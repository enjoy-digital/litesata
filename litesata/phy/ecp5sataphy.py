#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from math import ceil

from migen.genlib.cdc       import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from liteiclink.serdes.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

from litesata.common import *
from litesata.common import _PulseSynchronizer
from litesata.phy.ctrl import LiteSATAPHYCtrl

# Pads ---------------------------------------------------------------------------------------------

class Pads:
    def __init__(self, p, n):
        self.p = p
        self.n = n

# ECP5LiteSATAPHYCRG -------------------------------------------------------------------------------

class ECP5LiteSATAPHYCRG(LiteXModule):
    def __init__(self, phy):
        self.tx_reset = Signal()
        self.rx_reset = Signal()

        self.cd_sata_tx = ClockDomain()
        self.cd_sata_rx = ClockDomain()

        # # #

        serdes = phy.serdes
        self.comb += [
            self.cd_sata_tx.clk.eq(serdes.cd_tx.clk),
            self.cd_sata_rx.clk.eq(serdes.cd_rx.clk),
        ]
        self.specials += [
            AsyncResetSynchronizer(self.cd_sata_tx, ~serdes.tx_ready | self.tx_reset),
            AsyncResetSynchronizer(self.cd_sata_rx, ~serdes.rx_ready | self.rx_reset),
        ]

# ECP5 SATA OOB Generator --------------------------------------------------------------------------

class ECP5SATAOOBGenerator(LiteXModule):
    """Generate the six burst/gap pairs of a SATA OOB sequence."""
    def __init__(self, tx_clk_freq):
        self.cominit = Signal()
        self.comwake = Signal()
        self.finish  = Signal()
        self.active  = Signal()
        self.gap     = Signal()

        # # #

        # SATA OOB timing uses Gen1 unit intervals, also on Gen2 links.
        cycles       = lambda ui: round(ui*tx_clk_freq/1.5e9)
        burst_cycles = cycles(160)
        wake_cycles  = cycles(160)
        init_cycles  = cycles(480)
        assert burst_cycles >= 4

        count       = Signal(8)
        bursts_left = Signal(3)  # Five remaining after the first burst gives six total.
        is_wake     = Signal()

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.cominit | self.comwake,
                NextValue(is_wake, self.comwake & ~self.cominit),
                # Release electrical idle one TX cycle before the measured sequence.
                NextValue(count, 0),
                NextValue(bursts_left, 5),
                NextState("PRE")
            )
        )
        fsm.act("PRE",
            self.active.eq(1),
            NextValue(count, count - 1),
            If(count == 0,
                NextValue(count, burst_cycles - 1),
                NextState("BURST")
            )
        )
        fsm.act("BURST",
            self.active.eq(1),
            NextValue(count, count - 1),
            If(count == 0,
                NextValue(count, Mux(is_wake, wake_cycles - 1, init_cycles - 1)),
                NextState("GAP")
            )
        )
        fsm.act("GAP",
            self.active.eq(1),
            self.gap.eq(1),
            NextValue(count, count - 1),
            If(count == 0,
                If(bursts_left == 0,
                    NextState("FINISH")
                ).Else(
                    NextValue(bursts_left, bursts_left - 1),
                    NextValue(count, burst_cycles - 1),
                    NextState("BURST")
                )
            )
        )
        fsm.act("FINISH",
            self.active.eq(1),
            self.gap.eq(1),
            self.finish.eq(1),
            NextState("WAIT")
        )
        fsm.act("WAIT",
            If(~self.cominit & ~self.comwake,
                NextState("IDLE")
            )
        )

# ECP5 SATA OOB Checker ----------------------------------------------------------------------------

class ECP5SATAOOBChecker(LiteXModule):
    """Classify the idle gaps of received COMINIT and COMWAKE sequences."""
    def __init__(self, clk_freq):
        self.rx_idle     = Signal()
        self.cominit_det = Signal()
        self.comwake_det = Signal()

        # # #

        # Classify COMWAKE/COMINIT from ECP5 RLOS gap timing.
        wake_min = ceil( 55e-9*clk_freq)
        wake_max = int( 175e-9*clk_freq)
        init_min = ceil(175e-9*clk_freq)
        init_max = int( 525e-9*clk_freq)
        quiet    = 32

        rx_idle_d = Signal()
        gap_end   = Signal()
        gap_count = Signal(16)
        init_gaps = Signal(3)
        wake_gaps = Signal(3)
        self.sync += rx_idle_d.eq(self.rx_idle)
        self.comb += gap_end.eq(rx_idle_d & ~self.rx_idle)

        self.sync += [
            If(self.rx_idle,
                If(gap_count != (2**16 - 1),
                    gap_count.eq(gap_count + 1)
                )
            ).Else(
                gap_count.eq(0)
            )
        ]

        wake_gap = Signal()
        init_gap = Signal()
        self.comb += [
            wake_gap.eq((gap_count >= wake_min) & (gap_count <= wake_max)),
            init_gap.eq((gap_count >= init_min) & (gap_count <= init_max)),
        ]

        # Four gaps identify the six-burst sequence; clear detection after a quiet interval.
        self.sync += [
            If(gap_end,
                If(wake_gap,
                    If(wake_gaps != 3, wake_gaps.eq(wake_gaps + 1)),
                    init_gaps.eq(0),
                ).Elif(init_gap,
                    If(init_gaps != 3, init_gaps.eq(init_gaps + 1)),
                    wake_gaps.eq(0),
                ).Else(
                    init_gaps.eq(0),
                    wake_gaps.eq(0),
                )
            ),
            If(gap_count == quiet,
                self.cominit_det.eq(0),
                self.comwake_det.eq(0),
                init_gaps.eq(0),
                wake_gaps.eq(0),
            ),
            If(gap_end & wake_gap & (wake_gaps == 3),
                self.comwake_det.eq(1)
            ),
            If(gap_end & init_gap & (init_gaps == 3),
                self.cominit_det.eq(1)
            ),
        ]

# ECP5LiteSATAPHYCtrl ------------------------------------------------------------------------------

class ECP5LiteSATAPHYCtrl(LiteSATAPHYCtrl):
    """Adapt the OOB/ALIGN handoff to ECP5 RLOS behavior and early device ALIGN."""
    def __init__(self, trx, crg, clk_freq):
        LiteSATAPHYCtrl.__init__(self, trx, crg, clk_freq)

        sink        = self.sink
        source      = self.source
        fsm         = self.fsm
        align_count = Signal(2)

        # Latch ALIGN received before the controller has left COMWAKE.
        align_seen     = Signal()
        align_polarity = Signal()
        self.sync += If(fsm.ongoing("COMWAKE"),
            align_seen.eq(0)
        ).Elif(
            sink.valid &
            (sink.charisk == 0b0001) &
            ((sink.data == primitives["ALIGN"]) | (sink.data == primitives["ALIGN_N"])),
            align_seen.eq(1),
            align_polarity.eq(sink.data == primitives["ALIGN_N"])
        )

        # Bound sticky COMWAKE and require a short ALIGN dwell.
        nocomwake_timer    = WaitTimer(ceil(0.4e-6*clk_freq))
        align_accept_timer = WaitTimer(ceil(20e-6*clk_freq))
        self.submodules += nocomwake_timer, align_accept_timer

        # Qualify the full dword while the receive aligner settles.
        valid_non_align = 0
        for name, value in primitives.items():
            if name not in ["ALIGN", "ALIGN_N"]:
                valid_non_align = valid_non_align | (sink.data == value)

        # Override only the ECP5 handoff states; the generic controller remains unchanged.
        fsm.actions["AWAIT-NO-COMWAKE"] = [
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            nocomwake_timer.wait.eq(1),
            If(~trx.rx_comwake_stb | nocomwake_timer.done,
                NextState("AWAIT-ALIGN")
            )
        ]
        fsm.actions["AWAIT-ALIGN"] = [
            # Keep the CDR tracking the device's ALIGN stream.
            source.data.eq(0x4a4a4a4a),  # D10.2
            source.charisk.eq(0b0000),
            self.align_timer.wait.eq(1),
            If(align_seen,
                NextValue(trx.rx_polarity, align_polarity),
                # Require three consecutive qualified primitives before entering READY.
                NextValue(align_count, 2),
                NextState("SEND-ALIGN")
            )
        ]
        fsm.actions["SEND-ALIGN"] = [
            self.align_timer.wait.eq(1),
            align_accept_timer.wait.eq(1),
            source.data.eq(primitives["ALIGN"]),
            source.charisk.eq(0b0001),
            If(sink.valid,
                # Accept repeated ALIGN after a dwell; accept other primitives immediately.
                If((sink.charisk == 0b0001) &
                   (valid_non_align |
                    ((sink.data == primitives["ALIGN"]) & align_accept_timer.done)),
                    If(align_count == 0,
                        NextState("READY")
                    ).Else(
                        NextValue(align_count, align_count - 1)
                    )
                ).Else(
                    NextValue(align_count, 2)
                )
            )
        ]

# ECP5LiteSATAPHY ----------------------------------------------------------------------------------

class ECP5LiteSATAPHY(LiteXModule):
    def __init__(self, refclk, pads, gen, clk_freq, data_width=16, dual=0, channel=0,
        refclk_freq=150e6):
        assert gen == "gen2"
        assert data_width == 16
        assert isinstance(refclk, (Signal, ClockSignal))

        self.data_width = data_width

        # Control.
        self.ready          = Signal()
        self.tx_idle        = Signal()
        self.tx_polarity    = Signal()
        self.tx_cominit_stb = Signal()
        self.tx_cominit_ack = Signal()
        self.tx_comwake_stb = Signal()
        self.tx_comwake_ack = Signal()
        self.rx_idle        = Signal()
        self.rx_cdrhold     = Signal()
        self.rx_polarity    = Signal()
        self.rx_cominit_stb = Signal()
        self.rx_comwake_stb = Signal()
        self.rxdisperr      = Signal(data_width//8)
        self.rxnotintable   = Signal(data_width//8)

        # Datapath.
        self.sink   = stream.Endpoint(phy_description(data_width))
        self.source = stream.Endpoint(phy_description(data_width))

        # # #

        linerate    = 3e9
        tx_clk_freq = linerate/20

        self.pll = SerDesECP5PLL(refclk, refclk_freq=refclk_freq, linerate=linerate)
        self.serdes = serdes = SerDesECP5(self.pll,
            tx_pads        = Pads(pads.tx_p, pads.tx_n),
            rx_pads        = Pads(pads.rx_p, pads.rx_n),
            dual           = dual,
            channel        = channel,
            data_width     = 20,
            tx_polarity    = self.tx_polarity,
            rx_polarity    = self.rx_polarity,
            with_oob       = True,
        )
        serdes.add_stream_endpoints()

        # Use LUNA's ECP5 receive settings and decouple CDR acquisition from RLOS.
        serdes.serdes_params.update(
            p_CHX_REQ_EN         = "0b1",
            p_CHX_REQ_LVL_SET    = "0b01",
            p_CHX_RXTERM_CM      = "0b10",
            p_CHX_RX_LOS_HYST_EN = "0b0",
            p_CHX_RX_LOS_LVL     = "0b010",
            p_CHX_PDEN_SEL       = "0b0",
            p_D_CDR_LOL_SET      = "0b10",
        )

        # TX readiness starts COMRESET since no receive signal exists yet.
        self.comb += [
            self.ready.eq(serdes.tx_ready),
            serdes.tx_idle.eq(self.tx_idle),
            serdes.rx_cdr_hold.eq(self.rx_cdrhold),
            self.rx_idle.eq(serdes.rx_idle),
        ]

        # SATA datapath.
        tx_data    = Signal(data_width)
        tx_charisk = Signal(data_width//8)
        self.comb += [
            serdes.sink.data.eq(tx_data),
            serdes.sink.ctrl.eq(tx_charisk),
        ]
        self.sync.sata_rx += [
            self.source.valid.eq(1),
            self.source.charisk.eq(serdes.source.ctrl),
            self.source.data.eq(serdes.source.data),
        ]

        # Send SYNC while idle; persistent D0.0 makes devices drop a healthy link.
        sync_half = Signal()
        self.sync.sata_tx += [
            If(self.sink.valid,
                tx_charisk.eq(self.sink.charisk),
                tx_data.eq(self.sink.data),
                sync_half.eq(0),
            ).Else(
                sync_half.eq(~sync_half),
                If(sync_half,
                    tx_charisk.eq(0b00),
                    tx_data.eq(primitives["SYNC"] >> 16),
                ).Else(
                    tx_charisk.eq(0b01),
                    tx_data.eq(primitives["SYNC"]),
                )
            ),
            self.sink.ready.eq(1),
        ]

        rxnotintable = Signal(data_width//8)
        self.comb += [
            rxnotintable.eq(Cat(*[decoder.invalid for decoder in serdes.decoders])),
            self.rxdisperr.eq(0),
        ]
        self.specials += MultiReg(rxnotintable, self.rxnotintable, "sys")

        # TX OOB.
        self.com_gen = com_gen = ClockDomainsRenamer("tx")(ECP5SATAOOBGenerator(tx_clk_freq))
        tx_cominit = Signal()
        tx_comwake = Signal()
        self.specials += [
            MultiReg(self.tx_cominit_stb, tx_cominit, "tx"),
            MultiReg(self.tx_comwake_stb, tx_comwake, "tx"),
        ]
        self.comb += [
            com_gen.cominit.eq(tx_cominit),
            com_gen.comwake.eq(tx_comwake),
        ]

        tx_comfinish = Signal()
        self.submodules += _PulseSynchronizer(com_gen.finish, "tx", tx_comfinish, "sys")
        self.comb += [
            self.tx_cominit_ack.eq(self.tx_cominit_stb & tx_comfinish),
            self.tx_comwake_ack.eq(self.tx_comwake_stb & tx_comfinish),
        ]

        # Alternating Gen2 words form the Gen1 OOB carrier; a constant word forms the gap.
        pattern_toggle = Signal()
        self.sync.tx += pattern_toggle.eq(~pattern_toggle)
        self.comb += [
            serdes.tx_raw_enable.eq(com_gen.active),
            serdes.tx_raw_data.eq(Mux(com_gen.gap, 0,
                Mux(pattern_toggle, 0x0f0f0, 0xf0f0f))),
        ]

        # Debounce RLOS for three sys cycles before measuring OOB gaps.
        rx_idle_f = Signal()
        filter_count = Signal(2)
        self.sync += [
            If(serdes.rx_idle == rx_idle_f,
                filter_count.eq(0)
            ).Else(
                filter_count.eq(filter_count + 1),
                If(filter_count == 2,
                    rx_idle_f.eq(serdes.rx_idle),
                    filter_count.eq(0),
                )
            )
        ]

        self.com_check = com_check = ECP5SATAOOBChecker(clk_freq)
        self.comb += [
            com_check.rx_idle.eq(rx_idle_f),
            self.rx_cominit_stb.eq(com_check.cominit_det),
            self.rx_comwake_stb.eq(com_check.comwake_det),
        ]
