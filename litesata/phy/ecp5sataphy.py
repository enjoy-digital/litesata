#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# ECP5 SATA PHY.
#
# The ECP5 DCU has no hardware SATA OOB support (contrary to Xilinx transceivers), so the OOB
# sequencing (COMRESET/COMINIT/COMWAKE generation/detection) is implemented in the fabric:
# - TX: OOB bursts are generated through the DCU's LDR (Low Data Rate) direct pad drive
#   (CHx_FFC_LDR_CORE2TX_EN/CHx_LDR_CORE2TX), with electrical idle (CHx_FFC_EI_EN) held for the
#   whole OOB phase. Burst timing is defined solely by the fast LDR enable; the slow (~220ns)
#   response of the electrical idle control is therefore harmless.
# - RX: OOB bursts/gaps are detected by measuring the idle gap durations, using either the DCU's
#   RX Loss-Of-Signal detector (CHx_FFS_RLOS, default) or transition activity on the raw LDR line
#   observation (CHx_LDR_RX2CORE), selectable at runtime.

from math import ceil

from litesata.common import *
from litesata.common import _PulseSynchronizer, _RisingEdge

from migen.genlib.cdc       import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.soc.interconnect.csr import *

from litesata.phy.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

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

        # TX/RX clocking ---------------------------------------------------------------------------
        #   (gen1) sata_tx/sata_rx clks @  75MHz (16-bits)
        #   (gen2) sata_tx/sata_rx clks @ 150MHz (16-bits)
        self.comb += self.cd_sata_tx.clk.eq(serdes.cd_tx.clk)
        self.comb += self.cd_sata_rx.clk.eq(serdes.cd_rx.clk)

        # Reset for SATA TX/RX clock domains -------------------------------------------------------
        # Note: only the datapath domains are reset here; the OOB COMGenerator lives in the SerDes
        # local "tx" domain (reset by the SerDes init) and thus survives ctrl's tx/rx_reset pulses.
        self.specials += [
            AsyncResetSynchronizer(self.cd_sata_tx, ~serdes.tx_ready | self.tx_reset),
            AsyncResetSynchronizer(self.cd_sata_rx, ~serdes.rx_ready | self.rx_reset),
        ]

# COMGenerator -------------------------------------------------------------------------------------

class COMGenerator(LiteXModule):
    """SATA OOB burst generator (LDR direct pad drive).

    Generates COMRESET/COMINIT or COMWAKE sequences: 6 bursts of 160UI, separated by idle gaps of
    480UI (COMRESET/COMINIT) or 160UI (COMWAKE). OOB timing is always relative to the Gen1 UI
    (0.667ns) whatever the linerate. Burst content is a square wave at tx_clk/2/(toggle_div+1),
    detection on the device side being amplitude (squelch) based.

    Must be clocked in the SerDes "tx" (word clock) domain.
    """
    def __init__(self, tx_clk_freq):
        self.cominit     = Signal() # i (pulse or level)
        self.comwake     = Signal() # i (pulse or level)
        self.finish      = Signal() # o (pulse)
        self.active      = Signal() # o (level)
        self.toggle_div  = Signal(4) # i (square wave half-period, in tx_clk cycles, minus 1)

        self.tx_oob_en   = Signal() # o
        self.tx_oob_data = Signal() # o
        self.tx_idle     = Signal() # o

        # # #

        # OOB timing (in tx_clk cycles, from Gen1 UI = 1/1.5GHz) -----------------------------------
        cycles       = lambda ui: round(ui*tx_clk_freq/1.5e9)
        burst_cycles = cycles(160) # 106.7ns:  8 cycles @ 75MHz (gen1) / 16 cycles @ 150MHz (gen2).
        wake_cycles  = cycles(160) # 106.7ns COMWAKE          inter-burst gap.
        init_cycles  = cycles(480) # 320.0ns COMRESET/COMINIT inter-burst gap.
        assert burst_cycles >= 4
        self.burst_cycles = burst_cycles
        self.wake_cycles  = wake_cycles

        # Shaped EI request (compensates the slow FFC_EI_EN response, all runtime adjustable):
        # - ei_lead : raise the EI request this many cycles before the burst ends (the LDR drive
        #             keeps the burst alive until EI physically engages).
        # - ei_trail: drop the EI request this many cycles before the gap ends.
        # - wake_gap: COMWAKE inter-burst gap in cycles (device detect window is 55-175ns, so the
        #             gap may be stretched to gain EI engage margin).
        self.ei_lead  = Signal(5)                    # i
        self.ei_trail = Signal(4)                    # i
        self.wake_gap = Signal(6, reset=wake_cycles) # i
        self.gap_mode = Signal()                     # i: 0 = EI gaps / 1 = LDR-constant gaps
                                                     #    (no transitions, EI off in-sequence).
        self.ei_req   = Signal()                     # o

        # Square wave generation -------------------------------------------------------------------
        square    = Signal()
        in_gap    = Signal()
        div_count = Signal(4)
        self.sync += [
            If(div_count == 0,
                square.eq(~square),
                div_count.eq(self.toggle_div),
            ).Else(
                div_count.eq(div_count - 1)
            )
        ]
        # In LDR-constant gap mode the LDR keeps driving a constant low level during gaps.
        self.comb += self.tx_oob_data.eq(square & ~(in_gap & self.gap_mode))

        # Burst/Gap sequencing ---------------------------------------------------------------------
        count   = Signal(8)
        loops   = Signal(3)
        is_wake = Signal()

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.cominit | self.comwake,
                NextValue(is_wake, self.comwake & ~self.cominit),
                NextValue(count, burst_cycles - 1),
                NextValue(loops, 6 - 1),
                NextState("BURST")
            )
        )
        fsm.act("BURST",
            self.active.eq(1),
            self.tx_idle.eq(1),
            self.tx_oob_en.eq(1),
            self.ei_req.eq(~self.gap_mode & (count < self.ei_lead)), # Raise EI request in the burst tail.
            NextValue(count, count - 1),
            If(count == 0,
                NextValue(count, Mux(is_wake, self.wake_gap - 1, init_cycles - 1)),
                NextState("GAP")
            )
        )
        fsm.act("GAP",
            self.active.eq(1),
            self.tx_idle.eq(1),
            in_gap.eq(1),
            self.tx_oob_en.eq(self.gap_mode), # LDR-constant gaps: keep driving (constant level).
            self.ei_req.eq(~self.gap_mode & (count >= self.ei_trail)), # EI gaps: request w/ trail.
            NextValue(count, count - 1),
            If(count == 0,
                If(loops == 0,
                    NextState("FINISH")
                ).Else(
                    NextValue(loops, loops - 1),
                    NextValue(count, burst_cycles - 1),
                    NextState("BURST")
                )
            )
        )
        fsm.act("FINISH",
            self.active.eq(1),
            self.tx_idle.eq(1),
            self.ei_req.eq(1),
            self.finish.eq(1),
            NextState("IDLE")
        )

# COMChecker ---------------------------------------------------------------------------------------

class COMChecker(LiteXModule):
    """SATA OOB sequence detector (idle gap duration classifier).

    Measures the duration of idle gaps between received bursts on rx_idle (from RLOS or LDR line
    activity) and classifies them as COMWAKE (55-175ns) or COMINIT/COMRESET (175-525ns) gaps. After
    n_gaps qualifying gaps, asserts the corresponding detect output as a level (mirroring Xilinx
    RXCOMINITDET/RXCOMWAKEDET behavior expected by LiteSATAPHYCtrl), deasserted once the line has
    been quiet for > 2us.

    Runs in the sys clock domain (rx_idle must already be synchronized).
    """
    def __init__(self, clk_freq, n_gaps=4):
        self.rx_idle     = Signal() # i
        self.cominit_det = Signal() # o (level)
        self.comwake_det = Signal() # o (level)

        # Detection windows (in sys_clk cycles, runtime adjustable, reset = SATA spec).
        self.comwake_gap_min = Signal(16, reset=ceil( 55e-9*clk_freq))
        self.comwake_gap_max = Signal(16, reset=int( 175e-9*clk_freq))
        self.cominit_gap_min = Signal(16, reset=ceil(175e-9*clk_freq))
        self.cominit_gap_max = Signal(16, reset=int( 525e-9*clk_freq))

        # Status (for LiteScope/debug).
        self.gap_count    = Signal(16)
        self.cominit_gaps = Signal(3)
        self.comwake_gaps = Signal(3)

        # # #

        quiet_cycles = int(2e-6*clk_freq)
        assert quiet_cycles < 2**16

        rx_idle_d = Signal()
        gap_end   = Signal()
        self.sync += rx_idle_d.eq(self.rx_idle)
        self.comb += gap_end.eq(rx_idle_d & ~self.rx_idle) # Idle -> Burst transition.

        # Idle gap duration counter (saturating).
        self.sync += [
            If(self.rx_idle,
                If(self.gap_count != (2**16 - 1),
                    self.gap_count.eq(self.gap_count + 1)
                )
            ).Else(
                self.gap_count.eq(0)
            )
        ]

        # Gap classification on Idle -> Burst transition.
        comwake_gap = Signal()
        cominit_gap = Signal()
        self.comb += [
            comwake_gap.eq((self.gap_count >= self.comwake_gap_min) & (self.gap_count <= self.comwake_gap_max)),
            cominit_gap.eq((self.gap_count >= self.cominit_gap_min) & (self.gap_count <= self.cominit_gap_max)),
        ]
        self.sync += [
            If(gap_end,
                If(comwake_gap,
                    If(self.comwake_gaps != (n_gaps - 1),
                        self.comwake_gaps.eq(self.comwake_gaps + 1)
                    ),
                    self.cominit_gaps.eq(0),
                ).Elif(cominit_gap,
                    If(self.cominit_gaps != (n_gaps - 1),
                        self.cominit_gaps.eq(self.cominit_gaps + 1)
                    ),
                    self.comwake_gaps.eq(0),
                ).Else(
                    # Out of window gap: restart both counts.
                    self.cominit_gaps.eq(0),
                    self.comwake_gaps.eq(0),
                )
            ),
            # Quiet line: sequence is over, deassert detections and re-arm.
            If(self.gap_count == quiet_cycles,
                self.cominit_det.eq(0),
                self.comwake_det.eq(0),
                self.cominit_gaps.eq(0),
                self.comwake_gaps.eq(0),
            ),
            # Assert detections when enough qualifying gaps have been seen.
            If(gap_end & comwake_gap & (self.comwake_gaps == (n_gaps - 1)),
                self.comwake_det.eq(1)
            ),
            If(gap_end & cominit_gap & (self.cominit_gaps == (n_gaps - 1)),
                self.cominit_det.eq(1)
            ),
        ]

# ECP5LiteSATAPHY ----------------------------------------------------------------------------------

class ECP5LiteSATAPHY(LiteXModule):
    def __init__(self, refclk, pads, gen, clk_freq, data_width=16, dual=0, channel=0, refclk_freq=None,
        oob_config={"ei", "ldr_tx", "ldr_rx"}):
        assert data_width in [16]
        assert gen in ["gen1", "gen2"]
        # Common signals
        self.data_width     = data_width
        self.clk_freq       = clk_freq

        # Control
        self.ready          = Signal() # o

        self.tx_idle        = Signal() # i
        self.tx_polarity    = Signal() # i

        self.tx_cominit_stb = Signal() # i
        self.tx_cominit_ack = Signal() # o
        self.tx_comwake_stb = Signal() # i
        self.tx_comwake_ack = Signal() # o

        self.rx_idle        = Signal() # o
        self.rx_cdrhold     = Signal() # i
        self.rx_polarity    = Signal() # i

        self.rx_cominit_stb = Signal() # o
        self.rx_comwake_stb = Signal() # o

        self.rxdisperr      = Signal(data_width//8) # o
        self.rxnotintable   = Signal(data_width//8) # o

        # Datapath
        self.sink           = stream.Endpoint(phy_description(data_width))
        self.source         = stream.Endpoint(phy_description(data_width))

        # Debug/Experiment controls (quasi-static, see add_oob_csr).
        self.oob_rx_sel      = Signal()  # i: RX OOB squelch source: 0 = RLOS / 1 = LDR activity.
        self.ei_mode         = Signal()  # i: 0 = EI masked during LDR drive / 1 = EI held.
        self.oob_cdrhold_dis = Signal()  # i: 1 = ignore ctrl's rx_cdrhold request.
        self.oob_tx_test     = Signal()  # i: 1 = free-running COMWAKE generation (TX measurement).
        self.oob_toggle_div  = Signal(4) # i: OOB burst square wave half-period - 1 (tx_clk cycles).

        # ECP5 specific signals
        self.rxcharisk      = Signal(data_width//8)
        self.rxdata         = Signal(data_width)
        self.rxcominitdet   = Signal()
        self.rxcomwakedet   = Signal()

        self.txcharisk      = Signal(data_width//8)
        self.txdata         = Signal(data_width)
        self.txelecidle     = Signal(reset=1)
        self.txcomfinish    = Signal()
        self.txcominit      = Signal()
        self.txcomwake      = Signal()

        # # #

        linerate = {"gen1": 1.5e9, "gen2": 3.0e9}[gen]
        tx_clk_freq = linerate/20

        # PLL --------------------------------------------------------------------------------------
        # Default refclk = linerate/20, selecting the x20 DCU PLL multiplier: the x10 multiplier
        # setting has been observed non-functional on hardware (VCO at 2x the requested rate, TX
        # word clock dead); all known-working ECP5 SerDes configs use x20/x25.
        if refclk_freq is None:
            refclk_freq = linerate/20
        assert isinstance(refclk, (Signal, ClockSignal)), "ECP5 PHY expects a refclk Signal (from a fabric PLL)."
        self.pll = SerDesECP5PLL(refclk, refclk_freq=refclk_freq, linerate=linerate)

        # SerDes -----------------------------------------------------------------------------------
        self.serdes = serdes = SerDesECP5(self.pll,
            tx_pads     = Pads(pads.tx_p, pads.tx_n),
            rx_pads     = Pads(pads.rx_p, pads.rx_n),
            dual        = dual,
            channel     = channel,
            data_width  = 20,
            tx_polarity = self.tx_polarity,
            rx_polarity = self.rx_polarity,
            oob_config  = oob_config,
        )
        serdes.add_stream_endpoints()

        # Ready ------------------------------------------------------------------------------------
        self.comb += self.ready.eq(serdes.tx_ready & serdes.rx_ready)

        # Datapath ---------------------------------------------------------------------------------
        self.comb += [
            serdes.sink.data.eq(self.txdata),
            serdes.sink.ctrl.eq(self.txcharisk),
            self.rxdata.eq(serdes.source.data),
            self.rxcharisk.eq(serdes.source.ctrl),
        ]

        self.sync.sata_rx += [
            self.source.valid.eq(1),
            self.source.charisk.eq(self.rxcharisk),
            self.source.data.eq(self.rxdata)
        ]

        self.sync.sata_tx += [
            self.txcharisk.eq(self.sink.charisk),
            self.txdata.eq(self.sink.data),
            self.sink.ready.eq(1),
        ]

        # 8b10b decode errors (rx -> sys) ----------------------------------------------------------
        rxnotintable = Signal(data_width//8)
        self.comb += rxnotintable.eq(Cat(*[serdes.decoders[i].invalid for i in range(data_width//8)]))
        self.specials += MultiReg(rxnotintable, self.rxnotintable, "sys")
        self.comb += self.rxdisperr.eq(0) # Not provided by the fabric 8b10b decoder (status only).

        # Electrical idle / CDR hold ---------------------------------------------------------------
        self.comb += [
            self.txelecidle.eq(self.tx_idle),
            serdes.tx_idle.eq(self.txelecidle),
            serdes.ei_mode.eq(self.ei_mode),
            serdes.rx_cdr_hold.eq(self.rx_cdrhold & ~self.oob_cdrhold_dis),
            self.rx_idle.eq(serdes.rx_idle),
        ]

        # TX OOB (COMRESET/COMINIT/COMWAKE generation) ---------------------------------------------
        self.com_gen = com_gen = ClockDomainsRenamer("tx")(COMGenerator(tx_clk_freq))

        # Debug: skip COMWAKE (out-of-spec experiment): immediately ack the ctrl's COMWAKE request
        # and synthesize a device COMWAKE response, so ctrl proceeds to ALIGN transmission right
        # after the device's COMINIT (some devices proceed on sustained ALIGN activity).
        self.oob_force_wake = Signal()
        force_wake_cnt      = Signal(16)
        force_wake_stb      = Signal()
        self.sync += [
            If(~self.oob_force_wake,
                force_wake_cnt.eq(0),
            ).Elif(self.tx_comwake_ack,
                force_wake_cnt.eq(1),
            ).Elif(force_wake_cnt != 0,
                force_wake_cnt.eq(force_wake_cnt + 1),
            ),
        ]
        self.comb += force_wake_stb.eq((force_wake_cnt > int(200e-9*clk_freq)) &
                                       (force_wake_cnt < int(1.2e-6*clk_freq)))

        # Generic <-> specific handshake (mirrors Xilinx TXCOMINIT/TXCOMWAKE/TXCOMFINISH).
        self.comb += [
            self.tx_cominit_ack.eq(self.tx_cominit_stb & self.txcomfinish),
            self.tx_comwake_ack.eq(self.tx_comwake_stb & (self.txcomfinish | self.oob_force_wake)),
        ]
        self.submodules += _RisingEdge(self.tx_cominit_stb, self.txcominit)
        self.submodules += _RisingEdge(self.tx_comwake_stb, self.txcomwake)

        # sys clk -> tx clk
        txcominit      = Signal()
        txcomwake      = Signal()
        oob_tx_test_tx = Signal()
        toggle_div_tx  = Signal(4)
        self.submodules += [
            _PulseSynchronizer(self.txcominit, "sys", txcominit, "tx"),
            _PulseSynchronizer(self.txcomwake, "sys", txcomwake, "tx"),
        ]
        self.specials += [
            MultiReg(self.oob_tx_test,    oob_tx_test_tx, "tx"),
            MultiReg(self.oob_toggle_div, toggle_div_tx,  "tx"),
        ]
        # Shaped EI controls (quasi-static, sys -> tx).
        self.oob_ei_lead  = Signal(5)
        self.oob_ei_trail = Signal(4)
        self.oob_wake_gap = Signal(6, reset=com_gen.wake_cycles)
        self.oob_gap_mode = Signal()
        ei_lead_tx  = Signal(5)
        ei_trail_tx = Signal(4)
        wake_gap_tx = Signal(6, reset=com_gen.wake_cycles)
        gap_mode_tx = Signal()
        self.specials += [
            MultiReg(self.oob_ei_lead,  ei_lead_tx,  "tx"),
            MultiReg(self.oob_ei_trail, ei_trail_tx, "tx"),
            MultiReg(self.oob_wake_gap, wake_gap_tx, "tx"),
            MultiReg(self.oob_gap_mode, gap_mode_tx, "tx"),
        ]

        self.comb += [
            com_gen.cominit.eq(txcominit),
            com_gen.comwake.eq(txcomwake | oob_tx_test_tx),
            com_gen.toggle_div.eq(toggle_div_tx),
            com_gen.ei_lead.eq(ei_lead_tx),
            com_gen.ei_trail.eq(ei_trail_tx),
            com_gen.wake_gap.eq(wake_gap_tx),
            com_gen.gap_mode.eq(gap_mode_tx),
            serdes.tx_oob_en.eq(com_gen.tx_oob_en),
            serdes.tx_oob_data.eq(com_gen.tx_oob_data),
            serdes.tx_oob_idle.eq(com_gen.tx_idle),
            serdes.tx_oob_active.eq(com_gen.active),
            serdes.tx_oob_ei_req.eq(com_gen.ei_req),
        ]

        # tx clk -> sys clk
        self.submodules += _PulseSynchronizer(com_gen.finish, "tx", self.txcomfinish, "sys")

        # RX OOB (COMINIT/COMWAKE detection) -------------------------------------------------------
        # Source A (default): RLOS squelch (serdes.rx_idle, already synchronized to sys).
        # Source B          : transition activity on the raw LDR line observation.
        rx_oob_data_sys = Signal()
        rx_oob_data_d   = Signal()
        self.specials += MultiReg(serdes.rx_oob_data, rx_oob_data_sys, "sys")

        self.ldr_timeout = Signal(8, reset=max(2, ceil(40e-9*clk_freq)))
        ldr_count        = Signal(8)
        self.ldr_idle    = Signal()
        self.sync += [
            rx_oob_data_d.eq(rx_oob_data_sys),
            If(rx_oob_data_sys ^ rx_oob_data_d,
                ldr_count.eq(0)
            ).Elif(~self.ldr_idle,
                ldr_count.eq(ldr_count + 1)
            )
        ]
        self.comb += self.ldr_idle.eq(ldr_count >= self.ldr_timeout)

        self.com_check = com_check = COMChecker(clk_freq)
        self.comb += [
            com_check.rx_idle.eq(Mux(self.oob_rx_sel, self.ldr_idle, serdes.rx_idle)),
            self.rxcominitdet.eq(com_check.cominit_det),
            self.rxcomwakedet.eq(com_check.comwake_det),
            self.rx_cominit_stb.eq(self.rxcominitdet),
            self.rx_comwake_stb.eq(self.rxcomwakedet | force_wake_stb),
        ]

    def add_oob_csr(self):
        self._oob_control = CSRStorage(fields=[
            CSRField("rx_sel", size=1, offset=0, values=[
                ("``0b0``", "RX OOB detection from RLOS (loss of signal)."),
                ("``0b1``", "RX OOB detection from LDR line activity.")],
            ),
            CSRField("ei_mode", size=1, offset=1, values=[
                ("``0b0``", "Electrical idle masked during LDR drive (LUNA-style)."),
                ("``0b1``", "Electrical idle held during LDR drive.")],
            ),
            CSRField("cdrhold_dis", size=1, offset=2, description="Ignore ctrl's rx_cdrhold request."),
            CSRField("tx_test",     size=1, offset=3, description="Free-running COMWAKE generation (TX timing measurement)."),
            CSRField("toggle_div",  size=4, offset=4, description="OOB burst square wave half-period - 1 (tx_clk cycles)."),
            CSRField("ldr_timeout", size=8, offset=8, reset=self.ldr_timeout.reset.value,
                description="LDR activity timeout (sys_clk cycles without transition = idle)."),
            CSRField("gap_mode", size=1, offset=16, values=[
                ("``0b0``", "OOB inter-burst gaps via electrical idle."),
                ("``0b1``", "OOB inter-burst gaps via LDR constant level (EI off in-sequence).")],
            ),
            CSRField("force_wake", size=1, offset=17,
                description="Skip COMWAKE: auto-ack TX and synthesize a device COMWAKE response."),
        ])
        self.comb += [
            self.oob_rx_sel.eq(     self._oob_control.fields.rx_sel),
            self.ei_mode.eq(        self._oob_control.fields.ei_mode),
            self.oob_cdrhold_dis.eq(self._oob_control.fields.cdrhold_dis),
            self.oob_tx_test.eq(    self._oob_control.fields.tx_test),
            self.oob_toggle_div.eq( self._oob_control.fields.toggle_div),
            self.ldr_timeout.eq(    self._oob_control.fields.ldr_timeout),
            self.oob_gap_mode.eq(   self._oob_control.fields.gap_mode),
            self.oob_force_wake.eq( self._oob_control.fields.force_wake),
        ]

        # Shaped EI request (lead/trail compensation + COMWAKE gap stretch), see COMGenerator.
        self._oob_ei_shape = CSRStorage(fields=[
            CSRField("lead",     size=5, offset=0),
            CSRField("trail",    size=4, offset=5),
            CSRField("wake_gap", size=6, offset=9, reset=self.oob_wake_gap.reset.value),
        ])
        self.comb += [
            self.oob_ei_lead.eq( self._oob_ei_shape.fields.lead),
            self.oob_ei_trail.eq(self._oob_ei_shape.fields.trail),
            self.oob_wake_gap.eq(self._oob_ei_shape.fields.wake_gap),
        ]

        # RX OOB gap detection windows (sys_clk cycles).
        com_check = self.com_check
        self._oob_comwake_gap = CSRStorage(fields=[
            CSRField("min", size=16, offset= 0, reset=com_check.comwake_gap_min.reset.value),
            CSRField("max", size=16, offset=16, reset=com_check.comwake_gap_max.reset.value),
        ])
        self._oob_cominit_gap = CSRStorage(fields=[
            CSRField("min", size=16, offset= 0, reset=com_check.cominit_gap_min.reset.value),
            CSRField("max", size=16, offset=16, reset=com_check.cominit_gap_max.reset.value),
        ])
        self.comb += [
            com_check.comwake_gap_min.eq(self._oob_comwake_gap.fields.min),
            com_check.comwake_gap_max.eq(self._oob_comwake_gap.fields.max),
            com_check.cominit_gap_min.eq(self._oob_cominit_gap.fields.min),
            com_check.cominit_gap_max.eq(self._oob_cominit_gap.fields.max),
        ]

        # RX OOB burst/gap duration recorders (sys_clk cycles): measure what the device actually
        # sends; gaps longer than 1us (inter-sequence quiet) are excluded from min/max.
        self._oob_rec = CSRStorage(fields=[
            CSRField("enable", size=1, description="Enable recorders (0 clears/holds them).")
        ])
        self._oob_rx_gap   = CSRStatus(fields=[
            CSRField("min", size=16, offset= 0), CSRField("max", size=16, offset=16)])
        self._oob_rx_burst = CSRStatus(fields=[
            CSRField("min", size=16, offset= 0), CSRField("max", size=16, offset=16)])
        self._oob_rx_count = CSRStatus(16, description="RX OOB bursts seen.")

        rec_en    = self._oob_rec.fields.enable
        rx_idle   = com_check.rx_idle
        rx_idle_d = Signal()
        burst_len = Signal(16)
        gap_min   = self._oob_rx_gap.fields.min
        gap_max   = self._oob_rx_gap.fields.max
        burst_min = self._oob_rx_burst.fields.min
        burst_max = self._oob_rx_burst.fields.max
        gap_bound = int(1e-6*self.clk_freq)
        self.sync += [
            rx_idle_d.eq(rx_idle),
            If(~rec_en,
                gap_min.eq(  2**16 - 1),
                gap_max.eq(  0),
                burst_min.eq(2**16 - 1),
                burst_max.eq(0),
                burst_len.eq(0),
                self._oob_rx_count.status.eq(0),
            ).Else(
                # Burst length counter.
                If(~rx_idle,
                    If(burst_len != (2**16 - 1),
                        burst_len.eq(burst_len + 1)
                    )
                ).Else(
                    burst_len.eq(0)
                ),
                # On Burst -> Idle transition: record burst length.
                If(~rx_idle_d & rx_idle & (burst_len != 0),
                    If(burst_len < burst_min, burst_min.eq(burst_len)),
                    If(burst_len > burst_max, burst_max.eq(burst_len)),
                    If(self._oob_rx_count.status != (2**16 - 1),
                        self._oob_rx_count.status.eq(self._oob_rx_count.status + 1)
                    ),
                ),
                # On Idle -> Burst transition: record gap length (excluding long quiet).
                If(rx_idle_d & ~rx_idle & (com_check.gap_count < gap_bound),
                    If(com_check.gap_count < gap_min, gap_min.eq(com_check.gap_count)),
                    If(com_check.gap_count > gap_max, gap_max.eq(com_check.gap_count)),
                ),
            )
        ]
