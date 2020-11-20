#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2017 Johan Klockars <Johan.Klockars@hasselblad.com>
# Copyright (c) 2016 Olof Kindgren <olof.kindgren@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *

from migen.genlib.misc import WaitTimer

# LiteSATAPHYCtrl ----------------------------------------------------------------------------------

class LiteSATAPHYCtrl(Module):
    """SATA PHY Controller

    Manages link reset/initialization and OOB sequence.

    This modules is mainly a state machine that:
    - Commands and checks transceivers reset/initialization.
    - Manages the SATA OOB sequence with the device.
    - Wait for link to be stable before declaring it ready.
    - Re-initialize the link when invalid data are received.

    The state machine is robust enough to handle hot plug/ power off/on sequences of the device
    # without reseting the FPGA core.
    """
    def __init__(self, trx, crg, clk_freq):
        self.clk_freq = clk_freq
        self.ready    = Signal()
        self.sink     = sink   = stream.Endpoint(phy_description(32))
        self.source   = source = stream.Endpoint(phy_description(32))
        self.misalign = Signal()
        self.tx_idle  = Signal()
        self.rx_reset = Signal()
        self.tx_reset = Signal()
        self.rx_idle  = Signal()

        # # #

        # Always transmitting / receiving
        self.comb += source.valid.eq(1)
        self.comb += sink.ready.eq(1)

        # Retry / Align count/timers.
        retry_timer = WaitTimer(self.us(10000))
        align_timer = WaitTimer(self.us(873))
        align_count = Signal(4)
        self.submodules += align_timer, retry_timer

        # Drive Transceiver/CRG idle/reset from internal logic.
        self.sync += trx.tx_idle.eq(self.tx_idle)
        self.sync += crg.rx_reset.eq(self.rx_reset)
        self.sync += crg.tx_reset.eq(self.tx_reset)

        self.submodules.fsm = fsm = ResetInserter()(FSM(reset_state="RESET"))
        self.comb += fsm.reset.eq(retry_timer.done | align_timer.done)
        fsm.act("RESET",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            self.rx_reset.eq(1),
            self.tx_reset.eq(1),
            NextState("AWAIT-CRG-RESET")
        )
        fsm.act("AWAIT-CRG-RESET",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            NextValue(align_count, 4-1),
            If(trx.ready,
                # Set RX polarity to 0 (we don't know it at this point).
                NextValue(trx.rx_polarity, 0),
                # Alternate TX polarity on each retry.
                NextValue(trx.tx_polarity, ~trx.tx_polarity),
                NextState("COMINIT")
            )
        )
        fsm.act("COMINIT",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            trx.tx_cominit_stb.eq(1),
            If(trx.tx_cominit_ack & ~trx.rx_cominit_stb,
                NextState("AWAIT-COMINIT")
            )
        )
        fsm.act("AWAIT-COMINIT",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            retry_timer.wait.eq(1),
            If(trx.rx_cominit_stb,
                NextState("AWAIT-NO-COMINIT")
            )
        )
        fsm.act("AWAIT-NO-COMINIT",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            retry_timer.wait.eq(1),
            If(~trx.rx_cominit_stb,
                NextState("CALIBRATE")
            )
        )
        fsm.act("CALIBRATE",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            NextState("COMWAKE"),
        )
        fsm.act("COMWAKE",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            trx.tx_comwake_stb.eq(1),
            If(trx.tx_comwake_ack,
                NextState("AWAIT-COMWAKE")
            )
        )
        fsm.act("AWAIT-COMWAKE",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            retry_timer.wait.eq(1),
            If(trx.rx_comwake_stb,
                NextState("AWAIT-NO-COMWAKE")
            )
        )
        fsm.act("AWAIT-NO-COMWAKE",
            self.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            If(~trx.rx_comwake_stb,
                NextState("AWAIT-ALIGN")
            )
        )
        fsm.act("AWAIT-ALIGN",
            trx.rx_cdrhold.eq(1),
            source.data.eq(0x4a4a4a4a),  # D10.2
            source.charisk.eq(0b0000),
            align_timer.wait.eq(1),
            If(~trx.rx_idle,
                If(sink.valid & (self.sink.charisk == 0b0001) & (self.sink.data == primitives["ALIGN"]),
                    NextValue(trx.rx_polarity, 0),
                    NextState("SEND-ALIGN")
                ),
                If(sink.valid & (self.sink.charisk == 0b0001) & (self.sink.data == primitives["ALIGN_N"]),
                    NextValue(trx.rx_polarity, 1),
                    NextState("SEND-ALIGN")
                ),
            )
        )
        fsm.act("SEND-ALIGN",
            align_timer.wait.eq(1),
            source.data.eq(primitives["ALIGN"]),
            source.charisk.eq(0b0001),
            If(sink.valid & (sink.charisk == 0b0001),
                If(sink.data[0:8] == 0x7c,
                    NextValue(align_count, align_count - 1),
                ).Else(
                    NextValue(align_count, 4-1),
                )
            ),
            If(align_count == 0,
                NextState("READY")
            )
        )

        # Wait alignment stability for 5ms before declaring ctrl is ready, reset the RX part of
        # the transceiver when misalignment is detected.
        stability_timer = WaitTimer(int(5e-3*clk_freq))
        self.submodules += stability_timer

        fsm.act("READY",
            source.data.eq(primitives["SYNC"]),
            source.charisk.eq(0b0001),
            stability_timer.wait.eq(1),
            self.ready.eq(stability_timer.done),
            If(self.rx_idle,
                NextState("RESET"),
            ).Elif(self.misalign,
                self.rx_reset.eq(1),
                NextState("RESET_RX")
            )
        )
        fsm.act("RESET_RX",
            If(trx.ready,
                NextState("READY")
            )
        )

    def us(self, t):
        clk_period_us = 1e6/self.clk_freq
        return ceil(t/clk_period_us)
