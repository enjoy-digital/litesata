#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2017 Johan Klockars <Johan.Klockars@hasselblad.com>
# Copyright (c) 2016 Olof Kindgren <olof.kindgren@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from litex.gen import *

from litesata.common import *

from litex.gen.genlib.misc import WaitTimer

# LiteSATAPHYCtrl ----------------------------------------------------------------------------------

class LiteSATAPHYCtrl(LiteXModule):
    """SATA PHY Controller

    Manages link reset/initialization and OOB sequence.

    This modules is mainly a state machine that:
    - Commands and checks transceivers reset/initialization.
    - Manages the SATA OOB sequence with the device.
    - Wait for link to be stable before declaring it ready.
    - Re-initialize the link when invalid data are received.

    The state machine is robust enough to handle hot plug/ power off/on sequences of the device
    # without reseting the FPGA core.

    Link rate fallback (opt-in, rate_fallback=True): the host starts at rate_max (0: gen1, 1: gen2,
    2: gen3) and steps down after rate_attempts ALIGN timeouts at a rate (the device answered the
    OOB sequence but did not lock), wrapping back to rate_max below rate_min. The rate is handed to
    the transceiver through trx.rate while the controller holds it in reset, and the OOB sequence
    only resumes once trx.rate_done is back. A lost link restarts from rate_max.
    """
    def __init__(self, trx, crg, clk_freq, rate_fallback=False, rate_max=2, rate_min=0, rate_attempts=2):
        self.clk_freq = clk_freq
        self.ready    = Signal()
        self.sink     = sink   = stream.Endpoint(phy_description(32))
        self.source   = source = stream.Endpoint(phy_description(32))
        self.misalign = Signal()
        self.tx_idle  = Signal()
        self.rx_reset = Signal()
        self.tx_reset = Signal()
        self.rx_idle  = Signal()

        # Link rate fallback (0: gen1, 1: gen2, 2: gen3).
        self.rate_fallback = rate_fallback
        self.rate          = Signal(2, reset=rate_max)     # Current (negotiated once ready) rate.
        self.rate_max_i    = Signal(2, reset=rate_max)     # Highest rate to try (CSR-overridable).
        self.rate_min_i    = Signal(2, reset=rate_min)     # Lowest rate to try.
        self.rate_count    = [Signal(8) for _ in range(3)] # Attempts per rate (saturating).

        # # #

        # Transceiver rate handshake (only PHYs with runtime rate support expose it).
        trx_rate      = getattr(trx, "rate",      None)
        trx_rate_done = getattr(trx, "rate_done", None)
        if rate_fallback:
            assert trx_rate is not None and trx_rate_done is not None, "Rate fallback requires trx.rate/rate_done."
        crg_ready = Signal()
        self.comb += crg_ready.eq((trx.ready & trx_rate_done) if rate_fallback else trx.ready)

        # Always transmitting / receiving.
        self.comb += source.valid.eq(1)
        self.comb += sink.ready.eq(1)

        # Retry / Align count/timers.
        self.retry_timer = retry_timer = WaitTimer(self.us(10000))
        self.align_timer = align_timer = WaitTimer(self.us(873))
        align_count = Signal(4)

        # Drive Transceiver/CRG idle/reset from internal logic.
        self.sync += trx.tx_idle.eq(self.tx_idle)
        self.sync += crg.rx_reset.eq(self.rx_reset)
        self.sync += crg.tx_reset.eq(self.tx_reset)

        # FSM.
        self.fsm = fsm = ResetInserter()(FSM(reset_state="RESET"))
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
            If(crg_ready,
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
        stability_timer = WaitTimer(5e-3*clk_freq)
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

        # Rate fallback policy ---------------------------------------------------------------------
        if rate_fallback:
            rate         = self.rate
            attempt      = Signal(max=max(rate_attempts, 2))
            align_done_d = Signal()
            step         = Signal()
            link_lost    = Signal()
            rate_clamped = Signal(2)
            self.sync += align_done_d.eq(align_timer.done)
            self.comb += [
                # Only an ALIGN timeout steps the rate (the device answered the OOB sequence but did
                # not lock at this rate); COMINIT/COMWAKE timeouts are rate independent. The timer's
                # done stays high while the FSM is held in reset, so use its rising edge.
                step.eq(align_timer.done & ~align_done_d),
                link_lost.eq(fsm.ongoing("READY") & self.rx_idle),
                If(rate > self.rate_max_i,
                    rate_clamped.eq(self.rate_max_i)
                ).Elif(rate < self.rate_min_i,
                    rate_clamped.eq(self.rate_min_i)
                ).Else(
                    rate_clamped.eq(rate)
                ),
            ]
            self.sync += [
                If(step,
                    If(attempt == (rate_attempts - 1),
                        attempt.eq(0),
                        If(rate > self.rate_min_i,
                            rate.eq(rate - 1)
                        ).Else(
                            rate.eq(self.rate_max_i)
                        )
                    ).Else(
                        attempt.eq(attempt + 1)
                    )
                ).Elif(link_lost,
                    attempt.eq(0),
                    rate.eq(self.rate_max_i)
                ).Elif(fsm.ongoing("RESET"),
                    rate.eq(rate_clamped)
                ),
                # The transceiver rate only changes while the controller holds it in reset; the PHY
                # then drops rate_done until the new rate is applied.
                If(fsm.ongoing("RESET"),
                    trx_rate.eq(rate_clamped)
                ),
            ]
            # Attempts per rate (one per COMINIT).
            cominit_d = Signal()
            self.sync += cominit_d.eq(fsm.ongoing("COMINIT"))
            for i, count in enumerate(self.rate_count):
                self.sync += If(fsm.ongoing("COMINIT") & ~cominit_d & (rate == i) & (count != 0xff),
                    count.eq(count + 1)
                )

    def us(self, t):
        clk_period_us = 1e6/self.clk_freq
        return ceil(t/clk_period_us)

# LiteSATAPHYLinkMonitor ---------------------------------------------------------------------------

class LiteSATAPHYLinkMonitor(LiteXModule):
    """Debounced link up/down events derived from the PHY ready signal.

    link_up rises as soon as ready is asserted (ready already includes the controller's stability
    time) and falls only once ready has stayed low for down_cycles, so the short re-initializations
    performed by the controller (misalignment, idle retries) do not generate events while a device
    unplug does. connect/disconnect are single-cycle pulses (EventSourcePulse compatible).
    """
    def __init__(self, ready, down_cycles):
        self.link_up    = Signal()
        self.connect    = Signal()
        self.disconnect = Signal()

        # # #

        self.timer = timer = WaitTimer(down_cycles)
        link_up_d = Signal()
        self.comb += timer.wait.eq(~ready)
        self.sync += [
            If(ready,
                self.link_up.eq(1)
            ).Elif(timer.done,
                self.link_up.eq(0)
            ),
            link_up_d.eq(self.link_up),
            self.connect.eq(self.link_up & ~link_up_d),
            self.disconnect.eq(~self.link_up & link_up_d),
        ]
