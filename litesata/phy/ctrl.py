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
    """
    def __init__(self, trx, crg, clk_freq, oob_retries=None, oob_backoff=1e-1,
                 align_cdr_hold=True,
                 align_timeout_us=873, retry_timeout_us=10000, nocomwake_timeout_us=None, stability_us=5000,
                 misalign_tolerance=0, align_needs_signal=True, align_accept_align=False):
        self.clk_freq = clk_freq
        self.ready    = Signal()
        self.sink     = sink   = stream.Endpoint(phy_description(32))
        self.source   = source = stream.Endpoint(phy_description(32))
        self.misalign = Signal()
        self.tx_idle  = Signal()
        self.rx_reset = Signal()
        self.tx_reset = Signal()
        self.rx_idle  = Signal()
        self.d102_phase = Signal() # o: transmitting the post-COMWAKE D10.2 filler

        # # #

        # Always transmitting / receiving.
        self.comb += source.valid.eq(1)
        self.comb += sink.ready.eq(1)

        # Retry / Align count/timers.
        # The SATA host value for the ALIGN wait is 873.8us, but a device that starts speed
        # negotiation at a rate we cannot receive (Gen3) may only reach our Gen2 window after
        # several ~54.6us step-downs plus its own retry delays. Waiting longer costs nothing and
        # lets us catch the ALIGN burst instead of resetting OOB just before it arrives.
        # Bounded wait for the device's COMWAKE to clear (see AWAIT-NO-COMWAKE).
        # `None` keeps the original behaviour of waiting for the detection to deassert.
        nocomwake_timer = WaitTimer(self.us(nocomwake_timeout_us if nocomwake_timeout_us else 1))
        self.submodules += nocomwake_timer
        retry_timer = WaitTimer(self.us(retry_timeout_us))
        align_timer = WaitTimer(self.us(align_timeout_us))
        align_count = Signal(4)
        self.submodules += align_timer, retry_timer

        # Drive Transceiver/CRG idle/reset from internal logic.
        self.sync += trx.tx_idle.eq(self.tx_idle)
        self.sync += crg.rx_reset.eq(self.rx_reset)
        self.sync += crg.tx_reset.eq(self.tx_reset)

        # FSM.
        # Loopback self-test support (ECP5 bench): when the PHY exposes an active echo-mask mode
        # (TX externally looped to RX), transmit ALIGN during AWAIT-ALIGN so the handshake can
        # complete against our own echo; a real device link transmits D10.2 there per spec.
        loopback = getattr(trx, "oob_echo_mask", None)
        if loopback is None:
            loopback = Signal()

        # OOB bypass (bench debug): jump straight from reset to the ALIGN exchange, skipping the
        # whole COMRESET/COMINIT/COMWAKE handshake. A plain TX->RX loopback cannot complete a SATA
        # OOB handshake by construction - the host transmits COMINIT only while in the COMINIT
        # state and is silent in AWAIT-COMINIT, so it never hears its own burst - which otherwise
        # makes the loopback useless for validating everything that happens AFTER OOB. With this
        # set, the loopback exercises the real post-OOB path end to end: ALIGN exchange ->
        # SEND-ALIGN -> READY -> the core's SYNC idle stream.
        oob_bypass = getattr(trx, "oob_bypass", None)
        if oob_bypass is None:
            oob_bypass = Signal()

        # Early D10.2 (ECP5): the spec expects the host's continuous D10.2 within 533ns of the
        # device's last COMWAKE burst, and real hosts start it as soon as COMWAKE is DETECTED,
        # i.e. while it is still being received. On ECP5 the COMWAKE-end detection alone takes the
        # quiet threshold (~356ns) and releasing electrical idle another 213-427ns, so a host that
        # waits for AWAIT-ALIGN to un-mute cannot meet the budget. With this set, AWAIT-NO-COMWAKE
        # transmits D10.2 instead of holding EI, so the driver is live and the D10.2 stream already
        # flowing when the device finishes its COMWAKE. Zero when the PHY doesn't expose the knob,
        # which keeps the original (Xilinx) behaviour.
        early_d102 = getattr(trx, "oob_early_d102", None)
        if early_d102 is None:
            early_d102 = Signal()

        # Runtime-selectable lenient SEND-ALIGN exit (counts the drive's ALIGNs as well as its
        # SYNCs), for A/B against the spec exit without a rebuild.
        lenient_exit = getattr(trx, "oob_lenient_exit", None)
        if lenient_exit is None:
            lenient_exit = Signal()

        # Mid-link retrain offer (runtime, absent/0 = original behaviour): a rising edge while in
        # READY jumps straight back to SEND-ALIGN with no serdes touch, so the line carries
        # SYNC...ALIGN with zero discontinuity. Rationale: a device whose speed-negotiation
        # qualifier missed our in-window ALIGN reply (fresh rate-hop, untrained CDR) may accept
        # the same ALIGN stream once its receiver has trained on our idle for a while - offering
        # the exchange again mid-link asks it with a fully-trained RX.
        retrain = getattr(trx, "oob_retrain", None)
        if retrain is None:
            retrain = Signal()
        retrain_r = Signal()
        self.sync += retrain_r.eq(retrain)

        # Minimum SEND-ALIGN dwell (runtime, absent/0 = original behaviour): hold the ALIGN burst
        # for ~200us regardless of the exit conditions. Without it a device that keeps ALIGN-ing
        # (lenient exit) or SYNC-ing (retrain offer) terminates SEND-ALIGN within a few dwords and
        # its window qualifier never sees a sustained host burst.
        align_dwell = getattr(trx, "oob_align_dwell", None)
        if align_dwell is None:
            align_dwell = Signal()
        dwell_timer = WaitTimer(int(200e-6*clk_freq))
        self.submodules += dwell_timer

        # Sticky ALIGN/ALIGN_N detection (cleared with the FSM): the device's ALIGN bursts are
        # short and must not be missed while the FSM is between states.
        align_seen   = Signal()
        align_n_seen = Signal()
        align_rst    = Signal()
        self.sync += [
            If(align_rst,
                align_seen.eq(0),
                align_n_seen.eq(0),
            ).Else(
                If(sink.valid & (sink.charisk == 0b0001) & (sink.data == primitives["ALIGN"]),
                    align_seen.eq(1)),
                If(sink.valid & (sink.charisk == 0b0001) & (sink.data == primitives["ALIGN_N"]),
                    align_n_seen.eq(1)),
            )
        ]

        # Misalignment policy. A dword whose K character lands outside byte 0 is normal during
        # resynchronisation - the RX converter self-resets and the link recovers within a few
        # words. `misalign_tolerance` = 0 keeps the original behaviour (reset the RX on the first
        # event, as the Xilinx PHYs have always done); a non-zero value runs a leaky-bucket
        # integrator so only SUSTAINED misalignment resets the RX. On ECP5 the DCU word aligner
        # emits short bursts of 0xEE decode errors that the link recovers from unaided, and
        # tearing down on those prevents READY from ever completing its stability timer.
        misalign_flt = Signal()
        if misalign_tolerance:
            mis_score = Signal(max=2*misalign_tolerance + 2)
            self.sync += [
                If(self.misalign,
                    If(mis_score < 2*misalign_tolerance, mis_score.eq(mis_score + 1))
                ).Elif(mis_score != 0,
                    mis_score.eq(mis_score - 1)
                )
            ]
            self.comb += misalign_flt.eq(mis_score >= misalign_tolerance)
        else:
            self.comb += misalign_flt.eq(self.misalign)

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
            If(trx.ready,
                # Set RX polarity to 0 (we don't know it at this point).
                NextValue(trx.rx_polarity, 0),
                # Alternate TX polarity on each retry.
                NextValue(trx.tx_polarity, ~trx.tx_polarity),
                If(oob_bypass,
                    NextState("AWAIT-ALIGN")
                ).Else(
                    NextState("COMINIT")
                )
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
            align_rst.eq(1),
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
            If(early_d102,
                # COMWAKE is detected: start the continuous D10.2 stream now (see above).
                self.d102_phase.eq(1),
                source.data.eq(0x4a4a4a4a),
                source.charisk.eq(0b0000),
            ).Else(
                self.tx_idle.eq(1),
            ),
            trx.rx_cdrhold.eq(1),
            nocomwake_timer.wait.eq(1),
            If(~trx.rx_comwake_stb | (nocomwake_timer.done if nocomwake_timeout_us else 0),
                NextState("AWAIT-ALIGN")
            )
        )
        fsm.act("AWAIT-ALIGN",
            # The device transmits ALIGN bursts in this state, so there ARE transitions to lock to:
            # holding the CDR here freezes the receiver exactly when it must acquire the device's
            # ALIGNs. Measured on ECP5 with the hold effective: rx_idle 2040/2040 and all-zero
            # dwords for the entire state. Xilinx keeps the original behaviour by default.
            trx.rx_cdrhold.eq((~loopback) if align_cdr_hold else 0),
            self.d102_phase.eq(~loopback),
            source.data.eq(Mux(loopback, primitives["ALIGN"], 0x4a4a4a4a)),  # D10.2 (ALIGN in loopback)
            source.charisk.eq(Mux(loopback, 0b0001, 0b0000)),
            align_timer.wait.eq(1),
            # `align_needs_signal` keeps the original gate on the transceiver's rx_idle. On ECP5
            # the RLOS-derived rx_idle stays asserted right through a device's ALIGN bursts, so the
            # comparison would never be evaluated; decoding a valid ALIGN is itself proof of signal.
            If(align_seen & ((~trx.rx_idle) if align_needs_signal else 1),
                NextValue(trx.rx_polarity, 0),
                NextState("SEND-ALIGN")
            ),
            If(align_n_seen & ((~trx.rx_idle) if align_needs_signal else 1),
                NextValue(trx.rx_polarity, 1),
                NextState("SEND-ALIGN")
            )
        )
        fsm.act("SEND-ALIGN",
            align_timer.wait.eq(1),
            dwell_timer.wait.eq(1),
            source.data.eq(primitives["ALIGN"]),
            source.charisk.eq(0b0001),
            If(sink.valid & (sink.charisk == 0b0001),
                # Loopback: our own ALIGN echo (K28.5, 0xBC) counts too; a real device answers
                # with 0x7C-low-byte (K28.3 family) primitives.
                # Count SYNC (K28.3, low byte 0x7C) or ALIGN (K28.5, 0xBC): a device that is
                # still emitting ALIGNs after speed negotiation is just as valid a confirmation
                # that the link is established, and some devices linger on ALIGN.
                If((sink.data[0:8] == 0x7c) |
                   ((sink.data[0:8] == 0xbc) & lenient_exit) |
                   ((sink.data[0:8] == 0xbc) if align_accept_align else 0),
                    If(align_count != 0,
                        NextValue(align_count, align_count - 1),
                    )
                ).Else(
                    NextValue(align_count, 4-1),
                )
            ),
            If((align_count == 0) & (dwell_timer.done | ~align_dwell),
                NextState("READY")
            )
        )

        # Wait alignment stability for 5ms before declaring ctrl is ready, reset the RX part of
        # the transceiver when misalignment is detected.
        stability_timer = WaitTimer(int(stability_us*1e-6*clk_freq))
        self.submodules += stability_timer

        fsm.act("READY",
            source.data.eq(primitives["SYNC"]),
            source.charisk.eq(0b0001),
            stability_timer.wait.eq(1),
            self.ready.eq(stability_timer.done),
            # Loopback: RLOS-based rx_idle chatters through the doubly-AC-coupled loop; ignore it
            # (a real link drop is caught by misalign and upper layers).
            If(self.rx_idle & ~loopback,
                NextState("RESET"),
            ).Elif(misalign_flt,
                self.rx_reset.eq(1),
                NextState("RESET_RX")
            ).Elif(retrain & ~retrain_r,
                NextValue(align_count, 4-1),
                NextState("SEND-ALIGN")
            )
        )
        fsm.act("RESET_RX",
            If(trx.ready,
                NextState("READY")
            )
        )

        # Line test: continuously transmit ALIGN primitives regardless of FSM state (overrides the
        # FSM's source drive; used to answer a device's autonomous speed-negotiation windows).
        self.align_force = Signal()
        self.comb += If(self.align_force,
            source.valid.eq(1),
            source.data.eq(primitives["ALIGN"]),
            source.charisk.eq(0b0001),
        )

        # Optional polite-host retry limit: after oob_retries failed OOB attempts, hold the line
        # idle for oob_backoff seconds instead of hammering the device forever (some devices
        # wedge on sustained incoherent OOB streams until power-cycled).
        if oob_retries is not None:
            # The attempt counter lives outside the FSM (the FSM ResetInserter clears NextValue
            # state on every retry timeout).
            attempts      = Signal(max=oob_retries + 1)
            backoff_timer = WaitTimer(int(oob_backoff*clk_freq))
            self.submodules += backoff_timer
            self.comb += backoff_timer.wait.eq(fsm.ongoing("BACKOFF"))
            fsm.act("BACKOFF",
                self.tx_idle.eq(1),
                trx.rx_cdrhold.eq(1),
                If(backoff_timer.done,
                    NextState("RESET")
                )
            )
            reset_entry = Signal()
            self.sync += reset_entry.eq(fsm.ongoing("RESET"))
            self.sync += [
                If(fsm.ongoing("RESET") & ~reset_entry,
                    If(attempts != oob_retries,
                        attempts.eq(attempts + 1)
                    )
                ),
                If(fsm.ongoing("BACKOFF") & backoff_timer.done,
                    attempts.eq(0)
                ),
            ]
            # Divert to BACKOFF from RESET once the retry budget is exhausted.
            reset_state = fsm.actions["RESET"]
            fsm.actions["RESET"] = [
                If((attempts == oob_retries) & ~fsm.ongoing("BACKOFF"),
                    NextState("BACKOFF")
                ).Else(
                    *reset_state
                )
            ]

    def us(self, t):
        clk_period_us = 1e6/self.clk_freq
        return ceil(t/clk_period_us)
