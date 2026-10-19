#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import inspect
import unittest

from migen import Module, Signal
from migen.sim import run_simulation

from litesata.common import primitives
from litesata.phy.ctrl import LiteSATAPHYCtrl, LiteSATAPHYLinkMonitor
from litesata.phy.ecp5sataphy import ECP5LiteSATAPHYCtrl


class PHYInterface:
    def __init__(self):
        self.ready          = Signal()
        self.tx_idle        = Signal()
        self.rx_idle        = Signal()
        self.rx_cdrhold     = Signal()
        self.rx_polarity    = Signal()
        self.tx_polarity    = Signal()
        self.tx_cominit_stb = Signal()
        self.tx_cominit_ack = Signal()
        self.rx_cominit_stb = Signal()
        self.tx_comwake_stb = Signal()
        self.tx_comwake_ack = Signal()
        self.rx_comwake_stb = Signal()
        self.rate           = Signal(2)
        self.rate_done      = Signal(reset=1)


class CRGInterface:
    def __init__(self):
        self.tx_reset = Signal()
        self.rx_reset = Signal()


class DUT(Module):
    def __init__(self, with_early_align=False, **ctrl_kwargs):
        self.trx = PHYInterface()
        self.crg = CRGInterface()
        ctrl_cls = ECP5LiteSATAPHYCtrl if with_early_align else LiteSATAPHYCtrl
        self.submodules.ctrl = ctrl_cls(self.trx, self.crg, clk_freq=1e6, **ctrl_kwargs)


class TestPHYCtrl(unittest.TestCase):
    def test_constructor_keeps_legacy_positional_order(self):
        parameters = list(inspect.signature(LiteSATAPHYCtrl.__init__).parameters)
        self.assertEqual(parameters[:4], [
            "self",
            "trx",
            "crg",
            "clk_freq",
        ])

    def run_early_align(self, with_early_align):
        dut    = DUT(with_early_align=with_early_align)
        result = {}

        def wait_high(signal):
            for _ in range(32):
                if (yield signal):
                    return
                yield
            self.fail("PHY controller handshake timed out")

        def generator():
            yield dut.trx.ready.eq(1)
            yield dut.trx.tx_cominit_ack.eq(1)
            yield dut.trx.tx_comwake_ack.eq(1)
            yield dut.trx.rx_idle.eq(0)

            yield from wait_high(dut.trx.tx_cominit_stb)
            yield
            while (yield dut.trx.tx_cominit_stb):
                yield

            yield dut.trx.rx_cominit_stb.eq(1)
            yield
            yield
            yield dut.trx.rx_cominit_stb.eq(0)

            yield from wait_high(dut.trx.tx_comwake_stb)
            yield
            while (yield dut.trx.tx_comwake_stb):
                yield

            # Present ALIGN while COMWAKE is still asserted, then remove it before AWAIT-ALIGN.
            yield dut.trx.rx_comwake_stb.eq(1)
            yield dut.ctrl.sink.valid.eq(1)
            yield dut.ctrl.sink.charisk.eq(0b0001)
            yield dut.ctrl.sink.data.eq(primitives["ALIGN"])
            yield
            yield
            yield dut.ctrl.sink.valid.eq(0)
            yield dut.trx.rx_comwake_stb.eq(0)
            for _ in range(4):
                yield

            result["charisk"] = (yield dut.ctrl.source.charisk)
            result["data"]    = (yield dut.ctrl.source.data)

        run_simulation(dut, generator())
        return result

    def test_legacy_mode_does_not_latch_early_align(self):
        result = self.run_early_align(with_early_align=False)
        self.assertEqual(result["charisk"], 0)
        self.assertEqual(result["data"], 0x4a4a4a4a)

    def test_early_align_latch_is_opt_in(self):
        result = self.run_early_align(with_early_align=True)
        self.assertEqual(result["charisk"], 0b0001)
        self.assertEqual(result["data"], primitives["ALIGN"])


class RateDUT(DUT):
    """DUT exposing the controller states the device-side helpers wait for."""
    def __init__(self, **ctrl_kwargs):
        DUT.__init__(self, **ctrl_kwargs)
        self.states = {name: self.ctrl.fsm.ongoing(name) for name in [
            "AWAIT-CRG-RESET", "AWAIT-COMINIT", "AWAIT-COMWAKE", "AWAIT-ALIGN", "READY"]}


class TestPHYCtrlRateFallback(unittest.TestCase):
    """clk_freq=1e6 keeps the timers short: ALIGN timeout 873 cycles, stability 5000 cycles."""

    def wait_for(self, signal, value, timeout):
        for _ in range(timeout):
            if (yield signal) == value:
                return
            yield
        self.fail("Timeout waiting for a PHY controller signal/state")

    def setup(self, dut):
        yield dut.trx.ready.eq(1)
        yield dut.trx.tx_cominit_ack.eq(1)
        yield dut.trx.tx_comwake_ack.eq(1)
        yield

    def oob_handshake(self, dut):
        """Play the device side of COMINIT/COMWAKE until the controller waits for ALIGN."""
        yield from self.wait_for(dut.states["AWAIT-COMINIT"], 1, 64)
        yield dut.trx.rx_cominit_stb.eq(1)
        yield
        yield
        yield dut.trx.rx_cominit_stb.eq(0)
        yield from self.wait_for(dut.states["AWAIT-COMWAKE"], 1, 64)
        yield dut.trx.rx_comwake_stb.eq(1)
        yield
        yield
        yield dut.trx.rx_comwake_stb.eq(0)
        yield from self.wait_for(dut.states["AWAIT-ALIGN"], 1, 64)

    def align_timeout(self, dut):
        """No ALIGN from the device: the controller times out, restarts and sends COMINIT again."""
        yield from self.wait_for(dut.states["AWAIT-ALIGN"], 0, 1200)
        yield from self.wait_for(dut.states["AWAIT-COMINIT"], 1, 64)

    def test_rate_steps_down_and_wraps(self):
        dut = RateDUT(rate_fallback=True, rate_attempts=1)

        def generator():
            yield from self.setup(dut)
            yield from self.wait_for(dut.states["AWAIT-COMINIT"], 1, 16)
            self.assertEqual((yield dut.trx.rate), 2)
            for expected in [1, 0, 2, 1]:
                yield from self.oob_handshake(dut)
                yield from self.align_timeout(dut)
                self.assertEqual((yield dut.trx.rate), expected)
            # One COMINIT per attempt: 2 at gen3 (initial + wrap), 2 at gen2, 1 at gen1.
            self.assertEqual((yield dut.ctrl.rate_count[2]), 2)
            self.assertEqual((yield dut.ctrl.rate_count[1]), 2)
            self.assertEqual((yield dut.ctrl.rate_count[0]), 1)

        run_simulation(dut, generator())

    def test_rate_attempts_per_rate(self):
        dut = RateDUT(rate_fallback=True, rate_attempts=2)

        def generator():
            yield from self.setup(dut)
            yield from self.oob_handshake(dut)
            yield from self.align_timeout(dut)
            self.assertEqual((yield dut.trx.rate), 2)
            yield from self.oob_handshake(dut)
            yield from self.align_timeout(dut)
            self.assertEqual((yield dut.trx.rate), 1)

        run_simulation(dut, generator())

    def test_rate_done_gates_oob(self):
        dut = RateDUT(rate_fallback=True, rate_attempts=1)

        def generator():
            yield from self.setup(dut)
            yield from self.oob_handshake(dut)
            # The PHY is still applying the new rate when the controller restarts.
            yield dut.trx.rate_done.eq(0)
            yield from self.wait_for(dut.states["AWAIT-ALIGN"], 0, 1200)
            yield from self.wait_for(dut.states["AWAIT-CRG-RESET"], 1, 16)
            self.assertEqual((yield dut.trx.rate), 1)
            for _ in range(100):
                self.assertEqual((yield dut.states["AWAIT-CRG-RESET"]), 1)
                self.assertEqual((yield dut.trx.tx_cominit_stb), 0)
                yield
            yield dut.trx.rate_done.eq(1)
            yield from self.wait_for(dut.states["AWAIT-COMINIT"], 1, 16)

        run_simulation(dut, generator())

    def test_ready_latches_rate_and_link_loss_restarts_from_max(self):
        dut = RateDUT(rate_fallback=True, rate_attempts=1)

        def generator():
            yield from self.setup(dut)
            yield from self.oob_handshake(dut)
            yield from self.align_timeout(dut)
            self.assertEqual((yield dut.trx.rate), 1)
            # The device locks at gen2: its ALIGNs, then non-ALIGN primitives (SYNC), bring the
            # controller to READY, and the stability time to ready.
            yield from self.oob_handshake(dut)
            yield dut.ctrl.sink.valid.eq(1)
            yield dut.ctrl.sink.charisk.eq(0b0001)
            yield dut.ctrl.sink.data.eq(primitives["ALIGN"])
            yield from self.wait_for(dut.states["AWAIT-ALIGN"], 0, 16)
            yield dut.ctrl.sink.data.eq(primitives["SYNC"])
            yield from self.wait_for(dut.states["READY"], 1, 16)
            yield from self.wait_for(dut.ctrl.ready, 1, 6000)
            self.assertEqual((yield dut.ctrl.rate), 1)
            self.assertEqual((yield dut.trx.rate), 1)
            # Link lost: renegotiate from the highest rate.
            yield dut.ctrl.sink.valid.eq(0)
            yield dut.ctrl.rx_idle.eq(1)
            yield from self.wait_for(dut.states["READY"], 0, 16)
            yield dut.ctrl.rx_idle.eq(0)
            yield from self.wait_for(dut.states["AWAIT-COMINIT"], 1, 16)
            self.assertEqual((yield dut.trx.rate), 2)

        run_simulation(dut, generator())

    def test_rate_pinned(self):
        dut = RateDUT(rate_fallback=True, rate_attempts=1)

        def generator():
            # Pin gen1 (a step from gen3 would give gen2): applied at the next controller reset.
            yield dut.ctrl.rate_max_i.eq(0)
            yield dut.ctrl.rate_min_i.eq(0)
            yield from self.setup(dut)
            yield from self.oob_handshake(dut)
            yield from self.align_timeout(dut)
            self.assertEqual((yield dut.trx.rate), 0)
            yield from self.oob_handshake(dut)
            yield from self.align_timeout(dut)
            self.assertEqual((yield dut.trx.rate), 0)

        run_simulation(dut, generator())

    def test_no_fallback_leaves_trx_rate_alone(self):
        dut = RateDUT()

        def generator():
            yield from self.setup(dut)
            yield from self.oob_handshake(dut)
            yield from self.align_timeout(dut)
            self.assertEqual((yield dut.trx.rate), 0)

        run_simulation(dut, generator())


class TestPHYLinkMonitor(unittest.TestCase):
    def test_link_events(self):
        class DUT(Module):
            def __init__(self):
                self.ready = Signal()
                self.submodules.monitor = LiteSATAPHYLinkMonitor(self.ready, down_cycles=16)

        dut    = DUT()
        events = []

        def generator():
            def drive(ready, cycles):
                yield dut.ready.eq(ready)
                for _ in range(cycles):
                    yield
                    if (yield dut.monitor.connect):
                        events.append("connect")
                    if (yield dut.monitor.disconnect):
                        events.append("disconnect")

            yield from drive(0, 4)
            self.assertEqual(events, [])
            # Ready: immediate link up + one connect pulse.
            yield from drive(1, 4)
            self.assertEqual((yield dut.monitor.link_up), 1)
            self.assertEqual(events, ["connect"])
            # Short drop (re-initialization): no event.
            yield from drive(0, 8)
            yield from drive(1, 4)
            self.assertEqual((yield dut.monitor.link_up), 1)
            self.assertEqual(events, ["connect"])
            # Long drop (unplug): one disconnect pulse after the timeout.
            yield from drive(0, 32)
            self.assertEqual((yield dut.monitor.link_up), 0)
            self.assertEqual(events, ["connect", "disconnect"])
            # Back: one connect pulse.
            yield from drive(1, 4)
            self.assertEqual(events, ["connect", "disconnect", "connect"])

        run_simulation(dut, generator())


if __name__ == "__main__":
    unittest.main()
