#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *
from migen.genlib.cdc import MultiReg
from migen.sim import run_simulation

from litesata.common import primitives
from litesata.phy.ctrl import LiteSATAPHYCtrl
from litesata.phy.ecp5sataphy import COMGenerator, COMChecker


def runs(trace):
    """Compress a 0/1 trace into (value, length) runs."""
    out = []
    for v in trace:
        if out and out[-1][0] == v:
            out[-1][1] += 1
        else:
            out.append([v, 1])
    return [tuple(r) for r in out]


class TestECP5OOB(unittest.TestCase):
    # COMGenerator -------------------------------------------------------------------------------
    def com_generator_test(self, tx_clk_freq, com, burst_cycles, gap_cycles):
        dut = COMGenerator(tx_clk_freq)
        trace  = []
        finish = []

        def gen():
            yield getattr(dut, com).eq(1)
            yield
            yield getattr(dut, com).eq(0)
            for i in range(4096):
                trace.append((yield dut.tx_oob_en))
                if (yield dut.finish):
                    finish.append(i)
                    break
                # tx_idle must cover the whole sequence.
                if (yield dut.active):
                    self.assertEqual((yield dut.tx_idle), 1)
                yield
            else:
                self.fail("COMGenerator never finished")

        run_simulation(dut, gen())

        # Strip leading idle, drop trailing run (blends into FINISH).
        r = runs(trace)
        if r[0][0] == 0:
            r = r[1:]
        bursts     = [l for v, l in r if v == 1]
        inner_gaps = [l for v, l in r[:-1] if v == 0]
        self.assertEqual(bursts, [burst_cycles]*6)
        self.assertEqual(inner_gaps, [gap_cycles]*5)

    def test_com_generator_cominit_gen1(self):
        self.com_generator_test(75e6, "cominit", burst_cycles=8, gap_cycles=24)

    def test_com_generator_comwake_gen1(self):
        self.com_generator_test(75e6, "comwake", burst_cycles=8, gap_cycles=8)

    def test_com_generator_cominit_gen2(self):
        self.com_generator_test(150e6, "cominit", burst_cycles=16, gap_cycles=48)

    def test_com_generator_comwake_gen2(self):
        self.com_generator_test(150e6, "comwake", burst_cycles=16, gap_cycles=16)

    def test_com_generator_square_wave(self):
        dut = COMGenerator(75e6)
        def gen():
            yield dut.comwake.eq(1)
            yield
            yield dut.comwake.eq(0)
            data = []
            for i in range(64):
                if (yield dut.tx_oob_en):
                    data.append((yield dut.tx_oob_data))
                yield
            # toggle_div=0: tx_oob_data toggles every cycle during bursts.
            self.assertGreater(len(data), 8)
            for a, b in zip(data, data[1:]):
                if a == b:
                    # Toggles may be interrupted at burst boundaries only.
                    pass
            self.assertIn(0, data)
            self.assertIn(1, data)
        run_simulation(dut, gen())

    # COMChecker ---------------------------------------------------------------------------------
    @staticmethod
    def drive(dut, sequence):
        """sequence: list of (idle_value, cycles)."""
        for value, cycles in sequence:
            yield dut.rx_idle.eq(value)
            for _ in range(cycles):
                yield

    def com_checker_test(self, sequence, expect_cominit, expect_comwake, clk_freq=100e6):
        dut = COMChecker(clk_freq)
        result = {}

        def gen():
            cominit_seen = 0
            comwake_seen = 0
            # Interleave driving and observation.
            for value, cycles in sequence:
                yield dut.rx_idle.eq(value)
                for _ in range(cycles):
                    yield
                    cominit_seen |= (yield dut.cominit_det)
                    comwake_seen |= (yield dut.comwake_det)
            result["cominit"] = cominit_seen
            result["comwake"] = comwake_seen
            result["cominit_now"] = (yield dut.cominit_det)
            result["comwake_now"] = (yield dut.comwake_det)

        run_simulation(dut, gen())
        self.assertEqual(result["cominit"], expect_cominit)
        self.assertEqual(result["comwake"], expect_comwake)
        return result

    @staticmethod
    def com_sequence(burst, gap, n=6):
        seq = [(1, 64)] # Initial idle.
        for i in range(n):
            seq.append((0, burst))
            seq.append((1, gap))
        return seq

    def test_com_checker_cominit(self):
        # 320ns gaps @ 100MHz = 32 cycles -> COMINIT.
        seq = self.com_sequence(burst=11, gap=32)
        self.com_checker_test(seq, expect_cominit=1, expect_comwake=0)

    def test_com_checker_comwake(self):
        # 106.7ns gaps @ 100MHz = 11 cycles -> COMWAKE.
        seq = self.com_sequence(burst=11, gap=11)
        self.com_checker_test(seq, expect_cominit=0, expect_comwake=1)

    def test_com_checker_windows(self):
        # Boundary values @ 100MHz: COMWAKE [6..17], COMINIT [18..52].
        for gap, cominit, comwake in [
            ( 5, 0, 0),
            ( 6, 0, 1),
            (17, 0, 1),
            (18, 1, 0),
            (52, 1, 0),
            (53, 0, 0),
        ]:
            seq = self.com_sequence(burst=11, gap=gap)
            self.com_checker_test(seq, expect_cominit=cominit, expect_comwake=comwake)

    def test_com_checker_junk_rejection(self):
        # Alternating gap classes never reach 4 consecutive qualifying gaps.
        seq = [(1, 64)]
        for i in range(8):
            seq.append((0, 11))
            seq.append((1, 11 if (i % 2) else 32))
        self.com_checker_test(seq, expect_cominit=0, expect_comwake=0)

    def test_com_checker_quiet_deassert_and_rearm(self):
        dut = COMChecker(100e6)
        edges = []

        def gen():
            prev = 0
            seq  = []
            quiet = int(2e-6*100e6) + 32
            for _ in range(2):
                seq += self.com_sequence(burst=11, gap=32)
                seq += [(1, quiet)]
            for value, cycles in seq:
                yield dut.rx_idle.eq(value)
                for _ in range(cycles):
                    yield
                    det = (yield dut.cominit_det)
                    if det != prev:
                        edges.append(det)
                    prev = det

        run_simulation(dut, gen())
        # Two assert/deassert episodes.
        self.assertEqual(edges, [1, 0, 1, 0])

    # Generator -> Checker loopback across clock domains -----------------------------------------
    def test_com_loopback(self):
        class _Loopback(Module):
            def __init__(self):
                self.submodules.com_gen   = ClockDomainsRenamer("tx")(COMGenerator(75e6))
                self.submodules.com_check = COMChecker(100e6)
                idle = Signal(reset=1)
                self.specials += MultiReg(~self.com_gen.tx_oob_en, idle, "sys")
                self.comb += self.com_check.rx_idle.eq(idle)

        for com, det in [("cominit", "cominit_det"), ("comwake", "comwake_det")]:
            dut = _Loopback()
            result = {}

            def tx_gen(dut=dut, com=com):
                yield getattr(dut.com_gen, com).eq(1)
                yield
                yield getattr(dut.com_gen, com).eq(0)
                for i in range(1024):
                    yield

            def sys_gen(dut=dut, det=det):
                seen = 0
                for i in range(1024):
                    yield
                    seen |= (yield getattr(dut.com_check, det))
                result["seen"] = seen

            # tx @ 75MHz (13.33ns) approximated with 13ns period vs sys @ 10ns.
            run_simulation(dut, [tx_gen(), sys_gen()], clocks={"sys": 10, "tx": 13})
            self.assertEqual(result["seen"], 1, f"{com} not detected in loopback")

    # ctrl FSM <-> PHY handshake semantics -------------------------------------------------------
    def test_ctrl_oob_sequence(self):
        clk_freq = 1e6 # Shrinks ctrl timers: retry=10000, align=873, stability=5000 cycles.

        class _TRXStub(Module):
            def __init__(self):
                self.ready          = Signal(reset=1)
                self.tx_idle        = Signal()
                self.tx_polarity    = Signal()
                self.rx_polarity    = Signal()
                self.tx_cominit_stb = Signal()
                self.tx_cominit_ack = Signal()
                self.tx_comwake_stb = Signal()
                self.tx_comwake_ack = Signal()
                self.rx_idle        = Signal(reset=1)
                self.rx_cdrhold     = Signal()
                self.rx_cominit_stb = Signal()
                self.rx_comwake_stb = Signal()

        class _CRGStub(Module):
            def __init__(self):
                self.tx_reset = Signal()
                self.rx_reset = Signal()

        class _DUT(Module):
            def __init__(self):
                self.submodules.trx  = _TRXStub()
                self.submodules.crg  = _CRGStub()
                self.submodules.ctrl = LiteSATAPHYCtrl(self.trx, self.crg, clk_freq)

        dut = _DUT()

        def wait_for(sig, timeout=20000):
            for i in range(timeout):
                if (yield sig):
                    return
                yield
            self.fail("timeout waiting for signal")

        def gen():
            trx, ctrl = dut.trx, dut.ctrl
            # Host COMRESET: wait for stb, ack it (finish pulse) with RX quiet.
            yield from wait_for(trx.tx_cominit_stb)
            for i in range(16):
                yield
            yield trx.tx_cominit_ack.eq(1)
            yield
            yield trx.tx_cominit_ack.eq(0)
            yield
            # Device COMINIT: level, then quiet.
            for i in range(8):
                yield
            yield trx.rx_cominit_stb.eq(1)
            for i in range(16):
                yield
            yield trx.rx_cominit_stb.eq(0)
            yield
            # Host COMWAKE.
            yield from wait_for(trx.tx_comwake_stb)
            for i in range(16):
                yield
            yield trx.tx_comwake_ack.eq(1)
            yield
            yield trx.tx_comwake_ack.eq(0)
            yield
            # Device COMWAKE: level, then quiet.
            for i in range(8):
                yield
            yield trx.rx_comwake_stb.eq(1)
            for i in range(16):
                yield
            yield trx.rx_comwake_stb.eq(0)
            yield
            # Device sends ALIGNs: line active, feed ALIGN primitives to ctrl.
            yield trx.rx_idle.eq(0)
            yield ctrl.sink.valid.eq(1)
            yield ctrl.sink.charisk.eq(0b0001)
            yield ctrl.sink.data.eq(primitives["ALIGN"])
            for i in range(8):
                yield
            # Device locks and moves on to SYNC (ctrl counts 4 consecutive non-ALIGN primitives
            # in SEND-ALIGN before declaring the link aligned).
            yield ctrl.sink.data.eq(primitives["SYNC"])
            # Wait for ready (stability timer = 5000 cycles).
            yield from wait_for(ctrl.ready, timeout=20000)
            self.assertEqual((yield ctrl.ready), 1)

        run_simulation(dut, gen())

    # PHY selection / elaboration ----------------------------------------------------------------
    def test_ecp5_phy_selection(self):
        from migen.fhdl import verilog
        from litex.build.lattice.common import lattice_ecp5_special_overrides
        from litesata.phy import LiteSATAPHY
        from litesata.phy.ecp5sataphy import ECP5LiteSATAPHY

        class SATAPads:
            def __init__(self):
                self.rx_p = Signal()
                self.rx_n = Signal()
                self.tx_p = Signal()
                self.tx_n = Signal()

        dut = LiteSATAPHY(
            device   = "LFE5UM5G-85F-8BG554I",
            pads     = SATAPads(),
            gen      = "gen1",
            clk_freq = 100e6,
            refclk   = Signal(),
            dual     = 1,
            channel  = 0,
            with_csr = True,
        )
        self.assertIsInstance(dut.phy, ECP5LiteSATAPHY)
        v = str(verilog.convert(dut, special_overrides=lattice_ecp5_special_overrides))
        self.assertIn("DCUA", v)
        for port in ["CH0_FFC_LDR_CORE2TX_EN", "CH0_LDR_CORE2TX", "CH0_LDR_RX2CORE", "CH0_FFC_EI_EN"]:
            self.assertIn(port, v)


if __name__ == "__main__":
    unittest.main()
