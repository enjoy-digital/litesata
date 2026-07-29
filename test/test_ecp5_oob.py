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
from litesata.phy.datapath import LiteSATAPHYAlignTimer
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
                self.submodules.ctrl = LiteSATAPHYCtrl(
                    self.trx, self.crg, clk_freq,
                    align_full_primitive=True,
                )

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
            # Strict SATA behavior: the device's ALIGN does not end our SEND-ALIGN
            # response.  It must move on to a K28.3-family primitive such as SYNC.
            self.assertEqual(
                (yield ctrl.fsm.state),
                ctrl.fsm.encoding["SEND-ALIGN"],
            )
            # A corrupted K-led word with the SYNC low byte is not a complete SYNC and must
            # not end SEND-ALIGN on ECP5.
            yield ctrl.sink.data.eq(0x7878787c)
            for _ in range(8):
                yield
            self.assertEqual(
                (yield ctrl.fsm.state),
                ctrl.fsm.encoding["SEND-ALIGN"],
            )
            # Device locks and moves on to SYNC (ctrl counts 4 consecutive exact SYNC primitives
            # in SEND-ALIGN before declaring the link aligned).
            yield ctrl.sink.data.eq(primitives["SYNC"])
            # Wait for ready (stability timer = 5000 cycles).
            yield from wait_for(ctrl.ready, timeout=20000)
            self.assertEqual((yield ctrl.ready), 1)

        run_simulation(dut, gen())

    def test_ctrl_lenient_exit_obeys_align_dwell(self):
        clk_freq = 1e6

        class _TRXStub(Module):
            def __init__(self):
                self.ready              = Signal(reset=1)
                self.tx_idle            = Signal()
                self.tx_polarity        = Signal()
                self.rx_polarity        = Signal()
                self.tx_cominit_stb     = Signal()
                self.tx_cominit_ack     = Signal()
                self.tx_comwake_stb     = Signal()
                self.tx_comwake_ack     = Signal()
                self.rx_idle            = Signal(reset=1)
                self.rx_cdrhold         = Signal()
                self.rx_cominit_stb     = Signal()
                self.rx_comwake_stb     = Signal()
                self.oob_lenient_exit   = Signal(reset=1)
                self.oob_lenient_dwell  = Signal(16, reset=16)
                self.comb += [
                    self.tx_cominit_ack.eq(self.tx_cominit_stb),
                    self.tx_comwake_ack.eq(self.tx_comwake_stb),
                ]

        class _CRGStub(Module):
            def __init__(self):
                self.tx_reset = Signal()
                self.rx_reset = Signal()

        class _DUT(Module):
            def __init__(self):
                self.submodules.trx = _TRXStub()
                self.submodules.crg = _CRGStub()
                self.submodules.ctrl = LiteSATAPHYCtrl(
                    self.trx, self.crg, clk_freq,
                    align_timeout_us=1000,
                    stability_us=1,
                    align_full_primitive=True,
                )

        dut = _DUT()

        def wait_state(name, timeout=200):
            encoding = dut.ctrl.fsm.encoding[name]
            for _ in range(timeout):
                if (yield dut.ctrl.fsm.state) == encoding:
                    return
                yield
            self.fail(f"timeout waiting for {name}")

        def gen():
            yield from wait_state("AWAIT-COMINIT")
            yield dut.trx.rx_cominit_stb.eq(1)
            yield
            yield dut.trx.rx_cominit_stb.eq(0)
            yield from wait_state("AWAIT-COMWAKE")
            yield dut.trx.rx_comwake_stb.eq(1)
            yield
            yield dut.trx.rx_comwake_stb.eq(0)
            yield from wait_state("AWAIT-ALIGN")

            yield dut.trx.rx_idle.eq(0)
            yield dut.ctrl.sink.valid.eq(1)
            yield dut.ctrl.sink.charisk.eq(0b0001)
            yield dut.ctrl.sink.data.eq(primitives["ALIGN"])
            yield from wait_state("SEND-ALIGN")

            # Four consecutive ALIGNs normally trigger the diagnostic exit immediately.
            # The configured dwell must keep us in SEND-ALIGN first.
            for _ in range(12):
                yield
            self.assertEqual(
                (yield dut.ctrl.fsm.state),
                dut.ctrl.fsm.encoding["SEND-ALIGN"],
            )

            # A corrupted K-led word with ALIGN's low byte must not satisfy the diagnostic
            # full-primitive policy, even after the dwell has elapsed.
            yield dut.ctrl.sink.data.eq(0x787878bc)
            for _ in range(24):
                yield
            self.assertEqual(
                (yield dut.ctrl.fsm.state),
                dut.ctrl.fsm.encoding["SEND-ALIGN"],
            )

            yield dut.ctrl.sink.data.eq(primitives["ALIGN"])
            yield from wait_state("READY", timeout=16)
            self.assertEqual((yield dut.ctrl.source.data), primitives["SYNC"])

        run_simulation(dut, gen())

    def test_align_timer_accepts_any_k_led_primitive(self):
        dut = LiteSATAPHYAlignTimer(timeout=8)

        def gen():
            # Plain data does not prove that the far end is aligned.
            yield dut.sink.valid.eq(1)
            yield dut.sink.charisk.eq(0)
            for _ in range(10):
                yield
            self.assertEqual((yield dut.timer.done), 1)

            # Once negotiation ends, the far end sends SYNC rather than ALIGN.
            # Repeated SYNC must keep the line-activity timer re-armed.
            yield dut.sink.charisk.eq(0b0001)
            yield dut.sink.data.eq(primitives["SYNC"])
            yield  # WaitTimer reloads on the first non-waiting cycle.
            for _ in range(16):
                yield
                self.assertEqual((yield dut.timer.done), 0)

            yield dut.sink.charisk.eq(0)
            for _ in range(10):
                yield
            self.assertEqual((yield dut.timer.done), 1)

        run_simulation(dut, gen())

    def test_ctrl_backoff(self):
        clk_freq = 1e5  # retry timer -> 1000 cycles.

        class _Stub(Module):
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
                # Auto-ack COMINIT so ctrl proceeds to AWAIT-COMINIT and retries on timeout.
                self.comb += self.tx_cominit_ack.eq(self.tx_cominit_stb)

        class _CRG(Module):
            def __init__(self):
                self.tx_reset = Signal()
                self.rx_reset = Signal()

        class _DUT(Module):
            def __init__(self):
                self.submodules.trx  = _Stub()
                self.submodules.crg  = _CRG()
                self.submodules.ctrl = LiteSATAPHYCtrl(self.trx, self.crg, clk_freq,
                    oob_retries=2, oob_backoff=2e-3)  # backoff = 200 cycles.

        dut = _DUT()

        def gen():
            backoff_enc = dut.ctrl.fsm.encoding["BACKOFF"]
            backoff_cycles = 0
            resumed = 0
            for i in range(10000):
                yield
                st = (yield dut.ctrl.fsm.state)
                if st == backoff_enc:
                    backoff_cycles += 1
                elif backoff_cycles and (yield dut.trx.tx_cominit_stb):
                    resumed = 1
                    break
            self.assertGreater(backoff_cycles, 150, "BACKOFF state never/barely entered")
            self.assertEqual(resumed, 1, "retries never resumed after backoff")

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
            gen      = "gen2",
            clk_freq = 100e6,
            refclk   = Signal(),
            dual     = 1,
            channel  = 0,
            with_csr = True,
        )
        self.assertIsInstance(dut.phy, ECP5LiteSATAPHY)
        self.assertFalse(hasattr(dut.phy, "_oob_control"))
        self.assertEqual(dut.phy.ei_mode.reset.value, 1)
        self.assertEqual(dut.phy.oob_burst_mode.reset.value, 1)
        self.assertEqual(dut.phy.oob_zero_bus.reset.value, 1)
        self.assertEqual(dut.phy.oob_pat_alt.reset.value, 1)
        self.assertEqual(dut.phy.oob_deemph_gap.reset.value, 1)
        self.assertEqual(dut.phy.oob_early_d102.reset.value, 1)
        self.assertEqual(dut.phy.oob_pattern.reset.value, 0xF0F0)
        self.assertEqual(dut.phy.oob_align_nocomma.reset.value, 4096)
        self.assertEqual(dut.phy.oob_lenient_dwell.reset.value, 0)
        self.assertEqual(dut.phy.com_check.quiet_cycles.reset.value, 32)

        dut.phy.add_oob_csr()
        self.assertEqual(dut.phy._oob_control.storage.reset.value, 0x000C0402)
        self.assertEqual(dut.phy._oob_txctl.storage.reset.value, 0x00000260)
        self.assertEqual(dut.phy._oob_pattern.storage.reset.value, 0xF0F0)
        self.assertEqual(dut.phy._oob_align.storage.reset.value, (4096 << 16) | 64)
        self.assertEqual(dut.phy._oob_lenient_dwell.storage.reset.value, 0)
        self.assertEqual(dut.phy._oob_quiet.storage.reset.value, 32)
        v = str(verilog.convert(dut, special_overrides=lattice_ecp5_special_overrides))
        self.assertIn("DCUA", v)
        for port in ["CH0_FFC_LDR_CORE2TX_EN", "CH0_LDR_CORE2TX", "CH0_LDR_RX2CORE", "CH0_FFC_EI_EN"]:
            self.assertIn(port, v)

        with self.assertRaisesRegex(NotImplementedError, "Gen2 only"):
            LiteSATAPHY(
                device   = "LFE5UM5G-85F-8BG554I",
                pads     = SATAPads(),
                gen      = "gen1",
                clk_freq = 100e6,
                refclk   = Signal(),
            )



    def test_bypass_word_aligner(self):
        """The fabric word aligner must recover the symbol boundary at every bit offset.

        Feeds an encoded continuous-ALIGN bit stream, rotated by 0..19 bits, through
        BypassWordAligner + the fabric decoders, and requires K28.5 to decode and >=90%%
        clean ALIGN symbols at every offset. (The DCU comma aligner does not operate on
        the raw 10BSER datapath - this module is what does the job in bypass PCS.)
        """
        from litex.soc.cores.code_8b10b import Encoder, Decoder
        from litesata.phy.serdes_ecp5 import BypassWordAligner

        def enc_stream(pairs, n):
            e = Encoder(1, True); out = []
            def g():
                for i in range(n+4):
                    d, k = pairs[i % len(pairs)]
                    yield e.d[0].eq(d); yield e.k[0].eq(k); yield
                    out.append((yield e.output[0]))
            run_simulation(e, g()); return out[2:2+n]

        codes = enc_stream([(0xBC,1),(0x4A,0),(0x4A,0),(0x7B,0)], 80)
        bits  = []
        for c in codes: bits += [(c >> i) & 1 for i in range(10)]

        class DUT(Module):
            def __init__(self):
                self.al = BypassWordAligner()
                self.submodules += self.al
                self.decs = [Decoder(True) for _ in range(2)]
                self.submodules += self.decs
                self.sync += [
                    self.decs[0].input.eq(self.al.source[0:10]),
                    self.decs[1].input.eq(self.al.source[10:20]),
                ]

        good = [(0xBC,1),(0x4A,0),(0x7B,0)]
        for off in range(20):
            dut  = DUT()
            seen = []
            def g():
                stream = bits[off:] + bits[:off]
                words  = [stream[i:i+20] for i in range(0, len(stream)-20, 20)]
                for w in words*3:
                    yield dut.al.sink.eq(sum(b << i for i, b in enumerate(w)))
                    yield
                    seen.append(((yield dut.decs[0].d), (yield dut.decs[0].k),
                                 (yield dut.decs[1].d), (yield dut.decs[1].k)))
            run_simulation(dut, g())
            tail   = seen[20:]
            got_k  = sum(1 for d0,k0,d1,k1 in tail if (k0 and d0 == 0xBC) or (k1 and d1 == 0xBC))
            sym_ok = sum(1 for d0,k0,d1,k1 in tail if (d0,k0) in good and (d1,k1) in good)
            self.assertGreater(got_k, 0, f"offset {off}: no K28.5 decoded")
            self.assertGreaterEqual(sym_ok, 0.9*len(tail), f"offset {off}: {sym_ok}/{len(tail)}")

    def test_bypass_word_aligner_rejects_isolated_false_comma(self):
        from litesata.phy.serdes_ecp5 import BypassWordAligner

        dut = BypassWordAligner()
        result = {}

        def comma_word(offset):
            return 0x7c << offset

        def feed(word, cycles):
            for _ in range(cycles):
                yield dut.sink.eq(word)
                yield

        def gen():
            yield from feed(0, 5)
            # Two consecutive votes establish offset 5.
            yield from feed(comma_word(5), 6)
            yield from feed(0, 5)
            self.assertEqual((yield dut.slip), 5)
            moves = (yield dut.slip_mv)

            # One comma at offset 9 creates only a candidate.  A subsequent
            # confirmation at the current offset must clear that candidate.
            yield from feed(comma_word(9), 1)
            yield from feed(0, 5)
            yield from feed(comma_word(5), 1)
            yield from feed(0, 5)
            yield from feed(comma_word(9), 1)
            yield from feed(0, 5)

            result["slip"] = (yield dut.slip)
            result["moves"] = (yield dut.slip_mv)
            result["initial_moves"] = moves

        run_simulation(dut, gen())
        self.assertEqual(result["slip"], 5)
        self.assertEqual(result["moves"], result["initial_moves"])


if __name__ == "__main__":
    unittest.main()
