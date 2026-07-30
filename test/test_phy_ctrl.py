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
from litesata.phy.ctrl import LiteSATAPHYCtrl
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


class CRGInterface:
    def __init__(self):
        self.tx_reset = Signal()
        self.rx_reset = Signal()


class DUT(Module):
    def __init__(self, with_early_align=False):
        self.trx = PHYInterface()
        self.crg = CRGInterface()
        ctrl_cls = ECP5LiteSATAPHYCtrl if with_early_align else LiteSATAPHYCtrl
        self.submodules.ctrl = ctrl_cls(self.trx, self.crg, clk_freq=1e6)


class TestPHYCtrl(unittest.TestCase):
    def test_constructor_keeps_legacy_positional_order(self):
        parameters = list(inspect.signature(LiteSATAPHYCtrl.__init__).parameters)
        self.assertEqual(parameters, [
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


if __name__ == "__main__":
    unittest.main()
