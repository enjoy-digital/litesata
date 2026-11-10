#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest
from unittest import mock

from bench.acorn import SATATestSoC
from bench import test_init
from bench import test_bist


class TestInit(unittest.TestCase):
    def test_phy_is_reset_once_then_polled(self):
        class Register:
            def __init__(self, reads=None):
                self.reads  = list(reads or [])
                self.writes = []

            def read(self):
                return self.reads.pop(0)

            def write(self, value):
                self.writes.append(value)

        class Bus:
            def __init__(self):
                self.regs = mock.Mock(
                    sata_phy_enable = Register(),
                    sata_phy_status = Register([0x2, 0x6, 0xf]),
                )
                self.opened = False
                self.closed = False

            def open(self):
                self.opened = True

            def close(self):
                self.closed = True

        bus = Bus()
        with mock.patch.object(test_init, "RemoteClient", return_value=bus), \
             mock.patch.object(test_init.time, "sleep") as sleep:
            test_init.init_test(port=1234, retries=3, interval=0.1)

        self.assertTrue(bus.opened)
        self.assertTrue(bus.closed)
        self.assertEqual(bus.regs.sata_phy_enable.writes, [0, 1])
        self.assertEqual(bus.regs.sata_phy_status.reads, [])
        self.assertEqual(sleep.call_args_list, [
            mock.call(1e-3),
            mock.call(0.1),
            mock.call(0.1),
            mock.call(0.1),
        ])

    def test_sata_rate_rejects_undersized_system_clock(self):
        with self.assertRaisesRegex(ValueError, "at least 150.0MHz"):
            SATATestSoC(platform=None, sys_clk_freq=100e6, gen="gen3")


class TestIdentifyDecode(unittest.TestCase):
    def test_model_number_is_words_27_to_46(self):
        # ATA IDENTIFY: model number is 40 ASCII characters in words 27-46 (2 characters per word).
        model = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789WXYZ"
        data  = [0x2020]*256
        for i in range(20):
            data[27 + i] = int.from_bytes(model[2*i:2*i + 2].encode(), byteorder="big")
        identify = test_bist.LiteSATABISTIdentifyDriver(mock.Mock(), None, "sata_bist")
        identify.data = data
        identify.decode()
        self.assertEqual(identify.model_number, model)


if __name__ == "__main__":
    unittest.main()
