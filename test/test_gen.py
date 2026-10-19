#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import Instance, Signal

from litesata.gen import PHYS
from litesata.phy import LiteSATAPHY
from litesata.phy.a7sataphy import A7LiteSATAPHY
from litesata.phy.gth3sataphy import GTH3LiteSATAPHY
from litesata.phy.gth4sataphy import GTH4LiteSATAPHY
from litesata.phy.gty4sataphy import GTY4LiteSATAPHY
from litesata.phy.k7sataphy import K7LiteSATAPHY


class SATAPads:
    def __init__(self):
        self.rx_p = Signal()
        self.rx_n = Signal()
        self.tx_p = Signal()
        self.tx_n = Signal()


class TestGen(unittest.TestCase):
    def test_phys_select_expected_transceivers(self):
        expected = {
            "A7SATAPHY"   : (A7LiteSATAPHY,   "GTPE2_CHANNEL"),
            "K7SATAPHY"   : (K7LiteSATAPHY,   "GTXE2_CHANNEL"),
            "GTH3SATAPHY" : (GTH3LiteSATAPHY, "GTHE3_CHANNEL"),
            "GTH4SATAPHY" : (GTH4LiteSATAPHY, "GTHE4_CHANNEL"),
            "GTY4SATAPHY" : (GTY4LiteSATAPHY, "GTYE4_CHANNEL"),
        }
        self.assertEqual(set(PHYS.keys()), set(expected.keys()))
        for name, (device, gt_type, toolchain) in PHYS.items():
            phy_cls, primitive = expected[name]
            with self.subTest(phy=name):
                self.assertIn(toolchain, ["ise", "vivado"])
                dut = LiteSATAPHY(
                    device   = device,
                    pads     = SATAPads(),
                    gen      = "gen3",
                    clk_freq = 100e6,
                    refclk   = Signal(),
                    gt_type  = gt_type if gt_type is not None else "GTY",
                    with_csr = False,
                )
                instances = [special.of for special in dut.phy.get_fragment().specials
                    if isinstance(special, Instance)]
                self.assertIsInstance(dut.phy, phy_cls)
                self.assertIn(primitive, instances)


if __name__ == "__main__":
    unittest.main()
