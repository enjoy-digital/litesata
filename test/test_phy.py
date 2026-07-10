#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import Instance, Signal

from litesata.phy import LiteSATAPHY
from litesata.phy.gth3sataphy import GTH3LiteSATAPHYCRG, GTH3LiteSATAPHY
from litesata.phy.gth4sataphy import GTH4LiteSATAPHYCRG, GTH4LiteSATAPHY
from litesata.phy.gty4sataphy import GTY4LiteSATAPHYCRG, GTY4LiteSATAPHY
from litesata.phy.gthe4sataphy import GTHE4LiteSATAPHYCRG, GTHE4LiteSATAPHY
from litesata.phy.uspsataphy import USPLiteSATAPHYCRG, USPLiteSATAPHY
from litesata.phy.ussataphy import USLiteSATAPHYCRG, USLiteSATAPHY


class SATAPads:
    def __init__(self):
        self.rx_p = Signal()
        self.rx_n = Signal()
        self.tx_p = Signal()
        self.tx_n = Signal()


class TestPHY(unittest.TestCase):
    def assert_phy(self, device, gt_type, phy_cls, primitive):
        dut = LiteSATAPHY(
            device   = device,
            pads     = SATAPads(),
            gen      = "gen3",
            clk_freq = 100e6,
            refclk   = Signal(),
            gt_type  = gt_type,
            with_csr = False,
        )
        instances = [special.of for special in dut.phy.get_fragment().specials
            if isinstance(special, Instance)]
        self.assertIsInstance(dut.phy, phy_cls)
        self.assertIn(primitive, instances)

    def test_ultrascale_gth3(self):
        self.assert_phy(
            device    = "xcku040-ffva1156-2-e",
            gt_type   = "GTH",
            phy_cls   = GTH3LiteSATAPHY,
            primitive = "GTHE3_CHANNEL",
        )

    def test_ultrascale_plus_gth4(self):
        self.assert_phy(
            device    = "xcku15p-ffva1156-2-e",
            gt_type   = "GTH",
            phy_cls   = GTH4LiteSATAPHY,
            primitive = "GTHE4_CHANNEL",
        )

    def test_ultrascale_plus_gty4(self):
        self.assert_phy(
            device    = "xcvu9p-flga2104-2L-e",
            gt_type   = "GTY",
            phy_cls   = GTY4LiteSATAPHY,
            primitive = "GTYE4_CHANNEL",
        )

    def test_legacy_aliases(self):
        self.assertIs(USLiteSATAPHYCRG, GTH3LiteSATAPHYCRG)
        self.assertIs(USLiteSATAPHY,    GTH3LiteSATAPHY)
        self.assertIs(GTHE4LiteSATAPHYCRG, GTH4LiteSATAPHYCRG)
        self.assertIs(GTHE4LiteSATAPHY,    GTH4LiteSATAPHY)
        self.assertIs(USPLiteSATAPHYCRG, GTY4LiteSATAPHYCRG)
        self.assertIs(USPLiteSATAPHY,    GTY4LiteSATAPHY)


if __name__ == "__main__":
    unittest.main()
