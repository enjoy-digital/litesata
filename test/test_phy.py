#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import ClockDomain, Instance, Module, Signal
from migen.sim import run_simulation

from litex.soc.interconnect import stream

from litesata.phy import LiteSATAPHY
from litesata.phy.datapath import LiteSATAPHYDatapathRX
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

    def test_rx_realigns_with_resetless_payload(self):
        class DUT(Module):
            def __init__(self):
                self.clock_domains.cd_sys     = ClockDomain()
                self.clock_domains.cd_sata_rx = ClockDomain()
                self.submodules.rx = LiteSATAPHYDatapathRX(data_width=16)

        def find_submodule(module, cls):
            for _, submodule in module._submodules:
                if isinstance(submodule, cls):
                    return submodule
                match = find_submodule(submodule, cls)
                if match is not None:
                    return match

        dut       = DUT()
        realigner = dut.rx._submodules[0][1]
        converter = find_submodule(realigner, stream._UpConverter)
        self.assertIsNotNone(converter)
        stream.set_reset_less(converter.source.payload)

        realign_seen = []
        aligned_data = []

        def generator():
            yield dut.rx.sink.valid.eq(1)

            # Force a control character into the upper half of an assembled word.
            yield dut.rx.sink.charisk.eq(0b01)
            for _ in range(4):
                if (yield realigner.reset):
                    realign_seen.append(True)
                yield

            # The converter must leave reset and resume with ordinary data.
            yield dut.rx.sink.charisk.eq(0)
            yield dut.rx.sink.data.eq(0x1234)
            for _ in range(12):
                if (yield realigner.source.valid):
                    aligned_data.append((yield realigner.source.data))
                yield

        run_simulation(dut, {"sata_rx": generator()}, clocks={"sys": 10, "sata_rx": 10})

        self.assertTrue(realign_seen)
        self.assertIn(0x12341234, aligned_data)


if __name__ == "__main__":
    unittest.main()
