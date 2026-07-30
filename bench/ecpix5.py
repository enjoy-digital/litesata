#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse
import os

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.build.generic_platform import Pins, Subsignal
from litex.soc.cores.clock import ECP5PLL
from litex.soc.integration.builder import Builder
from litex.soc.integration.soc import SoCMini

from litex_boards.platforms import lambdaconcept_ecpix5

from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABIST
from litesata.phy import LiteSATAPHY

# IOs ----------------------------------------------------------------------------------------------

_sata_io = [
    ("sata_tx", 0,
        Subsignal("p", Pins("AD16")),
        Subsignal("n", Pins("AD17")),
    ),
    ("sata_rx", 0,
        Subsignal("p", Pins("AF15")),
        Subsignal("n", Pins("AF16")),
    ),
]

class SATAPads:
    def __init__(self, tx, rx):
        self.tx_p = tx.p
        self.tx_n = tx.n
        self.rx_p = rx.p
        self.rx_n = rx.n

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.cd_sys         = ClockDomain()
        self.cd_por         = ClockDomain(reset_less=True)
        self.cd_sata_refclk = ClockDomain(reset_less=True)

        # # #

        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        platform.add_period_constraint(clk100, 1e9/100e6)

        por_count = Signal(16, reset=2**16 - 1)
        por_done  = Signal()
        self.comb += [
            self.cd_por.clk.eq(ClockSignal()),
            por_done.eq(por_count == 0),
        ]
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        self.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, with_reset=False)
        pll.create_clkout(self.cd_sata_refclk, 150e6)
        self.specials += AsyncResetSynchronizer(
            self.cd_sys, ~por_done | ~pll.locked | ~rst_n)

# SATATestSoC --------------------------------------------------------------------------------------

class SATATestSoC(SoCMini):
    def __init__(self, platform, sys_clk_freq=int(90e6)):
        self.crg = _CRG(platform, sys_clk_freq)
        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteSATA bench on ECPIX-5.")

        self.add_uartbone(baudrate=1e6)

        self.sata_phy = LiteSATAPHY(platform.device,
            refclk     = self.crg.cd_sata_refclk.clk,
            pads       = SATAPads(platform.request("sata_tx"), platform.request("sata_rx")),
            gen        = "gen2",
            clk_freq   = sys_clk_freq,
            data_width = 16,
            dual       = 1,
            channel    = 0,
        )
        self.sata_core     = LiteSATACore(self.sata_phy)
        self.sata_crossbar = LiteSATACrossbar(self.sata_core)
        self.sata_bist     = LiteSATABIST(self.sata_crossbar, with_csr=True)

        platform.add_period_constraint(self.sata_phy.crg.cd_sata_tx.clk, 1e9/150e6)
        platform.add_period_constraint(self.sata_phy.crg.cd_sata_rx.clk, 1e9/150e6)
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.sata_phy.crg.cd_sata_tx.clk,
            self.sata_phy.crg.cd_sata_rx.clk,
        )

        counter = Signal(32)
        self.sync += counter.eq(counter + 1)
        self.comb += [
            platform.request("rgb_led", 0).g.eq(~counter[26]),
            platform.request("rgb_led", 1).g.eq(~self.sata_phy.ready),
        ]

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA bench on ECPIX-5.")
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream to SRAM.")
    parser.add_argument("--device", default="85F", help="FPGA device (85F or 45F).")
    parser.add_argument("--seed", default=3, type=int, help="nextpnr placement seed.")
    args = parser.parse_args()

    platform = lambdaconcept_ecpix5.Platform(device=args.device, toolchain="trellis")
    platform.add_extension(_sata_io)
    soc = SATATestSoC(platform)
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build, seed=args.seed)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
