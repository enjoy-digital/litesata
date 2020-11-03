#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import argparse

from migen import *

from litex_boards.platforms import nexys_video
from litex_boards.targets.nexys_video import _CRG

from litex.build.generic_platform import *

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litesata.common import *
from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABIST

from litescope import LiteScopeAnalyzer

# IOs ----------------------------------------------------------------------------------------------

_sata_io = [
    # AB09-FMCRAID / https://www.dgway.com/AB09-FMCRAID_E.html
    ("fmc2sata", 0,
        Subsignal("clk_p", Pins("LPC:GBTCLK0_M2C_P")),
        Subsignal("clk_n", Pins("LPC:GBTCLK0_M2C_N")),
        Subsignal("tx_p",  Pins("LPC:DP0_C2M_P")),
        Subsignal("tx_n",  Pins("LPC:DP0_C2M_N")),
        Subsignal("rx_p",  Pins("LPC:DP0_M2C_P")),
        Subsignal("rx_n",  Pins("LPC:DP0_M2C_N"))
    ),
]

# SATATestSoC --------------------------------------------------------------------------------------

class SATATestSoC(SoCMini):
    def __init__(self, platform, gen="gen2", with_pll_refclk=True, with_analyzer=False):
        assert gen in ["gen1", "gen2"]
        sys_clk_freq  = int(100e6)
        sata_clk_freq = {"gen1": 75e6, "gen2": 150e6}[gen]

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "LiteSATA bench on Nexys Video",
            ident_version = True,
            with_uart     = True,
            uart_name     = "bridge")

        # SATA -------------------------------------------------------------------------------------
        if with_pll_refclk:
            # RefClk, Generate 150MHz from PLL.
            self.clock_domains.cd_sata_refclk = ClockDomain()
            self.crg.pll.create_clkout(self.cd_sata_refclk, 150e6)
            platform.add_platform_command("set_property SEVERITY {{Warning}} [get_drc_checks REQP-49]")

        # PHY
        self.submodules.sata_phy = LiteSATAPHY(platform.device,
            refclk     = None if not with_pll_refclk else self.cd_sata_refclk.clk,
            pads       = platform.request("fmc2sata"),
            gen        = gen,
            clk_freq   = sys_clk_freq,
            data_width = 16)
        self.add_csr("sata_phy")

        # Core
        self.submodules.sata_core = LiteSATACore(self.sata_phy)

        # Crossbar
        self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_core)

        # BIST
        self.submodules.sata_bist = LiteSATABIST(self.sata_crossbar, with_csr=True)
        self.add_csr("sata_bist")

        # Timing constraints
        platform.add_period_constraint(self.sata_phy.crg.cd_sata_tx.clk, 1e9/sata_clk_freq)
        platform.add_period_constraint(self.sata_phy.crg.cd_sata_rx.clk, 1e9/sata_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.sata_phy.crg.cd_sata_tx.clk,
            self.sata_phy.crg.cd_sata_rx.clk)

        # Leds -------------------------------------------------------------------------------------
        # sys_clk
        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        self.comb += platform.request("user_led", 0).eq(sys_counter[26])
        # tx_clk
        tx_counter = Signal(32)
        self.sync.sata_tx += tx_counter.eq(tx_counter + 1)
        self.comb += platform.request("user_led", 1).eq(tx_counter[26])
        # rx_clk
        rx_counter = Signal(32)
        self.sync.sata_rx += rx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 2).eq(rx_counter[26])
        # ready
        self.comb += platform.request("user_led", 3).eq(self.sata_phy.ctrl.ready)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                self.sata_phy.phy.tx_init.fsm,
                self.sata_phy.phy.rx_init.fsm,
                self.sata_phy.ctrl.fsm,

                self.sata_phy.ctrl.ready,
                self.sata_phy.source,
                self.sata_phy.sink,

                self.sata_core.command.sink,
                self.sata_core.command.source,

                self.sata_core.link.rx.fsm,
                self.sata_core.link.tx.fsm,
                self.sata_core.transport.rx.fsm,
                self.sata_core.transport.tx.fsm,
                self.sata_core.command.rx.fsm,
                self.sata_core.command.tx.fsm,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 512, csr_csv="analyzer.csv")
            self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA bench on Nexys Video")
    parser.add_argument("--build",         action="store_true", help="Build bitstream")
    parser.add_argument("--load",          action="store_true", help="Load bitstream (to SRAM)")
    parser.add_argument("--gen",           default="2",         help="SATA Gen: 1 or 2 (default)")
    parser.add_argument("--pll-refclk",    action="store_true", help="Generate RefClk from PLL")
    parser.add_argument("--with-analyzer", action="store_true", help="Add LiteScope Analyzer")
    args = parser.parse_args()

    platform = nexys_video.Platform()
    platform.add_extension(_sata_io)
    soc = SATATestSoC(platform, "gen" + args.gen, with_pll_refclk=args.pll_refclk, with_analyzer=args.with_analyzer)
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
