#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import argparse

from migen import *

from litex_boards.platforms import xcu1525

from litex.build.generic_platform import *

from litex.soc.cores.clock import USPPLL
from litex.soc.interconnect.csr import *
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
    # SFP 2 SATA Adapter / https://shop.trenz-electronic.de/en/TE0424-01-SFP-2-SATA-Adapter
    ("qsfp2sata", 0,
        Subsignal("rx_n",  Pins("N3")),
        Subsignal("rx_p",  Pins("N4")),
        Subsignal("tx_n",  Pins("N8")),
        Subsignal("tx_p",  Pins("N9")),
    ),
    # PCIe 2 SATA Custom Adapter (With PCIe Riser / SATA cable mod).
    ("pcie2sata", 0,
        Subsignal("rx_n",  Pins("AF1")),
        Subsignal("rx_p",  Pins("AF2")),
        Subsignal("tx_n",  Pins("AF6")),
        Subsignal("tx_p",  Pins("AF7")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        # PLL
        self.submodules.pll = pll = USPPLL(speedgrade=-2)
        pll.register_clkin(platform.request("clk300"), 300e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

# SATATestSoC --------------------------------------------------------------------------------------

class SATATestSoC(SoCMini):
    def __init__(self, platform, connector="qsfp", gen="gen2", with_analyzer=False):
        assert connector in ["qsfp", "pcie"]
        assert gen in ["gen1", "gen2", "gen3"]

        sys_clk_freq  = int(187.5e6)
        sata_clk_freq = {"gen1": 75e6, "gen2": 150e6, "gen3": 300e6}[gen]

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "LiteSATA bench on XCU1525",
            ident_version = True,
            with_uart     = True,
            uart_name     = "bridge")

        # SATA -------------------------------------------------------------------------------------
        # RefClk / Generate 150MHz from PLL.
        self.clock_domains.cd_sata_refclk = ClockDomain()
        self.crg.pll.create_clkout(self.cd_sata_refclk, 150e6, buf=None)
        sata_refclk = ClockSignal("sata_refclk")

        # PHY
        self.submodules.sata_phy = LiteSATAPHY(platform.device,
            refclk     = sata_refclk,
            pads       = platform.request(connector+"2sata"),
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
        # rx_clk
        rx_counter = Signal(32)
        self.sync.sata_rx += rx_counter.eq(rx_counter + 1)
        self.comb += platform.request("user_led", 1).eq(rx_counter[26])
        # ready
        self.comb += platform.request("user_led", 2).eq(self.sata_phy.ctrl.ready)

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
    parser = argparse.ArgumentParser(description="LiteSATA bench on XCU1525")
    parser.add_argument("--build",         action="store_true", help="Build bitstream")
    parser.add_argument("--load",          action="store_true", help="Load bitstream (to SRAM)")
    parser.add_argument("--gen",           default="3",         help="SATA Gen: 1, 2 or 3 (default)")
    parser.add_argument("--connector",     default="qsfp",      help="SATA Connector: qsfp (default) or pcie")
    parser.add_argument("--with-analyzer", action="store_true", help="Add LiteScope Analyzer")
    args = parser.parse_args()

    platform = xcu1525.Platform()
    platform.add_extension(_sata_io)
    soc = SATATestSoC(platform, args.connector, "gen" + args.gen, with_analyzer=args.with_analyzer)
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
