#!/usr/bin/env python3

#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2023 Ryohei Niwase <niwase@lila.cs.tsukuba.ac.jp>
# SPDX-License-Identifier: BSD-2-Clause

from migen.fhdl.structure import ClockDomain, ClockSignal, Signal
from migen.fhdl.specials import Instance
from litex.gen.fhdl.module import LiteXModule
from litex.build.generic_platform import Subsignal, Pins
from litex.soc.cores.clock import USPMMCM
from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder

from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABIST

from litescope import LiteScopeAnalyzer

from litex_boards.platforms import xilinx_zcu102

# IOs ----------------------------------------------------------------------------------------------

_sata_io = [
    # AB09-FMCRAID / https://www.dgway.com/AB09-FMCRAID_E.html
    ("fmc2sata", 0,
        Subsignal("tx_p",  Pins("FMC_HPC1:DP0_C2M_P")),
        Subsignal("tx_n",  Pins("FMC_HPC1:DP0_C2M_N")),
        Subsignal("rx_p",  Pins("FMC_HPC1:DP0_M2C_P")),
        Subsignal("rx_n",  Pins("FMC_HPC1:DP0_M2C_N")),
    ),
    ("fmc2sata", 1,
        Subsignal("tx_p",  Pins("FMC_HPC1:DP1_C2M_P")),
        Subsignal("tx_n",  Pins("FMC_HPC1:DP1_C2M_N")),
        Subsignal("rx_p",  Pins("FMC_HPC1:DP1_M2C_P")),
        Subsignal("rx_n",  Pins("FMC_HPC1:DP1_M2C_N")),
    ),
    ("fmc2sata", 2,
        Subsignal("tx_p",  Pins("FMC_HPC1:DP2_C2M_P")),
        Subsignal("tx_n",  Pins("FMC_HPC1:DP2_C2M_N")),
        Subsignal("rx_p",  Pins("FMC_HPC1:DP2_M2C_P")),
        Subsignal("rx_n",  Pins("FMC_HPC1:DP2_M2C_N")),
    ),
    ("fmc2sata", 3,
        Subsignal("tx_p",  Pins("FMC_HPC1:DP3_C2M_P")),
        Subsignal("tx_n",  Pins("FMC_HPC1:DP3_C2M_N")),
        Subsignal("rx_p",  Pins("FMC_HPC1:DP3_M2C_P")),
        Subsignal("rx_n",  Pins("FMC_HPC1:DP3_M2C_N")),
    ),
    ("fmc2sata", 4,
        Subsignal("tx_p",  Pins("FMC_HPC1:DP4_C2M_P")),
        Subsignal("tx_n",  Pins("FMC_HPC1:DP4_C2M_N")),
        Subsignal("rx_p",  Pins("FMC_HPC1:DP4_M2C_P")),
        Subsignal("rx_n",  Pins("FMC_HPC1:DP4_M2C_N")),
    ),
    ("fmc2sata_refclk", 0,
        Subsignal("p",     Pins("FMC_HPC1:GBTCLK0_M2C_C_P")),
        Subsignal("n",     Pins("FMC_HPC1:GBTCLK0_M2C_C_N")),
    ),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, sata_refclk_src="internal"):
        self.cd_sys = ClockDomain()
        self.cd_sata_refclk = ClockDomain()

        # # #

        self.pll = pll = USPMMCM(speedgrade=-2)
        pll.register_clkin(platform.request("clk300"), 300e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        assert sata_refclk_src in ["internal", "external"]
        if sata_refclk_src == "internal":
            # Generate REFCLK from PLL
            pll.create_clkout(self.cd_sata_refclk, 150e6)
        elif sata_refclk_src == "external":
            # Supply REFCLK from MGTREFCLK
            sata_refclk_pads = platform.request("fmc2sata_refclk", 0)
            self.specials += Instance(
                "IBUFDS_GTE4",
                i_CEB = 0,
                i_I   = sata_refclk_pads.p,
                i_IB  = sata_refclk_pads.n,
                o_O   = self.cd_sata_refclk.clk
            )
            platform.add_period_constraint(
                platform.lookup_request("fmc2sata_refclk", 0, loose=True), 1e9 / 150e6
            )

# SATATestSoC --------------------------------------------------------------------------------------

class SATATestSoC(SoCCore):
    def __init__(self, sata_gen="gen3", sata_phy_dw=32, sata_refclk_src="external",
                 with_analyzer=False, analyzer_csv="analyzer.csv", **kwargs):
        platform = xilinx_zcu102.Platform()
        platform.add_extension(_sata_io)

        sys_clk_freq = 175e6
        sata_clk_freq = {"gen1": 75e6, "gen2": 150e6, "gen3": 300e6}[sata_gen]
        sata_clk_freq *= {16: 1., 32: 0.5}[sata_phy_dw]

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq, sata_refclk_src=sata_refclk_src)

        # SoCCore ----------------------------------------------------------------------------------
        SoCCore.__init__(self, platform, sys_clk_freq, ident="LiteSATA bench on ZCU102", **kwargs)

        # SATA -------------------------------------------------------------------------------------
        # PHY
        self.sata_phy = LiteSATAPHY(
            platform.device,
            pads          = platform.request("fmc2sata", 0),
            gen           = sata_gen,
            clk_freq      = sys_clk_freq,
            refclk        = ClockSignal("sata_refclk"),
            data_width    = sata_phy_dw,
            gt_type       = "GTH",  # Use GTHE4_CHANNEL
            use_gtgrefclk = sata_refclk_src == "internal"
        )

        # Core
        self.sata_core = LiteSATACore(self.sata_phy)

        # Crossbar
        self.sata_crossbar = LiteSATACrossbar(self.sata_core)

        # BIST
        self.sata_bist = LiteSATABIST(self.sata_crossbar, with_csr=True)

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
            self.analyzer = LiteScopeAnalyzer(analyzer_signals, 512, csr_csv=analyzer_csv)

# Build --------------------------------------------------------------------------------------------

def main():
    from litex.build.parser import LiteXArgumentParser
    parser = LiteXArgumentParser(platform=xilinx_zcu102.Platform, description="LiteSATA bench on ZCU102")
    parser.add_target_argument("--gen", type=int, default=3,
                               help="SATA Gen.", choices=[1, 2, 3])
    parser.add_target_argument("--dw", type=int, default=32,
                               help="PHY data width.", choices=[16, 32])
    parser.add_target_argument("--refclk", type=str, default="external",
                               help="PHY REFCLK source.",
                               choices=["internal", "external"])
    parser.add_target_argument("--with-analyzer", action="store_true", help="Enable LiteScope Analyzer.")
    parser.add_target_argument("--analyzer-csv", type=str, default="analyzer.csv",
                               help="Write LiteScope Analyzer mapping to specified CSV file.")
    parser.set_defaults(
        # Equivalent to SoCMini with UARTBone
        soc_csv              = "csr.csv",
        cpu_type             = "None",
        integrated_sram_size = 0,
        no_uart              = True,
        with_uartbone        = True,
        no_timer             = True,
    )
    args = parser.parse_args()

    soc = SATATestSoC(
        sata_gen        = "gen{}".format(args.gen),
        sata_phy_dw     = args.dw,
        sata_refclk_src = args.refclk,
        with_analyzer   = args.with_analyzer,
        analyzer_csv    = args.analyzer_csv,
        **parser.soc_argdict
    )

    builder = Builder(soc, **parser.builder_argdict)

    if args.build:
        builder.build(**parser.toolchain_argdict)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

if __name__ == "__main__":
    main()
