#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import argparse

from migen import *

from litex.boards.platforms import nexys_video
from litex.boards.targets.nexys_video import _CRG

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
    ("fmc_refclk", 0, # 150MHz
        Subsignal("p", Pins("LPC:GBTCLK0_M2C_P")),
        Subsignal("n", Pins("LPC:GBTCLK0_M2C_N"))
    ),
    ("fmc", 0,
        Subsignal("txp", Pins("LPC:DP0_C2M_P")),
        Subsignal("txn", Pins("LPC:DP0_C2M_N")),
        Subsignal("rxp", Pins("LPC:DP0_M2C_P")),
        Subsignal("rxn", Pins("LPC:DP0_M2C_N"))
    ),
]

# StatusLeds ---------------------------------------------------------------------------------------

class StatusLeds(Module):
    def __init__(self, platform, sata_phys):
        if not isinstance(sata_phys, list):
            sata_phys = [sata_phys]
            use_cd_num = False
        else:
            use_cd_num = True
        for i, sata_phy in enumerate(sata_phys):
            # 1Hz blinking sata_tx led
            tx_led = platform.request("user_led", 4*i+1)
            tx_cnt = Signal(32)
            freq = int(frequencies[sata_phy.revision]*1000*1000)
            tx_sync = getattr(self.sync, "sata_tx{}".format(str(i) if use_cd_num else ""))
            tx_sync += \
                If(tx_cnt == 0,
                    tx_led.eq(~tx_led),
                    tx_cnt.eq(freq//2)
                ).Else(
                    tx_cnt.eq(tx_cnt-1)
                )

            # 1Hz bliking sata_rx led
            rx_led = platform.request("user_led", 4*i)
            rx_cnt = Signal(32)
            freq = int(frequencies[sata_phy.revision]*1000*1000)
            rx_sync = getattr(self.sync, "sata_rx{}".format(str(i) if use_cd_num else ""))
            rx_sync += \
                If(rx_cnt == 0,
                    rx_led.eq(~rx_led),
                    rx_cnt.eq(freq//2)
                ).Else(
                    rx_cnt.eq(rx_cnt-1)
                )

            # 1Hz blinking sata_refclk led
            refclk_led = platform.request("user_led", 4*i+2)
            refclk_cnt = Signal(32)
            freq = int(frequencies[sata_phy.revision]*1000*1000)
            refclk_sync = getattr(self.sync, "sata_refclk{}".format(str(i) if use_cd_num else ""))
            refclk_sync += \
                If(refclk_cnt == 0,
                    refclk_led.eq(~refclk_led),
                    refclk_cnt.eq(freq//2)
                ).Else(
                    refclk_cnt.eq(refclk_cnt-1)
                )

            # ready led
            self.comb += platform.request("user_led", 4*i+3).eq(sata_phy.ctrl.ready)

# SATATestSoC --------------------------------------------------------------------------------------

class SATATestSoC(SoCMini):
    def __init__(self, platform, revision="sata_gen2", data_width=16, with_analyzer=False):
        sys_clk_freq = int(100e6)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "LiteSATA bench on Nexys Video",
            ident_version = True,
            with_uart     = True,
            uart_name     = "bridge")

        # SATA -------------------------------------------------------------------------------------
        # PHY
        self.submodules.sata_phy = LiteSATAPHY(platform.device,
            refclk     = platform.request("fmc_refclk"), # Use 150MHz refclk provided by FMC.
            pads       = platform.request("fmc"),
            revision   = revision,
            clk_freq   = sys_clk_freq,
            data_width = data_width)

        # Core
        self.submodules.sata_core = LiteSATACore(self.sata_phy)

        # Crossbar
        self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_core)

        # BIST
        self.submodules.sata_bist = LiteSATABIST(self.sata_crossbar, with_csr=True)
        self.add_csr("sata_bist")

        # Status Leds ------------------------------------------------------------------------------
        self.submodules.leds = StatusLeds(platform, self.sata_phy)

        # Timing constraints -----------------------------------------------------------------------
        # FIXME: update.
        self.sata_phy.crg.cd_sata_rx.clk.attr.add("keep")
        self.sata_phy.crg.cd_sata_tx.clk.attr.add("keep")
        platform.add_platform_command("""
create_clock -name sys_clk -period 10 [get_nets sys_clk]

create_clock -name sata_rx_clk -period {sata_clk_period} [get_nets sata_rx_clk]
create_clock -name sata_tx_clk -period {sata_clk_period} [get_nets sata_tx_clk]

set_false_path -from [get_clocks sys_clk] -to [get_clocks sata_rx_clk]
set_false_path -from [get_clocks sys_clk] -to [get_clocks sata_tx_clk]
set_false_path -from [get_clocks sata_rx_clk] -to [get_clocks sys_clk]
set_false_path -from [get_clocks sata_tx_clk] -to [get_clocks sys_clk]
""".format(sata_clk_period="6.6" if data_width == 16 else "13.2"))

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                self.sata_phy.crg.tx_startup_fsm,
                self.sata_phy.crg.rx_startup_fsm,
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
    parser.add_argument("--with-analyzer", action="store_true", help="Add LiteScope Analyzer")
    args = parser.parse_args()

    platform = nexys_video.Platform()
    platform.add_extension(_sata_io)
    soc = SATATestSoC(platform, with_analyzer=args.with_analyzer)
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
