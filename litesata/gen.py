#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2014-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
LiteSATA standalone core generator

LiteSATA aims to be directly used as a python package when the SoC is created using LiteX. However,
for some use cases it could be interesting to generate a standalone verilog file of the core:
- integration of the core in a SoC using a more traditional flow.
- need to version/package the core.
- avoid Migen/LiteX dependencies.
- etc...

The standalone core is generated from a YAML configuration file that allows the user to generate
easily a custom configuration of the core.

Current version of the generator is limited to:
- Xilinx 7-Series.
"""

import yaml
import argparse
import subprocess

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *

from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar

from litex.build.generic_platform import *

# IOs/Interfaces -----------------------------------------------------------------------------------

def get_clkin_ios():
    return [
        ("clk", 0, Pins(1)),
        ("rst", 0, Pins(1))
    ]

def get_sata_ios():
    return [
        ("sata", 0,
            Subsignal("clk_p", Pins(1)),
            Subsignal("clk_n", Pins(1)),
            Subsignal("rx_p",  Pins(1)),
            Subsignal("rx_n",  Pins(1)),
            Subsignal("tx_p",  Pins(1)),
            Subsignal("tx_n",  Pins(1)),
        ),
    ]

def get_ctrl_ios():
    return [
        ("enable", 0, Pins(1)),
        ("ready",  0, Pins(1)),
    ]

def get_native_user_port_ios(_id):
    return [
        ("user_port_{}".format(_id), 0,
            Subsignal("sink_valid",    Pins(1)),
            Subsignal("sink_last",     Pins(1)),
            Subsignal("sink_ready",    Pins(1)),
            Subsignal("sink_write",    Pins(1)),
            Subsignal("sink_read",     Pins(1)),
            Subsignal("sink_identify", Pins(1)),
            Subsignal("sink_sector",   Pins(48)),
            Subsignal("sink_count",    Pins(16)),
            Subsignal("sink_data",     Pins(32)),

            Subsignal("source_valid",    Pins(1)),
            Subsignal("source_last",     Pins(1)),
            Subsignal("source_ready",    Pins(1)),
            Subsignal("source_write",    Pins(1)),
            Subsignal("source_read",     Pins(1)),
            Subsignal("source_identify", Pins(1)),
            Subsignal("source_end",      Pins(1)),
            Subsignal("source_failed",   Pins(1)),
            Subsignal("source_data",     Pins(32)),
        ),
    ]

# CRG ----------------------------------------------------------------------------------------------

class LiteSATACRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        # # #

        platform.add_extension(get_clkin_ios())
        self.comb += self.cd_sys.clk.eq(platform.request("clk"))
        self.specials += AsyncResetSynchronizer(self.cd_sys, platform.request("rst"))

# Core ---------------------------------------------------------------------------------------------

class _LiteSATACore(SoCMini):
    SoCMini.mem_map["csr"] = 0x00000000
    SoCMini.csr_map = {
        "ctrl":           0,
    }
    def __init__(self, platform, core_config):
        platform.add_extension(get_sata_ios())
        platform.add_extension(get_ctrl_ios())
        sys_clk_freq = int(float(core_config.get("clk_freq")))
        sata_clk_freq = {"gen1": 75e6, "gen2": 150e6, "gen3": 300e6}[core_config["gen"]]

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq,
            csr_data_width = 32,
            ident          = "LiteSATA standalone core",
            ident_version  = True
        )

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = LiteSATACRG(platform, sys_clk_freq)

        # SATA PHY ---------------------------------------------------------------------------------
        self.submodules.sata_phy = LiteSATAPHY(platform.device,
            pads       = platform.request("sata"),
            gen        = core_config["gen"],
            clk_freq   = sys_clk_freq,
            data_width = 16,
            with_csr   = False)
        self.comb += self.sata_phy.enable.eq(platform.request("enable"))
        self.comb += platform.request("ready").eq(self.sata_phy.ready)

        # SATA Core --------------------------------------------------------------------------------
        self.submodules.sata_core = LiteSATACore(self.sata_phy)

        # SATA Crossbar ----------------------------------------------------------------------------
        self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_core)

        # SATA User Ports --------------------------------------------------------------------------
        for name, port in core_config["user_ports"].items():
            # Native
            if port["type"] == "native":
                assert port["data_width"] == 32
                user_port = self.sata_crossbar.get_port()
                platform.add_extension(get_native_user_port_ios(name))
                user_port_pads = platform.request("user_port_{}".format(name))

                self.comb += [
                    user_port.sink.valid.eq(user_port_pads.sink_valid),
                    user_port.sink.last.eq(user_port_pads.sink_last),
                    user_port.sink.write.eq(user_port_pads.sink_write),
                    user_port.sink.read.eq(user_port_pads.sink_read),
                    user_port.sink.identify.eq(user_port_pads.sink_identify),
                    user_port.sink.sector.eq(user_port_pads.sink_sector),
                    user_port.sink.count.eq(user_port_pads.sink_count),

                    user_port_pads.sink_ready.eq(user_port.sink.ready),
                ]

                self.comb += [
                    user_port_pads.source_valid.eq(user_port.source.valid),
                    user_port_pads.source_last.eq(user_port.source.last),
                    user_port_pads.source_write.eq(user_port.source.write),
                    user_port_pads.source_read.eq(user_port.source.read),
                    user_port_pads.source_identify.eq(user_port.source.identify),
                    user_port_pads.source_end.eq(user_port.source.end),
                    user_port_pads.source_failed.eq(user_port.source.failed),
                    user_port_pads.source_data.eq(user_port.source.data),

                    user_port.source.ready.eq(user_port_pads.source_ready),
                ]
            else:
                raise ValueError("Unsupported port type: {}".format(port["type"]))


        # Timing constraints
        platform.add_period_constraint(self.sata_phy.crg.cd_sata_tx.clk, 1e9/sata_clk_freq)
        platform.add_period_constraint(self.sata_phy.crg.cd_sata_rx.clk, 1e9/sata_clk_freq)
        self.platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.sata_phy.crg.cd_sata_tx.clk,
            self.sata_phy.crg.cd_sata_rx.clk)

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA standalone core generator")
    parser.add_argument("config", help="YAML config file")
    args = parser.parse_args()
    core_config = yaml.load(open(args.config).read(), Loader=yaml.Loader)

    # Convert YAML elements to Python/LiteX --------------------------------------------------------
    for k, v in core_config.items():
        replaces = {"False": False, "True": True, "None": None}
        for r in replaces.keys():
            if v == r:
                core_config[k] = replaces[r]

    # Generate core --------------------------------------------------------------------------------
    if core_config["phy"] == "A7SATAPHY":
        from litex.build.xilinx import XilinxPlatform
        platform = XilinxPlatform("xc7a", io=[])
    elif core_config["phy"] == "K7SATAPHY":
        from litex.build.xilinx import XilinxPlatform
        platform = XilinxPlatform("xc7k", io=[])
    else:
        raise ValueError("Unsupported SATA PHY: {}".format(core_config["phy"]))
    soc      = _LiteSATACore(platform, core_config)
    builder  = Builder(soc, output_dir="build", compile_gateware=False)
    builder.build(build_name="litesata_core", regular_comb=True)

if __name__ == "__main__":
    main()
