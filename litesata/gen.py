#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2014-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""
LiteSATA standalone core generator

LiteSATA aims to be directly used as a Python package when the SoC is created using LiteX. However,
for some use cases it could be interesting to generate a standalone Verilog file of the core:
- integration of the core in a SoC using a more traditional flow.
- need to version/package the core.
- avoid Migen/LiteX dependencies.
- etc...

The standalone core is generated from a YAML configuration file that allows the user to generate
easily a custom configuration of the core.

Two flavours can be generated:
- Native: the crossbar user ports are exposed as native streaming ports (no CPU required).
- Control/DMA: a Wishbone or AXI-Lite control slave (CSRs), a Wishbone DMA master and an IRQ output,
  integrating Identify, optional BIST and Sector2Mem/Mem2Sector DMAs through LiteX's add_sata. Native
  user ports can still be added to this flavour.

YAML keys:
- phy            : PHY name (see PHYS below).
- gen            : "gen1" | "gen2" | "gen3" (fixed link rate).
- clk_freq       : System clock frequency (Hz).
- data_width     : PHY datapath width, 16 | 32 (default: 16).
- gt_type        : "GTH" | "GTY" override for UltraScale+ (default: from the PHY name).
- rate_fallback  : Host-side Gen3 -> Gen2 -> Gen1 link rate fallback (default: False; requires a PHY
                   with runtime rate support, "gen" is then the highest rate).
- core           : "wishbone" | "axi-lite" control bus; omit for a native-only core.
- dma_data_width : Wishbone DMA master data width (default: 32).
- dma_mode       : "read" | "write" | "read+write" (default: "read+write").
- with_identify  : Identify frontend (default: True).
- with_bist      : BIST frontend (default: False).
- with_irq       : IRQ output (default: True).
- user_ports     : {name: {"type": "native", "data_width": 32 | 64}}.
"""

import os
import argparse

import yaml

from migen import *

from litex.gen import *

from litex.build.generic_platform import *
from litex.build.io import CRG

from litex.soc.interconnect import wishbone
from litex.soc.interconnect import axi
from litex.soc.integration.soc import *
from litex.soc.integration.builder import *

from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar

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
        ("enable",     0, Pins(1)),
        ("ready",      0, Pins(1)),
        ("link_up",    0, Pins(1)), # Debounced link status.
        ("connect",    0, Pins(1)), # Single-cycle pulse on link up.
        ("disconnect", 0, Pins(1)), # Single-cycle pulse on (debounced) link down.
    ]

def get_irq_ios():
    return [
        ("irq", 0, Pins(1)),
    ]

def get_native_user_port_ios(_id, dw=32):
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
            Subsignal("sink_data",     Pins(dw)),

            Subsignal("source_valid",    Pins(1)),
            Subsignal("source_last",     Pins(1)),
            Subsignal("source_ready",    Pins(1)),
            Subsignal("source_write",    Pins(1)),
            Subsignal("source_read",     Pins(1)),
            Subsignal("source_identify", Pins(1)),
            Subsignal("source_end",      Pins(1)),
            Subsignal("source_failed",   Pins(1)),
            Subsignal("source_data",     Pins(dw)),
        ),
    ]

# Core ---------------------------------------------------------------------------------------------

class _LiteSATACore(SoCMini):
    def __init__(self, platform, core_config):
        platform.add_extension(get_clkin_ios())
        platform.add_extension(get_sata_ios())
        platform.add_extension(get_ctrl_ios())

        # Parameters -------------------------------------------------------------------------------
        sys_clk_freq   = int(float(core_config["clk_freq"]))
        gen            = core_config["gen"]
        data_width     = int(core_config.get("data_width", 16))
        gt_type        = core_config.get("gt_type", "GTY")
        rate_fallback  = core_config.get("rate_fallback", False)
        bus_standard   = core_config.get("core", None)
        dma_data_width = int(core_config.get("dma_data_width", 32))
        dma_mode       = core_config.get("dma_mode", "read+write")
        with_identify  = core_config.get("with_identify", True)
        with_bist      = core_config.get("with_bist", False)
        with_irq       = core_config.get("with_irq", True)
        user_ports     = core_config.get("user_ports", {}) or {}
        assert gen in ["gen1", "gen2", "gen3"]
        assert data_width in [16, 32]
        assert bus_standard in [None, "wishbone", "axi-lite"]
        # sata_tx/sata_rx clock frequency (linerate / (10 * data_width)).
        sata_clk_freq = {"gen1": 75e6, "gen2": 150e6, "gen3": 300e6}[gen]*16/data_width

        # CRG --------------------------------------------------------------------------------------
        self.crg = CRG(platform.request("clk"), platform.request("rst"))

        # SoCMini ----------------------------------------------------------------------------------
        self.mem_map = {**SoCMini.mem_map, "csr": 0x00000000}
        SoCMini.__init__(self, platform, clk_freq=sys_clk_freq,
            bus_data_width = dma_data_width,
            csr_data_width = 32,
            ident          = "LiteSATA standalone core",
            ident_version  = True
        )

        # SATA PHY ---------------------------------------------------------------------------------
        self.sata_phy = LiteSATAPHY(platform.device,
            pads       = platform.request("sata"),
            gen        = gen,
            clk_freq   = sys_clk_freq,
            data_width = data_width,
            gt_type       = gt_type,
            rate_fallback = rate_fallback,
            with_csr      = bus_standard is not None)
        self.comb += self.sata_phy.enable.eq(platform.request("enable"))
        self.comb += platform.request("ready").eq(self.sata_phy.ready)
        self.comb += platform.request("link_up").eq(self.sata_phy.link_up)
        self.comb += platform.request("connect").eq(self.sata_phy.connect_irq)
        self.comb += platform.request("disconnect").eq(self.sata_phy.disconnect_irq)

        if bus_standard is None:
            # Native core: SATA Core + Crossbar only -----------------------------------------------
            self.sata_core     = LiteSATACore(self.sata_phy)
            self.sata_crossbar = LiteSATACrossbar(self.sata_core)

            # Timing constraints.
            platform.add_period_constraint(self.sata_phy.crg.cd_sata_tx.clk, 1e9/sata_clk_freq)
            platform.add_period_constraint(self.sata_phy.crg.cd_sata_rx.clk, 1e9/sata_clk_freq)
            platform.add_false_path_constraints(
                self.crg.cd_sys.clk,
                self.sata_phy.crg.cd_sata_tx.clk,
                self.sata_phy.crg.cd_sata_rx.clk)
        else:
            # Control bus ----------------------------------------------------------------------------
            if bus_standard == "wishbone":
                ctrl_bus = wishbone.Interface()
                platform.add_extension(ctrl_bus.get_ios("wb_ctrl"))
                self.comb += ctrl_bus.connect_to_pads(platform.request("wb_ctrl"), mode="slave")
            else:
                ctrl_bus = axi.AXILiteInterface(address_width=32, data_width=32)
                platform.add_extension(ctrl_bus.get_ios("axil_ctrl"))
                self.comb += ctrl_bus.connect_to_pads(platform.request("axil_ctrl"), mode="slave")
            self.bus.add_master(name="ctrl", master=ctrl_bus)

            # DMA bus --------------------------------------------------------------------------------
            # Wishbone DMA Master, only slave of a dedicated bus the SATA DMAs are added to.
            wb_dma = wishbone.Interface(data_width=dma_data_width)
            platform.add_extension(wb_dma.get_ios("wb_dma"))
            self.comb += wb_dma.connect_to_pads(platform.request("wb_dma"), mode="master")
            self.dma_bus = SoCBusHandler(
                name          = "SoCDMABusHandler",
                standard      = "wishbone",
                data_width    = dma_data_width,
                address_width = 32,
            )
            self.dma_bus.add_slave("dma", slave=wb_dma, region=SoCRegion(origin=0x00000000, size=0x100000000))

            # SATA Core/Crossbar/Frontends/DMAs (and timing constraints) through LiteX's add_sata.
            self.add_sata(name="sata",
                phy           = self.sata_phy,
                mode          = dma_mode,
                with_identify = with_identify,
                with_bist     = with_bist,
                with_irq      = with_irq,
            )

            # IRQ ------------------------------------------------------------------------------------
            if with_irq:
                platform.add_extension(get_irq_ios())
                self.comb += platform.request("irq").eq(self.sata_irq.irq)

        # SATA User Ports --------------------------------------------------------------------------
        for name, port in user_ports.items():
            # Native
            if port["type"] == "native":
                dw = int(port.get("data_width", 32))
                assert dw in [32, 64]
                user_port = self.sata_crossbar.get_port(dw)
                platform.add_extension(get_native_user_port_ios(name, dw))
                user_port_pads = platform.request("user_port_{}".format(name))

                self.comb += [
                    user_port.sink.valid.eq(user_port_pads.sink_valid),
                    user_port.sink.last.eq(user_port_pads.sink_last),
                    user_port.sink.write.eq(user_port_pads.sink_write),
                    user_port.sink.read.eq(user_port_pads.sink_read),
                    user_port.sink.identify.eq(user_port_pads.sink_identify),
                    user_port.sink.sector.eq(user_port_pads.sink_sector),
                    user_port.sink.count.eq(user_port_pads.sink_count),
                    user_port.sink.data.eq(user_port_pads.sink_data),

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

# PHYs ---------------------------------------------------------------------------------------------

# PHY name -> (Device, GT type, Toolchain). The device only selects the transceiver PHY; the generated
# core is independent of the exact part/package.
PHYS = {
    "A7SATAPHY"   : ("xc7a",    None,  "vivado"), # Artix-7 GTP.
    "K7SATAPHY"   : ("xc7k",    None,  "vivado"), # Kintex-7 GTX.
    "GTH3SATAPHY" : ("xcku040", "GTH", "vivado"), # Kintex/Virtex UltraScale GTH.
    "GTH4SATAPHY" : ("xcau25p", "GTH", "vivado"), # Artix/Kintex/Zynq UltraScale+ GTH.
    "GTY4SATAPHY" : ("xcvu9p",  "GTY", "vivado"), # Kintex/Virtex UltraScale+ GTY.
}

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA standalone core generator")
    parser.add_argument("config",                           help="YAML config file")
    parser.add_argument("--output-dir", default="build",    help="Output directory")
    parser.add_argument("--name",       default="litesata_core", help="Name of the generated core")
    args = parser.parse_args()
    with open(args.config, "r", encoding="utf-8") as f:
        core_config = yaml.load(f, Loader=yaml.SafeLoader)

    # Convert YAML elements to Python/LiteX --------------------------------------------------------
    for k, v in core_config.items():
        replaces = {"False": False, "True": True, "None": None}
        for r in replaces.keys():
            if v == r:
                core_config[k] = replaces[r]

    # Generate core --------------------------------------------------------------------------------
    if core_config["phy"] not in PHYS:
        raise ValueError("Unsupported SATA PHY: {} (supported: {})".format(
            core_config["phy"], ", ".join(PHYS.keys())))
    device, gt_type, toolchain = PHYS[core_config["phy"]]
    if gt_type is not None:
        core_config["gt_type"] = gt_type
    from litex.build.xilinx import XilinxPlatform
    platform = XilinxPlatform(device, io=[], toolchain=toolchain)
    soc      = _LiteSATACore(platform, core_config)
    builder  = Builder(soc,
        output_dir       = args.output_dir,
        compile_gateware = False,
        csr_csv          = os.path.join(args.output_dir, "csr.csv"))
    builder.build(build_name=args.name)

if __name__ == "__main__":
    main()
