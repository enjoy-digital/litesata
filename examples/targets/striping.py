#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen.genlib.cdc import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import *
from litex.soc.cores.uart import UARTWishboneBridge

from litesata.common import *
from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.raid import LiteSATAStriping
from litesata.frontend.bist import LiteSATABIST

from targets.bist import CRG, StatusLeds

# StripingSoC --------------------------------------------------------------------------------------

class StripingSoC(SoCMini):
    default_platform = "kc705"
    def __init__(self, platform, revision="sata_gen3", data_width=16, nphys=4):
        self.nphys = nphys
        sys_clk_freq = int(200e6)
        SoCMini.__init__(self, platform, sys_clk_freq,
            csr_data_width = 32,
            ident          = "LiteSATA example design",
            ident_version  = True
        )

        # Serial Bridge ----------------------------------------------------------------------------
        self.submodules.bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq, baudrate=115200)
        self.add_wb_master(self.bridge.wishbone)
        self.submodules.crg = CRG(platform)

        # SATA PHYs --------------------------------------------------------------------------------
        self.sata_phys = []
        for i in range(self.nphys):
            sata_phy = LiteSATAPHY(platform.device,
                platform.request("sata_clocks") if i == 0 else self.sata_phys[0].crg.refclk,
                platform.request("sata", i),
                revision,
                sys_clk_freq,
                data_width)
            sata_phy = ClockDomainsRenamer({"sata_rx": "sata_rx{}".format(str(i)),
                                            "sata_tx": "sata_tx{}".format(str(i))})(sata_phy)
            setattr(self.submodules, "sata_phy{}".format(str(i)), sata_phy)
            self.sata_phys.append(sata_phy)

        # SATA Cores -------------------------------------------------------------------------------
        self.sata_cores = []
        for i in range(self.nphys):
            sata_core = LiteSATACore(self.sata_phys[i])
            setattr(self.submodules, "sata_core{}".format(str(i)), sata_core)
            self.sata_cores.append(sata_core)

        # SATA Frontend ----------------------------------------------------------------------------
        self.submodules.sata_striping = LiteSATAStriping(self.sata_cores)
        self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_striping)

        # SATA Application -------------------------------------------------------------------------
        self.submodules.sata_bist = LiteSATABIST(self.sata_crossbar, with_csr=True)
        self.add_csr("sata_bist")

        # Status Leds ------------------------------------------------------------------------------
        self.submodules.status_leds = StatusLeds(platform, self.sata_phys)

        # Timing constraints -----------------------------------------------------------------------
        platform.add_platform_command("""
create_clock -name sys_clk -period 5 [get_nets sys_clk]
""")

        for i in range(len(self.sata_phys)):
            self.sata_phys[i].crg.cd_sata_rx.clk.attr.add("keep")
            self.sata_phys[i].crg.cd_sata_tx.clk.attr.add("keep")
            platform.add_platform_command("""
create_clock -name {sata_rx_clk} -period {sata_clk_period} [get_nets {sata_rx_clk}]
create_clock -name {sata_tx_clk} -period {sata_clk_period} [get_nets {sata_tx_clk}]

set_false_path -from [get_clocks sys_clk] -to [get_clocks {sata_rx_clk}]
set_false_path -from [get_clocks sys_clk] -to [get_clocks {sata_tx_clk}]
set_false_path -from [get_clocks {sata_rx_clk}] -to [get_clocks sys_clk]
set_false_path -from [get_clocks {sata_tx_clk}] -to [get_clocks sys_clk]
""".format(sata_rx_clk="sata_rx{}_clk".format(str(i)),
           sata_tx_clk="sata_tx{}_clk".format(str(i)),
           sata_clk_period="3.3" if data_width == 16 else "6.6"))


# StripingSoCDevel ---------------------------------------------------------------------------------

class StripingSoCDevel(StripingSoC):
    csr_map = {
        "analyzer": 17
    }
    csr_map.update(StripingSoC.csr_map)
    def __init__(self, platform):
        from litescope import LiteScopeAnalyzer
        StripingSoC.__init__(self, platform)
        analyzer_signals = [
            self.sata_phy0.ctrl.ready,
            self.sata_phy1.ctrl.ready,
            self.sata_phy2.ctrl.ready,
            self.sata_phy3.ctrl.ready,

            self.sata_core0.link.tx.fsm,
            self.sata_core0.link.rx.fsm,

            self.sata_core0.sink.valid,
            self.sata_core0.source.valid,

            self.sata_phy0.source.valid,
            self.sata_phy0.source.data,
            self.sata_phy0.source.charisk,

            self.sata_phy0.sink.valid,
            self.sata_phy0.sink.data,
            self.sata_phy0.sink.charisk,

            self.sata_core1.link.tx.fsm,
            self.sata_core1.link.rx.fsm,

            self.sata_core1.sink.valid,
            self.sata_core1.source.valid,

            self.sata_phy1.source.valid,
            self.sata_phy1.source.data,
            self.sata_phy1.source.charisk,

            self.sata_phy1.sink.valid,
            self.sata_phy1.sink.data,
            self.sata_phy1.sink.charisk,

            self.sata_core2.link.tx.fsm,
            self.sata_core2.link.rx.fsm,

            self.sata_core2.sink.valid,
            self.sata_core2.source.valid,

            self.sata_phy2.source.valid,
            self.sata_phy2.source.data,
            self.sata_phy2.source.charisk,

            self.sata_phy2.sink.valid,
            self.sata_phy2.sink.data,
            self.sata_phy2.sink.charisk,

            self.sata_core3.link.tx.fsm,
            self.sata_core3.link.rx.fsm,

            self.sata_core3.sink.valid,
            self.sata_core3.source.valid,

            self.sata_phy3.source.valid,
            self.sata_phy3.source.data,
            self.sata_phy3.source.charisk,

            self.sata_phy3.sink.valid,
            self.sata_phy3.sink.data,
            self.sata_phy3.sink.charisk
        ]
        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 2048, csr_csv="test/analyzer.csv")

default_subtarget = StripingSoC