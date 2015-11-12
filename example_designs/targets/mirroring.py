from migen.genlib.cdc import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCCore
from litex.soc.cores.uart.bridge import UARTWishboneBridge

from litesata.common import *
from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.raid import LiteSATAMirroring
from litesata.frontend.bist import LiteSATABIST

from targets.bist import CRG, StatusLeds


class MirroringSoC(SoCCore):
    default_platform = "kc705"
    csr_map = {
        "sata_bist0": 16,
        "sata_bist1": 17,
        "sata_bist2": 18,
        "sata_bist3": 19,
    }
    csr_map.update(SoCCore.csr_map)
    def __init__(self, platform, revision="sata_gen3", trx_dw=16, nphys=4):
        self.nphys = nphys
        clk_freq = 200*1000000
        SoCCore.__init__(self, platform, clk_freq,
            cpu_type=None,
            csr_data_width=32,
            with_uart=False,
            ident="LiteSATA example design",
            with_timer=False)
        self.add_cpu_or_bridge(UARTWishboneBridge(platform.request("serial"), clk_freq, baudrate=115200))
        self.add_wb_master(self.cpu_or_bridge.wishbone)
        self.submodules.crg = CRG(platform)

        # SATA PHYs
        self.sata_phys = []
        for i in range(self.nphys):
            sata_phy = LiteSATAPHY(platform.device,
                                   platform.request("sata_clocks") if i == 0 else self.sata_phys[0].crg.refclk,
                                   platform.request("sata", i),
                                   revision,
                                   clk_freq,
                                   trx_dw)
            sata_phy = ClockDomainsRenamer({"sata_rx": "sata_rx{}".format(str(i)),
                                            "sata_tx": "sata_tx{}".format(str(i))})(sata_phy) 
            setattr(self.submodules, "sata_phy{}".format(str(i)), sata_phy)
            self.sata_phys.append(sata_phy)

        # SATA Cores
        self.sata_cores = []
        for i in range(self.nphys):
            sata_core = LiteSATACore(self.sata_phys[i])
            setattr(self.submodules, "sata_core{}".format(str(i)), sata_core)
            self.sata_cores.append(sata_core)

        # SATA Frontend
        self.submodules.sata_mirroring = LiteSATAMirroring(self.sata_cores)
        self.sata_crossbars = []
        for i in range(self.nphys):
            sata_crossbar = LiteSATACrossbar(self.sata_mirroring.ports[i])
            setattr(self.submodules, "sata_crossbar{}".format(str(i)), sata_crossbar)
            self.sata_crossbars.append(sata_crossbar)

        # SATA Application
        self.sata_bists = []
        for i in range(self.nphys):
            sata_bist = LiteSATABIST(self.sata_crossbars[i], with_csr=True)
            setattr(self.submodules, "sata_bist{}".format(str(i)), sata_bist)
            self.sata_bists.append(sata_bist)

        # Status Leds
        self.submodules.status_leds = StatusLeds(platform, self.sata_phys)


        platform.add_platform_command("""
create_clock -name sys_clk -period 5 [get_nets sys_clk]
""")

        for i in range(len(self.sata_phys)):
            platform.add_platform_command("""
create_clock -name {sata_rx_clk} -period {sata_clk_period} [get_nets {sata_rx_clk}]
create_clock -name {sata_tx_clk} -period {sata_clk_period} [get_nets {sata_tx_clk}]

set_false_path -from [get_clocks sys_clk] -to [get_clocks {sata_rx_clk}]
set_false_path -from [get_clocks sys_clk] -to [get_clocks {sata_tx_clk}]
set_false_path -from [get_clocks {sata_rx_clk}] -to [get_clocks sys_clk]
set_false_path -from [get_clocks {sata_tx_clk}] -to [get_clocks sys_clk]
""".format(sata_rx_clk="sata_rx{}_clk".format(str(i)),
           sata_tx_clk="sata_tx{}_clk".format(str(i)),
           sata_clk_period="3.3" if trx_dw == 16 else "6.6"))

default_subtarget = MirroringSoC