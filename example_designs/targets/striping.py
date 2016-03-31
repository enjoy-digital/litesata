from litex.gen.fhdl.specials import Keep
from litex.gen.genlib.cdc import *
from litex.gen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCCore
from litex.soc.cores.uart.bridge import UARTWishboneBridge

from litesata.common import *
from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.raid import LiteSATAStriping
from litesata.frontend.bist import LiteSATABIST

from targets.bist import CRG, StatusLeds


class StripingSoC(SoCCore):
    default_platform = "kc705"
    csr_map = {
        "sata_bist": 16
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
            with_timer=False
        )
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
        self.submodules.sata_striping = LiteSATAStriping(self.sata_cores)
        self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_striping)

        # SATA Application
        self.submodules.sata_bist = LiteSATABIST(self.sata_crossbar, with_csr=True)

        # Status Leds
        self.submodules.status_leds = StatusLeds(platform, self.sata_phys)


        platform.add_platform_command("""
create_clock -name sys_clk -period 5 [get_nets sys_clk]
""")

        for i in range(len(self.sata_phys)):
            self.specials += [
                Keep(ClockSignal("sata_rx{}".format(str(i)))),
                Keep(ClockSignal("sata_tx{}".format(str(i))))
            ]
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


class StripingSoCDevel(StripingSoC):
    csr_map = {
        "analyzer": 17
    }
    csr_map.update(StripingSoC.csr_map)
    def __init__(self, platform):
        from litescope import LiteScopeAnalyzer
        StripingSoC.__init__(self, platform)

        self.sata_core0_link_tx_fsm_state = Signal(4)
        self.sata_core0_link_rx_fsm_state = Signal(4)

        self.sata_core1_link_tx_fsm_state = Signal(4)
        self.sata_core1_link_rx_fsm_state = Signal(4)

        self.sata_core2_link_tx_fsm_state = Signal(4)
        self.sata_core2_link_rx_fsm_state = Signal(4)

        self.sata_core3_link_tx_fsm_state = Signal(4)
        self.sata_core3_link_rx_fsm_state = Signal(4)

        debug = [
            self.sata_phy0.ctrl.ready,
            self.sata_phy1.ctrl.ready,
            self.sata_phy2.ctrl.ready,
            self.sata_phy3.ctrl.ready,

            self.sata_core0_link_tx_fsm_state,
            self.sata_core0_link_rx_fsm_state,

            self.sata_core0.sink.valid,
            self.sata_core0.source.valid,

            self.sata_phy0.source.valid,
            self.sata_phy0.source.data,
            self.sata_phy0.source.charisk,

            self.sata_phy0.sink.valid,
            self.sata_phy0.sink.data,
            self.sata_phy0.sink.charisk,

            self.sata_core1_link_tx_fsm_state,
            self.sata_core1_link_rx_fsm_state,

            self.sata_core1.sink.valid,
            self.sata_core1.source.valid,

            self.sata_phy1.source.valid,
            self.sata_phy1.source.data,
            self.sata_phy1.source.charisk,

            self.sata_phy1.sink.valid,
            self.sata_phy1.sink.data,
            self.sata_phy1.sink.charisk,

            self.sata_core2_link_tx_fsm_state,
            self.sata_core2_link_rx_fsm_state,

            self.sata_core2.sink.valid,
            self.sata_core2.source.valid,

            self.sata_phy2.source.valid,
            self.sata_phy2.source.data,
            self.sata_phy2.source.charisk,

            self.sata_phy2.sink.valid,
            self.sata_phy2.sink.data,
            self.sata_phy2.sink.charisk,

            self.sata_core3_link_tx_fsm_state,
            self.sata_core3_link_rx_fsm_state,

            self.sata_core3.sink.valid,
            self.sata_core3.source.valid,

            self.sata_phy3.source.valid,
            self.sata_phy3.source.data,
            self.sata_phy3.source.charisk,

            self.sata_phy3.sink.valid,
            self.sata_phy3.sink.data,
            self.sata_phy3.sink.charisk
        ]

        self.submodules.analyzer = LiteScopeAnalyzer(debug, 2048)

    def do_finalize(self):
        StripingSoC.do_finalize(self)
        self.comb += [
            self.sata_core0_link_rx_fsm_state.eq(self.sata_core0.link.rx.fsm.state),
            self.sata_core0_link_tx_fsm_state.eq(self.sata_core0.link.tx.fsm.state),
            self.sata_core1_link_rx_fsm_state.eq(self.sata_core1.link.rx.fsm.state),
            self.sata_core1_link_tx_fsm_state.eq(self.sata_core1.link.tx.fsm.state),
            self.sata_core2_link_rx_fsm_state.eq(self.sata_core2.link.rx.fsm.state),
            self.sata_core2_link_tx_fsm_state.eq(self.sata_core2.link.tx.fsm.state),
            self.sata_core3_link_rx_fsm_state.eq(self.sata_core3.link.rx.fsm.state),
            self.sata_core3_link_tx_fsm_state.eq(self.sata_core3.link.tx.fsm.state)
        ]

    def do_exit(self, vns):
        self.analyzer.export_csv(vns, "test/analyzer.csv")


default_subtarget = StripingSoC