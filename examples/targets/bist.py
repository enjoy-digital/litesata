from migen.genlib.cdc import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCCore
from litex.soc.cores.uart import UARTWishboneBridge

from litesata.common import *
from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABIST


class CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()

        clk200 = platform.request("clk200")
        clk200_se = Signal()
        self.specials += Instance("IBUFDS", i_I=clk200.p, i_IB=clk200.n, o_O=clk200_se)

        try:
            cpu_reset = platform.request("cpu_reset")
        except:
            cpu_reset = ~platform.request("cpu_reset_n")

        pll_locked = Signal()
        pll_fb = Signal()
        pll_sys = Signal()
        self.specials += [
            Instance("PLLE2_BASE",
                p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

                # VCO @ 1GHz
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=5.0,
                p_CLKFBOUT_MULT=5, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=clk200_se, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

                # 200MHz
                p_CLKOUT0_DIVIDE=5, p_CLKOUT0_PHASE=0.0, o_CLKOUT0=pll_sys,

                p_CLKOUT1_DIVIDE=2, p_CLKOUT1_PHASE=0.0, #o_CLKOUT1=,

                p_CLKOUT2_DIVIDE=2, p_CLKOUT2_PHASE=0.0, #o_CLKOUT2=,

                p_CLKOUT3_DIVIDE=2, p_CLKOUT3_PHASE=0.0, #o_CLKOUT3=,

                p_CLKOUT4_DIVIDE=2, p_CLKOUT4_PHASE=0.0, #o_CLKOUT4=
            ),
            Instance("BUFG", i_I=pll_sys, o_O=self.cd_sys.clk),
            AsyncResetSynchronizer(self.cd_sys, ~pll_locked | cpu_reset),
        ]


class StatusLeds(Module):
    def __init__(self, platform, sata_phys):
        if not isinstance(sata_phys, list):
            sata_phys = [sata_phys]
            use_cd_num = False
        else:
            use_cd_num = True
        for i, sata_phy in enumerate(sata_phys):
            # 1Hz blinking leds (sata_rx and sata_tx clocks)
            rx_led = platform.request("user_led", 2*i)

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

            # ready leds
            self.comb += platform.request("user_led", 2*i+1).eq(sata_phy.ctrl.ready)


class BISTSoC(SoCCore):
    default_platform = "kc705"
    csr_map = {
        "sata_bist": 16
    }
    csr_map.update(SoCCore.csr_map)
    def __init__(self, platform, revision="sata_gen3", data_width=16):
        clk_freq = int(200e6)
        SoCCore.__init__(self, platform, clk_freq,
            cpu_type=None,
            csr_data_width=32,
            with_uart=False,
            ident="LiteSATA example design", ident_version=True,
            with_timer=False)
        self.add_cpu(UARTWishboneBridge(platform.request("serial"), clk_freq, baudrate=115200))
        self.add_wb_master(self.cpu.wishbone)
        self.submodules.crg = CRG(platform)

        # SATA PHY/Core/Frontend
        self.submodules.sata_phy = LiteSATAPHY(platform.device,
                                               platform.request("sata_clocks"),
                                               platform.request("sata", 0),
                                               revision,
                                               clk_freq,
                                               data_width)
        self.submodules.sata_core = LiteSATACore(self.sata_phy)
        self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_core)
        self.submodules.sata_bist = LiteSATABIST(self.sata_crossbar, with_csr=True)

        # Status Leds
        self.submodules.leds = StatusLeds(platform, self.sata_phy)

        self.sata_phy.crg.cd_sata_rx.clk.attr.add("keep")
        self.sata_phy.crg.cd_sata_tx.clk.attr.add("keep")
        platform.add_platform_command("""
create_clock -name sys_clk -period 5 [get_nets sys_clk]

create_clock -name sata_rx_clk -period {sata_clk_period} [get_nets sata_rx_clk]
create_clock -name sata_tx_clk -period {sata_clk_period} [get_nets sata_tx_clk]

set_false_path -from [get_clocks sys_clk] -to [get_clocks sata_rx_clk]
set_false_path -from [get_clocks sys_clk] -to [get_clocks sata_tx_clk]
set_false_path -from [get_clocks sata_rx_clk] -to [get_clocks sys_clk]
set_false_path -from [get_clocks sata_tx_clk] -to [get_clocks sys_clk]
""".format(sata_clk_period="3.3" if data_width == 16 else "6.6"))

class BISTSoCDevel(BISTSoC):
    csr_map = {
        "analyzer": 17
    }
    csr_map.update(BISTSoC.csr_map)
    def __init__(self, platform):
        from litescope import LiteScopeAnalyzer
        BISTSoC.__init__(self, platform)

        self.sata_core_link_rx_fsm_state = Signal(4)
        self.sata_core_link_tx_fsm_state = Signal(4)
        self.sata_core_transport_rx_fsm_state = Signal(4)
        self.sata_core_transport_tx_fsm_state = Signal(4)
        self.sata_core_command_rx_fsm_state = Signal(4)
        self.sata_core_command_tx_fsm_state = Signal(4)

        debug = [
            self.sata_phy.ctrl.ready,

            self.sata_phy.source.valid,
            self.sata_phy.source.data,
            self.sata_phy.source.charisk,

            self.sata_phy.sink.valid,
            self.sata_phy.sink.data,
            self.sata_phy.sink.charisk,

            self.sata_core.command.sink.valid,
            self.sata_core.command.sink.last,
            self.sata_core.command.sink.ready,
            self.sata_core.command.sink.write,
            self.sata_core.command.sink.read,
            self.sata_core.command.sink.identify,

            self.sata_core.command.source.valid,
            self.sata_core.command.source.last,
            self.sata_core.command.source.ready,
            self.sata_core.command.source.write,
            self.sata_core.command.source.read,
            self.sata_core.command.source.identify,
            self.sata_core.command.source.failed,
            self.sata_core.command.source.data,

            self.sata_core_link_rx_fsm_state,
            self.sata_core_link_tx_fsm_state,
            self.sata_core_transport_rx_fsm_state,
            self.sata_core_transport_tx_fsm_state,
            self.sata_core_command_rx_fsm_state,
            self.sata_core_command_tx_fsm_state,
        ]

        self.submodules.analyzer = LiteScopeAnalyzer(debug, 2048)

    def do_finalize(self):
        BISTSoC.do_finalize(self)
        self.comb += [
            self.sata_core_link_rx_fsm_state.eq(self.sata_core.link.rx.fsm.state),
            self.sata_core_link_tx_fsm_state.eq(self.sata_core.link.tx.fsm.state),
            self.sata_core_transport_rx_fsm_state.eq(self.sata_core.transport.rx.fsm.state),
            self.sata_core_transport_tx_fsm_state.eq(self.sata_core.transport.tx.fsm.state),
            self.sata_core_command_rx_fsm_state.eq(self.sata_core.command.rx.fsm.state),
            self.sata_core_command_tx_fsm_state.eq(self.sata_core.command.tx.fsm.state)
        ]

    def do_exit(self, vns):
        self.analyzer.export_csv(vns, "test/analyzer.csv")

default_subtarget = BISTSoC
