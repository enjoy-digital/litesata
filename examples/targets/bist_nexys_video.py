# This file is Copyright (c) 2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

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

        clk_se = platform.request("clk100")

        cpu_reset = ~platform.request("cpu_reset") # FIXME

        pll_locked = Signal()
        pll_fb = Signal()
        pll_sys = Signal()
        self.specials += [
            Instance("PLLE2_BASE",
                p_STARTUP_WAIT="FALSE", o_LOCKED=pll_locked,

                # VCO @ 1GHz
                p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=10.0,
                p_CLKFBOUT_MULT=10, p_DIVCLK_DIVIDE=1,
                i_CLKIN1=clk_se, i_CLKFBIN=pll_fb, o_CLKFBOUT=pll_fb,

                # 100MHz
                p_CLKOUT0_DIVIDE=10, p_CLKOUT0_PHASE=0.0, o_CLKOUT0=pll_sys
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


class BISTSoC(SoCCore):
    default_platform = "kc705"
    csr_map = {
        "sata_bist": 16
    }
    csr_map.update(SoCCore.csr_map)
    def __init__(self, platform, revision="sata_gen2", data_width=16):
        sys_clk_freq = int(100e6)
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type=None,
            csr_data_width=32,
            with_uart=False,
            ident="LiteSATA example design", ident_version=True,
            with_timer=False)
        self.submodules.bridge = UARTWishboneBridge(platform.request("serial"), clk_freq, baudrate=115200)
        self.add_wb_master(self.bridge.wishbone)
        self.submodules.crg = CRG(platform)

        # SATA PHY/Core/Frontend
        self.submodules.sata_phy = LiteSATAPHY(platform.device,
                                               platform.request("sata_clocks"),
                                               platform.request("sata", 0),
                                               revision,
                                               sys_clk_freq,
                                               data_width)
        self.submodules.sata_core = LiteSATACore(self.sata_phy)
        self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_core)
        self.submodules.sata_bist = LiteSATABIST(self.sata_crossbar, with_csr=True)

        # Status Leds
        self.submodules.leds = StatusLeds(platform, self.sata_phy)

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

class BISTSoCDevel(BISTSoC):
    csr_map = {
        "analyzer": 17
    }
    csr_map.update(BISTSoC.csr_map)
    def __init__(self, platform):
        from litescope import LiteScopeAnalyzer
        BISTSoC.__init__(self, platform)

        # analyzer signals
        analyzer_signals = [
            self.sata_phy.crg.tx_startup_fsm,
            self.sata_phy.crg.rx_startup_fsm,
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

        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 256, csr_csv="test/analyzer.csv")

default_subtarget = BISTSoCDevel
