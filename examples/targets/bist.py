# This file is Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from migen.genlib.cdc import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.cores.uart import UARTWishboneBridge

from litesata.common import *
from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABIST

# CRG ----------------------------------------------------------------------------------------------

class CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()

        clk200 = platform.request("clk200")
        try:
            cpu_reset = platform.request("cpu_reset")
        except:
            cpu_reset = ~platform.request("cpu_reset_n")

        self.submodules.pll = pll = S7PLL(speedgrade=-1)
        self.comb += pll.reset.eq(cpu_reset)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, 200e6)

# StatusLeds ---------------------------------------------------------------------------------------

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

            freq = int(frequencies[sata_phy.revision]*1e6)

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

# BISTSoC ------------------------------------------------------------------------------------------

class BISTSoC(SoCMini):
    default_platform = "kc705"
    def __init__(self, platform, revision="sata_gen3", data_width=16):
        sys_clk_freq = int(200e6)
        SoCMini.__init__(self, platform, sys_clk_freq,
            csr_data_width = 32,
            ident          = "LiteSATA example design",
            ident_version  = True)

        # Serial Bridge ----------------------------------------------------------------------------
        self.submodules.bridge = UARTWishboneBridge(platform.request("serial"), sys_clk_freq, baudrate=115200)
        self.add_wb_master(self.bridge.wishbone)
        self.submodules.crg = CRG(platform)

        # SATA PHY/Core/Frontend -------------------------------------------------------------------
        self.submodules.sata_phy = LiteSATAPHY(platform.device,
            platform.request("sata_clocks"),
            platform.request("sata", 0),
            revision,
            sys_clk_freq,
            data_width)
        self.submodules.sata_core     = LiteSATACore(self.sata_phy)
        self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_core)
        self.submodules.sata_bist     = LiteSATABIST(self.sata_crossbar, with_csr=True)
        self.add_csr("sata_bist")

        # Status Leds ------------------------------------------------------------------------------
        self.submodules.leds = StatusLeds(platform, self.sata_phy)

        # Timing constraints -----------------------------------------------------------------------
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

# BISTSoCDevel -------------------------------------------------------------------------------------

class BISTSoCDevel(BISTSoC):
    def __init__(self, platform):
        from litescope import LiteScopeAnalyzer
        BISTSoC.__init__(self, platform)
        analyzer_signals = [
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
        self.submodules.analyzer = LiteScopeAnalyzer(analyzer_signals, 2048, csr_csv="test/analyzer.csv")
        self.add_csr("analyzer")

default_subtarget = BISTSoC
