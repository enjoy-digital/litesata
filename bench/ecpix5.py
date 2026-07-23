#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# LiteSATA bench on LambdaConcept ECPIX-5 (ECP5-5G), SATA connector on DCU1/CH0.
#
# Build/Use:
# ./ecpix5.py --gen 1 --with-analyzer --build --load
# litex_server --uart --uart-port=/dev/ttyUSB2 --uart-baudrate=1000000
# ./test_init.py
# litescope_cli (see --help)

import os
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *

from litex.build.generic_platform import *

from litex_boards.platforms import lambdaconcept_ecpix5

from litex.soc.cores.clock import *
from litex.soc.integration.soc import *
from litex.soc.integration.builder import *

from litesata.common               import *
from litesata.phy                  import LiteSATAPHY
from litesata.core                 import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist        import LiteSATABIST

from litescope import LiteScopeAnalyzer

# IOs ----------------------------------------------------------------------------------------------

_sata_io = [
    # SATA connector, wired to DCU1/CH0 (bare differential pads, no IOStandard).
    ("sata_tx", 0,
        Subsignal("p", Pins("AD16")),
        Subsignal("n", Pins("AD17")),
    ),
    ("sata_rx", 0,
        Subsignal("p", Pins("AF15")),
        Subsignal("n", Pins("AF16")),
    ),
]

class SATAPads:
    def __init__(self, tx, rx):
        self.tx_p = tx.p
        self.tx_n = tx.n
        self.rx_p = rx.p
        self.rx_n = rx.n

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, refclk_freq=150e6):
        self.cd_sys         = ClockDomain()
        self.cd_por         = ClockDomain(reset_less=True)
        self.cd_sata_refclk = ClockDomain(reset_less=True)

        # # #

        # Clk / Rst.
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        platform.add_period_constraint(clk100, 1e9/100e6)

        # Power on reset.
        por_count = Signal(16, reset=2**16-1)
        por_done  = Signal()
        self.comb += self.cd_por.clk.eq(ClockSignal())
        self.comb += por_done.eq(por_count == 0)
        self.sync.por += If(~por_done, por_count.eq(por_count - 1))

        # PLL: sys clk + 150MHz SATA SerDes refclk (the onboard 100MHz EXTREF cannot synthesize
        # the 1.5/3.0Gbps SATA linerates with the DCU PLL multipliers).
        self.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, with_reset=False)
        pll.create_clkout(self.cd_sata_refclk, refclk_freq)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~por_done | ~pll.locked | ~rst_n)

# SATATestSoC --------------------------------------------------------------------------------------

class SATATestSoC(SoCMini):
    def __init__(self, platform, sys_clk_freq=int(100e6), gen="gen1",
        with_bist       = False,
        with_analyzer   = False,
        analyzer_domain = "sys",
    ):
        assert gen in ["gen1", "gen2"]
        assert analyzer_domain in ["sys", "tx", "rx"]
        sata_clk_freq = {"gen1": 75e6, "gen2": 150e6}[gen]

        # CRG --------------------------------------------------------------------------------------
        self.crg = _CRG(platform, sys_clk_freq)

        # SoCMini ----------------------------------------------------------------------------------
        SoCMini.__init__(self, platform, sys_clk_freq, ident="LiteSATA bench on ECPIX-5.")

        # UARTBone ---------------------------------------------------------------------------------
        self.add_uartbone(baudrate=1e6)

        # SATA -------------------------------------------------------------------------------------
        # PHY
        self.sata_phy = LiteSATAPHY(platform.device,
            refclk     = self.crg.cd_sata_refclk.clk,
            pads       = SATAPads(platform.request("sata_tx"), platform.request("sata_rx")),
            gen        = gen,
            clk_freq   = sys_clk_freq,
            data_width = 16,
            dual       = 1,
            channel    = 0,
        )

        # Core / Crossbar / BIST
        if with_bist:
            self.sata_core     = LiteSATACore(self.sata_phy)
            self.sata_crossbar = LiteSATACrossbar(self.sata_core)
            self.sata_bist     = LiteSATABIST(self.sata_crossbar, with_csr=True)

        # Timing constraints
        platform.add_period_constraint(self.sata_phy.crg.cd_sata_tx.clk, 1e9/sata_clk_freq)
        platform.add_period_constraint(self.sata_phy.crg.cd_sata_rx.clk, 1e9/sata_clk_freq)
        platform.add_false_path_constraints(
            self.crg.cd_sys.clk,
            self.sata_phy.crg.cd_sata_tx.clk,
            self.sata_phy.crg.cd_sata_rx.clk)

        # Leds -------------------------------------------------------------------------------------
        # sys_clk
        sys_counter = Signal(32)
        self.sync.sys += sys_counter.eq(sys_counter + 1)
        self.comb += platform.request("rgb_led", 0).g.eq(~sys_counter[26])
        # tx_clk
        tx_counter = Signal(32)
        self.sync.sata_tx += tx_counter.eq(tx_counter + 1)
        self.comb += platform.request("rgb_led", 1).g.eq(~tx_counter[26])
        # rx_clk
        rx_counter = Signal(32)
        self.sync.sata_rx += rx_counter.eq(rx_counter + 1)
        self.comb += platform.request("rgb_led", 2).g.eq(~rx_counter[26])
        # ready
        self.comb += platform.request("rgb_led", 3).g.eq(~self.sata_phy.ctrl.ready)

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            phy    = self.sata_phy.phy
            serdes = phy.serdes
            if analyzer_domain == "sys":
                analyzer_signals = {
                    # Group 0: OOB/ctrl bring-up.
                    0: [
                        self.sata_phy.ctrl.fsm,
                        self.sata_phy.ctrl.ready,
                        serdes.init.fsm,
                        serdes.init.tx_lol,
                        serdes.init.rx_lol,
                        serdes.init.rx_los,
                        phy.tx_cominit_stb,
                        phy.tx_cominit_ack,
                        phy.tx_comwake_stb,
                        phy.tx_comwake_ack,
                        phy.rx_cominit_stb,
                        phy.rx_comwake_stb,
                        phy.rx_idle,
                        phy.ldr_idle,
                        phy.com_check.gap_count,
                        phy.com_check.cominit_gaps,
                        phy.com_check.comwake_gaps,
                        phy.tx_polarity,
                        phy.rx_polarity,
                        self.sata_phy.ctrl.rx_idle,
                        self.sata_phy.ctrl.misalign,
                        phy.rxnotintable,
                    ],
                    # Group 1: post-OOB datapath (32-bit, sys domain).
                    1: [
                        self.sata_phy.source,
                        self.sata_phy.sink,
                    ],
                }
            if analyzer_domain == "tx":
                analyzer_signals = {
                    0: [
                        phy.com_gen.fsm,
                        phy.com_gen.tx_oob_en,
                        phy.com_gen.tx_oob_data,
                        phy.com_gen.tx_idle,
                        phy.com_gen.active,
                        serdes.sink.data,
                        serdes.sink.ctrl,
                    ],
                }
            if analyzer_domain == "rx":
                analyzer_signals = {
                    0: [
                        serdes.source.data,
                        serdes.source.ctrl,
                        Cat(*[serdes.decoders[i].invalid for i in range(2)]),
                        phy.source,
                    ],
                }
            self.analyzer = LiteScopeAnalyzer(analyzer_signals,
                depth        = 1024,
                clock_domain = analyzer_domain,
                samplerate   = {"sys": sys_clk_freq, "tx": sata_clk_freq, "rx": sata_clk_freq}[analyzer_domain],
                csr_csv      = "analyzer.csv",
            )

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA bench on ECPIX-5.")
    parser.add_argument("--build",           action="store_true", help="Build bitstream.")
    parser.add_argument("--load",            action="store_true", help="Load bitstream (to SRAM).")
    parser.add_argument("--toolchain",       default="trellis",   help="FPGA toolchain: trellis (default) or diamond.")
    parser.add_argument("--device",          default="85F",       help="FPGA device (85F or 45F).")
    parser.add_argument("--sys-clk-freq",    default=100e6, type=float, help="System clock frequency (default: 100MHz).")
    parser.add_argument("--gen",             default="1", choices=["1", "2"], help="SATA generation (default: 1).")
    parser.add_argument("--with-bist",       action="store_true", help="Add SATA Core/Crossbar/BIST.")
    parser.add_argument("--with-analyzer",   action="store_true", help="Add LiteScope Analyzer.")
    parser.add_argument("--analyzer-domain", default="sys", choices=["sys", "tx", "rx"],
        help="LiteScope Analyzer clock domain/probe set (default: sys).")
    args = parser.parse_args()

    platform = lambdaconcept_ecpix5.Platform(device=args.device, toolchain=args.toolchain)
    platform.add_extension(_sata_io)
    soc = SATATestSoC(platform,
        sys_clk_freq    = int(args.sys_clk_freq),
        gen             = "gen" + args.gen,
        with_bist       = args.with_bist,
        with_analyzer   = args.with_analyzer,
        analyzer_domain = args.analyzer_domain,
    )
    builder = Builder(soc, csr_csv="csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
