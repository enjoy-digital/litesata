#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# LiteSATA bench on LambdaConcept ECPIX-5 (ECP5-5G), SATA connector on DCU1/CH0.
#
# Build/Use:
# ./ecpix5.py --with-bist --with-analyzer --build --load
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

# The original LiteICLink values are retained by default. These diagnostic
# profiles make the transmitter-side CMU differences found in Lattice-generated
# low-rate examples and LUNA's 5Gbps ECPIX-5 PHY independently reproducible.
_dcu_cmu_profiles = {
    "legacy": {},
    "clarity-low-rate": {
        "p_D_SETIRPOLY_AUX": "0b10",
        "p_D_SETIRPOLY_CH":  "0b10",
    },
    "luna-5g": {
        "p_D_CMUSETI4CPP":   "0d4",
        "p_D_CMUSETZGM":     "0b100",
        "p_D_SETIRPOLY_AUX": "0b10",
        "p_D_SETIRPOLY_CH":  "0b10",
    },
}

class SATAPads:
    def __init__(self, tx, rx):
        self.tx_p = tx.p
        self.tx_n = tx.n
        self.rx_p = rx.p
        self.rx_n = rx.n

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq, refclk_freq=150e6,
        split_sata_refclk_pll=False):
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
        # the 1.5/3.0Gbps SATA linerates with the DCU PLL multipliers). The optional split is a
        # bench-only reference-quality A/B: it moves the SATA PLL VCO from 450MHz to 750MHz and
        # removes the sys-clock output load while preserving the exact 150MHz DCU reference.
        self.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, with_reset=False)
        if split_sata_refclk_pll:
            self.sata_refclk_pll = sata_refclk_pll = ECP5PLL()
            sata_refclk_pll.register_clkin(clk100, 100e6)
            sata_refclk_pll.create_clkout(self.cd_sata_refclk, refclk_freq)
            plls_locked = pll.locked & sata_refclk_pll.locked
        else:
            pll.create_clkout(self.cd_sata_refclk, refclk_freq)
            plls_locked = pll.locked
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~por_done | ~plls_locked | ~rst_n)

# SATATestSoC --------------------------------------------------------------------------------------

class SATATestSoC(SoCMini):
    def __init__(self, platform, sys_clk_freq=int(90e6),
        with_bist       = False,
        with_analyzer   = False,
        analyzer_domain = "sys",
        dcu_cmu_profile = "legacy",
        split_sata_refclk_pll = False,
    ):
        assert analyzer_domain in ["sys", "tx", "rx"]
        assert dcu_cmu_profile in _dcu_cmu_profiles
        gen = "gen2"
        sata_clk_freq = 150e6
        # The 16->32 RX StrideConverter requires sys_clk > sata_rx_clk/2 (see acorn.py).
        min_sys_clk_freq = sata_clk_freq*16/32
        assert sys_clk_freq >= min_sys_clk_freq, \
            f"sys_clk_freq must be >= {min_sys_clk_freq/1e6:.1f}MHz for {gen}."

        # CRG --------------------------------------------------------------------------------------
        # SATA SerDes refclk = linerate/20 (x20 DCU PLL multiplier, see ecp5sataphy.py).
        self.crg = _CRG(platform, sys_clk_freq,
            refclk_freq           = sata_clk_freq,
            split_sata_refclk_pll = split_sata_refclk_pll,
        )

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
        # Apply before SerDes finalization/Instance creation. The low-rate profile changes only
        # the two regulator-current selections used by Lattice's 2.5Gbps Clarity output. The
        # LUNA profile adds LUNA's two 5Gbps CMU loop values. RX DCO parameters are deliberately
        # untouched: the drive's Gen2 ALIGN stream is already received and decoded cleanly.
        self.sata_phy.phy.serdes.serdes_params.update(_dcu_cmu_profiles[dcu_cmu_profile])
        # ECPIX-5 bring-up controls/counters are intentionally bench-only; the
        # production ECP5 PHY comes up with the evidence-backed Gen2 defaults.
        self.sata_phy.phy.add_oob_csr()

        # SerDes TX/RX word clock measurement (debug).
        self.sata_phy.phy.serdes.add_clock_cycles()

        # Core / Crossbar / BIST.
        # The core is ALWAYS instantiated (as on every other bench): LiteSATAPHYDatapath hands the
        # transmitter from ctrl to the core the instant ctrl.ready asserts, and the PHY copies
        # sink.data ignoring sink.valid - so a dangling sink transmits D0.0 for ever, the device
        # stops answering and the link tears down ~41us later. with_bist only gates the BIST CSRs.
        self.sata_core     = LiteSATACore(self.sata_phy)
        self.sata_crossbar = LiteSATACrossbar(self.sata_core)
        self.sata_bist     = LiteSATABIST(
            self.sata_crossbar,
            with_csr         = with_bist,
            # ATA requires SRST asserted for at least 5us. Use 6us so the
            # interval remains comfortably above the minimum after rounding.
            soft_reset_cycles = int(sys_clk_freq*6e-6),
        )

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

        # Long-activity trigger: RX line continuously active >5us (far longer than any beacon
        # burst) - isolates the rare long-carrier events for litescope content inspection.
        long_act_cnt = Signal(10)
        self.long_activity = Signal()
        self.sync += [
            If(self.sata_phy.phy.rx_idle,
                long_act_cnt.eq(0)
            ).Elif(~self.long_activity,
                long_act_cnt.eq(long_act_cnt + 1)
            ),
            self.long_activity.eq(long_act_cnt == 500),
        ]

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            phy    = self.sata_phy.phy
            serdes = phy.serdes
            identify = (self.sata_bist.identify.bist_identify
                if with_bist else self.sata_bist.identify)

            # Stable names for the unscrambled link input. Sampling valid & ready here records
            # exactly the FIS dwords accepted by the CRC/scrambler pipeline, independently of
            # any later wire-level decoding.
            self.link_tx_payload_valid = Signal()
            self.link_tx_payload_ready = Signal()
            self.link_tx_payload_last  = Signal()
            self.link_tx_payload_data  = Signal(32)
            self.comb += [
                self.link_tx_payload_valid.eq(self.sata_core.link.sink.valid),
                self.link_tx_payload_ready.eq(self.sata_core.link.sink.ready),
                self.link_tx_payload_last.eq(self.sata_core.link.sink.last),
                self.link_tx_payload_data.eq(self.sata_core.link.sink.data),
            ]

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
                        serdes.init.tx_pll_rst,
                        serdes.init.tx_pcs_rst,
                        serdes.init.rx_cdr_rst,
                        serdes.init.rx_pcs_rst,
                        phy.tx_cominit_stb,
                        phy.tx_cominit_ack,
                        phy.tx_comwake_stb,
                        phy.tx_comwake_ack,
                        phy.txcomfinish,
                        phy.rx_cominit_stb,
                        phy.rx_comwake_stb,
                        phy.rx_idle,
                        phy.ldr_idle,
                        phy.com_check.gap_count,
                        phy.com_check.cominit_gaps,
                        phy.com_check.comwake_gaps,
                        phy.tx_polarity,
                        phy.rx_polarity,
                        phy.oob_active_sys_dbg,
                        phy.oob_is_wake_sys_dbg,
                        phy.oob_ei_req_sys_dbg,
                        phy.oob_gap_sys_dbg,
                        phy.oob_d102_sys_dbg,
                        self.sata_phy.ctrl.d102_phase,
                        phy.tx_idle_tx_sys_dbg,
                        phy.tx_ei_en_sys_dbg,
                        self.long_activity,
                        *([serdes.rx_word_data, serdes.rx_word_ctrl]
                          if hasattr(serdes, "rx_word_data") else []),
                        # Raw DCU RX bus: in bypass PCS there is no rx_word_data, so without this
                        # the only RX view is the already-decoded datapath source. Needed to tell a
                        # mis-decode (bus non-zero) from a non-sampling deserializer (bus zero).
                        *([serdes.rx_bus_dbg] if hasattr(serdes, "rx_bus_dbg") else []),
                        *([serdes.bp_src_dbg, serdes.bp_dec_d, serdes.bp_dec_k, serdes.bp_dec_inv,
                           serdes.bp_slip_dbg] if hasattr(serdes, "bp_src_dbg") else []),
                        *([serdes.rx_lol_dbg] if hasattr(serdes, "rx_lol_dbg") else []),
                        self.sata_phy.datapath.rx.source.valid,
                        self.sata_phy.datapath.rx.source.data,
                        self.sata_phy.datapath.rx.source.charisk,
                        # The ACTUAL transceiver TX input (mux.source -> tx -> trx.sink). The
                        # datapath.sink probes in groups 1/2 are the CORE-facing stream, which is
                        # STALLED while ctrl owns the TX (ctrl.ready=0) - campaign 37 mistook that
                        # frozen ALIGNInserter output for the wire content.
                        phy.sink.valid,
                        phy.sink.data,
                        phy.sink.charisk,
                        *([serdes.tx_data_dbg, serdes.tx_bus_dbg]
                          if hasattr(serdes, "tx_data_dbg") else []),
                        self.sata_phy.ctrl.rx_idle,
                        self.sata_phy.ctrl.misalign,
                        phy.rxnotintable,
                    ],
                    # Group 1: post-OOB datapath (32-bit, sys domain).
                    1: [
                        self.sata_phy.source,
                        self.sata_phy.sink,
                    ],
                    # Group 2: link layer - what the core actually transmits once ctrl.ready hands
                    # the transmitter over. Healthy idle = SYNC (0xb5b5957c) x254 then ALIGN
                    # (0x7b4a4abc) x2, charisk 0b0001.
                    2: [
                        self.sata_core.link.tx.fsm,
                        self.sata_core.link.rx.fsm,
                        self.sata_core.link.tx.from_rx.idle,
                        self.sata_core.link.tx.from_rx.insert,
                        self.sata_core.link.tx.from_rx.primitive_valid,
                        self.sata_core.link.tx.from_rx.primitive,
                        self.sata_core.link.tx.error,
                        self.link_tx_payload_valid,
                        self.link_tx_payload_ready,
                        self.link_tx_payload_last,
                        self.link_tx_payload_data,
                        self.sata_core.link.tx_align.source.valid,
                        self.sata_core.link.tx_align.source.data,
                        self.sata_core.link.tx_align.source.charisk,
                        self.sata_phy.sink.valid,
                        self.sata_phy.sink.data,
                        self.sata_phy.sink.charisk,
                        self.sata_phy.ctrl.ready,
                        self.sata_phy.ctrl.rx_idle,
                    ],
                    # Group 3: command-path trace (identify pulse stage-by-stage).
                    3: [
                        identify.fsm,
                        self.sata_core.command.tx.fsm,
                        self.sata_core.command.rx.fsm,
                        self.sata_core.transport.tx.fsm,
                        self.sata_core.transport.rx.fsm,
                        self.sata_core.command.tx.sink.valid,
                        self.sata_core.command.tx.sink.ready,
                        self.sata_core.command.tx.sink.identify,
                        self.sata_core.link.sink.valid,
                        self.sata_core.link.sink.ready,
                        self.sata_core.link.source.valid,
                        self.sata_core.link.source.ready,
                        self.sata_phy.ctrl.ready,
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
                        phy.com_gen.is_wake,
                        phy.com_gen.ei_req,
                        phy.oob_d102_active,
                        serdes.tx_oob_active,
                        serdes.tx_oob_ei_req,
                        serdes.tx_oob_gap,
                        serdes.tx_oob_deemph,
                        serdes.tx_idle_tx_dbg,
                        serdes.tx_ei_en_dbg,
                        serdes.tx_produce_pattern,
                        serdes.sink.data,
                        serdes.sink.ctrl,
                        *([serdes.tx_data_dbg, serdes.tx_bus_dbg]
                          if hasattr(serdes, "tx_data_dbg") else []),
                    ],
                }
            if analyzer_domain == "rx":
                analyzer_signals = {
                    0: [
                        serdes.source.data,
                        serdes.source.ctrl,
                        serdes.decoders[0].invalid,
                        serdes.decoders[1].invalid,
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
    parser.add_argument("--seed",            default=3, type=int, help="nextpnr placement seed (default: 3).")
    parser.add_argument("--sys-clk-freq",    default=90e6, type=float, help="System clock frequency (default: 90MHz).")
    parser.add_argument("--with-bist",       action="store_true", help="Add SATA Core/Crossbar/BIST.")
    parser.add_argument("--with-analyzer",   action="store_true", help="Add LiteScope Analyzer.")
    parser.add_argument("--analyzer-domain", default="sys", choices=["sys", "tx", "rx"],
        help="LiteScope Analyzer clock domain/probe set (default: sys).")
    parser.add_argument("--dcu-cmu-profile", default="legacy", choices=_dcu_cmu_profiles,
        help="Diagnostic DCU transmitter CMU profile (default: legacy).")
    parser.add_argument("--split-sata-refclk-pll", action="store_true",
        help="Generate the SATA reference with a dedicated EHXPLLL.")
    args = parser.parse_args()

    platform = lambdaconcept_ecpix5.Platform(device=args.device, toolchain=args.toolchain)
    platform.add_extension(_sata_io)
    soc = SATATestSoC(platform,
        sys_clk_freq    = int(args.sys_clk_freq),
        with_bist       = args.with_bist,
        with_analyzer   = args.with_analyzer,
        analyzer_domain = args.analyzer_domain,
        dcu_cmu_profile = args.dcu_cmu_profile,
        split_sata_refclk_pll = args.split_sata_refclk_pll,
    )
    builder = Builder(soc, csr_csv="csr.csv")
    build_kwargs = {"seed": args.seed} if args.toolchain == "trellis" else {}
    builder.build(run=args.build, **build_kwargs)

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

if __name__ == "__main__":
    main()
