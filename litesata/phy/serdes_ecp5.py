#
# This file is part of LiteICLink.
#
# Copyright (c) 2019-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.misc import WaitTimer
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream
from litex.soc.cores.prbs import PRBSTX, PRBSRX
from litex.soc.cores.code_8b10b import Encoder, Decoder

# SerDesECP5PLL ------------------------------------------------------------------------------------

class SerDesECP5PLL(Module):
    def __init__(self, refclk, refclk_freq, linerate):
        self.refclk = refclk
        self.config = self.compute_config(refclk_freq, linerate)

    @staticmethod
    def compute_config(refclk_freq, linerate):
        multipliers = [8, 10, 16, 20]
        if refclk_freq == 100e6:
            multipliers.append(25) # x25 for 100MHz refclk only.
        dividers = [1, 2, 4, 8, 16, 32]
        for d in reversed(dividers): # Use highest possible VCO.
            for m in multipliers:
                vco_freq = refclk_freq*m
                current_linerate = vco_freq/d
                if current_linerate == linerate:
                    return {
                        "clkin": refclk_freq,
                        "m":               m,
                        "d":               d,
                        "vco_freq": vco_freq,
                        "linerate": linerate,
                    }
        msg = "No config found for {:3.4f} MHz refclk / {:3.4f} Gbps linerate.\n"
        msg += "Possible refclk frequencies:\n"
        for m in multipliers:
            for d in dividers:
                f = linerate*d/m
                if f <= 250e6:
                    msg += " - {:3.4f}MHz\n".format(linerate*d/m*1e-6)
        msg = msg[:-1]
        raise ValueError(msg.format(refclk_freq/1e6, linerate/1e9))

    def __repr__(self):
        config = self.config
        r = """
SerDesECP5PLL
==============
  overview:
  ---------
       +---------------------------+
       | +-----+  +-----+  +-----+ |
       | |     |  |     |  |     | |
CLKIN +-->  M  +--> VCO +--> /D  +--> LINERATE
       | |     |  |     |  |     | |
       | +-----+  +-----+  +-----+ |
       +---------------------------+

  config:
  -------
    CLKIN    = {clkin}MHz
    VCO      = CLKIN x M = {clkin}MHz x {m} = {vco_freq}GHz
    LINERATE = VCO / D   = {vco_freq}GHz / {d}
             = {linerate}GHz
""".format(clkin    = config["clkin"]/1e6,
           m        = config["m"],
           d        = config["d"],
           vco_freq = config["vco_freq"]/1e9,
           linerate = config["linerate"]/1e9)
        return r

# SerDesSCI ----------------------------------------------------------------------------------------

class SerDesECP5SCI(Module):
    def __init__(self, serdes):
        self.dual_sel = Signal()
        self.chan_sel = Signal()
        self.re       = Signal()
        self.we       = Signal()
        self.done     = Signal()
        self.adr      = Signal(6)
        self.dat_w    = Signal(8)
        self.dat_r    = Signal(8)

        # # #

        self.sci_rd    = sci_rd    = Signal()
        self.sci_wrn   = sci_wrn   = Signal(reset=1)
        self.sci_addr  = sci_addr  = Signal(6)
        self.sci_wdata = sci_wdata = Signal(8)
        self.sci_rdata = sci_rdata = Signal(8)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            If(self.we,
                NextState("WRITE")
            ).Elif(self.re,
                sci_rd.eq(1),
                NextState("READ")
            )
        )
        fsm.act("WRITE",
            sci_wrn.eq(0),
            NextState("IDLE")
        )
        fsm.act("READ",
            sci_rd.eq(1),
            NextValue(self.dat_r, sci_rdata),
            NextState("IDLE")
        )
        self.comb += [
            sci_addr.eq(self.adr),
            sci_wdata.eq(self.dat_w)
        ]

        serdes.serdes_params.update(
             **{"i_D_SCIWDATA%d" % n: sci_wdata[n] for n in range(8)},
             **{"i_D_SCIADDR%d"   % n: sci_addr[n]   for n in range(6)},
             **{"o_D_SCIRDATA%d" % n: sci_rdata[n] for n in range(8)},
             i_D_SCIENAUX  = self.dual_sel,
             i_D_SCISELAUX = self.dual_sel,
             i_CHX_SCIEN   = self.chan_sel,
             i_CHX_SCISEL  = self.chan_sel,
             i_D_SCIRD     = sci_rd,
             i_D_SCIWSTN   = sci_wrn,
        )

@ResetInserter()
class SerDesECP5SCIReconfig(Module):
    def __init__(self, serdes):
        self.loopback    = Signal()
        self.rx_polarity = Signal()
        self.tx_polarity = Signal()
        self.rx_cdr_hold = Signal()

        # # #

        sci = SerDesECP5SCI(serdes)
        self.submodules.sci = sci

        first = Signal()
        data  = Signal(8)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextState("READ-CH-01"),
        )
        fsm.act("READ-CH-01",
            sci.chan_sel.eq(1),
            sci.re.eq(1),
            sci.adr.eq(0x01),
            If(~first & sci.done,
                sci.re.eq(0),
                NextValue(data, sci.dat_r),
                NextState("WRITE-CH-01"),
            )
        )
        fsm.act("WRITE-CH-01",
            sci.chan_sel.eq(1),
            sci.we.eq(1),
            sci.adr.eq(0x01),
            sci.dat_w.eq(data),
            sci.dat_w[0].eq(self.rx_polarity),
            sci.dat_w[1].eq(self.tx_polarity),
            If(~first & sci.done,
                sci.we.eq(0),
                NextState("READ-CH-15")
            )
        )
        fsm.act("READ-CH-15",
            sci.chan_sel.eq(1),
            sci.re.eq(1),
            sci.adr.eq(0x15),
            If(~first & sci.done,
                sci.re.eq(0),
                NextValue(data, sci.dat_r),
                NextState("WRITE-CH-15"),
            )
        )
        fsm.act("WRITE-CH-15",
            sci.chan_sel.eq(1),
            sci.we.eq(1),
            sci.adr.eq(0x15),
            sci.dat_w.eq(data),
            If(self.loopback,
                sci.dat_w[0:4].eq(0b0001) # lb_ctl
            ),
            If(~first & sci.done,
                sci.we.eq(0),
                NextState("READ-CH-18")
            )
        )
        fsm.act("READ-CH-18",
            sci.chan_sel.eq(1),
            sci.re.eq(1),
            sci.adr.eq(0x18),
            If(~first & sci.done,
                sci.re.eq(0),
                NextValue(data, sci.dat_r),
                NextState("WRITE-CH-18"),
            )
        )
        fsm.act("WRITE-CH-18",
            sci.chan_sel.eq(1),
            sci.we.eq(1),
            sci.adr.eq(0x18),
            sci.dat_w.eq(data),
            sci.dat_w[6].eq(self.rx_cdr_hold),
            If(~first & sci.done,
                sci.we.eq(0),
                NextState("IDLE")
            )
        )
        fsm.finalize()

        last_fsm_state = Signal(4)
        self.sync += last_fsm_state.eq(fsm.state)
        self.comb += first.eq(fsm.state != last_fsm_state)

# SerdesInit ---------------------------------------------------------------------------------------

class SerdesInit(Module):
    def __init__(self, tx_lol, rx_lol, rx_los):
        self.tx_rst   = Signal()
        self.rx_rst   = Signal()
        self.pcs_rst  = Signal()
        self.tx_ready = Signal()
        self.rx_ready = Signal()

        # # #

        self.tx_lol = _tx_lol = Signal()
        self.rx_lol = _rx_lol = Signal()
        self.rx_los = _rx_los = Signal()
        self.specials += MultiReg(tx_lol, _tx_lol)
        self.specials += MultiReg(rx_lol, _rx_lol)

        timer = WaitTimer(1024)
        self.submodules += timer
        timer.wait.reset = 1

        self.submodules.fsm = fsm = FSM(reset_state="RESET-ALL")
        fsm.act("RESET-ALL",
            # Reset TX Serdes, RX Serdes and PCS.
            self.tx_rst.eq(1),
            self.rx_rst.eq(1),
            self.pcs_rst.eq(1),
            If(timer.done,
                timer.wait.eq(0),
                NextState("RESET-RX-PCS-WAIT-TX-PLL-LOCK")
            ),
            NextValue(self.tx_ready, 0),
            NextValue(self.rx_ready, 0),
        )
        fsm.act("RESET-RX-PCS-WAIT-TX-PLL-LOCK",
            # Reset RX Serdes and PCS, wait for TX PLL lock.
            self.rx_rst.eq(1),
            self.pcs_rst.eq(1),
            If(timer.done & ~_tx_lol,
                timer.wait.eq(0),
                NextValue(self.tx_ready, 1),
                NextState("RESET-PCS-WAIT-RX-CDR-LOCK")
            ),
            NextValue(self.rx_ready, 0),
        )
        fsm.act("RESET-PCS-WAIT-RX-CDR-LOCK",
            # Reset PCS, wait for RX CDR lock.
            self.pcs_rst.eq(1),
            If(timer.done & ~_rx_lol,
                timer.wait.eq(0),
                NextValue(self.rx_ready, 1),
                NextState("READY")
            )
        )
        fsm.act("READY",
            If(_tx_lol,
                NextState("RESET-ALL")
            ),
            If(_rx_lol,
                NextState("RESET-RX-PCS-WAIT-TX-PLL-LOCK")
            )
        )

# SerDesECP5 ---------------------------------------------------------------------------------------

class SerDesECP5(Module, AutoCSR):
    def __init__(self, pll, tx_pads, rx_pads,
        dual        = 0,
        channel     = 0,
        data_width  = 20,
        tx_polarity = 0,
        rx_polarity = 0):
        assert dual       in [0, 1]
        assert channel    in [0, 1]
        assert data_width in [20]
        self.dual    = dual
        self.channel = channel

        # TX controls
        self.tx_enable              = Signal(reset=1)
        self.tx_ready               = Signal()
        self.tx_inhibit             = Signal() # FIXME
        self.tx_produce_square_wave = Signal()
        self.tx_produce_pattern     = Signal()
        self.tx_pattern             = Signal(data_width)
        self.tx_prbs_config         = Signal(2)
        self.tx_idle                = Signal()

        # RX controls
        self.rx_enable              = Signal(reset=1)
        self.rx_ready               = Signal()
        self.rx_align               = Signal(reset=1)
        self.rx_prbs_config         = Signal(2)
        self.rx_prbs_pause          = Signal()
        self.rx_prbs_errors         = Signal(32)
        self.rx_idle                = Signal()
        self.rx_cdr_hold            = Signal()

        # Loopback
        self.loopback               = Signal() # FIXME: reconfigure lb_ctl to 0b0001 but does not seem enough

        # # #

        self.nwords = nwords = data_width//10

        self.submodules.encoder  = ClockDomainsRenamer("tx")(Encoder(nwords, True))
        self.submodules.decoders = [ClockDomainsRenamer("rx")(Decoder(True)) for _ in range(nwords)]

        # Transceiver direct clock outputs (useful to specify clock constraints)
        self.txoutclk = Signal()
        self.rxoutclk = Signal()

        self.tx_clk_freq = pll.config["linerate"]/data_width
        self.rx_clk_freq = pll.config["linerate"]/data_width

        # Internal signals -------------------------------------------------------------------------
        rx_los     = Signal()
        rx_lol     = Signal()
        rx_lsm     = Signal()
        rx_align   = Signal()
        rx_data    = Signal(20)
        rx_bus     = Signal(24)

        tx_lol     = Signal()
        tx_data    = Signal(20)
        tx_bus     = Signal(24)

        # Control/Status CDC -----------------------------------------------------------------------
        tx_produce_square_wave = Signal()
        tx_produce_pattern     = Signal()
        tx_pattern             = Signal(20)
        tx_prbs_config         = Signal(2)

        rx_prbs_config         = Signal(2)
        rx_prbs_pause          = Signal()
        rx_prbs_errors         = Signal(32)

        self.specials += [
            MultiReg(self.tx_produce_square_wave, tx_produce_square_wave, "tx"),
            MultiReg(self.tx_produce_pattern, tx_produce_pattern, "tx"),
            MultiReg(self.tx_pattern, tx_pattern, "tx"),
            MultiReg(self.tx_prbs_config, tx_prbs_config, "tx"),
        ]

        self.specials += [
            MultiReg(self.rx_align, rx_align, "rx"),
            MultiReg(self.rx_prbs_config, rx_prbs_config, "rx"),
            MultiReg(self.rx_prbs_pause, rx_prbs_pause, "rx"),
            MultiReg(rx_los, self.rx_idle, "sys"),
            MultiReg(rx_prbs_errors, self.rx_prbs_errors, "sys"),
        ]

        # DCU init ---------------------------------------------------------------------------------
        self.submodules.init = init = SerdesInit(tx_lol, rx_lol, rx_los)

        # Clocking ---------------------------------------------------------------------------------
        self.clock_domains.cd_tx = ClockDomain()
        self.comb += self.cd_tx.clk.eq(self.txoutclk)
        self.specials += AsyncResetSynchronizer(self.cd_tx, ~init.tx_ready)
        self.comb += self.tx_ready.eq(init.tx_ready)

        self.clock_domains.cd_rx = ClockDomain()
        self.comb += self.cd_rx.clk.eq(self.rxoutclk)
        self.specials += AsyncResetSynchronizer(self.cd_rx, ~init.rx_ready)
        self.comb += self.rx_ready.eq(init.rx_ready)

        # DCU instance -----------------------------------------------------------------------------
        self.serdes_params = dict(
            # ECP5's DCU parameters/signals/instance have been documented by whitequark as part of
            #             Yumewatari project: https://github.com/whitequark/Yumewatari
            #                  Copyright (C) 2018 whitequark@whitequark.org
            # DCU ----------------------------------------------------------------------------------
            # DCU — power management
            p_D_MACROPDB            = "0b1",
            p_D_IB_PWDNB            = "0b1",    # undocumented (required for RX)
            p_D_TXPLL_PWDNB         = "0b1",
            i_D_FFC_MACROPDB        = 1,

            # DCU — reset
            i_D_FFC_MACRO_RST       = ResetSignal("sys"),
            i_D_FFC_DUAL_RST        = ResetSignal("sys"),

            # DCU — clocking
            i_D_REFCLKI             = pll.refclk,
            o_D_FFS_PLOL            = tx_lol,
            p_D_REFCK_MODE          = {
                25: "0b100",
                20: "0b000",
                16: "0b010",
                10: "0b001",
                 8: "0b011"}[pll.config["m"]],
            p_D_TX_MAX_RATE         = "5.0",    # 5.0 Gbps
            p_D_TX_VCO_CK_DIV       = {
                32: "0b111",
                16: "0b110",
                 8: "0b101",
                 4: "0b100",
                 2: "0b010",
                 1: "0b000"}[pll.config["d"]],
            p_D_BITCLK_LOCAL_EN     = "0b1",    # Use clock from local PLL

            # DCU ­— unknown
            p_D_CMUSETBIASI         = "0b00",   # begin undocumented (10BSER sample code used)
            p_D_CMUSETI4CPP         = "0d3",
            p_D_CMUSETI4CPZ         = "0d3",
            p_D_CMUSETI4VCO         = "0b00",
            p_D_CMUSETICP4P         = "0b01",
            p_D_CMUSETICP4Z         = "0b101",
            p_D_CMUSETINITVCT       = "0b00",
            p_D_CMUSETISCL4VCO      = "0b000",
            p_D_CMUSETP1GM          = "0b000",
            p_D_CMUSETP2AGM         = "0b000",
            p_D_CMUSETZGM           = "0b000",
            p_D_SETIRPOLY_AUX       = "0b01",
            p_D_SETICONST_AUX       = "0b01",
            p_D_SETIRPOLY_CH        = "0b01",
            p_D_SETICONST_CH        = "0b10",
            p_D_SETPLLRC            = "0d1",
            p_D_RG_EN               = "0b0",
            p_D_RG_SET              = "0b00",
            p_D_REQ_ISET            = "0b011",
            p_D_PD_ISET             = "0b11",   # end undocumented

            # DCU — FIFOs
            p_D_LOW_MARK            = "0d4",    # Clock compensation FIFO low  water mark (mean=8)
            p_D_HIGH_MARK           = "0d12",   # Clock compensation FIFO high water mark (mean=8)

            # CHX common ---------------------------------------------------------------------------
            # CHX — protocol
            p_CHX_PROTOCOL          = "10BSER",
            p_CHX_UC_MODE           = "0b1",
            p_CHX_ENC_BYPASS        = "0b1",    # Bypass 8b10b encoder
            p_CHX_DEC_BYPASS        = "0b1",    # Bypass 8b10b encoder

            # CHX receive --------------------------------------------------------------------------
            # CHX RX ­— power management
            p_CHX_RPWDNB            = "0b1",
            i_CHX_FFC_RXPWDNB       = 1,

            # CHX RX ­— reset
            i_CHX_FFC_RRST          = ~self.rx_enable | init.rx_rst,
            i_CHX_FFC_LANE_RX_RST   = ~self.rx_enable | init.pcs_rst,

            # CHX RX ­— input
            i_CHX_HDINP             = rx_pads.p,
            i_CHX_HDINN             = rx_pads.n,

            p_CHX_REQ_EN            = "0b0",    # Enable equalizer
            p_CHX_RX_RATE_SEL       = "0d10",   # Equalizer  pole position
            p_CHX_RTERM_RX          = {
                "5k-ohms": "0d00",
                "80-ohms": "0d01",
                "75-ohms": "0d04",
                "70-ohms": "0d06",
                "60-ohms": "0d11",
                "50-ohms": "0d19",
                "46-ohms": "0d25"}["50-ohms"],
            p_CHX_RXIN_CM           = "0b11",   # CMFB (wizard value used)
            p_CHX_RXTERM_CM         = "0b11",   # RX Input (wizard value used)

            # CHX RX ­— clocking
            i_CHX_RX_REFCLK         = pll.refclk,
            o_CHX_FF_RX_PCLK        = self.rxoutclk,
            i_CHX_FF_RXI_CLK        = ClockSignal("rx"),

            p_CHX_CDR_MAX_RATE      = "5.0",    # 5.0 Gbps
            p_CHX_RX_DCO_CK_DIV     = {
                32: "0b111",
                16: "0b110",
                 8: "0b101",
                 4: "0b100",
                 2: "0b010",
                 1: "0b000"}[pll.config["d"]],
            p_CHX_RX_GEAR_MODE      = "0b1",    # 1:2 gearbox
            p_CHX_FF_RX_H_CLK_EN    = "0b1",    # enable  DIV/2 output clock
            p_CHX_FF_RX_F_CLK_DIS   = "0b1",    # disable DIV/1 output clock
            p_CHX_SEL_SD_RX_CLK     = "0b1",    # FIFO driven by recovered clock

            p_CHX_AUTO_FACQ_EN      = "0b1",    # undocumented (wizard value used)
            p_CHX_AUTO_CALIB_EN     = "0b1",    # undocumented (wizard value used)
            p_CHX_PDEN_SEL          = "0b1",    # phase detector disabled on LOS

            p_CHX_DCOATDCFG         = "0b00",   # begin undocumented (sample code used)
            p_CHX_DCOATDDLY         = "0b00",
            p_CHX_DCOBYPSATD        = "0b1",
            p_CHX_DCOCALDIV         = "0b000",
            p_CHX_DCOCTLGI          = "0b011",
            p_CHX_DCODISBDAVOID     = "0b0",
            p_CHX_DCOFLTDAC         = "0b00",
            p_CHX_DCOFTNRG          = "0b001",
            p_CHX_DCOIOSTUNE        = "0b010",
            p_CHX_DCOITUNE          = "0b00",
            p_CHX_DCOITUNE4LSB      = "0b010",
            p_CHX_DCOIUPDNX2        = "0b1",
            p_CHX_DCONUOFLSB        = "0b100",
            p_CHX_DCOSCALEI         = "0b01",
            p_CHX_DCOSTARTVAL       = "0b010",
            p_CHX_DCOSTEP           = "0b11",   # end undocumented

            # CHX RX — loss of signal
            o_CHX_FFS_RLOS          = rx_los,
            p_CHX_RLOS_SEL          = "0b1",
            p_CHX_RX_LOS_EN         = "0b1",
            p_CHX_RX_LOS_LVL        = "0b100",  # Lattice "TBD" (wizard value used)
            p_CHX_RX_LOS_CEQ        = "0b11",   # Lattice "TBD" (wizard value used)

            # CHX RX — loss of lock
            o_CHX_FFS_RLOL          = rx_lol,

            # CHx_RXLSM? CHx_RXWA?

            # CHX RX — link state machine
            i_CHX_FFC_SIGNAL_DETECT = rx_align & (self.rx_prbs_config == 0),
            o_CHX_FFS_LS_SYNC_STATUS= rx_lsm,
            p_CHX_ENABLE_CG_ALIGN   = "0b1",
            p_CHX_UDF_COMMA_MASK    = "0x3ff",  # compare all 10 bits
            p_CHX_UDF_COMMA_A       = "0x283",  # K28.5 inverted
            p_CHX_UDF_COMMA_B       = "0x17C",  # K28.5

            p_CHX_CTC_BYPASS        = "0b1",    # bypass CTC FIFO
            p_CHX_MIN_IPG_CNT       = "0b11",   # minimum interpacket gap of 4
            p_CHX_MATCH_2_ENABLE    = "0b0",    # 2 character skip matching
            p_CHX_MATCH_4_ENABLE    = "0b0",    # 4 character skip matching
            p_CHX_CC_MATCH_1        = "0x000",
            p_CHX_CC_MATCH_2        = "0x000",
            p_CHX_CC_MATCH_3        = "0x000",
            p_CHX_CC_MATCH_4        = "0x000",

            # CHX RX — data
            **{"o_CHX_FF_RX_D_%d" % n: rx_bus[n] for n in range(rx_bus.nbits)},

            # CHX transmit -------------------------------------------------------------------------
            # CHX TX — power management
            p_CHX_TPWDNB            = "0b1",
            i_CHX_FFC_TXPWDNB       = 1,

            # CHX TX ­— reset
            i_D_FFC_TRST            = ~self.tx_enable | init.tx_rst,
            i_CHX_FFC_LANE_TX_RST   = ~self.tx_enable | init.pcs_rst,

            # CHX TX ­— output
            o_CHX_HDOUTP            = tx_pads.p,
            o_CHX_HDOUTN            = tx_pads.n,

            p_CHX_TXAMPLITUDE       = "0d1100",  # 1000 mV
            p_CHX_RTERM_TX          = {
                "5k-ohms": "0d00",
                "80-ohms": "0d01",
                "75-ohms": "0d04",
                "70-ohms": "0d06",
                "60-ohms": "0d11",
                "50-ohms": "0d19",
                "46-ohms": "0d25"}["50-ohms"],

            p_CHX_TDRV_SLICE0_CUR   = "0b011",  # 400 uA
            p_CHX_TDRV_SLICE0_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE1_CUR   = "0b000",  # 100 uA
            p_CHX_TDRV_SLICE1_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE2_CUR   = "0b11",   # 3200 uA
            p_CHX_TDRV_SLICE2_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE3_CUR   = "0b10",   # 2400 uA
            p_CHX_TDRV_SLICE3_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE4_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_SLICE4_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE5_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_SLICE5_SEL   = "0b00",   # power down

            # CHX TX ­— clocking
            o_CHX_FF_TX_PCLK        = self.txoutclk,
            i_CHX_FF_TXI_CLK        = ClockSignal("tx"),

            p_CHX_TX_GEAR_MODE      = "0b1",    # 1:2 gearbox
            p_CHX_FF_TX_H_CLK_EN    = "0b1",    # enable  DIV/2 output clock
            p_CHX_FF_TX_F_CLK_DIS   = "0b1",    # disable DIV/1 output clock

            # CHX TX — data
            **{"i_CHX_FF_TX_D_%d" % n: tx_bus[n] for n in range(tx_bus.nbits)},

            i_CHX_FFC_EI_EN = self.tx_idle
        )

        # SCI Reconfiguration ----------------------------------------------------------------------
        sci_reconfig = SerDesECP5SCIReconfig(self)
        self.submodules.sci_reconfig = sci_reconfig
        self.comb += sci_reconfig.reset.eq(~self.init.tx_ready)
        self.comb += sci_reconfig.sci.dual_sel.eq(dual)
        self.comb += sci_reconfig.loopback.eq(self.loopback)
        self.comb += sci_reconfig.rx_polarity.eq(rx_polarity)
        self.comb += sci_reconfig.tx_polarity.eq(tx_polarity)
        self.comb += sci_reconfig.rx_cdr_hold.eq(self.rx_cdr_hold)

        # TX Datapath and PRBS ---------------------------------------------------------------------
        self.submodules.tx_prbs = ClockDomainsRenamer("tx")(PRBSTX(data_width, True))
        self.comb += self.tx_prbs.config.eq(tx_prbs_config)
        self.comb += [
            self.tx_prbs.i.eq(Cat(*[self.encoder.output[i] for i in range(nwords)])),
            If(True,
            #If(tx_produce_square_wave,
                # square wave @ linerate/data_width for scope observation
                tx_data.eq(Signal(data_width, reset=(1<<(data_width//2))-1))
            ).Elif(tx_produce_pattern,
                tx_data.eq(tx_pattern)
            ).Else(
                tx_data.eq(self.tx_prbs.o)
            ),
            tx_bus[ 0:10].eq(tx_data[ 0:10]),
            tx_bus[12:22].eq(tx_data[10:20]),
        ]

        # RX Datapath and PRBS ---------------------------------------------------------------------
        self.submodules.rx_prbs = ClockDomainsRenamer("rx")(PRBSRX(data_width, True))
        self.comb += [
            self.rx_prbs.config.eq(rx_prbs_config),
            self.rx_prbs.pause.eq(rx_prbs_pause),
            rx_prbs_errors.eq(self.rx_prbs.errors),
            rx_data[ 0:10].eq(rx_bus[ 0:10]),
            rx_data[10:20].eq(rx_bus[12:22]),
        ]
        for i in range(nwords):
            self.sync.rx += self.decoders[i].input.eq(rx_data[10*i:10*(i+1)])
        self.comb += self.rx_prbs.i.eq(rx_data)

    def add_stream_endpoints(self):
        self.sink   =   sink = stream.Endpoint([("data", self.nwords*8), ("ctrl", self.nwords)])
        self.source = source = stream.Endpoint([("data", self.nwords*8), ("ctrl", self.nwords)])

        self.comb += sink.ready.eq(1)
        self.comb += source.valid.eq(1)
        for i in range(self.nwords):
            self.comb += [
                self.encoder.k[i].eq(sink.ctrl[i]),
                self.encoder.d[i].eq(sink.data[8*i:8*(i+1)]),
                source.ctrl[i].eq(self.decoders[i].k),
                source.data[8*i:8*(i+1)].eq(self.decoders[i].d),
            ]

    def add_base_control(self, auto_enable=True):
        self._tx_enable = CSRStorage(fields=[
                CSRField("enable", size=1, values=[
                    ("``0b0``", "TX disabled."),
                    ("``0b1``", "TX enabled.")
                ], reset=int(auto_enable))
            ])
        self._tx_ready = CSRStatus(fields=[
                CSRField("ready", size=1, values=[
                    ("``0b0``", "TX not initialized."),
                    ("``0b1``", "TX initialized and ready.")
                ])
            ])
        self._tx_inhibit = CSRStorage(fields=[
                CSRField("inhibit", size=1, values=[
                    ("``0b0``", "Normal operation."),
                    ("``0b1``", "TX inhibited.")
                ])
            ])
        self._tx_produce_square_wave = CSRStorage(fields=[
                CSRField("enable", size=1, values=[
                    ("``0b0``", "Normal operation."),
                    ("``0b1``", "TX square wave genration enabled (linerate observation/checks).")
                ])
            ])
        self._rx_enable = CSRStorage(fields=[
                CSRField("enable", size=1, values=[
                    ("``0b0``", "RX disabled."),
                    ("``0b1``", "RX enabled.")
                ], reset=int(auto_enable))
            ])
        self._rx_ready = CSRStatus(fields=[
                CSRField("ready", size=1, values=[
                    ("``0b0``", "RX not initialized."),
                    ("``0b1``", "RX initialized and ready.")
                ])
            ])
        self.comb += [
            self.tx_enable.eq(self._tx_enable.fields.enable),
            self._tx_ready.fields.ready.eq(self.tx_ready),
            self.tx_inhibit.eq(self._tx_inhibit.fields.inhibit),
            self.tx_produce_square_wave.eq(self._tx_produce_square_wave.fields.enable),
            self.rx_enable.eq(self._rx_enable.fields.enable),
            self._rx_ready.fields.ready.eq(self.rx_ready),
        ]

    def add_prbs_control(self):
        self._tx_prbs_config = CSRStorage(fields=[
            CSRField("config", size=2, values=[
                ("``0b00``", "PRBS   Disabled."),
                ("``0b01``", "PRBS7  Enabled."),
                ("``0b10``", "PRBS15 Enabled."),
                ("``0b11``", "PRBS31 Enabled."),
            ])
        ])
        self._rx_prbs_config = CSRStorage(fields=[
            CSRField("config", size=2, values=[
                ("``0b00``", "PRBS   Disabled."),
                ("``0b01``", "PRBS7  Enabled."),
                ("``0b10``", "PRBS15 Enabled."),
                ("``0b11``", "PRBS31 Enabled."),
            ])
        ])
        self._rx_prbs_pause  = CSRStorage(description="Pause RX PRBS.")
        self._rx_prbs_errors = CSRStatus(32, description="RX PRBS errors.")
        self.comb += [
            self.tx_prbs_config.eq(self._tx_prbs_config.fields.config),
            self.rx_prbs_config.eq(self._rx_prbs_config.fields.config),
            self.rx_prbs_pause.eq(self._rx_prbs_pause.storage),
            self._rx_prbs_errors.status.eq(self.rx_prbs_errors)
        ]

    def add_loopback_control(self):
        self._loopback = CSRStorage()
        self.comb += self.loopback.eq(self._loopback.storage)


    def add_controls(self, auto_enable=True):
        self.add_base_control(auto_enable)
        self.add_prbs_control()
        self.add_loopback_control()

    def add_clock_cycles(self):
        self.clock_latch    = CSRStorage(description="Write to latch TX/RX clock cycles")
        self.clock_tx_cycles = CSRStorage(32, description="TX clock cycles")
        self.clock_rx_cycles = CSRStorage(32, description="RX clock cycles")

        tx_cycles = Signal(32)
        rx_cycles = Signal(32)
        self.sync.tx += tx_cycles.eq(tx_cycles + 1)
        self.sync.rx += rx_cycles.eq(rx_cycles + 1)

        self.sync += If(self.clock_latch.re,
            self.clock_tx_cycles.storage.eq(tx_cycles),
            self.clock_rx_cycles.storage.eq(rx_cycles),
        )

    def do_finalize(self):
        serdes_params = dict()
        for k, v in self.serdes_params.items():
            k = k.replace("CHX", "CH{}".format(self.channel))
            serdes_params[k] = v
        self.specials.dcu0 = Instance("DCUA", **serdes_params)
        self.dcu0.attr.add(("LOC", "DCU{}".format(self.dual)))
        self.dcu0.attr.add(("CHAN", "CH{}".format(self.channel)))
        self.dcu0.attr.add(("BEL", "X42/Y71/DCU"))
