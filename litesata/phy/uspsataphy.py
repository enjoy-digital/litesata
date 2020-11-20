#
# This file is part of LiteSATA.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2020 Qui Nguyen <qui@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *
from litesata.common import _PulseSynchronizer, _RisingEdge

from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from liteiclink.serdes.common import *
from liteiclink.serdes.gty_ultrascale_init import GTYTXInit, GTYRXInit


class USPLiteSATAPHYCRG(Module):
    def __init__(self, refclk, pads, gty, gen):
        self.tx_reset = Signal()
        self.rx_reset = Signal()

        self.clock_domains.cd_sata_tx = ClockDomain()
        self.clock_domains.cd_sata_rx = ClockDomain()

        # CPLL -------------------------------------------------------------------------------------
        #   (gen3) 150MHz / VCO @ 3GHz / Linerate @ 6Gbps
        #   (gen2 & gen1) VCO still @ 3 GHz, Linerate is decreased with output dividers.
        if isinstance(refclk, (Signal, ClockSignal)):
            self.refclk = refclk
        else:
            self.refclk = Signal()
            self.specials += Instance("IBUFDS_GTE4",
                i_CEB = 0,
                i_I   = pads.clk_p,
                i_IB  = pads.clk_n,
                o_O   = self.refclk
            )
        self.comb += gty.refclk.eq(self.refclk)

        # TX clocking ------------------------------------------------------------------------------
        #   (gen3) 150MHz from CPLL TXOUTCLK, sata_tx clk @ 300MHz (16-bits) /  150MHz (32-bits)
        #   (gen2) 150MHz from CPLL TXOUTCLK, sata_tx clk @ 150MHz (16-bits) /   75MHz (32-bits)
        #   (gen1) 150MHz from CPLL TXOUTCLK, sata_tx clk @ 75MHz  (16-bits) / 37.5MHz (32-bits)
        self.specials += Instance("BUFG_GT", i_I=gty.txoutclk, o_O=self.cd_sata_tx.clk)

        self.comb += gty.txusrclk.eq(self.cd_sata_tx.clk)
        self.comb += gty.txusrclk2.eq(self.cd_sata_tx.clk)

        # RX clocking ------------------------------------------------------------------------------
        #   (gen3) sata_rx recovered clk @ 300MHz (16-bits) /  150MHz (32-bits) from GTY RXOUTCLK
        #   (gen2) sata_rx recovered clk @ 150MHz (16-bits) /   75MHz (32-bits) from GTY RXOUTCLK
        #   (gen1) sata_rx recovered clk @ 75MHz  (16-bits) / 37.5MHz (32-bits) from GTY RXOUTCLK
        self.specials += Instance("BUFG_GT", i_I=gty.rxoutclk, o_O=self.cd_sata_rx.clk)

        self.comb += gty.rxusrclk.eq(self.cd_sata_rx.clk)
        self.comb += gty.rxusrclk2.eq(self.cd_sata_rx.clk)

        # Reset for SATA TX/RX clock domains -------------------------------------------------------
        self.specials += [
            AsyncResetSynchronizer(self.cd_sata_tx, ~gty.cplllock | self.tx_reset),
            AsyncResetSynchronizer(self.cd_sata_rx, ~gty.cplllock | self.rx_reset)
        ]

# --------------------------------------------------------------------------------------------------

class USPLiteSATAPHY(Module):
    def __init__(self, pads, gen, clk_freq, data_width=16, tx_buffer_enable=False, rx_buffer_enable=False):
        assert data_width in [16, 32]
        # Common signals
        self.data_width     = data_width

        # Control
        self.ready          = Signal() # o

        self.tx_idle        = Signal() # i
        self.tx_polarity    = Signal() # i

        self.tx_cominit_stb = Signal() # i
        self.tx_cominit_ack = Signal() # o
        self.tx_comwake_stb = Signal() # i
        self.tx_comwake_ack = Signal() # o

        self.rx_idle        = Signal() # o
        self.rx_cdrhold     = Signal() # i
        self.rx_polarity    = Signal() # i

        self.rx_cominit_stb = Signal() # o
        self.rx_comwake_stb = Signal() # o

        # Datapath
        self.sink           = stream.Endpoint(phy_description(data_width))
        self.source         = stream.Endpoint(phy_description(data_width))

        # PLL
        self.refclk         = Signal()
        self.cplllock       = Signal()
        self.cpllpd         = Signal()
        self.cpllreset      = Signal()

        # Receive Ports - 8b10b Decoder
        self.rxctrl0        = Signal(data_width//8)
        self.rxctrl1        = Signal(data_width//8)
        self.rxctrl2        = Signal(data_width//8)
        self.rxctrl3        = Signal(data_width//8)

        self.rxcharisk        = Signal(data_width//8)
        self.rxdisperr      = Signal(data_width//8) # o
        self.rxnotintable   = Signal(data_width//8) # o
        rxdisperr           = Signal(data_width//8)

        self.comb += [
            self.rxcharisk.eq(self.rxctrl0),
            rxdisperr.eq(self.rxctrl1)
        ]

        # Receive Ports - RX Data Path interface
        self.rxdata         = Signal(data_width)
        self.rxoutclk       = Signal()
        self.rxusrclk       = Signal()
        self.rxusrclk2      = Signal()

        # Receive Ports - RX Ports for SATA
        self.rxcominitdet   = Signal()
        self.rxcomwakedet   = Signal()

        # Transmit Ports - 8b10b Encoder Control Ports
        self.txcharisk      = Signal(data_width//8)

        # Transmit Ports - TX Data Path interface
        self.txdata         = Signal(data_width)
        self.txoutclk       = Signal()
        self.txusrclk       = Signal()
        self.txusrclk2      = Signal()

        # Transmit Ports - TX Ports for PCI Express
        self.txelecidle     = Signal(reset=1)

        # Transmit Ports - TX Ports for SATA
        self.txcomfinish    = Signal()
        self.txcominit      = Signal()
        self.txcomwake      = Signal()

        # Power-down signals
        self.rxpd           = Signal()
        self.txpd           = Signal()

        # Config at startup
        div_config = {
            "gen1": 4,
            "gen2": 2,
            "gen3": 1,
        }
        rxout_div = div_config[gen]
        txout_div = div_config[gen]

        progdiv = {
            "gen1": 40.0,
            "gen2": 20.0,
            "gen3": 10.0,
        }
        tx_progdiv_cfg = progdiv[gen]
        rx_progdiv_cfg = progdiv[gen]

        # TX Init ----------------------------------------------------------------------------------
        self.submodules.tx_init = tx_init = GTYTXInit(clk_freq, buffer_enable=tx_buffer_enable)
        self.comb += tx_init.plllock.eq(self.cplllock)
        self.comb += self.cpllreset.eq(tx_init.pllreset)

        # RX Init ----------------------------------------------------------------------------------
        self.submodules.rx_init = rx_init = GTYRXInit(clk_freq, buffer_enable=rx_buffer_enable)
        self.comb += rx_init.plllock.eq(self.cplllock)

        # Ready ------------------------------------------------------------------------------------
        self.comb += self.ready.eq(tx_init.done & rx_init.done)

        # Specific / Generic signals encoding/decoding ---------------------------------------------
        self.comb += [
            self.txelecidle.eq(self.tx_idle | self.txpd),
            self.tx_cominit_ack.eq(self.tx_cominit_stb & self.txcomfinish),
            self.tx_comwake_ack.eq(self.tx_comwake_stb & self.txcomfinish),
            self.rx_cominit_stb.eq(self.rxcominitdet),
            self.rx_comwake_stb.eq(self.rxcomwakedet),
        ]
        self.submodules += _RisingEdge(self.tx_cominit_stb, self.txcominit)
        self.submodules += _RisingEdge(self.tx_comwake_stb, self.txcomwake)

        self.sync.sata_rx += [
            self.source.valid.eq(1),
            self.source.charisk.eq(self.rxcharisk),
            self.source.data.eq(self.rxdata)
        ]

        self.sync.sata_tx += [
            self.txcharisk.eq(self.sink.charisk),
            self.txdata.eq(self.sink.data),
            self.sink.ready.eq(1),
        ]

        # Internals and clock domain crossing ------------------------------------------------------
        # sys_clk --> sata_tx clk
        txpd       = Signal()
        txelecidle = Signal(reset=1)
        txcominit  = Signal()
        txcomwake  = Signal()
        self.specials += [
            MultiReg(self.txpd,             txpd, "sata_tx"),
            MultiReg(self.txelecidle, txelecidle, "sata_tx"),
        ]
        self.submodules += [
            _PulseSynchronizer(self.txcominit,  "sys",  txcominit, "sata_tx"),
            _PulseSynchronizer(self.txcomwake,  "sys",  txcomwake, "sata_tx"),
        ]

        # sata_tx clk --> sys clk
        txcomfinish = Signal()
        self.submodules += _PulseSynchronizer(txcomfinish, "sata_tx", self.txcomfinish, "sys")

        # sata_rx clk --> sys clk
        rxcominitdet = Signal()
        rxcomwakedet = Signal()
        rxratedone   = Signal()
        rxdisperr    = Signal(data_width//8)
        rxnotintable = Signal(data_width//8)
        self.specials += [
            MultiReg(rxcominitdet, self.rxcominitdet, "sys"),
            MultiReg(rxcomwakedet, self.rxcomwakedet, "sys"),
            MultiReg(rxdisperr, self.rxdisperr, "sys"),
            MultiReg(rxnotintable, self.rxnotintable, "sys")
        ]

        # DRP mux ----------------------------------------------------------------------------------
        self.drp = DRPInterface()
        self.submodules.drp_mux = drp_mux = DRPMux()
        drp_mux.add_interface(self.drp)

        # GTYE4_CHANNEL instance -------------------------------------------------------------------
        class Open(Signal): pass
        rxphaligndone = Signal()
        self.gty_params = dict(
            p_ACJTAG_DEBUG_MODE              = 0b0,
            p_ACJTAG_MODE                    = 0b0,
            p_ACJTAG_RESET                   = 0b0,
            p_ADAPT_CFG0                     = 0b0000000000000000,
            p_ADAPT_CFG1                     = 0b1111100000011100,
            p_ADAPT_CFG2                     = 0b0000000000000000,
            p_ALIGN_COMMA_DOUBLE             = "FALSE",
            p_ALIGN_COMMA_ENABLE             = 0b1111111111,
            p_ALIGN_COMMA_WORD               = 2,
            p_ALIGN_MCOMMA_DET               = "TRUE",
            p_ALIGN_MCOMMA_VALUE             = 0b1010000011,
            p_ALIGN_PCOMMA_DET               = "TRUE",
            p_ALIGN_PCOMMA_VALUE             = 0b0101111100,
            p_A_RXOSCALRESET                 = 0b0,
            p_A_RXPROGDIVRESET               = 0b0,
            p_A_RXTERMINATION                = 0b1,
            p_A_TXDIFFCTRL                   = 0b01100,
            p_A_TXPROGDIVRESET               = 0b0,
            p_CBCC_DATA_SOURCE_SEL           = "DECODED",
            p_CDR_SWAP_MODE_EN               = 0b0,
            p_CFOK_PWRSVE_EN                 = 0b1,
            p_CHAN_BOND_KEEP_ALIGN           = "FALSE",
            p_CHAN_BOND_MAX_SKEW             = 1,
            p_CHAN_BOND_SEQ_1_1              = 0b0000000000,
            p_CHAN_BOND_SEQ_1_2              = 0b0000000000,
            p_CHAN_BOND_SEQ_1_3              = 0b0000000000,
            p_CHAN_BOND_SEQ_1_4              = 0b0000000000,
            p_CHAN_BOND_SEQ_1_ENABLE         = 0b1111,
            p_CHAN_BOND_SEQ_2_1              = 0b0000000000,
            p_CHAN_BOND_SEQ_2_2              = 0b0000000000,
            p_CHAN_BOND_SEQ_2_3              = 0b0000000000,
            p_CHAN_BOND_SEQ_2_4              = 0b0000000000,
            p_CHAN_BOND_SEQ_2_ENABLE         = 0b1111,
            p_CHAN_BOND_SEQ_2_USE            = "FALSE",
            p_CHAN_BOND_SEQ_LEN              = 1,
            p_CH_HSPMUX                      = 0b0010000000100000,
            p_CKCAL1_CFG_0                   = 0b1100000011000000,
            p_CKCAL1_CFG_1                   = 0b0001000011000000,
            p_CKCAL1_CFG_2                   = 0b0010000000001000,
            p_CKCAL1_CFG_3                   = 0b0000000000000000,
            p_CKCAL2_CFG_0                   = 0b1100000011000000,
            p_CKCAL2_CFG_1                   = 0b1000000011000000,
            p_CKCAL2_CFG_2                   = 0b0001000000000000,
            p_CKCAL2_CFG_3                   = 0b0000000000000000,
            p_CKCAL2_CFG_4                   = 0b0000000000000000,
            p_CLK_CORRECT_USE                = "FALSE",
            p_CLK_COR_KEEP_IDLE              = "FALSE",
            p_CLK_COR_MAX_LAT                = 20 if not rx_buffer_enable else {16:6, 32:12}[data_width],
            p_CLK_COR_MIN_LAT                = 18 if not rx_buffer_enable else {16:4, 32: 8}[data_width],
            p_CLK_COR_PRECEDENCE             = "TRUE",
            p_CLK_COR_REPEAT_WAIT            = 0,
            p_CLK_COR_SEQ_1_1                = 0b0100000000,
            p_CLK_COR_SEQ_1_2                = 0b0100000000,
            p_CLK_COR_SEQ_1_3                = 0b0100000000,
            p_CLK_COR_SEQ_1_4                = 0b0100000000,
            p_CLK_COR_SEQ_1_ENABLE           = 0b1111,
            p_CLK_COR_SEQ_2_1                = 0b0100000000,
            p_CLK_COR_SEQ_2_2                = 0b0100000000,
            p_CLK_COR_SEQ_2_3                = 0b0100000000,
            p_CLK_COR_SEQ_2_4                = 0b0100000000,
            p_CLK_COR_SEQ_2_ENABLE           = 0b1111,
            p_CLK_COR_SEQ_2_USE              = "FALSE",
            p_CLK_COR_SEQ_LEN                = 1,
            p_CPLL_CFG0                      = 0b0000000111111010,
            p_CPLL_CFG1                      = 0b0000000000101011,
            p_CPLL_CFG2                      = 0b0000000000000010,
            p_CPLL_CFG3                      = 0b0000000000000000,
            p_CPLL_FBDIV                     = 5,
            p_CPLL_FBDIV_45                  = 4,
            p_CPLL_INIT_CFG0                 = 0b0000001010110010,
            p_CPLL_LOCK_CFG                  = 0b0000000111101000,
            p_CPLL_REFCLK_DIV                = 1,
            p_CTLE3_OCAP_EXT_CTRL            = 0b000,
            p_CTLE3_OCAP_EXT_EN              = 0b0,
            p_DDI_CTRL                       = 0b00,
            p_DDI_REALIGN_WAIT               = 15,
            p_DEC_MCOMMA_DETECT              = "TRUE",
            p_DEC_PCOMMA_DETECT              = "TRUE",
            p_DEC_VALID_COMMA_ONLY           = "TRUE",
            p_DELAY_ELEC                     = 0b0,
            p_DMONITOR_CFG0                  = 0b0000000000,
            p_DMONITOR_CFG1                  = 0b00000000,
            p_ES_CLK_PHASE_SEL               = 0b0,
            p_ES_CONTROL                     = 0b000000,
            p_ES_ERRDET_EN                   = "FALSE",
            p_ES_EYE_SCAN_EN                 = "FALSE",
            p_ES_HORZ_OFFSET                 = 0b000000000000,
            p_ES_PRESCALE                    = 0b00000,
            p_ES_QUALIFIER0                  = 0b0000000000000000,
            p_ES_QUALIFIER1                  = 0b0000000000000000,
            p_ES_QUALIFIER2                  = 0b0000000000000000,
            p_ES_QUALIFIER3                  = 0b0000000000000000,
            p_ES_QUALIFIER4                  = 0b0000000000000000,
            p_ES_QUALIFIER5                  = 0b0000000000000000,
            p_ES_QUALIFIER6                  = 0b0000000000000000,
            p_ES_QUALIFIER7                  = 0b0000000000000000,
            p_ES_QUALIFIER8                  = 0b0000000000000000,
            p_ES_QUALIFIER9                  = 0b0000000000000000,
            p_ES_QUAL_MASK0                  = 0b0000000000000000,
            p_ES_QUAL_MASK1                  = 0b0000000000000000,
            p_ES_QUAL_MASK2                  = 0b0000000000000000,
            p_ES_QUAL_MASK3                  = 0b0000000000000000,
            p_ES_QUAL_MASK4                  = 0b0000000000000000,
            p_ES_QUAL_MASK5                  = 0b0000000000000000,
            p_ES_QUAL_MASK6                  = 0b0000000000000000,
            p_ES_QUAL_MASK7                  = 0b0000000000000000,
            p_ES_QUAL_MASK8                  = 0b0000000000000000,
            p_ES_QUAL_MASK9                  = 0b0000000000000000,
            p_ES_SDATA_MASK0                 = 0b0000000000000000,
            p_ES_SDATA_MASK1                 = 0b0000000000000000,
            p_ES_SDATA_MASK2                 = 0b0000000000000000,
            p_ES_SDATA_MASK3                 = 0b0000000000000000,
            p_ES_SDATA_MASK4                 = 0b0000000000000000,
            p_ES_SDATA_MASK5                 = 0b0000000000000000,
            p_ES_SDATA_MASK6                 = 0b0000000000000000,
            p_ES_SDATA_MASK7                 = 0b0000000000000000,
            p_ES_SDATA_MASK8                 = 0b0000000000000000,
            p_ES_SDATA_MASK9                 = 0b0000000000000000,
            p_EYESCAN_VP_RANGE               = 0,
            p_EYE_SCAN_SWAP_EN               = 0b0,
            p_FTS_DESKEW_SEQ_ENABLE          = 0b1111,
            p_FTS_LANE_DESKEW_CFG            = 0b1111,
            p_FTS_LANE_DESKEW_EN             = "FALSE",
            p_GEARBOX_MODE                   = 0b00000,
            p_ISCAN_CK_PH_SEL2               = 0b0,
            p_LOCAL_MASTER                   = 0b1,
            p_LPBK_BIAS_CTRL                 = 4,
            p_LPBK_EN_RCAL_B                 = 0b0,
            p_LPBK_EXT_RCAL                  = 0b1000,
            p_LPBK_IND_CTRL0                 = 5,
            p_LPBK_IND_CTRL1                 = 5,
            p_LPBK_IND_CTRL2                 = 5,
            p_LPBK_RG_CTRL                   = 2,
            p_OOBDIVCTL                      = 0b01,
            p_OOB_PWRUP                      = 0b1,
            p_PCI3_AUTO_REALIGN              = "OVR_1K_BLK",
            p_PCI3_PIPE_RX_ELECIDLE          = 0b0,
            p_PCI3_RX_ASYNC_EBUF_BYPASS      = 0b00,
            p_PCI3_RX_ELECIDLE_EI2_ENABLE    = 0b0,
            p_PCI3_RX_ELECIDLE_H2L_COUNT     = 0b000000,
            p_PCI3_RX_ELECIDLE_H2L_DISABLE   = 0b000,
            p_PCI3_RX_ELECIDLE_HI_COUNT      = 0b000000,
            p_PCI3_RX_ELECIDLE_LP4_DISABLE   = 0b0,
            p_PCI3_RX_FIFO_DISABLE           = 0b0,
            p_PCIE3_CLK_COR_EMPTY_THRSH      = 0b00000,
            p_PCIE3_CLK_COR_FULL_THRSH       = 0b010000,
            p_PCIE3_CLK_COR_MAX_LAT          = 0b00100,
            p_PCIE3_CLK_COR_MIN_LAT          = 0b00000,
            p_PCIE3_CLK_COR_THRSH_TIMER      = 0b001000,
            p_PCIE_64B_DYN_CLKSW_DIS         = "FALSE",
            p_PCIE_BUFG_DIV_CTRL             = 0b0001000000000000,
            p_PCIE_GEN4_64BIT_INT_EN         = "FALSE",
            p_PCIE_PLL_SEL_MODE_GEN12        = 0b00,
            p_PCIE_PLL_SEL_MODE_GEN3         = 0b11,
            p_PCIE_PLL_SEL_MODE_GEN4         = 0b10,
            p_PCIE_RXPCS_CFG_GEN3            = 0b0000101010100101,
            p_PCIE_RXPMA_CFG                 = 0b0010100000001010,
            p_PCIE_TXPCS_CFG_GEN3            = 0b0010110010100100,
            p_PCIE_TXPMA_CFG                 = 0b0010100000001010,
            p_PCS_PCIE_EN                    = "FALSE",
            p_PCS_RSVD0                      = 0b0000000000000000,
            p_PD_TRANS_TIME_FROM_P2          = 0b000000111100,
            p_PD_TRANS_TIME_NONE_P2          = 0b00011001,
            p_PD_TRANS_TIME_TO_P2            = 0b01100100,
            p_PREIQ_FREQ_BST                 = 0,
            p_RATE_SW_USE_DRP                = 0b1,
            p_RCLK_SIPO_DLY_ENB              = 0b0,
            p_RCLK_SIPO_INV_EN               = 0b0,
            p_RTX_BUF_CML_CTRL               = 0b011,
            p_RTX_BUF_TERM_CTRL              = 0b00,
            p_RXBUFRESET_TIME                = 0b00011,
            p_RXBUF_ADDR_MODE                = "FAST",
            p_RXBUF_EIDLE_HI_CNT             = 0b1000,
            p_RXBUF_EIDLE_LO_CNT             = 0b0000,
            p_RXBUF_EN                       = "TRUE" if rx_buffer_enable else "FALSE",
            p_RXBUF_RESET_ON_CB_CHANGE       = "TRUE",
            p_RXBUF_RESET_ON_COMMAALIGN      = "FALSE",
            p_RXBUF_RESET_ON_EIDLE           = "FALSE",
            p_RXBUF_RESET_ON_RATE_CHANGE     = "TRUE",
            p_RXBUF_THRESH_OVFLW             = 0 if not rx_buffer_enable else {16:61, 32:57}[data_width],
            p_RXBUF_THRESH_OVRD              = "TRUE" if rx_buffer_enable else "FALSE",
            p_RXBUF_THRESH_UNDFLW            = 4 if not rx_buffer_enable else {16:1, 32:3}[data_width],
        )

        self.gty_params.update(
            p_RXCDRFREQRESET_TIME            = 0b00001,
            p_RXCDRPHRESET_TIME              = 0b00001,
            p_RXCDR_CFG0                     = 0b0000000000000011,
            p_RXCDR_CFG0_GEN3                = 0b0000000000000011,
            p_RXCDR_CFG1                     = 0b0000000000000000,
            p_RXCDR_CFG1_GEN3                = 0b0000000000000000,
            p_RXCDR_CFG2                     = 0b0000001001000010 if gen == "gen1" else
                                               0b0000001001010011 if gen == "gen2" else
                                               0b0000001001100011,
            p_RXCDR_CFG2_GEN2                = 0b1001000010 if gen == "gen1" else
                                               0b1001010011 if gen == "gen2" else
                                               0b1001100011,
            p_RXCDR_CFG2_GEN3                = 0b0000001001000010 if gen == "gen1" else
                                               0b0000001001010011 if gen == "gen2" else
                                               0b0000001001100011,
            p_RXCDR_CFG2_GEN4                = 0b0000000101100100,
            p_RXCDR_CFG3                     = 0b0000000000011010,
            p_RXCDR_CFG3_GEN2                = 0b011010,
            p_RXCDR_CFG3_GEN3                = 0b0000000000011010,
            p_RXCDR_CFG3_GEN4                = 0b0000000000011010,
            p_RXCDR_CFG4                     = 0b0101110011110110,
            p_RXCDR_CFG4_GEN3                = 0b0101110011110110,
            p_RXCDR_CFG5                     = 0b1011010001101011,
            p_RXCDR_CFG5_GEN3                = 0b0001010001101011,
            p_RXCDR_FR_RESET_ON_EIDLE        = 0b0,
            p_RXCDR_HOLD_DURING_EIDLE        = 0b0,
            p_RXCDR_LOCK_CFG0                = 0b0010001000000001,
            p_RXCDR_LOCK_CFG1                = 0b1001111111111111,
            p_RXCDR_LOCK_CFG2                = 0b000000000000000,
            p_RXCDR_LOCK_CFG3                = 0b0000000000000000,
            p_RXCDR_LOCK_CFG4                = 0b0000000000000000,
            p_RXCDR_PH_RESET_ON_EIDLE        = 0b0,
            p_RXCFOK_CFG0                    = 0b0000000000000000,
            p_RXCFOK_CFG1                    = 0b1000000000010101,
            p_RXCFOK_CFG2                    = 0b0000001010101110,
            p_RXCKCAL1_IQ_LOOP_RST_CFG       = 0b0000000000000000,
            p_RXCKCAL1_I_LOOP_RST_CFG        = 0b0000000000000000,
            p_RXCKCAL1_Q_LOOP_RST_CFG        = 0b0000000000000000,
            p_RXCKCAL2_DX_LOOP_RST_CFG       = 0b0000000000000000,
            p_RXCKCAL2_D_LOOP_RST_CFG        = 0b0000000000000000,
            p_RXCKCAL2_S_LOOP_RST_CFG        = 0b0000000000000000,
            p_RXCKCAL2_X_LOOP_RST_CFG        = 0b0000000000000000,
            p_RXDFELPMRESET_TIME             = 0b0001111,
            p_RXDFELPM_KL_CFG0               = 0b000000000000000,
            p_RXDFELPM_KL_CFG1               = 0b1010000010000010,
            p_RXDFELPM_KL_CFG2               = 0b0000000100000000,
            p_RXDFE_CFG0                     = 0b0000101000000000,
            p_RXDFE_CFG1                     = 0b0000000000000000,
            p_RXDFE_GC_CFG0                  = 0b0000000000000000,
            p_RXDFE_GC_CFG1                  = 0b1000000000000000,
            p_RXDFE_GC_CFG2                  = 0b1111111111100000,
            p_RXDFE_H2_CFG0                  = 0b0000000000000000,
            p_RXDFE_H2_CFG1                  = 0b0000000000000010,
            p_RXDFE_H3_CFG0                  = 0b0000000000000000,
            p_RXDFE_H3_CFG1                  = 0b1000000000000010,
            p_RXDFE_H4_CFG0                  = 0b0000000000000000,
            p_RXDFE_H4_CFG1                  = 0b1000000000000010,
            p_RXDFE_H5_CFG0                  = 0b0000000000000000,
            p_RXDFE_H5_CFG1                  = 0b1000000000000010,
            p_RXDFE_H6_CFG0                  = 0b0000000000000000,
            p_RXDFE_H6_CFG1                  = 0b1000000000000010,
            p_RXDFE_H7_CFG0                  = 0b0000000000000000,
            p_RXDFE_H7_CFG1                  = 0b1000000000000010,
            p_RXDFE_H8_CFG0                  = 0b0000000000000000,
            p_RXDFE_H8_CFG1                  = 0b1000000000000010,
            p_RXDFE_H9_CFG0                  = 0b0000000000000000,
            p_RXDFE_H9_CFG1                  = 0b1000000000000010,
            p_RXDFE_HA_CFG0                  = 0b0000000000000000,
            p_RXDFE_HA_CFG1                  = 0b1000000000000010,
            p_RXDFE_HB_CFG0                  = 0b0000000000000000,
            p_RXDFE_HB_CFG1                  = 0b1000000000000010,
            p_RXDFE_HC_CFG0                  = 0b0000000000000000,
            p_RXDFE_HC_CFG1                  = 0b1000000000000010,
            p_RXDFE_HD_CFG0                  = 0b0000000000000000,
            p_RXDFE_HD_CFG1                  = 0b1000000000000010,
            p_RXDFE_HE_CFG0                  = 0b0000000000000000,
            p_RXDFE_HE_CFG1                  = 0b1000000000000010,
            p_RXDFE_HF_CFG0                  = 0b0000000000000000,
            p_RXDFE_HF_CFG1                  = 0b1000000000000010,
            p_RXDFE_KH_CFG0                  = 0b1000000000000000,
            p_RXDFE_KH_CFG1                  = 0b1111111000000000,
            p_RXDFE_KH_CFG2                  = 0b0000001000000000,
            p_RXDFE_KH_CFG3                  = 0b0100000100000001,
            p_RXDFE_OS_CFG0                  = 0b0010000000000000,
            p_RXDFE_OS_CFG1                  = 0b1000000000000000,
            p_RXDFE_UT_CFG0                  = 0b0000000000000000,
            p_RXDFE_UT_CFG1                  = 0b0000000000000011,
            p_RXDFE_UT_CFG2                  = 0b0000000000000000,
            p_RXDFE_VP_CFG0                  = 0b0000000000000000,
            p_RXDFE_VP_CFG1                  = 0b0000000000110011,
            p_RXDLY_CFG                      = 0b0000000000010000,
            p_RXDLY_LCFG                     = 0b0000000000110000,
            p_RXELECIDLE_CFG                 = "SIGCFG_4",
            p_RXGBOX_FIFO_INIT_RD_ADDR       = 4,
            p_RXGEARBOX_EN                   = "FALSE",
            p_RXISCANRESET_TIME              = 0b00001,
            p_RXLPM_CFG                      = 0b0000000000000000,
            p_RXLPM_GC_CFG                   = 0b1111100000000000,
            p_RXLPM_KH_CFG0                  = 0b0000000000000000,
            p_RXLPM_KH_CFG1                  = 0b1010000000000010,
            p_RXLPM_OS_CFG0                  = 0b0000000000000000,
            p_RXLPM_OS_CFG1                  = 0b1000000000000010,
            p_RXOOB_CFG                      = 0b000000110,
            p_RXOOB_CLK_CFG                  = "PMA",
            p_RXOSCALRESET_TIME              = 0b00011,
            p_RXOUT_DIV                      = rxout_div,
            p_RXPCSRESET_TIME                = 0b00011,
            p_RXPHBEACON_CFG                 = 0b0000000000000000,
            p_RXPHDLY_CFG                    = 0b0010000001110000,
            p_RXPHSAMP_CFG                   = 0b0010000100000000,
            p_RXPHSLIP_CFG                   = 0b1001100100110011,
            p_RXPH_MONITOR_SEL               = 0b00000,
            p_RXPI_CFG0                      = 0b0000001100000001,
            p_RXPI_CFG1                      = 0b0000000011111100,
            p_RXPMACLK_SEL                   = "DATA",
            p_RXPMARESET_TIME                = 0b00011,
            p_RXPRBS_ERR_LOOPBACK            = 0b0,
            p_RXPRBS_LINKACQ_CNT             = 15,
            p_RXREFCLKDIV2_SEL               = 0b0,
            p_RXSLIDE_AUTO_WAIT              = 7,
            p_RXSLIDE_MODE                   = "PCS",
            p_RXSYNC_MULTILANE               = 0b0,
            p_RXSYNC_OVRD                    = 0b0,
            p_RXSYNC_SKIP_DA                 = 0b0,
            p_RX_AFE_CM_EN                   = 0b0,
            p_RX_BIAS_CFG0                   = 0b0001001010110000,
            p_RX_BUFFER_CFG                  = 0b000000,
            p_RX_CAPFF_SARC_ENB              = 0b0,
            p_RX_CLK25_DIV                   = 6,
            p_RX_CLKMUX_EN                   = 0b1,
            p_RX_CLK_SLIP_OVRD               = 0b00000,
            p_RX_CM_BUF_CFG                  = 0b1010,
            p_RX_CM_BUF_PD                   = 0b0,
            p_RX_CM_SEL                      = 3,
            p_RX_CM_TRIM                     = 10,
            p_RX_CTLE_PWR_SAVING             = 0b0,
            p_RX_CTLE_RES_CTRL               = 0b0000,
            p_RX_DATA_WIDTH                  = 20 if data_width == 16 else 40,
            p_RX_DDI_SEL                     = 0b000000,
            p_RX_DEFER_RESET_BUF_EN          = "TRUE",
            p_RX_DEGEN_CTRL                  = 0b100,
            p_RX_DFELPM_CFG0                 = 10,
            p_RX_DFELPM_CFG1                 = 0b1,
            p_RX_DFELPM_KLKH_AGC_STUP_EN     = 0b1,
            p_RX_DFE_AGC_CFG1                = 2,
            p_RX_DFE_KL_LPM_KH_CFG0          = 3,
            p_RX_DFE_KL_LPM_KH_CFG1          = 2,
            p_RX_DFE_KL_LPM_KL_CFG0          = 0b11,
            p_RX_DFE_KL_LPM_KL_CFG1          = 2,
            p_RX_DFE_LPM_HOLD_DURING_EIDLE   = 0b0,
            p_RX_DISPERR_SEQ_MATCH           = "TRUE",
            p_RX_DIVRESET_TIME               = 0b00001,
            p_RX_EN_CTLE_RCAL_B              = 0b0,
            p_RX_EN_SUM_RCAL_B               = 0,
            p_RX_EYESCAN_VS_CODE             = 0b0000000,
            p_RX_EYESCAN_VS_NEG_DIR          = 0b0,
            p_RX_EYESCAN_VS_RANGE            = 0b10,
            p_RX_EYESCAN_VS_UT_SIGN          = 0b0,
            p_RX_FABINT_USRCLK_FLOP          = 0b0,
            p_RX_I2V_FILTER_EN               = 0b1,
            p_RX_INT_DATAWIDTH               = 0 if data_width == 16 else 1,
            p_RX_PMA_POWER_SAVE              = 0b0,
            p_RX_PMA_RSV0                    = 0b0000000000101111,
            p_RX_PROGDIV_CFG                 = rx_progdiv_cfg,
            p_RX_PROGDIV_RATE                = 0b0000000000000001,
            p_RX_RESLOAD_CTRL                = 0b0000,
            p_RX_RESLOAD_OVRD                = 0b0,
            p_RX_SAMPLE_PERIOD               = 0b111,
            p_RX_SIG_VALID_DLY               = 11,
            p_RX_SUM_DEGEN_AVTT_OVERITE      = 0,
            p_RX_SUM_DFETAPREP_EN            = 0b0,
            p_RX_SUM_IREF_TUNE               = 0b0000,
            p_RX_SUM_PWR_SAVING              = 0,
            p_RX_SUM_RES_CTRL                = 0b0000,
            p_RX_SUM_VCMTUNE                 = 0b0011,
            p_RX_SUM_VCM_BIAS_TUNE_EN        = 0b1,
            p_RX_SUM_VCM_OVWR                = 0b0,
            p_RX_SUM_VREF_TUNE               = 0b100,
            p_RX_TUNE_AFE_OS                 = 0b10,
            p_RX_VREG_CTRL                   = 0b010,
            p_RX_VREG_PDB                    = 0b1,
            p_RX_WIDEMODE_CDR                = 0b00,
            p_RX_WIDEMODE_CDR_GEN3           = 0b00,
            p_RX_WIDEMODE_CDR_GEN4           = 0b01,
            p_RX_XCLK_SEL                    = "RXDES" if rx_buffer_enable else "RXUSR",
            p_RX_XMODE_SEL                   = 0b1,
            p_SAMPLE_CLK_PHASE               = 0b0,
            p_SAS_12G_MODE                   = 0b0,
            p_SATA_BURST_SEQ_LEN             = 0b0110,
            p_SATA_BURST_VAL                 = 0b100,
            p_SATA_CPLL_CFG                  = "VCO_3000MHZ",
            p_SATA_EIDLE_VAL                 = 0b100,
            p_SHOW_REALIGN_COMMA             = "FALSE",
            p_SIM_DEVICE                     = "ULTRASCALE_PLUS",
            p_SIM_MODE                       = "FAST",
            p_SIM_RECEIVER_DETECT_PASS       = "TRUE",
            p_SIM_RESET_SPEEDUP              = "TRUE",
            p_SIM_TX_EIDLE_DRIVE_LEVEL       = "Z",
            p_SRSTMODE                       = 0b0,
            p_TAPDLY_SET_TX                  = 0b00,
            p_TERM_RCAL_CFG                  = 0b100001000000010,
            p_TERM_RCAL_OVRD                 = 0b001,
            p_TRANS_TIME_RATE                = 0b00001110,
            p_TST_RSV0                       = 0b00000000,
            p_TST_RSV1                       = 0b00000000,
        )

        self.gty_params.update(
            p_TXBUF_EN                       = "TRUE" if tx_buffer_enable else "FALSE",
            p_TXBUF_RESET_ON_RATE_CHANGE     = "TRUE",
            p_TXDLY_CFG                      = 0b1000000000010000,
            p_TXDLY_LCFG                     = 0b0000000000110000,
            p_TXDRV_FREQBAND                 = 0,
            p_TXFE_CFG0                      = 0b0000001111000010,
            p_TXFE_CFG1                      = 0b0110110000000000,
            p_TXFE_CFG2                      = 0b0110110000000000,
            p_TXFE_CFG3                      = 0b0110110000000000,
            p_TXFIFO_ADDR_CFG                = "LOW",
            p_TXGBOX_FIFO_INIT_RD_ADDR       = 4,
            p_TXGEARBOX_EN                   = "FALSE",
            p_TXOUT_DIV                      = txout_div,
            p_TXPCSRESET_TIME                = 0b00011,
            p_TXPHDLY_CFG0                   = 0b0110000001110000,
            p_TXPHDLY_CFG1                   = 0b0000000000001010 if gen == "gen3" else
                                               0b0000000000001111,
            p_TXPH_CFG                       = 0b0000011100100011 if gen == "gen1" else
                                               0b0000001100100011,
            p_TXPH_CFG2                      = 0b0000000000000000,
            p_TXPH_MONITOR_SEL               = 0b00000,
            p_TXPI_CFG0                      = 0b0000001100000000,
            p_TXPI_CFG1                      = 0b0111010101010101,
            p_TXPI_GRAY_SEL                  = 0b0,
            p_TXPI_INVSTROBE_SEL             = 0b0,
            p_TXPI_PPM                       = 0b0,
            p_TXPI_PPM_CFG                   = 0b00000000,
            p_TXPI_SYNFREQ_PPM               = 0b001,
            p_TXPMARESET_TIME                = 0b00011,
            p_TXREFCLKDIV2_SEL               = 0b0,
            p_TXSWBST_BST                    = 1,
            p_TXSWBST_EN                     = 0,
            p_TXSWBST_MAG                    = 4,
            p_TXSYNC_MULTILANE               = 0b0,
            p_TXSYNC_OVRD                    = 0b0,
            p_TXSYNC_SKIP_DA                 = 0b0,
            p_TX_CLK25_DIV                   = 6,
            p_TX_CLKMUX_EN                   = 0b1,
            p_TX_DATA_WIDTH                  = 20 if data_width == 16 else 40,
            p_TX_DCC_LOOP_RST_CFG            = 0b0000000000000100,
            p_TX_DEEMPH0                     = 0b000000,
            p_TX_DEEMPH1                     = 0b000000,
            p_TX_DEEMPH2                     = 0b000000,
            p_TX_DEEMPH3                     = 0b000000,
            p_TX_DIVRESET_TIME               = 0b00001,
            p_TX_DRIVE_MODE                  = "DIRECT",
            p_TX_EIDLE_ASSERT_DELAY          = 0b100,
            p_TX_EIDLE_DEASSERT_DELAY        = 0b011,
            p_TX_FABINT_USRCLK_FLOP          = 0b0,
            p_TX_FIFO_BYP_EN                 = 0 if tx_buffer_enable else 1,
            p_TX_IDLE_DATA_ZERO              = 0b0,
            p_TX_INT_DATAWIDTH               = 0 if data_width == 16 else 1,
            p_TX_LOOPBACK_DRIVE_HIZ          = "FALSE",
            p_TX_MAINCURSOR_SEL              = 0b0,
            p_TX_MARGIN_FULL_0               = 0b1011000,
            p_TX_MARGIN_FULL_1               = 0b1010111,
            p_TX_MARGIN_FULL_2               = 0b1010101,
            p_TX_MARGIN_FULL_3               = 0b1010011,
            p_TX_MARGIN_FULL_4               = 0b1010001,
            p_TX_MARGIN_LOW_0                = 0b1001100,
            p_TX_MARGIN_LOW_1                = 0b1001011,
            p_TX_MARGIN_LOW_2                = 0b1001000,
            p_TX_MARGIN_LOW_3                = 0b1000010,
            p_TX_MARGIN_LOW_4                = 0b1000000,
            p_TX_PHICAL_CFG0                 = 0b0000000000100000,
            p_TX_PHICAL_CFG1                 = 0b0000000001000000,
            p_TX_PI_BIASSET                  = 0,
            p_TX_PMADATA_OPT                 = 0b0,
            p_TX_PMA_POWER_SAVE              = 0b0,
            p_TX_PMA_RSV0                    = 0b0000000000000000,
            p_TX_PMA_RSV1                    = 0b0000000000000000,
            p_TX_PROGCLK_SEL                 = "PREPI",
            p_TX_PROGDIV_CFG                 = tx_progdiv_cfg,
            p_TX_PROGDIV_RATE                = 0b0000000000000001,
            p_TX_RXDETECT_CFG                = 0b00000000110010,
            p_TX_RXDETECT_REF                = 5,
            p_TX_SAMPLE_PERIOD               = 0b111,
            p_TX_SW_MEAS                     = 0b00,
            p_TX_VREG_CTRL                   = 0b011,
            p_TX_VREG_PDB                    = 0b1,
            p_TX_VREG_VREFSEL                = 0b10,
            p_TX_XCLK_SEL                    = "TXOUT" if tx_buffer_enable else "TXUSR",
            p_USB_BOTH_BURST_IDLE            = 0b0,
            p_USB_BURSTMAX_U3WAKE            = 0b1111111,
            p_USB_BURSTMIN_U3WAKE            = 0b1100011,
            p_USB_CLK_COR_EQ_EN              = 0b0,
            p_USB_EXT_CNTL                   = 0b1,
            p_USB_IDLEMAX_POLLING            = 0b1010111011,
            p_USB_IDLEMIN_POLLING            = 0b0100101011,
            p_USB_LFPSPING_BURST             = 0b000000101,
            p_USB_LFPSPOLLING_BURST          = 0b000110001,
            p_USB_LFPSPOLLING_IDLE_MS        = 0b000000100,
            p_USB_LFPSU1EXIT_BURST           = 0b000011101,
            p_USB_LFPSU2LPEXIT_BURST_MS      = 0b001100011,
            p_USB_LFPSU3WAKE_BURST_MS        = 0b111110011,
            p_USB_LFPS_TPERIOD               = 0b0011,
            p_USB_LFPS_TPERIOD_ACCURATE      = 0b1,
            p_USB_MODE                       = 0b0,
            p_USB_PCIE_ERR_REP_DIS           = 0b0,
            p_USB_PING_SATA_MAX_INIT         = 21,
            p_USB_PING_SATA_MIN_INIT         = 12,
            p_USB_POLL_SATA_MAX_BURST        = 8,
            p_USB_POLL_SATA_MIN_BURST        = 4,
            p_USB_RAW_ELEC                   = 0b0,
            p_USB_RXIDLE_P0_CTRL             = 0b1,
            p_USB_TXIDLE_TUNE_ENABLE         = 0b1,
            p_USB_U1_SATA_MAX_WAKE           = 7,
            p_USB_U1_SATA_MIN_WAKE           = 4,
            p_USB_U2_SAS_MAX_COM             = 64,
            p_USB_U2_SAS_MIN_COM             = 36,
            p_USE_PCS_CLK_PHASE_SEL          = 0b0,
            p_Y_ALL_MODE                     = 0b0
        )
        self.gty_params.update(
            #
            i_PCSRSVDIN       = 0x00,
            i_GTRSVD          = 0x0000,
            i_TSTIN           = 0, #2**20-1,

            # Reset modes
            i_GTTXRESETSEL    = 0,
            i_GTRXRESETSEL    = 0,
            i_RESETOVRD       = 0,

            # DRP
            i_DRPADDR         = drp_mux.addr,
            i_DRPCLK          = drp_mux.clk,
            i_DRPDI           = drp_mux.di,
            o_DRPDO           = drp_mux.do,
            i_DRPEN           = drp_mux.en,
            o_DRPRDY          = drp_mux.rdy,
            i_DRPWE           = drp_mux.we,

            # CPLL
            i_CPLLRESET       = 0,
            i_CPLLPD          = self.cpllreset,
            o_CPLLLOCK        = self.cplllock,
            i_CPLLLOCKEN      = 1,
            i_CPLLREFCLKSEL   = 0b111,
            i_GTGREFCLK       = self.refclk,

            # QPLL
            i_QPLL0CLK        = 0,
            i_QPLL0REFCLK     = 0,
            i_QPLL1CLK        = 0,
            i_QPLL1REFCLK     = 0,
            i_QPLL0FREQLOCK   = 0,
            i_QPLL1FREQLOCK   = 0,

            # TX clock
            o_TXOUTCLK        = self.txoutclk,
            i_TXSYSCLKSEL     = 0b00,
            i_TXPLLCLKSEL     = 0b00,
            i_TXOUTCLKSEL     = 0b010 if tx_buffer_enable else 0b101,

            # TX Startup/Reset
            i_GTTXRESET       = tx_init.gtXxreset,
            o_TXRESETDONE     = tx_init.Xxresetdone,
            i_TXDLYSRESET     = tx_init.Xxdlysreset,
            o_TXDLYSRESETDONE = tx_init.Xxdlysresetdone,
            o_TXPHALIGNDONE   = tx_init.Xxphaligndone,
            i_TXUSERRDY       = tx_init.Xxuserrdy,
            i_TXSYNCMODE      = 1,
            i_TXDLYBYPASS     = 1 if tx_buffer_enable else 0,
            i_TXPHDLYPD       = 1 if tx_buffer_enable else 0,

            # Transmit Ports - TX 8B/10B Encoder Ports
            i_TX8B10BBYPASS   = 0,

            # TX data
            i_TXCTRL0         = 0,
            i_TXCTRL1         = 0,
            i_TXCTRL2         = self.txcharisk,
            i_TXDATA          = self.txdata,
            i_TXUSRCLK        = self.txusrclk,
            i_TXUSRCLK2       = self.txusrclk2,

            # Power-Down Ports
            i_RXPD             = Replicate(self.rxpd, 2),
            i_TXPD             = Replicate(txpd, 2),

            # TX electrical
            i_TXDIFFCTRL      = 0b1100,
            i_TXINHIBIT       = 0,

            # Transmit Ports - PCI Express Ports
            i_TXPDELECIDLEMODE= 0,
            i_TXELECIDLE      = txelecidle,
            i_TXMARGIN        = 0b000,
            i_TXRATE          = 0b000,
            i_TXSWING         = 0,

            # Internal Loopback
            i_LOOPBACK        = 0b000,

            # RX Startup/Reset
            i_GTRXRESET       = rx_init.gtXxreset,
            o_RXRESETDONE     = rx_init.Xxresetdone,
            i_RXDLYSRESET     = rx_init.Xxdlysreset,
            o_RXDLYSRESETDONE = rx_init.Xxdlysresetdone,
            i_RXPMARESET      = 0,
            o_RXPHALIGNDONE   = rxphaligndone,
            i_RXSYNCALLIN     = rxphaligndone,
            i_RXUSERRDY       = rx_init.Xxuserrdy,
            i_RXSYNCIN        = 0,
            i_RXSYNCMODE      = 1,
            o_RXSYNCDONE      = rx_init.Xxsyncdone,
            i_RXDLYBYPASS     = 1 if rx_buffer_enable else 0,
            i_RXPHDLYPD       = 1 if rx_buffer_enable else 0,

            # RX AFE
            i_RXDFEXYDEN      = 1,
            i_RXLPMEN         = 1,

            # RX clock
            i_RXRATE          = 0,
            i_RXSYSCLKSEL     = 0b00,
            i_RXOUTCLKSEL     = 0b010,
            i_RXPLLCLKSEL     = 0b00,
            o_RXOUTCLK        = self.rxoutclk,
            i_RXUSRCLK        = self.rxusrclk,
            i_RXUSRCLK2       = self.rxusrclk2,

            # FPGA RX Interface Datapath Configuration
            i_RX8B10BEN       = 1,

            # Receive Ports - CDR Ports
            i_RXCDRFREQRESET  = 0,
            i_RXCDRHOLD       = self.rx_cdrhold,
            o_RXCDRLOCK       = Open(),
            i_RXCDROVRDEN     = 0,
            i_RXCDRRESET      = 0,
            i_RXOSCALRESET    = 0,
            o_RXOSINTDONE     = Open(),
            o_RXOSINTSTARTED  = Open(),
            o_RXOSINTSTROBEDONE    = Open(),
            o_RXOSINTSTROBESTARTED = Open(),

            # Receive Ports - RX Byte and Word Alignment Ports
            o_RXBYTEISALIGNED = Open(),
            o_RXBYTEREALIGN   = Open(),
            o_RXCOMMADET      = Open(),
            i_RXCOMMADETEN    = 1,
            i_RXMCOMMAALIGNEN = 1,
            i_RXPCOMMAALIGNEN = 1,
            i_RXSLIDE         = 0,

            o_GTPOWERGOOD     = Open(),

            # Receive Ports - RX OOB Signaling ports
            o_RXCOMSASDET     = Open(),
            o_RXCOMWAKEDET    = rxcomwakedet,
            o_RXCOMINITDET    = rxcominitdet,

            # Receive Ports - RX OOB signalling Ports
            o_RXELECIDLE      = Open(),
            i_RXELECIDLEMODE  = 0b00,

            # Transmit Ports - TX OOB signaling Ports
            o_TXCOMFINISH     = txcomfinish,
            i_TXCOMINIT       = txcominit,
            i_TXCOMSAS        = 0,
            i_TXCOMWAKE       = txcomwake,

            # FPGA TX Interface Datapath Configuration
            i_TX8B10BEN       = 1,

            # RX data
            o_RXCTRL0         = self.rxctrl0,
            o_RXCTRL1         = self.rxctrl1,
            o_RXCTRL2         = self.rxctrl2,
            o_RXCTRL3         = self.rxctrl3,
            o_RXDATA          = self.rxdata,

            # Polarity
            i_TXPOLARITY      = self.tx_polarity,
            i_RXPOLARITY      = self.rx_polarity,

            # Pads
            i_GTYRXP          = pads.rx_p,
            i_GTYRXN          = pads.rx_n,
            o_GTYTXP          = pads.tx_p,
            o_GTYTXN          = pads.tx_n
        )

        self.specials += Instance("GTYE4_CHANNEL", **self.gty_params)
