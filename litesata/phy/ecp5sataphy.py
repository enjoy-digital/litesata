#
# This file is part of LiteSATA.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *
from litesata.common import _PulseSynchronizer, _RisingEdge

from migen.genlib.cdc import MultiReg
from migen.genlib.resetsync import AsyncResetSynchronizer

from litesata.phy.serdes_ecp5 import SerDesECP5PLL, SerDesECP5

# --------------------------------------------------------------------------------------------------

class Pads:
    def __init__(self, p, n):
        self.p = p
        self.n = n

# --------------------------------------------------------------------------------------------------

class ECP5LiteSATAPHYCRG(Module):
    def __init__(self, serdes):
        self.tx_reset = Signal()
        self.rx_reset = Signal()

        self.clock_domains.cd_sata_tx = ClockDomain()
        self.clock_domains.cd_sata_rx = ClockDomain()

        # TX clocking ------------------------------------------------------------------------------
        #  (gen2) 150MHz from SerDes, sata_tx clk @ 150MHz (16-bits)
        #  (gen1)  75MHz from SerDes, sata_tx clk @ 75MHz  (16-bits)
        self.comb += self.cd_sata_tx.clk.eq(serdes.serdes.cd_tx.clk)

        # RX clocking ------------------------------------------------------------------------------
        #  (gen2) sata_rx recovered clk @ 150MHz (16-bits)
        #  (gen1) sata_rx recovered clk @ 75MHz  (16-bits)
        self.comb += self.cd_sata_rx.clk.eq(serdes.serdes.cd_rx.clk)

        # Reset for SATA TX/RX clock domains -------------------------------------------------------
        self.specials += AsyncResetSynchronizer(self.cd_sata_tx, ~serdes.serdes.tx_ready | self.tx_reset)
        self.specials += AsyncResetSynchronizer(self.cd_sata_rx, ~serdes.serdes.rx_ready | self.rx_reset)

# COMGenerator -------------------------------------------------------------------------------------

class COMGenerator(Module):
    def __init__(self, tx_clk_freq, cominit_stb=None, cominit_ack=None, comwake_stb=None, comwake_ack=None):
        assert tx_clk_freq == 150e6
        # Control
        self.cominit_stb = Signal() if cominit_stb is None else cominit_stb
        self.cominit_ack = Signal() if cominit_ack is None else cominit_ack
        self.comwake_stb = Signal() if comwake_stb is None else comwake_stb
        self.comwake_ack = Signal() if comwake_ack is None else comwake_ack

        # Transceiver
        self.sink    = sink   = stream.Endpoint([("data", 16), ("ctrl", 2)])
        self.source  = source = stream.Endpoint([("data", 16), ("ctrl", 2)])
        self.tx_idle = Signal()

        # # #

        count = Signal(16)
        loops = Signal(16)

        cominit = Signal()
        comwake = Signal()

        self.comb += [
            source.valid.eq(1),
            source.data.eq(0x4a4a),
            source.ctrl.eq(0b00),
        ]

        delay_burst = 64
        delay_idle  = 1

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.tx_idle.eq(1),
            sink.connect(source),
            NextValue(cominit, self.cominit_stb),
            NextValue(comwake, self.comwake_stb),
            NextValue(count,   delay_burst-1),
            NextValue(loops,   0),
            If(self.cominit_stb | self.comwake_stb,
                NextState("TX_BURST")
            ),
        )
        fsm.act("TX_BURST",
            NextValue(count, count - 1),
            If(count == 0,
                If(cominit,
                    NextValue(count, delay_idle-1),
                ),
                If(comwake,
                    NextValue(count, delay_idle-1),
                ),
                NextState("TX_IDLE")
            )
        )
        fsm.act("TX_IDLE",
            self.tx_idle.eq(1),
            NextValue(count, count - 1),
            If(count == 0,
                NextValue(loops, loops + 1),
                If(loops == 5,
                    self.cominit_ack.eq(cominit),
                    self.comwake_ack.eq(comwake),
                    NextState("IDLE")
                ).Else(
                    NextValue(count, delay_burst-1),
                    NextState("TX_BURST")
                )
            )
        )


# COMChecker ---------------------------------------------------------------------------------------

class COMChecker(Module):
    def __init__(self, rx_clk_freq, cominit_stb=None, comwake_stb=None):
        assert rx_clk_freq == 150e6
        # Status
        self.cominit_stb = Signal() if cominit_stb is None else cominit_stb
        self.comwake_stb = Signal() if comwake_stb is None else comwake_stb

        # Transceiver
        self.rx_idle = Signal()

        # # #

        rx_idle = Signal()
        self.specials += MultiReg(self.rx_idle, rx_idle)

        self.count = count = Signal(8)
        self.loops = loops = Signal(8)

        self.cominit = cominit = Signal()
        self.comwake = comwake = Signal()

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            NextValue(count,   0),
            NextValue(loops,   0),
            NextValue(cominit, 0),
            NextValue(comwake, 0),
            If(rx_idle,
                NextState("RX_IDLE")
            ),
        )
        fsm.act("RX_IDLE",
            NextValue(count, count + 1),
            If(~rx_idle,
                If((count > 10) & (count < 20), # FIXME: refine.
                    If(cominit,
                        NextState("IDLE")
                    ).Else(
                        NextValue(comwake, 1),
                        NextValue(loops, loops + 1),
                        If(loops == 3,
                            NextValue(count, 32*4),
                            NextState("WAIT")
                        ).Else(
                            NextState("RX_BURST")
                        )
                    )
                ).Elif((count > 40) & (count < 60), # FIXME: refine.
                    If(comwake,
                        NextState("IDLE")
                    ).Else(
                        NextValue(cominit, 1),
                        NextValue(loops, loops + 1),
                        If(loops == 3,
                            NextValue(count, 32*4),
                            NextState("WAIT")
                        ).Else(
                            NextState("RX_BURST")
                        )
                    )
                )
            )
        )
        fsm.act("RX_BURST",
            NextValue(count, 0),
            If(rx_idle,
                NextState("RX_IDLE")
            )
        )
        fsm.act("WAIT",
            NextValue(count, count - 1),
            If(count == 0,
                NextState("END")
            )
        )
        fsm.act("END",
            self.cominit_stb.eq(cominit),
            self.comwake_stb.eq(comwake),
            NextState("IDLE")
        )

# --------------------------------------------------------------------------------------------------

class ECP5LiteSATAPHY(Module):
    def __init__(self, refclk, pads, gen, clk_freq, data_width=16, channel=0):
        assert data_width in [16]
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

        self.rxdisperr      = Signal(data_width//8) # o
        self.rxnotintable   = Signal(data_width//8) # o

        # Datapath
        self.sink           = stream.Endpoint(phy_description(data_width))
        self.source         = stream.Endpoint(phy_description(data_width))

        # Receive Ports - 8b10b Decoder
        self.rxcharisk      = Signal(data_width//8)

        # Receive Ports - RX Data Path interface
        self.rxdata         = Signal(data_width)

        # Transmit Ports - RX Ports for PCI Express
        self.rxelecidle     = Signal(reset=1)

        # Receive Ports - RX Ports for SATA
        self.rxcominitdet   = Signal()
        self.rxcomwakedet   = Signal()

        # Transmit Ports - 8b10b Encoder Control Ports
        self.txcharisk      = Signal(data_width//8)

        # Transmit Ports - TX Data Path interface
        self.txdata         = Signal(data_width)

        # Transmit Ports - TX Ports for PCI Express
        self.txelecidle     = Signal(reset=1)

        # Transmit Ports - TX Ports for SATA
        self.txcomfinish    = Signal()
        self.txcominit      = Signal()
        self.txcomwake      = Signal()

        # Power-down signals
        self.cpllpd         = Signal()
        self.rxpd           = Signal()
        self.txpd           = Signal()

        # SerDes RefClk ----------------------------------------------------------------------------
        if isinstance(refclk, (Signal, ClockSignal)):
            pass
        else:
            refclk = Signal()
            self.specials.extref0 = Instance("EXTREFB",
                i_REFCLKP     = refclk_pads.p,
                i_REFCLKN     = refclk_pads.n,
                o_REFCLKO     = refclk,
                p_REFCK_PWDNB = "0b1",
                p_REFCK_RTERM = "0b1", # 100 Ohm
            )
            self.extref0.attr.add(("LOC", "EXTREF0"))

        # Serdes PLL -------------------------------------------------------------------------------
        serdes_pll = SerDesECP5PLL(refclk, refclk_freq=150e6, linerate={"gen2": 3e9, "gen1": 1.5e9}[gen])
        self.submodules += serdes_pll


        # OOB --------------------------------------------------------------------------------------
        com_gen = COMGenerator(tx_clk_freq=150e6,
            #cominit_stb = self.tx_cominit_stb,
            #cominit_ack = self.tx_cominit_ack,
            #comwake_stb = self.tx_comwake_stb,
            #comwake_ack = self.tx_comwake_ack,
            cominit_stb = Signal(reset=1),
            cominit_ack = Signal(),
            comwake_stb = Signal(),
            comwake_ack = Signal()
        )
        com_gen = ClockDomainsRenamer("tx")(com_gen)
        self.submodules.com_gen = com_gen

        com_check = COMChecker(rx_clk_freq=150e6,
            cominit_stb = self.rx_cominit_stb,
            comwake_stb = self.rx_comwake_stb
        )
        com_check = ClockDomainsRenamer("rx")(com_check)
        self.submodules.com_check = com_check

        # SerDes -----------------------------------------------------------------------------------
        self.submodules.serdes = serdes = SerDesECP5(
            pll         = serdes_pll,
            tx_pads     = Pads(p=pads.tx_p, n=pads.tx_n),
            rx_pads     = Pads(p=pads.rx_p, n=pads.rx_n),
            channel     = channel,
            tx_polarity = self.tx_polarity,
            rx_polarity = self.rx_polarity,
        )
        serdes.add_stream_endpoints()
        self.comb += [
            # RX
            serdes.source.ready.eq(1),
            self.rxdata.eq(serdes.source.ctrl),
            self.rxcharisk.eq(serdes.source.data),
            serdes.rx_cdr_hold.eq(self.rx_cdrhold),

            # TX
            com_gen.sink.valid.eq(1),
            com_gen.sink.ctrl.eq(self.txcharisk),
            com_gen.sink.data.eq(self.txdata),
            com_gen.source.connect(serdes.sink),

            # Electrical
            serdes.tx_idle.eq(self.txelecidle),
            self.rxelecidle.eq(serdes.rx_idle),
        ]

        # Ready ------------------------------------------------------------------------------------
        self.comb += self.ready.eq(serdes.init.tx_ready & serdes.init.rx_ready)

        # Specific / Generic signals encoding/decoding ---------------------------------------------
        self.comb += [
            #self.txelecidle.eq(self.tx_idle | self.txpd | self.com_gen.tx_idle),
            self.txelecidle.eq(self.com_gen.tx_idle),
            com_check.rx_idle.eq(self.rxelecidle),
        ]

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
            MultiReg(self.rxelecidle, self.rx_idle,   "sys"),
            MultiReg(rxcominitdet, self.rxcominitdet, "sys"),
            MultiReg(rxcomwakedet, self.rxcomwakedet, "sys"),
            MultiReg(rxdisperr,    self.rxdisperr,    "sys"),
            MultiReg(rxnotintable, self.rxnotintable, "sys")
        ]
