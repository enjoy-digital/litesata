from litesata.common import *

from litex.gen.genlib.cdc import MultiReg
from litex.gen.genlib.resetsync import AsyncResetSynchronizer
from litex.gen.genlib.misc import WaitTimer


class K7LiteSATAPHYCRG(Module):
    def __init__(self, clock_pads_or_refclk, pads, gtx, revision, clk_freq):
        self.tx_reset = Signal()
        self.rx_reset = Signal()
        self.ready = Signal()
        self.cplllock = Signal()

        self.clock_domains.cd_sata_tx = ClockDomain()
        self.clock_domains.cd_sata_rx = ClockDomain()

        # CPLL
        #   (sata_gen3) 150MHz / VCO @ 3GHz / Line rate @ 6Gbps
        #   (sata_gen2 & sata_gen1) VCO still @ 3 GHz, Line rate is
        #   decreased with output dividers.
        if isinstance(clock_pads_or_refclk, Signal):
            self.refclk = clock_pads_or_refclk
        else:
            self.refclk = Signal()
            clock_pads = clock_pads_or_refclk
            self.specials += Instance("IBUFDS_GTE2",
                i_CEB=0,
                i_I=clock_pads.refclk_p,
                i_IB=clock_pads.refclk_n,
                o_O=self.refclk
            )

        self.comb += gtx.gtrefclk0.eq(self.refclk)

        # TX clocking
        #   (sata_gen3) 150MHz from CPLL TXOUTCLK, sata_tx clk @ 300MHz (16-bits) /  150MHz (32-bits)
        #   (sata_gen2) 150MHz from CPLL TXOUTCLK, sata_tx clk @ 150MHz (16-bits) /   75MHz (32-bits)
        #   (sata_gen1) 150MHz from CPLL TXOUTCLK, sata_tx clk @ 75MHz  (16-bits) / 37.5MHz (32-bits)
        mmcm_mult = 8.0
        mmcm_div_config = {
            "sata_gen1":   16.0*gtx.dw/16,
            "sata_gen2":    8.0*gtx.dw/16,
            "sata_gen3":    4.0*gtx.dw/16
        }
        mmcm_div = mmcm_div_config[revision]
        use_mmcm = mmcm_mult/mmcm_div != 1.0

        if use_mmcm:
            mmcm_reset = Signal()
            mmcm_locked_async = Signal()
            mmcm_locked = Signal()
            mmcm_fb = Signal()
            mmcm_clk_i = Signal()
            mmcm_clk0_o = Signal()
            self.specials += [
                Instance("BUFG", i_I=gtx.txoutclk, o_O=mmcm_clk_i),
                Instance("MMCME2_ADV",
                     p_BANDWIDTH="HIGH", p_COMPENSATION="ZHOLD", i_RST=mmcm_reset, o_LOCKED=mmcm_locked_async,

                     # DRP
                     i_DCLK=0, i_DEN=0, i_DWE=0, #o_DRDY=,
                     i_DADDR=0, i_DI=0, #o_DO=,

                     # VCO
                     p_REF_JITTER1=0.01, p_CLKIN1_PERIOD=6.66667,
                     p_CLKFBOUT_MULT_F=mmcm_mult, p_CLKFBOUT_PHASE=0.000, p_DIVCLK_DIVIDE=1,
                     i_CLKIN1=mmcm_clk_i, i_CLKFBIN=mmcm_fb, o_CLKFBOUT=mmcm_fb,

                     # CLK0
                     p_CLKOUT0_DIVIDE_F=mmcm_div, p_CLKOUT0_PHASE=0.000, o_CLKOUT0=mmcm_clk0_o,
                ),
                Instance("BUFG", i_I=mmcm_clk0_o, o_O=self.cd_sata_tx.clk),
                MultiReg(mmcm_locked_async, mmcm_locked, "sys"),
            ]
        else:
            mmcm_locked = Signal(reset=1)
            mmcm_reset = Signal()
            self.specials += Instance("BUFG", i_I=gtx.txoutclk, o_O=self.cd_sata_tx.clk)

        self.comb += [
            gtx.txusrclk.eq(self.cd_sata_tx.clk),
            gtx.txusrclk2.eq(self.cd_sata_tx.clk)
        ]

        # RX clocking
        #   (sata_gen3) sata_rx recovered clk @  @ 300MHz (16-bits) /  150MHz (32-bits) from GTX RXOUTCLK
        #   (sata_gen2) sata_rx recovered clk @  @ 150MHz (16-bits) /   75MHz (32-bits) from GTX RXOUTCLK
        #   (sata_gen1) sata_rx recovered clk @  @ 75MHz  (16-bits) / 37.5MHz (32-bits) from GTX RXOUTCLK
        self.specials += [
            Instance("BUFG", i_I=gtx.rxoutclk, o_O=self.cd_sata_rx.clk),
        ]
        self.comb += [
            gtx.rxusrclk.eq(self.cd_sata_rx.clk),
            gtx.rxusrclk2.eq(self.cd_sata_rx.clk)
        ]

        # Configuration Reset
        #   After configuration, GTX's resets have to stay low for at least 500ns
        #   See AR43482
        startup_cycles = ceil(500*clk_freq/1000000000)
        startup_timer = WaitTimer(startup_cycles)
        self.submodules += startup_timer
        self.comb += startup_timer.wait.eq(~(self.tx_reset | self.rx_reset))

        # TX Startup FSM
        self.tx_ready = Signal() 
        self.gttxreset = Signal()
        self.cpllreset = Signal()
        self.txuserrdy = Signal()
        self.tx_startup_fsm = tx_startup_fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.submodules += tx_startup_fsm

        txphaligndone = Signal(reset=1)
        txphaligndone_rising = Signal()
        self.sync += txphaligndone.eq(gtx.txphaligndone)
        self.sync += gtx.gttxreset.eq(self.gttxreset)
        self.sync += gtx.cpllreset.eq(self.cpllreset)
        self.sync += gtx.txuserrdy.eq(self.txuserrdy)
        self.comb += txphaligndone_rising.eq(gtx.txphaligndone & ~txphaligndone)

        # Wait 500ns of AR43482
        tx_startup_fsm.act("IDLE",
            If(startup_timer.done,
                NextState("RESET_ALL")
            )
        )
        # Reset CPLL, MMCM, GTX
        tx_startup_fsm.act("RESET_ALL",
            self.cpllreset.eq(1),
            mmcm_reset.eq(1),
            self.gttxreset.eq(1),
            If(~self.cplllock,
               NextState("RELEASE_CPLL")
            )
        )
        # Release CPLL reset and wait for lock
        tx_startup_fsm.act("RELEASE_CPLL",
            mmcm_reset.eq(1),
            self.gttxreset.eq(1),
            If(self.cplllock,
                NextState("RELEASE_MMCM")
            )
        )
        # Release MMCM reset and wait for lock
        tx_startup_fsm.act("RELEASE_MMCM",
            self.gttxreset.eq(1),
            If(mmcm_locked,
                NextState("RELEASE_GTX")
            )
        )
        # Release GTX reset and wait for GTX resetdone
        # (from UG476, GTX is reseted on falling edge
        # of gttxreset)
        tx_startup_fsm.act("RELEASE_GTX",
            self.txuserrdy.eq(1),
            If(gtx.txresetdone,
                NextState("ALIGN")
            )
        )
        # Start Delay alignment (Pulse)
        tx_startup_fsm.act("ALIGN",
            self.txuserrdy.eq(1),
            gtx.txdlyreset.eq(1),
            NextState("WAIT_ALIGN")
        )
        # Wait Delay alignment
        tx_startup_fsm.act("WAIT_ALIGN",
            self.txuserrdy.eq(1),
            If(gtx.txdlyresetdone,
                NextState("WAIT_FIRST_ALIGN_DONE")
            )
        )
        # Wait 2 rising edges of txphaligndone
        # (from UG476 in buffer bypass config)
        tx_startup_fsm.act("WAIT_FIRST_ALIGN_DONE",
            self.txuserrdy.eq(1),
            If(txphaligndone_rising,
               NextState("WAIT_SECOND_ALIGN_DONE")
            )
        )
        tx_startup_fsm.act("WAIT_SECOND_ALIGN_DONE",
            self.txuserrdy.eq(1),
            If(txphaligndone_rising,
               NextState("READY")
            )
        )
        tx_startup_fsm.act("READY",
            self.txuserrdy.eq(1),
            self.tx_ready.eq(1)
        )

        tx_ready_timer = WaitTimer(2*clk_freq//1000)
        self.submodules += tx_ready_timer
        self.comb += [
            tx_ready_timer.wait.eq(~self.tx_ready & ~tx_startup_fsm.reset),
            tx_startup_fsm.reset.eq(self.tx_reset | tx_ready_timer.done),
        ]


        # RX Startup FSM
        self.rx_ready = Signal() 
        self.gtrxreset = Signal()
        self.rxuserrdy = Signal()
        self.rx_startup_fsm = rx_startup_fsm = ResetInserter()(FSM(reset_state="IDLE"))
        self.submodules += rx_startup_fsm

        cdr_stable_timer = WaitTimer(1024)
        self.submodules += cdr_stable_timer

        rxphaligndone = Signal(reset=1)
        rxphaligndone_rising = Signal()
        self.sync += rxphaligndone.eq(gtx.rxphaligndone)
        self.sync += gtx.gtrxreset.eq(self.gtrxreset)
        self.sync += gtx.rxuserrdy.eq(self.rxuserrdy)
        self.comb += rxphaligndone_rising.eq(gtx.rxphaligndone & ~rxphaligndone)

        # Wait 500ns of AR43482
        rx_startup_fsm.act("IDLE",
            If(startup_timer.done,
                NextState("RESET_GTX")
            )
        )
        # Reset GTX
        rx_startup_fsm.act("RESET_GTX",
            self.gtrxreset.eq(1),
            If(~self.gttxreset,
               NextState("WAIT_CPLL")
            )
        )
        # Wait for CPLL lock
        rx_startup_fsm.act("WAIT_CPLL",
            self.gtrxreset.eq(1),
            If(self.cplllock,
                NextState("RELEASE_GTX")
            )
        )
        # Release GTX reset and wait for GTX resetdone
        # (from UG476, GTX is reseted on falling edge
        # of gttxreset)
        rx_startup_fsm.act("RELEASE_GTX",
            self.rxuserrdy.eq(1),
            cdr_stable_timer.wait.eq(1),
            If(gtx.rxresetdone &  cdr_stable_timer.done,
                NextState("ALIGN")
            )
        )
        # Start Delay alignment (Pulse)
        rx_startup_fsm.act("ALIGN",
            self.rxuserrdy.eq(1),
            gtx.rxdlyreset.eq(1),
            NextState("WAIT_ALIGN")
        )
        # Wait Delay alignment
        rx_startup_fsm.act("WAIT_ALIGN",
            self.rxuserrdy.eq(1),
            If(gtx.rxdlyresetdone,
                NextState("WAIT_FIRST_ALIGN_DONE")
            )
        )
        # Wait 2 rising edges of rxphaligndone
        # (from UG476 in buffer bypass config)
        rx_startup_fsm.act("WAIT_FIRST_ALIGN_DONE",
            self.rxuserrdy.eq(1),
            If(rxphaligndone_rising,
               NextState("WAIT_SECOND_ALIGN_DONE")
            )
        )
        rx_startup_fsm.act("WAIT_SECOND_ALIGN_DONE",
            self.rxuserrdy.eq(1),
            If(rxphaligndone_rising,
               NextState("READY")
            )
        )
        rx_startup_fsm.act("READY",
            self.rxuserrdy.eq(1),
            self.rx_ready.eq(1)
        )

        rx_ready_timer = WaitTimer(2*clk_freq//1000)
        self.submodules += rx_ready_timer
        self.comb += [
            rx_ready_timer.wait.eq(~self.rx_ready & ~rx_startup_fsm.reset),
            rx_startup_fsm.reset.eq(self.rx_reset | rx_ready_timer.done),
        ]

        # Ready
        self.comb += self.ready.eq(self.tx_ready & self.rx_ready)

        # Reset for SATA TX/RX clock domains
        self.specials += [
            AsyncResetSynchronizer(self.cd_sata_tx, ~(gtx.cplllock & mmcm_locked) | self.tx_reset),
            AsyncResetSynchronizer(self.cd_sata_rx, ~gtx.cplllock | self.rx_reset),
            MultiReg(gtx.cplllock, self.cplllock, "sys"),
        ]
