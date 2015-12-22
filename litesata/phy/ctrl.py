from litesata.common import *

from litex.gen.genlib.misc import WaitTimer


class LiteSATAPHYCtrl(Module):
    def __init__(self, trx, crg, clk_freq):
        self.clk_freq = clk_freq
        self.ready = Signal()
        self.sink = sink = Sink(phy_description(32))
        self.source = source = Source(phy_description(32))
        self.misalign = Signal()

        # # #

        self.comb += [
            source.stb.eq(1),
            sink.ack.eq(1)
        ]

        retry_timer = WaitTimer(self.us(10000))
        align_timer = WaitTimer(self.us(873))
        self.submodules += align_timer, retry_timer

        align_det = Signal()
        misalign_det = Signal()
        non_align_counter = Signal(4)
        non_align_counter_reset = Signal()
        non_align_counter_ce = Signal()
        self.sync += \
            If(non_align_counter_reset,
                non_align_counter.eq(0)
            ).Elif(non_align_counter_ce,
                non_align_counter.eq(non_align_counter + 1)
            )

        self.comb +=  [
            If(sink.stb,
                align_det.eq((self.sink.charisk == 0b0001) &
                             (self.sink.data == primitives["ALIGN"]))
            )
        ]

        self.fsm = fsm = ResetInserter()(FSM(reset_state="RESET"))
        self.submodules += fsm
        self.comb += fsm.reset.eq(retry_timer.done | align_timer.done)
        fsm.act("RESET",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            crg.rx_reset.eq(1),
            crg.tx_reset.eq(1),
            NextState("AWAIT_CRG_RESET")
        )
        fsm.act("AWAIT_CRG_RESET",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            non_align_counter_reset.eq(1),
            If(crg.ready,
                NextState("COMINIT")
            )
        )
        fsm.act("COMINIT",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            trx.tx_cominit_stb.eq(1),
            If(trx.tx_cominit_ack & ~trx.rx_cominit_stb,
                NextState("AWAIT_COMINIT")
            )
        )
        fsm.act("AWAIT_COMINIT",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            retry_timer.wait.eq(1),
            If(trx.rx_cominit_stb,
                NextState("AWAIT_NO_COMINIT")
            )
        )
        fsm.act("AWAIT_NO_COMINIT",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            retry_timer.wait.eq(1),
            If(~trx.rx_cominit_stb,
                NextState("CALIBRATE")
            )
        )
        fsm.act("CALIBRATE",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            NextState("COMWAKE"),
        )
        fsm.act("COMWAKE",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            trx.tx_comwake_stb.eq(1),
            If(trx.tx_comwake_ack,
                NextState("AWAIT_COMWAKE")
            )
        )
        fsm.act("AWAIT_COMWAKE",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            retry_timer.wait.eq(1),
            If(trx.rx_comwake_stb,
                NextState("AWAIT_NO_COMWAKE")
            )
        )
        fsm.act("AWAIT_NO_COMWAKE",
            trx.tx_idle.eq(1),
            trx.rx_cdrhold.eq(1),
            If(~trx.rx_comwake_stb,
                NextState("AWAIT_ALIGN")
            )
        )
        fsm.act("AWAIT_ALIGN",
            trx.rx_cdrhold.eq(1),
            source.data.eq(0x4A4A4A4A),  # D10.2
            source.charisk.eq(0b0000),
            align_timer.wait.eq(1),
            If(align_det & ~trx.rx_idle,
                crg.rx_reset.eq(1),
                NextState("SEND_ALIGN")
            )
        )
        fsm.act("SEND_ALIGN",
            align_timer.wait.eq(1),
            source.data.eq(primitives["ALIGN"]),
            source.charisk.eq(0b0001),
            If(sink.stb & 
			   (sink.charisk == 0b0001),
               If(sink.data[0:8] == 0x7C,
                   non_align_counter_ce.eq(1)
               ).Else(
                   non_align_counter_reset.eq(1)
               )
            ),
            If(non_align_counter == 3,
                NextState("READY")
            )
        )

        # wait alignement stability for 10ms before declaring ctrl is ready,
        # reset the RX part of the transceiver when misalignment is detected.
        stability_timer = WaitTimer(10*clk_freq//1000)
        self.submodules += stability_timer

        fsm.act("READY",
            source.data.eq(primitives["SYNC"]),
            source.charisk.eq(0b0001),
            stability_timer.wait.eq(1),
            self.ready.eq(stability_timer.done),
            If(trx.rx_idle,
                NextState("RESET"),
            ).Elif(self.misalign |
                   trx.rxdisperr |
                   trx.rxnotintable,
                crg.rx_reset.eq(1),
                NextState("RESET_RX")
            )
        )
        fsm.act("RESET_RX",
            If(crg.ready,
                NextState("READY")
            )
        )


    def us(self, t):
        clk_period_us = 1000000/self.clk_freq
        return ceil(t/clk_period_us)
