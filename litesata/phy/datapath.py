from litesata.common import *


class LiteSATAPHYDatapathRX(Module):
    def __init__(self, trx_dw):
        self.sink = sink = stream.Endpoint(phy_description(trx_dw))
        self.source = source = stream.Endpoint(phy_description(32))

        # # #

        # width convertion and byte alignment
        byte_alignment = Signal(trx_dw//8)
        last_charisk = Signal(trx_dw//8)
        last_data = Signal(trx_dw)
        sr_charisk = Signal(2*trx_dw//8)
        sr_data = Signal(2*trx_dw)
        self.sync.sata_rx += \
            If(sink.stb & sink.ack,
                If(sink.charisk != 0,
                    byte_alignment.eq(sink.charisk)
                ),
                last_charisk.eq(sink.charisk),
                last_data.eq(sink.data)
            )
        self.comb += [
            sr_charisk.eq(Cat(last_charisk, sink.charisk)),
            sr_data.eq(Cat(last_data, sink.data)),
        ]


        converter = Converter(phy_description(trx_dw),
                              phy_description(32),
                              reverse=False)
        if trx_dw == 16: # when trx_dw=32, converter is just direct connection
            converter = ResetInserter()(ClockDomainsRenamer("sata_rx")(converter))
        self.submodules += converter
        cases = {}
        for i in range(trx_dw//8):
            cases[2**i] = [
                converter.sink.charisk.eq(sr_charisk[trx_dw//8-i:2*trx_dw//8-i]),
                converter.sink.data.eq(sr_data[trx_dw-8*i:2*trx_dw-8*i])
            ]
        self.comb += [
            converter.sink.stb.eq(sink.stb),
            Case(byte_alignment, cases),
            sink.ack.eq(converter.sink.ack)
        ]
        if trx_dw == 16:
            self.comb += converter.reset.eq(converter.source.charisk[2:] != 0)

        # clock domain crossing
        #   (sata_gen3) 300MHz (16 bits) / 150MHz (32 bits) sata_rx clk to sys_clk
        #   (sata_gen2) 150MHz (16 bits) / 75MHz (32 bits) sata_rx clk to sys_clk
        #   (sata_gen1) 75MHz (16 bits) / 37.5MHz (32 bits) sata_rx clk to sys_clk
        #   requirements:
        #     due to the convertion ratio of 2, sys_clk need to be > sata_rx/2
        #     source destination is always able to accept data (ack always 1)
        fifo = stream.AsyncFIFO(phy_description(32), 8)
        fifo = ClockDomainsRenamer({"write": "sata_rx", "read": "sys"})(fifo)
        self.submodules += fifo
        self.comb += [
            converter.source.connect(fifo.sink),
            fifo.source.connect(source)
        ]


class LiteSATAPHYDatapathTX(Module):
    def __init__(self, trx_dw):
        self.sink = sink = stream.Endpoint(phy_description(32))
        self.source = source = stream.Endpoint(phy_description(trx_dw))

        # # #

        # clock domain crossing
        #   (sata_gen3) sys_clk to 300MHz (16 bits) / 150MHz (32 bits) sata_tx clk
        #   (sata_gen2) sys_clk to 150MHz (16 bits) / 75MHz (32 bits) sata_tx clk
        #   (sata_gen1) sys_clk to 75MHz (16 bits) / 37.5MHz (32 bits) sata_tx clk
        #   requirements:
        #     source destination is always able to accept data (ack always 1)
        fifo = stream.AsyncFIFO(phy_description(32), 8)
        fifo = ClockDomainsRenamer({"write": "sys", "read": "sata_tx"})(fifo)
        self.submodules += fifo
        self.comb += sink.connect(fifo.sink)

        # width convertion
        converter = Converter(phy_description(32),
                              phy_description(trx_dw),
                              reverse=False)
        converter = ClockDomainsRenamer("sata_tx")(converter)
        self.submodules += converter
        self.comb += [
            fifo.source.connect(converter.sink),
            converter.source.connect(source)
        ]


class LiteSATAPHYDatapath(Module):
    def __init__(self, trx, ctrl):
        self.sink = sink = stream.Endpoint(phy_description(32))
        self.source = source = stream.Endpoint(phy_description(32))

        self.misalign = Signal()

        # # #

        # TX path
        mux = Multiplexer(phy_description(32), 2)
        tx = LiteSATAPHYDatapathTX(trx.dw)
        self.submodules += mux, tx
        self.comb += [
            mux.sel.eq(ctrl.ready),
            ctrl.source.connect(mux.sink0),
            sink.connect(mux.sink1),
            mux.source.connect(tx.sink),
            tx.source.connect(trx.sink)
        ]

        # RX path
        rx = LiteSATAPHYDatapathRX(trx.dw)
        demux = Demultiplexer(phy_description(32), 2)
        self.submodules += rx, demux
        self.comb += [
            demux.sel.eq(ctrl.ready),
            trx.source.connect(rx.sink),
            rx.source.connect(demux.sink),
            demux.source0.connect(ctrl.sink),
            demux.source1.connect(source)
        ]

        self.comb += self.misalign.eq(rx.source.stb & ((rx.source.charisk & 0b1110) != 0))
