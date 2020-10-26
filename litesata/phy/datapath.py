#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *

from migen.genlib.misc import WaitTimer

# LiteSATAPHYDatapathRX ----------------------------------------------------------------------------

class LiteSATAPHYDatapathRX(Module):
    """SATA PHY RX datapath

    Manages the RX datapath between the transceiver and the SATA core.

    This modules receives data/special characters from the transceiver and needs
    to:
    - realign received data on 32 bits word boundaries.
    - change clock domain from "sata_rx" to "sys".

    It handles both 32 bits and 16 bits transceiver data width. The realignment is
    done by storing two complete words from the transceiver, detecting the byte
    alignement with the special character and use this information to select the
    data we will output.
    """
    def __init__(self, data_width):
        self.sink   = sink   = stream.Endpoint(phy_description(data_width))
        self.source = source = stream.Endpoint(phy_description(32))

        # # #

        # width convertion and byte alignment
        byte_alignment = Signal(data_width//8)
        last_charisk   = Signal(data_width//8)
        last_data      = Signal(data_width)
        sr_charisk     = Signal(2*data_width//8)
        sr_data        = Signal(2*data_width)
        self.sync.sata_rx += \
            If(sink.valid & sink.ready,
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


        converter = StrideConverter(phy_description(data_width),
                                    phy_description(32),
                                    reverse=False)
        if data_width == 16: # when data_width=32, converter is just direct connection
            converter = ResetInserter()(ClockDomainsRenamer("sata_rx")(converter))
        self.submodules += converter
        cases = {}
        for i in range(data_width//8):
            cases[2**i] = [
                converter.sink.charisk.eq(sr_charisk[data_width//8-i:2*data_width//8-i]),
                converter.sink.data.eq(sr_data[data_width-8*i:2*data_width-8*i])
            ]
        self.comb += [
            converter.sink.valid.eq(sink.valid),
            Case(byte_alignment, cases),
            sink.ready.eq(converter.sink.ready)
        ]
        if data_width == 16:
            self.comb += converter.reset.eq(converter.source.charisk[2:] != 0)

        # clock domain crossing
        #   (gen3) 300MHz (16 bits) / 150MHz (32 bits) sata_rx clk to sys_clk
        #   (gen2) 150MHz (16 bits) / 75MHz (32 bits) sata_rx clk to sys_clk
        #   (gen1) 75MHz (16 bits) / 37.5MHz (32 bits) sata_rx clk to sys_clk
        #   requirements:
        #     due to the convertion ratio of 2, sys_clk need to be > sata_rx/2
        #     source destination is always able to accept data (ready always 1)
        fifo = stream.AsyncFIFO(phy_description(32), 8)
        fifo = ClockDomainsRenamer({"write": "sata_rx", "read": "sys"})(fifo)
        self.submodules += fifo
        self.comb += [
            converter.source.connect(fifo.sink),
            fifo.source.connect(source)
        ]

# LiteSATAPHYDatapathTX ----------------------------------------------------------------------------

class LiteSATAPHYDatapathTX(Module):
    """SATA PHY TX Datapath

    Manages the TX Datapath between SATA core and the transceiver.

    This modules receives datas / special characters from the core and needs
    to:
    - convert 32 bits data to the transceiver data width (32 bits or 16 bits)
    - change clock domain from "sys" to "sata_rx".
    """
    def __init__(self, data_width):
        self.sink   = sink   = stream.Endpoint(phy_description(32))
        self.source = source = stream.Endpoint(phy_description(data_width))

        # # #

        # clock domain crossing
        #   (gen3) sys_clk to 300MHz (16 bits) / 150MHz (32 bits) sata_tx clk
        #   (gen2) sys_clk to 150MHz (16 bits) / 75MHz (32 bits) sata_tx clk
        #   (gen1) sys_clk to 75MHz (16 bits) / 37.5MHz (32 bits) sata_tx clk
        #   requirements:
        #     source destination is always able to accept data (ready always 1)
        fifo = stream.AsyncFIFO(phy_description(32), 8)
        fifo = ClockDomainsRenamer({"write": "sys", "read": "sata_tx"})(fifo)
        self.submodules += fifo
        self.comb += sink.connect(fifo.sink)

        # width convertion
        converter = StrideConverter(phy_description(32),
                                    phy_description(data_width),
                                    reverse=False)
        converter = ClockDomainsRenamer("sata_tx")(converter)
        self.submodules += converter
        self.comb += [
            fifo.source.connect(converter.sink),
            converter.source.connect(source)
        ]

# LiteSATAPHYAlignTimer ----------------------------------------------------------------------------

class LiteSATAPHYAlignTimer(Module):
    """SATA PHY align timer

    This modules detects ALIGN primitives that we receive from the device and
    decide whether or not our device is returning valid data (Using RX idle
    signal from the transceiver is not recommended by vendors).
    """
    def __init__(self):
        self.sink = sink = stream.Endpoint(phy_description(32))

        # # #

        self.submodules.timer = WaitTimer(256*16)

        charisk_match = sink.charisk == 0b0001
        data_match    = sink.data == primitives["ALIGN"]

        self.comb += \
            If(sink.valid &
              (sink.charisk == 0b0001) &
              (sink.data == primitives["ALIGN"]),
                self.timer.wait.eq(0)
            ).Else(
                self.timer.wait.eq(1),
            )

# LiteSATAPHYDatapath ------------------------------------------------------------------------------

class LiteSATAPHYDatapath(Module):
    """SATA PHY datapath

    Manages the datapath between the core and the transceiver.

    This modules does mainly data width convertion (core is always 32 bits,
    transceiver can either be 16 bits or 32 bits depending on the FPGA speedgrade)
    and the cross domain crossing:
    - "sys" to "sata_tx" for the TX datapath.
    - "sata_rx" to "sys" for the RX datapath.

    Misalign and Idle signals are also generated to have a state of the link and
    are used by the SATA PHY controller

    """
    def __init__(self, trx, ctrl):
        self.sink   = sink   = stream.Endpoint(phy_description(32))
        self.source = source = stream.Endpoint(phy_description(32))

        self.misalign = Signal()
        self.rx_idle  = Signal()

        # # #

        # TX path
        mux = Multiplexer(phy_description(32), 2)
        tx  = LiteSATAPHYDatapathTX(trx.data_width)
        self.submodules += mux, tx
        self.comb += [
            mux.sel.eq(ctrl.ready),
            ctrl.source.connect(mux.sink0),
            sink.connect(mux.sink1),
            mux.source.connect(tx.sink),
            tx.source.connect(trx.sink)
        ]

        # RX path
        rx    = LiteSATAPHYDatapathRX(trx.data_width)
        demux = Demultiplexer(phy_description(32), 2)
        align_timer = LiteSATAPHYAlignTimer()
        self.submodules += rx, demux, align_timer
        self.comb += [
            demux.sel.eq(ctrl.ready),
            trx.source.connect(rx.sink),
            rx.source.connect(demux.sink),
            rx.source.connect(align_timer.sink, omit=set(["ready"])),
            demux.source0.connect(ctrl.sink),
            demux.source1.connect(source),
        ]

        self.comb += [
            self.misalign.eq(rx.source.valid & ((rx.source.charisk & 0b1110) != 0)),
            self.rx_idle.eq(align_timer.timer.done)
        ]
