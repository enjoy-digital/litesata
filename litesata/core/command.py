#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2016 Olof Kindgren <olof.kindgren@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *

# Layouts ------------------------------------------------------------------------------------------

tx_to_rx = [
    ("write",    1),
    ("read",     1),
    ("identify", 1),
    ("count",    16)
]

rx_to_tx = [
    ("dma_activate", 1),
    ("d2h_error",    1)
]

# LiteSATA Command TX ------------------------------------------------------------------------------

class LiteSATACommandTX(Module):
    def __init__(self, transport):
        self.sink    = sink    = stream.Endpoint(command_tx_description(32))
        self.to_rx   = to_rx   = stream.Endpoint(tx_to_rx)
        self.from_rx = from_rx = stream.Endpoint(rx_to_tx)

        # # #

        self.comb += [
            transport.sink.pm_port.eq(0),
            transport.sink.features.eq(0),
            transport.sink.lba.eq(sink.sector),
            transport.sink.device.eq(0xe0),
            transport.sink.count.eq(sink.count),
            transport.sink.icc.eq(0),
            transport.sink.control.eq(0),
            transport.sink.data.eq(sink.data)
        ]

        is_write       = Signal()
        is_read        = Signal()
        is_identify    = Signal()
        dwords_counter = Signal(max=fis_max_dwords)

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            sink.ready.eq(0),
            If(sink.valid,
                NextState("SEND_CMD")
            ).Else(
                sink.ready.eq(1)
            )
        )
        self.sync += \
            If(fsm.ongoing("IDLE"),
                is_write.eq(sink.write),
                is_read.eq(sink.read),
                is_identify.eq(sink.identify),
            )

        fsm.act("SEND_CMD",
            transport.sink.valid.eq(sink.valid),
            transport.sink.last.eq(1),
            transport.sink.c.eq(1),
            If(transport.sink.valid & transport.sink.ready,
                If(is_write,
                    NextState("WAIT_DMA_ACTIVATE")
                ).Else(
                    sink.ready.eq(1),
                    NextState("IDLE")
                )
            )
        )
        fsm.act("WAIT_DMA_ACTIVATE",
            NextValue(dwords_counter, 0),
            If(from_rx.dma_activate,
                NextState("SEND_DATA")
            ).Elif(from_rx.d2h_error,
                sink.ready.eq(1),
                NextState("IDLE")
            )
        )
        fsm.act("SEND_DATA",
            transport.sink.valid.eq(sink.valid),
            transport.sink.last.eq((dwords_counter == (fis_max_dwords-1)) | sink.last),
            sink.ready.eq(transport.sink.ready),
            If(sink.valid & sink.ready,
                NextValue(dwords_counter, dwords_counter + 1),
                If(sink.last,
                    NextState("IDLE")
                ).Elif(dwords_counter == (fis_max_dwords-1),
                    NextState("WAIT_DMA_ACTIVATE")
                )
            )
        )
        self.comb += \
            If(fsm.ongoing("SEND_DATA"),
                transport.sink.type.eq(fis_types["DATA"]),
            ).Else(
                transport.sink.type.eq(fis_types["REG_H2D"]),
                If(is_write,
                    transport.sink.command.eq(regs["WRITE_DMA_EXT"])
                ).Elif(is_read,
                    transport.sink.command.eq(regs["READ_DMA_EXT"]),
                ).Else(
                    transport.sink.command.eq(regs["IDENTIFY_DEVICE"]),
                )
            )
        self.comb += [
            If(sink.valid,
                to_rx.write.eq(sink.write),
                to_rx.read.eq(sink.read),
                to_rx.identify.eq(sink.identify),
                to_rx.count.eq(sink.count)
            )
        ]

# LiteSATA Command RX ------------------------------------------------------------------------------

class LiteSATACommandRX(Module):
    def __init__(self, transport):
        self.source  = source  = stream.Endpoint(command_rx_description(32))
        self.to_tx   = to_tx   = stream.Endpoint(rx_to_tx)
        self.from_tx = from_tx = stream.Endpoint(tx_to_rx)

        # Debug
        self.d2h_status = Signal(8)
        self.d2h_errors = Signal(8)

        # # #

        def test_type(name):
            return transport.source.type == fis_types[name]

        is_identify     = Signal()
        is_dma_activate = Signal()
        read_ndwords    = Signal(max=sectors2dwords(2**16))
        dwords_counter  = Signal(max=sectors2dwords(2**16))
        read_done       = Signal()

        self.sync += \
            If(from_tx.read,
                read_ndwords.eq(from_tx.count*sectors2dwords(1) - 1)
            )
        self.comb += read_done.eq(dwords_counter == read_ndwords)

        d2h_error     = Signal()
        clr_d2h_error = Signal()
        set_d2h_error = Signal()
        self.sync += \
            If(clr_d2h_error,
                d2h_error.eq(0)
            ).Elif(set_d2h_error,
                d2h_error.eq(1)
            )

        read_error     = Signal()
        clr_read_error = Signal()
        set_read_error = Signal()
        self.sync += \
            If(clr_read_error,
                read_error.eq(0)
            ).Elif(set_read_error,
                read_error.eq(1)
            )

        update_d2h = Signal()
        self.sync += \
            If(update_d2h,
                self.d2h_status.eq(transport.source.status),
                self.d2h_errors.eq(transport.source.errors)
            )

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            NextValue(dwords_counter, 0),
            transport.source.ready.eq(1),
            clr_d2h_error.eq(1),
            clr_read_error.eq(1),
            If(from_tx.write,
                NextState("WAIT_WRITE_ACTIVATE_OR_REG_D2H")
            ).Elif(from_tx.read,
                NextState("WAIT_READ_DATA_OR_REG_D2H"),
            ).Elif(from_tx.identify,
                NextState("WAIT_PIO_SETUP_D2H"),
            )
        )
        self.sync += \
            If(fsm.ongoing("IDLE"),
                is_identify.eq(from_tx.identify)
            )
        fsm.act("WAIT_WRITE_ACTIVATE_OR_REG_D2H",
            transport.source.ready.eq(1),
            If(transport.source.valid,
                If(test_type("DMA_ACTIVATE_D2H"),
                    is_dma_activate.eq(1),
                ).Elif(test_type("REG_D2H"),
                    update_d2h.eq(1),
                    set_d2h_error.eq(transport.source.status[reg_d2h_status["err"]]),
                    NextState("PRESENT_WRITE_RESPONSE")
                )
            )
        )
        fsm.act("PRESENT_WRITE_RESPONSE",
            source.valid.eq(1),
            source.last.eq(1),
            source.write.eq(1),
            source.end.eq(1),
            source.failed.eq(transport.source.error | d2h_error),
            If(source.valid & source.ready,
                NextState("IDLE")
            )
        )
        fsm.act("WAIT_READ_DATA_OR_REG_D2H",
            transport.source.ready.eq(1),
            If(transport.source.valid,
                transport.source.ready.eq(0),
                If(test_type("DATA"),
                    NextState("PRESENT_READ_DATA")
                ).Elif(test_type("REG_D2H"),
                    update_d2h.eq(1),
                    set_d2h_error.eq(transport.source.status[reg_d2h_status["err"]]),
                    NextState("PRESENT_READ_RESPONSE")
                )
            )
        )
        fsm.act("WAIT_PIO_SETUP_D2H",
            transport.source.ready.eq(1),
            If(transport.source.valid,
                transport.source.ready.eq(0),
                If(test_type("PIO_SETUP_D2H"),
                    NextState("PRESENT_PIO_SETUP_D2H")
                ).Else(
                    NextState("FLUSH")
                )
            )
        )
        fsm.act("PRESENT_PIO_SETUP_D2H",
            transport.source.ready.eq(1),
            If(transport.source.valid & transport.source.last,
                NextState("WAIT_READ_DATA_OR_REG_D2H")
            )
        )

        fsm.act("PRESENT_READ_DATA",
            set_read_error.eq(transport.source.error),
            source.valid.eq(transport.source.valid),
            source.last.eq(transport.source.last),
            source.read.eq(~is_identify),
            source.identify.eq(is_identify),
            source.failed.eq(transport.source.error),
            source.end.eq(is_identify),
            source.data.eq(transport.source.data),
            transport.source.ready.eq(source.ready),
            If(source.valid & source.ready,
                If(~read_done, NextValue(dwords_counter, dwords_counter + 1)),
                If(source.last,
                    If(is_identify,
                        NextState("IDLE")
                    ).Else(
                        NextState("WAIT_READ_DATA_OR_REG_D2H")
                    )
                )
            )
        )

        fsm.act("PRESENT_READ_RESPONSE",
            source.valid.eq(1),
            source.last.eq(1),
            source.read.eq(1),
            source.end.eq(1),
            source.failed.eq(~read_done | read_error | d2h_error),
            If(source.valid & source.ready,
                NextState("IDLE")
            )
        )

        fsm.act("FLUSH",
            transport.source.ready.eq(1),
            If(transport.source.valid & transport.source.last,
               NextState("WAIT_PIO_SETUP_D2H")
            )
        )
        self.comb += [
            to_tx.dma_activate.eq(is_dma_activate),
            to_tx.d2h_error.eq(d2h_error)
        ]

# LiteSATA Command ---------------------------------------------------------------------------------

class LiteSATACommand(Module):
    def __init__(self, transport):
        self.submodules.tx = LiteSATACommandTX(transport)
        self.submodules.rx = LiteSATACommandRX(transport)
        self.comb += [
            self.rx.to_tx.connect(self.tx.from_rx),
            self.tx.to_rx.connect(self.rx.from_tx)
        ]
        self.sink, self.source = self.tx.sink, self.rx.source
