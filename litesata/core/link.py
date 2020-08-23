#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2016 Olof Kindgren <olof.kindgren@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from collections import OrderedDict
from functools import reduce
from operator import xor

from litesata.common import *

# Link CRC -----------------------------------------------------------------------------------------

class CRCEngine(Module):
    """Cyclic Redundancy Check Engine

    Compute next CRC value from last CRC value and data input using
    an optimized asynchronous LFSR.

    Parameters
    ----------
    width : int
        Width of the data bus and CRC.
    polynom : int
        Polynom of the CRC (ex: 0x04C11DB7 for IEEE 802.3 CRC)

    Attributes
    ----------
    d : in
        Data input.
    last : in
        last CRC value.
    next :
        next CRC value.
    """
    def __init__(self, width, polynom):
        self.data = Signal(width)
        self.last = Signal(width)
        self.next = Signal(width)

        # # #

        def _optimize_xors(l):
            """
            remove an even numbers of XORs with the same bit
            replace an odd number of XORs with a single XOR
            """
            d = OrderedDict()
            for e in l:
                try:
                    d[e] += 1
                except:
                    d[e] = 1
            r = []
            for k, v in d.items():
                if v%2:
                    r.append(k)
            return r

        new = Signal(32)
        self.comb += new.eq(self.last ^ self.data)

        # compute and optimize the parallel implementation of the CRC's LFSR
        taps = [x for x in range(width) if (1 << x) & polynom]
        curval = [[("new", i)] for i in range(width)]
        for i in range(width):
            feedback = curval.pop()
            for j in range(width-1):
                if j + 1 in taps:
                    curval[j] += feedback
                curval[j] = _optimize_xors(curval[j])
            curval.insert(0, feedback)

        # implement logic
        for i in range(width):
            xors = []
            for t, n in curval[i]:
                if t == "new":
                    xors += [new[n]]
            self.comb += self.next[i].eq(reduce(xor, xors))


@ResetInserter()
@CEInserter()
class LiteSATACRC(Module):
    """SATA CRC

    Implement a SATA CRC generator/checker

    Attributes
    ----------
    value : out
        CRC value (used for generator).
    error : out
        CRC error (used for checker).
    """
    width   = 32
    polynom = 0x04C11DB7
    init    = 0x52325032
    check   = 0x00000000

    def __init__(self):
        self.data  = Signal(self.width)
        self.value = Signal(self.width)
        self.error = Signal()

        # # #

        engine = CRCEngine(self.width, self.polynom)
        self.submodules += engine
        reg_i = Signal(self.width, reset=self.init)
        self.sync += reg_i.eq(engine.next)
        self.comb += [
            engine.data.eq(self.data),
            engine.last.eq(reg_i),

            self.value.eq(reg_i),
            self.error.eq(engine.next != self.check)
        ]


class LiteSATACRCInserter(Module):
    """SATA CRC Inserter

    Append a CRC at the end of each packet.

    Parameters
    ----------
    layout : layout
        Layout of the dataflow.

    Attributes
    ----------
    sink : in
        Packets input without CRC.
    source : out
        Packets output with CRC.
    """
    def __init__(self, description):
        self.sink   = sink   = stream.Endpoint(description)
        self.source = source = stream.Endpoint(description)
        self.busy   = Signal()

        # # #

        crc = LiteSATACRC()
        fsm = FSM(reset_state="IDLE")
        self.submodules += crc, fsm

        fsm.act("IDLE",
            crc.reset.eq(1),
            sink.ready.eq(1),
            If(sink.valid,
                sink.ready.eq(0),
                NextState("COPY"),
            )
        )
        fsm.act("COPY",
            crc.ce.eq(sink.valid & source.ready),
            crc.data.eq(sink.data),
            sink.connect(source),
            source.last.eq(0),
            If(sink.valid & sink.last & source.ready,
                NextState("INSERT"),
            )
        )
        fsm.act("INSERT",
            source.valid.eq(1),
            source.last.eq(1),
            source.data.eq(crc.value),
            If(source.ready, NextState("IDLE"))
        )
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))


class LiteSATACRCChecker(Module):
    """SATA CRC Checker

    Check CRC at the end of each packet.

    Parameters
    ----------
    layout : layout
        Layout of the dataflow.

    Attributes
    ----------
    sink : in
        Packets input with CRC.
    source : out
        Packets output without CRC and "error" set to 0
        on last when CRC OK / set to 1 when CRC KO.
    """
    def __init__(self, description):
        self.sink   = sink   = stream.Endpoint(description)
        self.source = source = stream.Endpoint(description)
        self.busy   = Signal()

        # # #

        crc = LiteSATACRC()
        self.submodules += crc

        error = Signal()
        fifo  = ResetInserter()(stream.SyncFIFO(description, 2))
        self.submodules += fifo

        fsm = FSM(reset_state="RESET")
        self.submodules += fsm

        fifo_in   = Signal()
        fifo_out  = Signal()
        fifo_full = Signal()

        self.comb += [
            fifo_full.eq(fifo.level == 1),
            fifo_in.eq(sink.valid & (~fifo_full | fifo_out)),
            fifo_out.eq(source.valid & source.ready),

            sink.connect(fifo.sink),
            fifo.sink.valid.eq(fifo_in),
            self.sink.ready.eq(fifo_in),

            source.valid.eq(sink.valid & fifo_full),
            source.last.eq(sink.last),
            fifo.source.ready.eq(fifo_out),
            source.payload.eq(fifo.source.payload),

            source.error.eq(sink.error | (crc.error & source.last)),
        ]

        fsm.act("RESET",
            crc.reset.eq(1),
            fifo.reset.eq(1),
            NextState("IDLE"),
        )
        fsm.act("IDLE",
            crc.data.eq(sink.data),
            If(sink.valid & sink.ready,
                crc.ce.eq(1),
                NextState("COPY")
            )
        )
        fsm.act("COPY",
            crc.data.eq(sink.data),
            If(sink.valid & sink.ready,
                crc.ce.eq(1),
                If(sink.last,
                    NextState("RESET")
                )
            )
        )
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))

# Link Scrambling ----------------------------------------------------------------------------------

@CEInserter()
class Scrambler(Module):
    """SATA Scrambler

    Implement a SATA Scrambler

    Attributes
    ----------
    value : out
        Scrambled value.
    """
    def __init__(self):
        self.value = Signal(32)

        # # #

        context    = Signal(16, reset=0xf0f6)
        next_value = Signal(32)
        self.sync += context.eq(next_value[16:32])

        # XXX: from SATA specification, replace it with
        # a generic implementation using polynoms.
        lfsr_coefs = (
            (15, 13, 4, 0),  # 0
            (15, 14, 13, 5, 4, 1, 0),
            (14, 13, 6, 5, 4, 2, 1, 0),
            (15, 14, 7, 6, 5, 3, 2, 1),
            (13, 8, 7, 6, 3, 2, 0),
            (14, 9, 8, 7, 4, 3, 1),
            (15, 10, 9, 8, 5, 4, 2),
            (15, 13, 11, 10, 9, 6, 5, 4, 3, 0),
            (15, 14, 13, 12, 11, 10, 7, 6, 5, 1, 0),
            (14, 12, 11, 8, 7, 6, 4, 2, 1, 0),
            (15, 13, 12, 9, 8, 7, 5, 3, 2, 1),
            (15, 14, 10, 9, 8, 6, 3, 2, 0),
            (13, 11, 10, 9, 7, 3, 1, 0),
            (14, 12, 11, 10, 8, 4, 2, 1),
            (15, 13, 12, 11, 9, 5, 3, 2),
            (15, 14, 12, 10, 6, 3, 0),

            (11, 7, 1, 0),  # 16
            (12, 8, 2, 1),
            (13, 9, 3, 2),
            (14, 10, 4, 3),
            (15, 11, 5, 4),
            (15, 13, 12, 6, 5, 4, 0),
            (15, 14, 7, 6, 5, 4, 1, 0),
            (13, 8, 7, 6, 5, 4, 2, 1, 0),
            (14, 9, 8, 7, 6, 5, 3, 2, 1),
            (15, 10, 9, 8, 7, 6, 4, 3, 2),
            (15, 13, 11, 10, 9, 8, 7, 5, 3, 0),
            (15, 14, 13, 12, 11, 10, 9, 8, 6, 1, 0),
            (14, 12, 11, 10, 9, 7, 4, 2, 1, 0),
            (15, 13, 12, 11, 10, 8, 5, 3, 2, 1),
            (15, 14, 12, 11, 9, 6, 3, 2, 0),
            (12, 10, 7, 3, 1, 0),
        )

        for n, coefs in enumerate(lfsr_coefs):
            eq = [context[i] for i in coefs]
            self.comb += next_value[n].eq(reduce(xor, eq))

        self.comb += self.value.eq(next_value)


@ResetInserter()
class LiteSATAScrambler(Module):
    def __init__(self, description):
        self.sink   = sink   = stream.Endpoint(description)
        self.source = source = stream.Endpoint(description)

        # # #

        scrambler = Scrambler()
        self.submodules += scrambler
        self.comb += [
            scrambler.ce.eq(sink.valid & sink.ready),
            sink.connect(source),
            source.data.eq(sink.data ^ scrambler.value)
        ]

# Link Clock Compensation --------------------------------------------------------------------------

class LiteSATACONTInserter(Module):
    def __init__(self, description):
        self.sink   = sink   = stream.Endpoint(description)
        self.source = source = stream.Endpoint(description)

        # # #

        counter       = Signal(max=4)
        counter_reset = Signal()
        counter_ce    = Signal()
        self.sync += \
            If(counter_reset,
                counter.eq(0)
            ).Elif(counter_ce,
                counter.eq(counter + 1)
            )

        is_data  = Signal()
        was_data = Signal()
        was_hold = Signal()
        change   = Signal()
        self.comb += is_data.eq(sink.charisk == 0)

        last_data = Signal(32)
        last_primitive = Signal(32)
        last_charisk = Signal(4)
        self.sync += [
            If(sink.valid & source.ready,
                last_data.eq(sink.data),
                last_charisk.eq(sink.charisk),
                If(~is_data,
                    last_primitive.eq(sink.data),
                ),
                was_data.eq(is_data),
                was_hold.eq(last_primitive == primitives["HOLD"])
            )
        ]
        self.comb += change.eq(
            (sink.data != last_data) |
            (sink.charisk != last_charisk) |
            is_data
        )

        # Scrambler
        scrambler = ResetInserter()(Scrambler())
        self.submodules += scrambler

        # Datapath
        self.comb += [
            sink.connect(source),
            If(sink.valid,
                If(~change,
                    counter_ce.eq(sink.ready & (counter != 2)),
                    # insert CONT
                    If(counter == 1,
                        source.charisk.eq(0b0001),
                        source.data.eq(primitives["CONT"])
                    # insert scrambled data for EMI
                    ).Elif(counter == 2,
                        scrambler.ce.eq(sink.ready),
                        source.charisk.eq(0b0000),
                        source.data.eq(scrambler.value)
                    )
                ).Else(
                    counter_reset.eq(source.ready),
                    If(counter == 2,
                        # Reinsert last primitive
                        If(is_data | (~is_data & was_hold),
                            source.valid.eq(1),
                            sink.ready.eq(0),
                            source.charisk.eq(0b0001),
                            source.data.eq(last_primitive)
                        )
                    )
                )
            )
        ]

class LiteSATACONTRemover(Module):
    def __init__(self, description):
        self.sink   = sink   = stream.Endpoint(description)
        self.source = source = stream.Endpoint(description)

        # # #

        is_data      = Signal()
        is_cont      = Signal()
        in_cont      = Signal()
        cont_ongoing = Signal()

        self.comb += [
            is_data.eq(sink.charisk == 0),
            is_cont.eq(~is_data & (sink.data == primitives["CONT"]))
        ]
        self.sync += \
            If(sink.valid & sink.ready,
                If(is_cont,
                    in_cont.eq(1)
                ).Elif(~is_data,
                    in_cont.eq(0)
                )
            )
        self.comb += cont_ongoing.eq(is_cont | (in_cont & is_data))

        # Datapath
        last_primitive = Signal(32)
        self.sync += [
            If(sink.valid & sink.ready,
                If(~is_data & ~is_cont,
                    last_primitive.eq(sink.data)
                )
            )
        ]
        self.comb += [
            sink.connect(source),
            If(cont_ongoing,
                source.charisk.eq(0b0001),
                source.data.eq(last_primitive)
            )
        ]

# Link Alignment -----------------------------------------------------------------------------------

class LiteSATAALIGNInserter(Module):
    def __init__(self, description):
        self.sink   = sink   = stream.Endpoint(description)
        self.source = source = stream.Endpoint(description)

        # # #

        # send 2 ALIGN every 256 DWORDs
        # used for clock compensation between
        # HOST and device
        cnt  = Signal(8)
        send = Signal()
        self.sync += \
            If(source.valid & source.ready,
                cnt.eq(cnt+1)
            )
        self.comb += [
            send.eq(cnt < 2),
            If(send,
                source.valid.eq(1),
                source.charisk.eq(0b0001),
                source.data.eq(primitives["ALIGN"]),
                sink.ready.eq(0)
            ).Else(
                source.valid.eq(sink.valid),
                source.data.eq(sink.data),
                source.charisk.eq(sink.charisk),
                sink.ready.eq(source.ready)
            )
        ]


class LiteSATAALIGNRemover(Module):
    def __init__(self, description):
        self.sink   = sink   = stream.Endpoint(description)
        self.source = source = stream.Endpoint(description)

        # # #

        charisk_match = sink.charisk == 0b0001
        data_match    = sink.data == primitives["ALIGN"]

        self.comb += \
            If(sink.valid & charisk_match & data_match,
                sink.ready.eq(1),
            ).Else(
                sink.connect(source)
            )

# Link TX ------------------------------------------------------------------------------------------

from_rx = [
    ("idle",            1),
    ("insert",          32),
    ("primitive_valid", 1),
    ("primitive",       32)
]

class LiteSATALinkTX(Module):
    def __init__(self):
        self.sink    = sink   = stream.Endpoint(link_description(32))
        self.source  = source = stream.Endpoint(phy_description(32))
        self.from_rx = stream.Endpoint(from_rx)

        self.error = Signal()

        # # #

        # CRC / Scrambler
        crc       = LiteSATACRCInserter(link_description(32))
        scrambler = LiteSATAScrambler(link_description(32))
        pipeline  = Pipeline(sink, crc, scrambler)
        self.submodules += crc, scrambler, pipeline

        # datas / primitives mux
        insert = Signal(32)
        copy   = Signal()
        self.comb += [
            If(self.from_rx.insert,
                source.valid.eq(1),
                source.data.eq(self.from_rx.insert),
                source.charisk.eq(0b0001),
            ).Elif(insert,
                source.valid.eq(1),
                source.data.eq(insert),
                source.charisk.eq(0b0001),
            ).Elif(copy,
                source.valid.eq(1),
                pipeline.source.ready.eq(source.ready),
                If(pipeline.source.valid,
                    source.data.eq(pipeline.source.data),
                    source.charisk.eq(0b0000)
                ).Else(
                    source.data.eq(primitives["HOLD"]),
                    source.charisk.eq(0b0001)
                )
            )
        ]

        # FSM
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            scrambler.reset.eq(1),
            If(self.from_rx.idle,
                insert.eq(primitives["SYNC"]),
                If(pipeline.source.valid,
                    If(self.from_rx.primitive_valid &
                       (self.from_rx.primitive == primitives["SYNC"]),
                        NextState("RDY")
                    )
                )
            )
        )
        fsm.act("RDY",
            insert.eq(primitives["X_RDY"]),
            If(~self.from_rx.idle,
                NextState("IDLE")
            ).Elif(self.from_rx.primitive_valid &
                   (self.from_rx.primitive == primitives["R_RDY"]),
                NextState("SOF")
            )
        )
        fsm.act("SOF",
            insert.eq(primitives["SOF"]),
            If(source.ready,
                NextState("COPY")
            )
        )
        fsm.act("COPY",
            copy.eq(1),
            If(pipeline.source.valid &
               pipeline.source.last &
               pipeline.source.ready,
                NextState("EOF")
            ).Elif(self.from_rx.primitive_valid &
               (self.from_rx.primitive == primitives["HOLD"]),
               NextState("HOLDA")
            )
        )
        fsm.act("HOLDA",
            insert.eq(primitives["HOLDA"]),
            If(self.from_rx.primitive_valid &
               (self.from_rx.primitive != primitives["HOLD"]),
                NextState("COPY")
            ).Elif(self.error,
                NextState("IDLE")
            )
        )
        fsm.act("EOF",
            insert.eq(primitives["EOF"]),
            If(source.ready,
                NextState("WTRM")
            )
        )
        fsm.act("WTRM",
            insert.eq(primitives["WTRM"]),
            If(self.from_rx.primitive_valid,
                If(self.from_rx.primitive == primitives["R_OK"],
                    NextState("IDLE")
                ).Elif(self.from_rx.primitive == primitives["R_ERR"],
                    NextState("IDLE")
                )
            ).Elif(self.error,
                NextState("IDLE")
            )
        )

        # error detection
        self.sync += [
            # generate error if receiving SYNC during transfer (disk returns to IDLE)
            If(~(fsm.ongoing("IDLE") | fsm.ongoing("RDY")),
                self.error.eq(self.from_rx.primitive_valid &
                              (self.from_rx.primitive == primitives["SYNC"]))
            )
        ]

# Link RX ------------------------------------------------------------------------------------------

class LiteSATALinkRX(Module):
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(phy_description(32))
        self.source = source = stream.Endpoint(link_description(32))
        self.hold   = Signal()
        self.to_tx  = stream.Endpoint(from_rx)

        # # #

        # Always ready from phy
        self.comb += sink.ready.eq(1)

        # Datas / primitives detection
        insert          = Signal(32)
        data_valid      = Signal()
        primitive_valid = Signal()
        primitive       = Signal(32)
        self.comb += [
            If(sink.valid,
                data_valid.eq(sink.charisk == 0),
                primitive_valid.eq(sink.charisk == 0b0001)
            ),
            primitive.eq(sink.data)
        ]

        # Descrambler / CRC
        descrambler = LiteSATAScrambler(link_description(32))
        crc         = LiteSATACRCChecker(link_description(32))
        pipeline    = Pipeline(descrambler, crc, source)
        self.submodules += descrambler, crc, pipeline

        # Internal logic
        self.crc_error = crc_error = Signal()
        self.sync += \
            If(crc.source.valid & crc.source.last & crc.source.ready,
                crc_error.eq(crc.source.error)
            )

        # FSM
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            descrambler.reset.eq(1),
            If(primitive_valid &
               (primitive == primitives["X_RDY"]),
                NextState("RDY")
            )
        )
        fsm.act("RDY",
            insert.eq(primitives["R_RDY"]),
            If(primitive_valid &
               (primitive == primitives["SOF"]),
                NextState("WAIT_FIRST")
            )
        )
        fsm.act("WAIT_FIRST",
            insert.eq(primitives["R_IP"]),
            If(data_valid,
                NextState("COPY")
            )
        )
        fsm.act("COPY",
            pipeline.sink.valid.eq(data_valid),
            insert.eq(primitives["R_IP"]),
            If(primitive_valid,
                If(primitive == primitives["HOLD"],
                    insert.eq(primitives["HOLDA"])
                ).Elif(primitive == primitives["EOF"],
                    # 1 clock cycle latency
                    pipeline.sink.valid.eq(1),
                    pipeline.sink.last.eq(1),
                    NextState("WTRM")
                )
            ).Elif(self.hold,
                insert.eq(primitives["HOLD"])
            )
        )
        # 1 clock cycle latency
        self.sync += If(data_valid, pipeline.sink.data.eq(sink.data))
        fsm.act("EOF",
            insert.eq(primitives["R_IP"]),
            If(primitive_valid &
               (primitive == primitives["WTRM"]),
                NextState("WTRM")
            )
        )
        fsm.act("WTRM",
            insert.eq(primitives["R_IP"]),
            If(~crc_error,
                NextState("R_OK")
            ).Else(
                NextState("R_ERR")
            )
        )
        fsm.act("R_OK",
            insert.eq(primitives["R_OK"]),
            If(primitive_valid &
               (primitive == primitives["SYNC"]),
                NextState("IDLE")
            )
        )
        fsm.act("R_ERR",
            insert.eq(primitives["R_ERR"]),
            If(primitive_valid &
               (primitive == primitives["SYNC"]),
                NextState("IDLE")
            )
        )

        # To TX
        self.comb += [
            self.to_tx.idle.eq(fsm.ongoing("IDLE")),
            self.to_tx.insert.eq(insert),
            self.to_tx.primitive_valid.eq(primitive_valid),
            self.to_tx.primitive.eq(primitive)
        ]

# Link ---------------------------------------------------------------------------------------------

class LiteSATALink(Module):
    def __init__(self, phy):
        # TX ---------------------------------------------------------------------------------------
        self.submodules.tx = BufferizeEndpoints({"source": DIR_SOURCE})(LiteSATALinkTX())
        self.submodules.tx_align = LiteSATAALIGNInserter(phy_description(32))
        self.submodules.tx_pipeline = Pipeline(self.tx, self.tx_align, phy)

        # RX ---------------------------------------------------------------------------------------
        self.submodules.rx_align = LiteSATAALIGNRemover(phy_description(32))
        self.submodules.rx_cont = LiteSATACONTRemover(phy_description(32))
        self.submodules.rx = BufferizeEndpoints({"sink": DIR_SINK})(LiteSATALinkRX())
        self.submodules.rx_buffer = stream.SyncFIFO(link_description(32), 128)
        self.submodules.rx_pipeline = Pipeline(phy, self.rx_align, self.rx_cont, self.rx, self.rx_buffer)

        # RX --> TX --------------------------------------------------------------------------------
        self.comb += self.rx.to_tx.connect(self.tx.from_rx)

        self.sink, self.source = self.tx_pipeline.sink, self.rx_pipeline.source

        # Hold -------------------------------------------------------------------------------------
        self.comb += self.rx.hold.eq(self.rx_buffer.level > 64)
