from collections import OrderedDict
from functools import reduce
from operator import xor

from litesata.common import *

from litex.soc.interconnect.stream_packet import Buffer

# link crc

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
        self.d = Signal(width)
        self.last = Signal(width)
        self.next = Signal(width)

        # # #

        def _optimize_eq(l):
            """
            Replace even numbers of XORs in the equation
            with an equivalent XOR
            """
            d = OrderedDict()
            for e in l:
                if e in d:
                    d[e] += 1
                else:
                    d[e] = 1
            r = []
            for key, value in d.items():
                if value%2 != 0:
                    r.append(key)
            return r

        new = Signal(32)
        self.comb += new.eq(self.last ^ self.d)

        # compute and optimize CRC's LFSR
        curval = [[("new", i)] for i in range(width)]
        for i in range(width):
            feedback = curval.pop()
            for j in range(width-1):
                if (polynom & (1<<(j+1))):
                    curval[j] += feedback
                curval[j] = _optimize_eq(curval[j])
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
    width = 32
    polynom = 0x04C11DB7
    init = 0x52325032
    check = 0x00000000

    def __init__(self):
        self.d = Signal(self.width)
        self.value = Signal(self.width)
        self.error = Signal()

        # # #

        engine = CRCEngine(self.width, self.polynom)
        self.submodules += engine
        reg_i = Signal(self.width, reset=self.init)
        self.sync += reg_i.eq(engine.next)
        self.comb += [
            engine.d.eq(self.d),
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
        self.sink = sink = Sink(description)
        self.source = source = Source(description)
        self.busy = Signal()

        # # #

        crc = LiteSATACRC()
        fsm = FSM(reset_state="IDLE")
        self.submodules += crc, fsm

        fsm.act("IDLE",
            crc.reset.eq(1),
            sink.ack.eq(1),
            If(sink.stb & sink.sop,
                sink.ack.eq(0),
                NextState("COPY"),
            )
        )
        fsm.act("COPY",
            crc.ce.eq(sink.stb & source.ack),
            crc.d.eq(sink.d),
            Record.connect(sink, source),
            source.eop.eq(0),
            If(sink.stb & sink.eop & source.ack,
                NextState("INSERT"),
            )
        )
        fsm.act("INSERT",
            source.stb.eq(1),
            source.eop.eq(1),
            source.d.eq(crc.value),
            If(source.ack, NextState("IDLE"))
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
        on eop when CRC OK / set to 1 when CRC KO.
    """
    def __init__(self, description):
        self.sink = sink = Sink(description)
        self.source = source = Source(description)
        self.busy = Signal()

        # # #

        crc = LiteSATACRC()
        self.submodules += crc

        error = Signal()
        fifo = ResetInserter()(SyncFIFO(description, 2))
        self.submodules += fifo

        fsm = FSM(reset_state="RESET")
        self.submodules += fsm

        fifo_in = Signal()
        fifo_out = Signal()
        fifo_full = Signal()

        self.comb += [
            fifo_full.eq(fifo.level == 1),
            fifo_in.eq(sink.stb & (~fifo_full | fifo_out)),
            fifo_out.eq(source.stb & source.ack),

            Record.connect(sink, fifo.sink),
            fifo.sink.stb.eq(fifo_in),
            self.sink.ack.eq(fifo_in),

            source.stb.eq(sink.stb & fifo_full),
            source.sop.eq(fifo.source.sop),
            source.eop.eq(sink.eop),
            fifo.source.ack.eq(fifo_out),
            source.payload.eq(fifo.source.payload),

            source.error.eq(sink.error | crc.error),
        ]

        fsm.act("RESET",
            crc.reset.eq(1),
            fifo.reset.eq(1),
            NextState("IDLE"),
        )
        fsm.act("IDLE",
            crc.d.eq(sink.d),
            If(sink.stb & sink.sop & sink.ack,
                crc.ce.eq(1),
                NextState("COPY")
            )
        )
        fsm.act("COPY",
            crc.d.eq(sink.d),
            If(sink.stb & sink.ack,
                crc.ce.eq(1),
                If(sink.eop,
                    NextState("RESET")
                )
            )
        )
        self.comb += self.busy.eq(~fsm.ongoing("IDLE"))

# link scrambler

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

        context = Signal(16, reset=0xf0f6)
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
        self.sink = sink = Sink(description)
        self.source = source = Source(description)

        # # #

        scrambler = Scrambler()
        self.submodules += scrambler
        self.comb += [
            scrambler.ce.eq(sink.stb & sink.ack),
            Record.connect(sink, source),
            source.d.eq(sink.d ^ scrambler.value)
        ]

# link cont

class LiteSATACONTInserter(Module):
    def __init__(self, description):
        self.sink = sink = Sink(description)
        self.source = source = Source(description)

        # # #

        counter = Signal(max=4)
        counter_reset = Signal()
        counter_ce = Signal()
        self.sync += \
            If(counter_reset,
                counter.eq(0)
            ).Elif(counter_ce,
                counter.eq(counter + 1)
            )

        is_data = Signal()
        was_data = Signal()
        was_hold = Signal()
        change = Signal()
        self.comb += is_data.eq(sink.charisk == 0)

        last_data = Signal(32)
        last_primitive = Signal(32)
        last_charisk = Signal(4)
        self.sync += [
            If(sink.stb & source.ack,
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

        # scrambler
        scrambler = ResetInserter()(Scrambler())
        self.submodules += scrambler

        # Datapath
        self.comb += [
            Record.connect(sink, source),
            If(sink.stb,
                If(~change,
                    counter_ce.eq(sink.ack & (counter != 2)),
                    # insert CONT
                    If(counter == 1,
                        source.charisk.eq(0b0001),
                        source.data.eq(primitives["CONT"])
                    # insert scrambled data for EMI
                    ).Elif(counter == 2,
                        scrambler.ce.eq(sink.ack),
                        source.charisk.eq(0b0000),
                        source.data.eq(scrambler.value)
                    )
                ).Else(
                    counter_reset.eq(source.ack),
                    If(counter == 2,
                        # Reinsert last primitive
                        If(is_data | (~is_data & was_hold),
                            source.stb.eq(1),
                            sink.ack.eq(0),
                            source.charisk.eq(0b0001),
                            source.data.eq(last_primitive)
                        )
                    )
                )
            )
        ]


class LiteSATACONTRemover(Module):
    def __init__(self, description):
        self.sink = sink = Sink(description)
        self.source = source = Source(description)

        # # #

        is_data = Signal()
        is_cont = Signal()
        in_cont = Signal()
        cont_ongoing = Signal()

        self.comb += [
            is_data.eq(sink.charisk == 0),
            is_cont.eq(~is_data & (sink.data == primitives["CONT"]))
        ]
        self.sync += \
            If(sink.stb & sink.ack,
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
            If(sink.stb & sink.ack,
                If(~is_data & ~is_cont,
                    last_primitive.eq(sink.data)
                )
            )
        ]
        self.comb += [
            Record.connect(sink, source),
            If(cont_ongoing,
                source.charisk.eq(0b0001),
                source.data.eq(last_primitive)
            )
        ]

# link tx

from_rx = [
    ("idle", 1),
    ("insert", 32),
    ("valid", 1),
    ("det", 32)
]

class LiteSATALinkTX(Module):
    def __init__(self, phy):
        self.sink = Sink(link_description(32))
        self.from_rx = Sink(from_rx)

        # # #

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        # insert CRC
        crc = LiteSATACRCInserter(link_description(32))
        self.submodules += crc

        # scramble
        scrambler = LiteSATAScrambler(link_description(32))
        self.submodules += scrambler

        # connect CRC / scrambler
        self.comb += [
            Record.connect(self.sink, crc.sink),
            Record.connect(crc.source, scrambler.sink)
        ]

        # inserter CONT and scrambled data between
        # CONT and next primitive
        cont = BufferizeEndpoints("sink")(LiteSATACONTInserter(phy_description(32)))
        self.submodules += cont

        # datas / primitives mux
        insert = Signal(32)
        copy = Signal()
        self.comb += [
            If(self.from_rx.insert,
                cont.sink.stb.eq(1),
                cont.sink.data.eq(self.from_rx.insert),
                cont.sink.charisk.eq(0x0001),
            ).Elif(insert,
                cont.sink.stb.eq(1),
                cont.sink.data.eq(insert),
                cont.sink.charisk.eq(0x0001),
            ).Elif(copy,
                cont.sink.stb.eq(scrambler.source.stb),
                cont.sink.data.eq(scrambler.source.d),
                scrambler.source.ack.eq(cont.sink.ack),
                cont.sink.charisk.eq(0)
            ),
            Record.connect(cont.source, phy.sink)
        ]

        # FSM
        fsm.act("IDLE",
            scrambler.reset.eq(1),
            If(self.from_rx.idle,
                insert.eq(primitives["SYNC"]),
                If(scrambler.source.stb & scrambler.source.sop,
                    If(self.from_rx.det == primitives["SYNC"],
                        NextState("RDY")
                    )
                )
            )
        )
        fsm.act("RDY",
            insert.eq(primitives["X_RDY"]),
            If(~self.from_rx.idle,
                NextState("IDLE")
            ).Elif(self.from_rx.det == primitives["R_RDY"],
                NextState("SOF")
            )
        )
        fsm.act("SOF",
            insert.eq(primitives["SOF"]),
            If(phy.sink.ack,
                NextState("COPY")
            )
        )
        fsm.act("COPY",
            copy.eq(1),
            If(self.from_rx.valid &
               (self.from_rx.det == primitives["HOLD"]),
               NextState("HOLDA")
            ).Elif(~scrambler.source.stb,
                insert.eq(primitives["HOLD"])
            ).Elif(scrambler.source.stb &
                   scrambler.source.eop &
                   scrambler.source.ack,
                NextState("EOF")
            )
        )
        fsm.act("HOLDA",
            insert.eq(primitives["HOLDA"]),
            If(self.from_rx.valid &
               (self.from_rx.det != primitives["HOLD"]),
                NextState("COPY")
            )
        )
        fsm.act("EOF",
            insert.eq(primitives["EOF"]),
            If(phy.sink.ack,
                NextState("WTRM")
            )
        )
        fsm.act("WTRM",
            insert.eq(primitives["WTRM"]),
            If(self.from_rx.det == primitives["R_OK"],
                NextState("IDLE")
            ).Elif(self.from_rx.det == primitives["R_ERR"],
                NextState("IDLE")
            )
        )

# link rx

class LiteSATALinkRX(Module):
    def __init__(self, phy):
        self.source = Source(link_description(32))
        self.hold = Signal()
        self.to_tx = Source(from_rx)

        # # #

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm

        # CONT remover
        cont = BufferizeEndpoints("source")(LiteSATACONTRemover(phy_description(32)))
        self.submodules += cont
        self.comb += Record.connect(phy.source, cont.sink)

        # datas / primitives detection
        insert = Signal(32)
        det = Signal(32)
        self.comb += \
            If(cont.source.stb & (cont.source.charisk == 0b0001),
                det.eq(cont.source.data)
            )

        # descrambler
        scrambler = LiteSATAScrambler(link_description(32))
        self.submodules += scrambler

        # check CRC
        crc = LiteSATACRCChecker(link_description(32))
        self.submodules += crc

        idle = Signal()
        copy = Signal()

        sop = Signal()
        eop = Signal()
        self.sync += \
            If(idle,
                sop.eq(1),
            ).Elif(copy,
                If(scrambler.sink.stb & scrambler.sink.ack,
                    sop.eq(0)
                )
            )
        self.comb += eop.eq(det == primitives["EOF"])

        crc_error = Signal()
        self.sync += \
            If(crc.source.stb & crc.source.eop & crc.source.ack,
                crc_error.eq(crc.source.error)
            )

        # graph
        self.comb += [
            cont.source.ack.eq(1),
            Record.connect(scrambler.source, crc.sink),
            Record.connect(crc.source, self.source),
        ]
        cont_source_data_d = Signal(32)
        self.sync += \
            If(cont.source.stb & (det == 0),
                scrambler.sink.d.eq(cont.source.data)
            )

        # FSM
        fsm.act("IDLE",
            idle.eq(1),
            scrambler.reset.eq(1),
            If(det == primitives["X_RDY"],
                NextState("RDY")
            )
        )
        fsm.act("RDY",
            insert.eq(primitives["R_RDY"]),
            If(det == primitives["SOF"],
                NextState("WAIT_FIRST")
            )
        )
        fsm.act("WAIT_FIRST",
            insert.eq(primitives["R_IP"]),
            If(cont.source.stb & (det == 0),
                NextState("COPY")
            )
        )
        self.comb += [
            scrambler.sink.sop.eq(sop),
            scrambler.sink.eop.eq(eop)
        ]
        fsm.act("COPY",
            copy.eq(1),
            scrambler.sink.stb.eq(cont.source.stb & ((det == 0) | eop)),
            insert.eq(primitives["R_IP"]),
            If(det == primitives["HOLD"],
                insert.eq(primitives["HOLDA"])
            ).Elif(det == primitives["EOF"],
                NextState("WTRM")
            ).Elif(self.hold,
                insert.eq(primitives["HOLD"])
            )
        )
        fsm.act("EOF",
            insert.eq(primitives["R_IP"]),
            If(det == primitives["WTRM"],
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
            If(det == primitives["SYNC"],
                NextState("IDLE")
            )
        )
        fsm.act("R_ERR",
            insert.eq(primitives["R_ERR"]),
            If(det == primitives["SYNC"],
                NextState("IDLE")
            )
        )

        # to TX
        self.comb += [
            self.to_tx.idle.eq(fsm.ongoing("IDLE")),
            self.to_tx.insert.eq(insert),
            self.to_tx.valid.eq(cont.source.stb),
            self.to_tx.det.eq(det)
        ]

# link

class LiteSATALink(Module):
    def __init__(self, phy, buffer_depth):
        self.submodules.tx_buffer = Buffer(link_description(32), buffer_depth)
        self.submodules.tx = LiteSATALinkTX(phy)
        self.submodules.rx = LiteSATALinkRX(phy)
        self.submodules.rx_buffer = Buffer(link_description(32), buffer_depth,
                                                 almost_full=3*buffer_depth//4)
        self.comb += [
            Record.connect(self.tx_buffer.source, self.tx.sink),
            Record.connect(self.rx.to_tx, self.tx.from_rx),
            Record.connect(self.rx.source, self.rx_buffer.sink),
            self.rx.hold.eq(self.rx_buffer.almost_full)
        ]
        self.sink, self.source = self.tx_buffer.sink, self.rx_buffer.source
