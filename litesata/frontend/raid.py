from functools import reduce
from operator import and_, or_

from litesata.common import *
from litesata.frontend.arbitration import LiteSATAUserPort

from litex.soc.interconnect.stream_packet import Status, Arbiter, Dispatcher

# striping

class LiteSATAStripingTX(Module):
    """SATA Striping TX

    Split cmds and writes data on N different controllers.

    This module provides a mirroring_mode that is used by the mirroring module to
    dispatch identicals writes to the controllers. This avoid code duplication in
    between striping and mirroring modules. In this special case, port's data width
    is dw (same as controllers)
    """
    def __init__(self, n, dw, mirroring_mode=False):
        self.sink = sink = Sink(command_tx_description(dw*n if not mirroring_mode else dw))
        self.sources = sources = [Source(command_tx_description(dw)) for i in range(n)]

        # # #

        split = Signal()

        already_acked = Signal(n)
        self.sync += If(split & sink.stb,
                already_acked.eq(already_acked | Cat(*[s.ack for s in sources])),
                If(sink.ack, already_acked.eq(0))
            )

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            sink.ack.eq(0),
            If(sink.stb & sink.sop,
                NextState("SPLIT")
            ).Else(
                sink.ack.eq(1)
            )
        )

        # split data and ctrl signals (except stb & ack managed in fsm)
        for i, s in enumerate(sources):
            self.comb += Record.connect(sink, s, leave_out=set(["stb", "ack", "data"]))
            if mirroring_mode:
                self.comb += s.data.eq(sink.data)
            else:
                self.comb += s.data.eq(sink.data[i*dw:(i+1)*dw])

        fsm.act("SPLIT",
            split.eq(1),
            [s.stb.eq(sink.stb & ~already_acked[i]) for i, s in enumerate(sources)],
            sink.ack.eq(reduce(and_, [s.ack | already_acked[i] for i, s in enumerate(sources)])),
            If(sink.stb & sink.eop & sink.ack,
                NextState("IDLE")
            )
        )

class LiteSATAStripingRX(Module):
    """SATA Striping RX

    Combine acknowledges and reads data from N different controllers.

    This module provides a mirroring_mode that is used by the mirroring module to
    dispatch identicals writes to the controllers. This avoid code duplication in
    between striping and mirroring modules. In this special case, port's data width
    is dw (same as controllers)
    """
    def __init__(self, n, dw, mirroring_mode=False):
        self.sinks = sinks = [Sink(command_rx_description(dw)) for i in range(n)]
        self.source = source = Source(command_rx_description(dw*n if not mirroring_mode else dw))

        # # #

        sop = Signal()
        self.comb += sop.eq(reduce(and_, [s.stb & s.sop for s in sinks]))

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            If(sop,
                NextState("COMBINE")
            )
        )

        # use first sink for ctrl signals (except for stb, ack & failed)
        self.comb += Record.connect(sinks[0], source, leave_out=set(["stb", "ack", "failed", "data"]))
		# combine datas
        if mirroring_mode:
            self.comb += source.data.eq(0) # mirroring only used for writes
        else:
            for i, s in enumerate(sinks):
                self.comb += source.data[i*dw:(i+1)*dw].eq(s.data)


        fsm.act("COMBINE",
            source.failed.eq(reduce(or_, [s.failed for s in sinks])), # XXX verify this is enough
            source.stb.eq(reduce(and_, [s.stb for s in sinks])),
            [s.ack.eq(source.stb & source.ack) for s in sinks],
            If(source.stb & source.eop & source.ack,
                NextState("IDLE")
            )
        )


class LiteSATAStriping(Module):
    """SATA Striping

    Segment data so that data is stored on N different controllers.
                     +----> controller0 (dw)
    port (N*dw) <----+----> controllerX (dw)
                     +----> controllerN (dw)

    Characteristics:
        - port's visible capacity = N x controller's visible capacity
        - port's throughput = N x (slowest) controller's throughput

    Can be used to increase capacity and writes/reads throughput.
    """
    def __init__(self, controllers):

        # # #
        n = len(controllers)
        dw = len(controllers[0].sink.data)

        self.submodules.tx = LiteSATAStripingTX(n, dw)
        self.submodules.rx = LiteSATAStripingRX(n, dw)
        for i in range(n):
            self.comb += [
                Record.connect(self.tx.sources[i], controllers[i].sink),
                Record.connect(controllers[i].source, self.rx.sinks[i])
            ]
        self.sink, self.source = self.tx.sink, self.rx.source

# mirroring


class LiteSATAMirroringCtrl(Module):
    def __init__(self, n):
        self.new_cmds = Signal(n)
        self.ack_cmds = Signal(n)

        self.reading = Signal()
        self.writing = Signal()

        self.wants_write = Signal()
        self.write_sel = Signal(max=n)

        # # #

        pending_cmds = Signal(n)
        self.sync += pending_cmds.eq(self.new_cmds | (pending_cmds & ~self.ack_cmds))
        can_commute = Signal()
        self.comb += can_commute.eq((pending_cmds | self.new_cmds)  == 0)

        self.fsm = fsm = FSM(reset_state="READ")
        self.submodules += fsm
        fsm.act("READ",
            self.reading.eq(1),
            If(self.wants_write & can_commute,
                NextState("WRITE")
            )
        )
        fsm.act("WRITE",
            self.writing.eq(1),
            If(~self.wants_write & can_commute,
                NextState("READ")
            )
        )


class LiteSATAMirroringTX(Module):
    def __init__(self, n, dw, ctrl):
        self.sinks = sinks = [Sink(command_tx_description(dw)) for i in range(n)]
        self.sources = sources = [Source(command_tx_description(dw)) for i in range(n)]

        # # #

        wants_write = Signal()

        reading = Signal()
        writing = Signal()

        reads = [Sink(command_tx_description(dw)) for i in range(dw)]
        writes = [Sink(command_tx_description(dw)) for i in range(dw)]
        for sink, read, write in zip(sinks, reads, writes):
            read_stall = Signal()
            read_status = Status(read)
            self.submodules += read_status
            self.comb += [
                Record.connect(sink, read, leave_out=set(["stb", "ack"])),
                Record.connect(sink, write, leave_out=set(["stb", "ack"])),
                read.stb.eq(sink.stb & (sink.read | sink.identify) & ~read_stall),
                write.stb.eq(sink.stb & sink.write),
                If(sink.read | sink.identify,
                    sink.ack.eq((read.ack & ~read_stall))
                ).Else(
                    sink.ack.eq(write.ack)
                )
            ]
            self.sync += \
                If(~ctrl.wants_write,
                    read_stall.eq(0)
                ).Elif(~read_status.ongoing,
                    read_stall.eq(1)
                )

        write_striper = LiteSATAStripingTX(n, dw, mirroring_mode=True)
        write_arbiter = Arbiter(writes, write_striper.sink)
        self.submodules += write_striper, write_arbiter

        for i in range(n):
            source_status = Status(sources[i])
            self.submodules += source_status
            self.comb += [
                If(ctrl.reading,
                    Record.connect(reads[i], sources[i]) # independent reads
                ).Elif(ctrl.writing,
                    Record.connect(write_striper.sources[i], sources[i]) # identical writes
                ),
                ctrl.new_cmds[i].eq(source_status.eop)
            ]
        write_striper_sink_status = Status(write_striper.sink)
        self.submodules += write_striper_sink_status
        self.comb += [
            ctrl.wants_write.eq(write_striper_sink_status.ongoing),
            ctrl.write_sel.eq(write_arbiter.rr.grant)
        ]


class LiteSATAMirroringRX(Module):
    def __init__(self, n, dw, ctrl):
        self.sinks = sinks = [Sink(command_rx_description(dw)) for i in range(n)]
        self.sources = sources = [Source(command_rx_description(dw)) for i in range(n)]

        # # #

        muxs = [Multiplexer(command_rx_description(dw), 2) for i in range(n)]
        self.submodules += muxs

        writes = [mux.sink0 for mux in muxs]
        reads = [mux.sink1 for mux in muxs]

        for mux, source in zip(muxs, sources):
            self.comb += [
                mux.sel.eq(ctrl.reading),
                Record.connect(mux.source, source)
            ]

        write_striper = LiteSATAStripingRX(n, dw, mirroring_mode=True)
        write_dispatcher = Dispatcher(write_striper.source, writes)
        self.comb += write_dispatcher.sel.eq(ctrl.write_sel)
        self.submodules += write_striper, write_dispatcher

        for i in range(n):
            sink_status = Status(sinks[i])
            self.submodules += sink_status
            self.comb += [
                Record.connect(sinks[i], reads[i], leave_out=set(["stb", "ack"])),
                Record.connect(sinks[i], write_striper.sinks[i], leave_out=set(["stb", "ack"])),
                reads[i].stb.eq(sinks[i].stb & ctrl.reading),
                write_striper.sinks[i].stb.eq(sinks[i].stb & ctrl.writing),
                sinks[i].ack.eq(reads[i].ack | write_striper.sinks[i].ack),
                ctrl.ack_cmds[i].eq(sink_status.eop & sinks[i].last)
            ]


class LiteSATAMirroring(Module):
    """SATA Mirroring

    The mirroring module handles N controllers and provides N ports.
    Each port has its dedicated controller for reads:
        port0 <----> controller0
        portX <----> controllerX
        portN <----> controllerN

    Writes are mirrored on each controller:
                   (port0 write)           |            (portN write)
        port0 ----------+----> controller0 | port0 (stalled) +-----> controller0
        portX (stalled) +----> controllerX | portX (stalled) +-----> controllerX
        portN (stalled) +----> controllerN | portN ----------+-----> controllerN

    Writes have priority on reads. When a write is presented on one of the port, the
    module waits for all ongoing reads to finish and commute to write mode. Once all writes are
    serviced it returns to read mode.

    Characteristics:
        - port's visible capacity = controller's visible capacity
        - total writes throughput = (slowest) controller's throughput
        - total reads throughput = N x controller's throughput

    Can be used for data redundancy and/or to increase total reads speed.
    """
    def __init__(self, controllers):
        n = len(controllers)
        dw = len(controllers[0].sink.data)
        self.ports = [LiteSATAUserPort(dw) for i in range(n)]

        # # #

        self.submodules.ctrl = LiteSATAMirroringCtrl(n)
        self.submodules.tx = LiteSATAMirroringTX(n, dw, self.ctrl)
        self.submodules.rx = LiteSATAMirroringRX(n, dw, self.ctrl)
        for i in range(n):
            self.comb += [
                Record.connect(self.ports[i].sink, self.tx.sinks[i]),
                Record.connect(self.tx.sources[i], controllers[i].sink),

                Record.connect(controllers[i].source, self.rx.sinks[i]),
                Record.connect(self.rx.sources[i], self.ports[i].source)
            ]
