#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from functools import reduce
from operator import and_, or_

from litesata.common import *
from litesata.frontend.arbitration import LiteSATAUserPort

from litex.soc.interconnect.packet import Status, Arbiter, Dispatcher

# LiteSATA Striping --------------------------------------------------------------------------------

class LiteSATAStripingTX(Module):
    """SATA Striping TX

    Split cmds and writes data on N different controllers.

    This module provides a mirroring_mode that is used by the mirroring module to
    dispatch identicals writes to the controllers. This avoid code duplication in
    between striping and mirroring modules. In this special case, port's data width
    is dw (same as controllers)
    """
    def __init__(self, n, dw, mirroring_mode=False):
        self.sink = sink = stream.Endpoint(command_tx_description(dw*n if not mirroring_mode else dw))
        self.sources = sources = [stream.Endpoint(command_tx_description(dw)) for i in range(n)]

        # # #

        split = Signal()

        already_acked = Signal(n)
        self.sync += If(split & sink.valid,
                already_acked.eq(already_acked | Cat(*[s.ready for s in sources])),
                If(sink.ready, already_acked.eq(0))
            )

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            sink.ready.eq(0),
            If(sink.valid,
                NextState("SPLIT")
            ).Else(
                sink.ready.eq(1)
            )
        )

        # split data and ctrl signals (except valid & ready managed in fsm)
        for i, s in enumerate(sources):
            self.comb += sink.connect(s, omit=set(["valid", "ready", "data"]))
            if mirroring_mode:
                self.comb += s.data.eq(sink.data)
            else:
                self.comb += s.data.eq(sink.data[i*dw:(i+1)*dw])

        fsm.act("SPLIT",
            split.eq(1),
            [s.valid.eq(sink.valid & ~already_acked[i]) for i, s in enumerate(sources)],
            sink.ready.eq(reduce(and_, [s.ready | already_acked[i] for i, s in enumerate(sources)])),
            If(sink.valid & sink.last & sink.ready,
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
        self.sinks = sinks = [stream.Endpoint(command_rx_description(dw)) for i in range(n)]
        self.source = source = stream.Endpoint(command_rx_description(dw*n if not mirroring_mode else dw))

        # # #

        valid_all = Signal()
        self.comb += valid_all.eq(reduce(and_, [s.valid for s in sinks]))

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            If(valid_all,
                NextState("COMBINE")
            )
        )

        # use first sink for ctrl signals (except for valid, ready & failed)
        self.comb += sinks[0].connect(source, omit=set(["valid", "ready", "failed", "data"]))
        # combine datas
        if mirroring_mode:
            self.comb += source.data.eq(0) # mirroring only used for writes
        else:
            for i, s in enumerate(sinks):
                self.comb += source.data[i*dw:(i+1)*dw].eq(s.data)


        fsm.act("COMBINE",
            source.failed.eq(reduce(or_, [s.failed for s in sinks])), # XXX verify this is enough
            source.valid.eq(reduce(and_, [s.valid for s in sinks])),
            [s.ready.eq(source.valid & source.ready) for s in sinks],
            If(source.valid & source.last & source.ready,
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

        n  = len(controllers)
        dw = len(controllers[0].sink.data)

        self.submodules.tx = LiteSATAStripingTX(n, dw)
        self.submodules.rx = LiteSATAStripingRX(n, dw)
        for i in range(n):
            self.comb += [
                self.tx.sources[i].connect(controllers[i].sink),
                controllers[i].source.connect(self.rx.sinks[i])
            ]
        self.sink, self.source = self.tx.sink, self.rx.source

# LiteSATA Mirroring -------------------------------------------------------------------------------

class LiteSATAMirroringCtrl(Module):
    def __init__(self, n):
        self.new_cmds    = Signal(n)
        self.ack_cmds    = Signal(n)

        self.reading     = Signal()
        self.writing     = Signal()

        self.wants_write = Signal()
        self.write_sel   = Signal(max=n)

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
        self.sinks = sinks = [stream.Endpoint(command_tx_description(dw)) for i in range(n)]
        self.sources = sources = [stream.Endpoint(command_tx_description(dw)) for i in range(n)]

        # # #

        wants_write = Signal()

        reading = Signal()
        writing = Signal()

        reads  = [stream.Endpoint(command_tx_description(dw)) for i in range(dw)]
        writes = [stream.Endpoint(command_tx_description(dw)) for i in range(dw)]
        for sink, read, write in zip(sinks, reads, writes):
            read_stall  = Signal()
            read_status = Status(read)
            self.submodules += read_status
            self.comb += [
                sink.connect(read, omit=set(["valid", "ready"])),
                sink.connect(write, omit=set(["valid", "ready"])),
                read.valid.eq(sink.valid & (sink.read | sink.identify) & ~read_stall),
                write.valid.eq(sink.valid & sink.write),
                If(sink.read | sink.identify,
                    sink.ready.eq((read.ready & ~read_stall))
                ).Else(
                    sink.ready.eq(write.ready)
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
                    reads[i].connect(sources[i]) # independent reads
                ).Elif(ctrl.writing,
                    write_striper.sources[i].connect(sources[i]) # identical writes
                ),
                ctrl.new_cmds[i].eq(source_status.last)
            ]
        write_striper_sink_status = Status(write_striper.sink)
        self.submodules += write_striper_sink_status
        self.comb += [
            ctrl.wants_write.eq(write_striper_sink_status.ongoing),
            ctrl.write_sel.eq(write_arbiter.rr.grant)
        ]


class LiteSATAMirroringRX(Module):
    def __init__(self, n, dw, ctrl):
        self.sinks = sinks = [stream.Endpoint(command_rx_description(dw)) for i in range(n)]
        self.sources = sources = [stream.Endpoint(command_rx_description(dw)) for i in range(n)]

        # # #

        muxs = [Multiplexer(command_rx_description(dw), 2) for i in range(n)]
        self.submodules += muxs

        writes = [mux.sink0 for mux in muxs]
        reads  = [mux.sink1 for mux in muxs]

        for mux, source in zip(muxs, sources):
            self.comb += [
                mux.sel.eq(ctrl.reading),
                mux.source.connect(source)
            ]

        write_striper = LiteSATAStripingRX(n, dw, mirroring_mode=True)
        write_dispatcher = Dispatcher(write_striper.source, writes)
        self.comb += write_dispatcher.sel.eq(ctrl.write_sel)
        self.submodules += write_striper, write_dispatcher

        for i in range(n):
            sink_status = Status(sinks[i])
            self.submodules += sink_status
            self.comb += [
                sinks[i].connect(reads[i], omit=set(["valid", "ready"])),
                sinks[i].connect(write_striper.sinks[i], omit=set(["valid", "ready"])),
                reads[i].valid.eq(sinks[i].valid & ctrl.reading),
                write_striper.sinks[i].valid.eq(sinks[i].valid & ctrl.writing),
                sinks[i].ready.eq(reads[i].ready | write_striper.sinks[i].ready),
                ctrl.ack_cmds[i].eq(sink_status.last & sinks[i].end)
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
        n  = len(controllers)
        dw = len(controllers[0].sink.data)
        self.ports = [LiteSATAUserPort(dw) for i in range(n)]

        # # #

        self.submodules.ctrl = LiteSATAMirroringCtrl(n)
        self.submodules.tx   = LiteSATAMirroringTX(n, dw, self.ctrl)
        self.submodules.rx   = LiteSATAMirroringRX(n, dw, self.ctrl)
        for i in range(n):
            self.comb += [
                self.ports[i].sink.connect(self.tx.sinks[i]),
                self.tx.sources[i].connect(controllers[i].sink),

                controllers[i].source.connect(self.rx.sinks[i]),
                self.rx.sources[i].connect(self.ports[i].source)
            ]
