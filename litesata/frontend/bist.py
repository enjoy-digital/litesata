#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2016 Olof Kindgren <olof.kindgren@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *
from litesata.core.link import Scrambler

from litex.soc.interconnect.csr import *

# LiteSATABISTGenerator ----------------------------------------------------------------------------

class LiteSATABISTGenerator(Module):
    def __init__(self, user_port, count_width=32):
        self.start   = Signal()
        self.sector  = Signal(48)
        self.count   = Signal(16)
        self.random  = Signal()

        self.done    = Signal()
        self.aborted = Signal()
        self.errors  = Signal(32)  # Note: Not used for writes

        # # #

        n          = user_port.dw//32
        count_mult = user_port.dw//user_port.controller_dw
        if count_mult == 0:
            raise ValueError

        source, sink = user_port.sink, user_port.source

        count     = Signal(count_width)
        scrambler = ResetInserter()(Scrambler())
        self.submodules += scrambler

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            scrambler.reset.eq(1),
            NextValue(count, 0),
            If(self.start,
                NextState("SEND-CMD-AND-DATA")
            )
        )
        self.comb += [
            source.last.eq(count == (logical_sector_size//4*self.count)-1),
            source.write.eq(1),
            source.sector.eq(self.sector),
            source.count.eq(self.count*count_mult),
            If(self.random,
                source.data.eq(Replicate(scrambler.value, n))
            ).Else(
                source.data.eq(Replicate(count, n))
            )
        ]
        fsm.act("SEND-CMD-AND-DATA",
            source.valid.eq(1),
            If(source.valid & source.ready,
                scrambler.ce.eq(1),
                NextValue(count, count + 1),
                If(source.last,
                    NextState("WAIT-ACK")
                )
            )
        )
        fsm.act("WAIT-ACK",
            sink.ready.eq(1),
            If(sink.valid,
                NextState("IDLE")
            )
        )

        self.sync += [
            If(self.start,
                self.aborted.eq(0)
            ).Elif(sink.valid & sink.ready,
                self.aborted.eq(self.aborted | sink.failed)
            )
        ]

# LiteSATABISTChecker ------------------------------------------------------------------------------

class LiteSATABISTChecker(Module):
    def __init__(self, user_port, count_width=32):
        self.start   = Signal()
        self.sector  = Signal(48)
        self.count   = Signal(16)
        self.random  = Signal()

        self.done    = Signal()
        self.aborted = Signal()
        self.errors  = Signal(count_width)

        # # #

        n          = user_port.dw//32
        count_mult = user_port.dw//user_port.controller_dw
        if count_mult == 0:
            raise ValueError

        source, sink = user_port.sink, user_port.source

        count  = Signal(count_width)
        errors = Signal(count_width)
        self.comb += self.errors.eq(errors)

        scrambler = ResetInserter()(Scrambler())
        self.submodules += scrambler

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            scrambler.reset.eq(1),
            NextValue(count, 0),
            If(self.start,
                NextValue(errors, 0),
                NextState("SEND-CMD")
            )
        )
        self.comb += [
            source.last.eq(1),
            source.read.eq(1),
            source.sector.eq(self.sector),
            source.count.eq(self.count*count_mult),
        ]
        fsm.act("SEND-CMD",
            source.valid.eq(1),
            If(source.ready,
                NextState("WAIT-ACK")
            )
        )
        fsm.act("WAIT-ACK",
            If(sink.valid & sink.read,
                NextState("RECEIVE-DATA")
            )
        )
        expected_data = Signal(n*32)
        self.comb += [
            If(self.random,
                expected_data.eq(Replicate(scrambler.value, n))
            ).Else(
                expected_data.eq(Replicate(count, n))
            )
        ]
        fsm.act("RECEIVE-DATA",
            sink.ready.eq(1),
            If(sink.valid,
                scrambler.ce.eq(1),
                NextValue(count, count + 1),
                If(sink.data != expected_data,
                    If(~sink.end,
                        NextValue(errors, errors + 1)
                    )
                ),
                If(sink.end,
                    If(sink.last,
                        NextState("IDLE")
                    ).Else(
                        NextState("WAIT-ACK")
                    )
                )
            )
        )

        self.sync += [
            If(self.start,
                self.aborted.eq(0)
            ).Elif(sink.valid & sink.ready,
                self.aborted.eq(self.aborted | sink.failed)
            )
        ]

# LiteSATABISTUnitCSR ------------------------------------------------------------------------------

class LiteSATABISTUnitCSR(Module, AutoCSR):
    def __init__(self, bist_unit):
        self._start   = CSR()
        self._sector  = CSRStorage(48)
        self._count   = CSRStorage(16)
        self._loops   = CSRStorage(8)
        self._random  = CSRStorage()

        self._done    = CSRStatus()
        self._aborted = CSRStatus()
        self._errors  = CSRStatus(32)
        self._cycles  = CSRStatus(32)

        # # #

        self.submodules += bist_unit

        start = self._start.r & self._start.re
        done  = self._done.status
        loops = self._loops.storage

        self.comb += [
            bist_unit.sector.eq(self._sector.storage),
            bist_unit.count.eq(self._count.storage),
            bist_unit.random.eq(self._random.storage),

            self._aborted.status.eq(bist_unit.aborted),
            self._errors.status.eq(bist_unit.errors)
        ]

        loop   = Signal(8)
        cycles = Signal(32)
        self.comb += self._cycles.status.eq(cycles)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self._done.status.eq(1),
            NextValue(loop, 0),
            If(start,
                NextValue(cycles, 0),
                NextState("CHECK")
            )
        )
        fsm.act("CHECK",
            If(loop < loops,
                NextState("START")
            ).Else(
                NextState("IDLE")
            ),
            NextValue(cycles, cycles + 1),
        )
        fsm.act("START",
            bist_unit.start.eq(1),
            NextState("WAIT-DONE"),
            NextValue(cycles, cycles + 1),
        )
        fsm.act("WAIT-DONE",
            If(bist_unit.done,
                NextValue(loop, loop + 1),
                NextState("CHECK")
            ),
            NextValue(cycles, cycles + 1),
        )


# LiteSATABISTIdentify -----------------------------------------------------------------------------

class LiteSATABISTIdentify(Module):
    def __init__(self, user_port):
        self.start      = Signal()
        self.done       = Signal()
        self.data_width = user_port.dw

        fifo = ResetInserter()(stream.SyncFIFO([("data", 32)], 512, buffered=True))
        self.submodules += fifo
        self.source = fifo.source

        # # #

        source, sink = user_port.sink, user_port.source

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            If(self.start,
                NextState("SEND-CMD")
            )
        )
        self.comb += [
            source.last.eq(1),
            source.identify.eq(1),
        ]
        fsm.act("SEND-CMD",
            fifo.reset.eq(1),
            source.valid.eq(1),
            If(source.valid & source.ready,
                NextState("WAIT-ACK")
            )
        )
        fsm.act("WAIT-ACK",
            If(sink.valid & sink.identify,
                NextState("RECEIVE-DATA")
            )
        )
        self.comb += fifo.sink.data.eq(sink.data)
        fsm.act("RECEIVE-DATA",
            sink.ready.eq(fifo.sink.ready),
            If(sink.valid,
                fifo.sink.valid.eq(1),
                If(sink.last,
                    NextState("IDLE")
                )
            )
        )

# LiteSATABISTIdentifyCSR --------------------------------------------------------------------------

class LiteSATABISTIdentifyCSR(Module, AutoCSR):
    def __init__(self, bist_identify):
        self._start        = CSR()
        self._done         = CSRStatus()
        self._data_width   = CSRStatus(16, reset=bist_identify.data_width)
        self._source_valid = CSRStatus()
        self._source_ready = CSR()
        self._source_data  = CSRStatus(32)

        # # #

        self.submodules += bist_identify
        self.comb += [
            bist_identify.start.eq(self._start.r & self._start.re),
            self._done.status.eq(bist_identify.done),

            self._source_valid.status.eq(bist_identify.source.valid),
            self._source_data.status.eq(bist_identify.source.data),
            bist_identify.source.ready.eq(self._source_ready.r & self._source_ready.re)
        ]

# LiteSATABIST --------------------------------------------------------------------------

class LiteSATABIST(Module, AutoCSR):
    def __init__(self, crossbar, with_csr=False, count_width=32):
        generator = LiteSATABISTGenerator(crossbar.get_port(), count_width)
        checker   = LiteSATABISTChecker(crossbar.get_port(), count_width)
        identify  = LiteSATABISTIdentify(crossbar.get_port())
        if with_csr:
            generator = LiteSATABISTUnitCSR(generator)
            checker   = LiteSATABISTUnitCSR(checker)
            identify  = LiteSATABISTIdentifyCSR(identify)
        self.submodules.generator = generator
        self.submodules.checker   = checker
        self.submodules.identify  = identify
