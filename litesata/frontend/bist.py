#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2016 Olof Kindgren <olof.kindgren@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *
from litesata.core.link import Scrambler

from litex.soc.interconnect.csr import *

# LiteSATABISTGenerator ----------------------------------------------------------------------------

class LiteSATABISTGenerator(Module):
    def __init__(self, user_port, counter_width=32):
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

        counter       = Signal(counter_width)
        counter_reset = Signal()
        counter_ce    = Signal()
        self.sync += \
            If(counter_reset,
                counter.eq(0)
            ).Elif(counter_ce,
                counter.eq(counter + 1)
            )

        scrambler = ResetInserter()(Scrambler())
        self.submodules += scrambler
        self.comb += [
            scrambler.reset.eq(counter_reset),
            scrambler.ce.eq(counter_ce)
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            self.done.eq(1),
            counter_reset.eq(1),
            If(self.start,
                NextState("SEND_CMD_AND_DATA")
            )
        )
        self.comb += [
            source.last.eq(counter == (logical_sector_size//4*self.count)-1),
            source.write.eq(1),
            source.sector.eq(self.sector),
            source.count.eq(self.count*count_mult),
            If(self.random,
                source.data.eq(Replicate(scrambler.value, n))
            ).Else(
                source.data.eq(Replicate(counter, n))
            )
        ]
        fsm.act("SEND_CMD_AND_DATA",
            source.valid.eq(1),
            If(source.valid & source.ready,
                counter_ce.eq(1),
                If(source.last,
                    NextState("WAIT_ACK")
                )
            )
        )
        fsm.act("WAIT_ACK",
            sink.ready.eq(1),
            If(sink.valid,
                NextState("IDLE")
            )
        )

        self.sync += \
            If(self.start,
                self.aborted.eq(0)
            ).Elif(sink.valid & sink.ready,
                self.aborted.eq(self.aborted | sink.failed)
            )

# LiteSATABISTChecker ------------------------------------------------------------------------------

class LiteSATABISTChecker(Module):
    def __init__(self, user_port, counter_width=32):
        self.start   = Signal()
        self.sector  = Signal(48)
        self.count   = Signal(16)
        self.random  = Signal()

        self.done    = Signal()
        self.aborted = Signal()
        self.errors  = Signal(counter_width)

        # # #

        n          = user_port.dw//32
        count_mult = user_port.dw//user_port.controller_dw
        if count_mult == 0:
            raise ValueError

        source, sink = user_port.sink, user_port.source

        counter       = Signal(counter_width)
        counter_ce    = Signal()
        counter_reset = Signal()
        self.sync += \
            If(counter_reset,
                counter.eq(0)
            ).Elif(counter_ce,
                counter.eq(counter + 1)
            )

        error_counter       = Signal(counter_width)
        error_counter_ce    = Signal()
        error_counter_reset = Signal()
        self.sync += \
            If(error_counter_reset,
                error_counter.eq(0)
            ).Elif(error_counter_ce,
                error_counter.eq(error_counter + 1)
            )

        self.comb += self.errors.eq(error_counter)

        scrambler = ResetInserter()(Scrambler())
        self.submodules += scrambler
        self.comb += [
            scrambler.reset.eq(counter_reset),
            scrambler.ce.eq(counter_ce)
        ]

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += self.fsm
        fsm.act("IDLE",
            self.done.eq(1),
            counter_reset.eq(1),
            If(self.start,
                error_counter_reset.eq(1),
                NextState("SEND_CMD")
            )
        )
        self.comb += [
            source.last.eq(1),
            source.read.eq(1),
            source.sector.eq(self.sector),
            source.count.eq(self.count*count_mult),
        ]
        fsm.act("SEND_CMD",
            source.valid.eq(1),
            If(source.ready,
                counter_reset.eq(1),
                NextState("WAIT_ACK")
            )
        )
        fsm.act("WAIT_ACK",
            If(sink.valid & sink.read,
                NextState("RECEIVE_DATA")
            )
        )
        expected_data = Signal(n*32)
        self.comb += \
            If(self.random,
                expected_data.eq(Replicate(scrambler.value, n))
            ).Else(
                expected_data.eq(Replicate(counter, n))
            )
        fsm.act("RECEIVE_DATA",
            sink.ready.eq(1),
            If(sink.valid,
                counter_ce.eq(1),
                If(sink.data != expected_data,
                    error_counter_ce.eq(~sink.end)
                ),
                If(sink.end,
                    If(sink.last,
                        NextState("IDLE")
                    ).Else(
                        NextState("WAIT_ACK")
                    )
                )
            )
        )

        self.sync += \
            If(self.start,
                self.aborted.eq(0)
            ).Elif(sink.valid & sink.ready,
                self.aborted.eq(self.aborted | sink.failed)
            )

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

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        loop_counter       = Signal(8)
        loop_counter_reset = Signal()
        loop_counter_ce    = Signal()
        self.sync += \
            If(loop_counter_reset,
                loop_counter.eq(0)
            ).Elif(loop_counter_ce,
                loop_counter.eq(loop_counter + 1)
            )

        fsm.act("IDLE",
            self._done.status.eq(1),
            loop_counter_reset.eq(1),
            If(start,
                NextState("CHECK")
            )
        )
        fsm.act("CHECK",
            If(loop_counter < loops,
                NextState("START")
            ).Else(
                NextState("IDLE")
            )
        )
        fsm.act("START",
            bist_unit.start.eq(1),
            NextState("WAIT_DONE")
        )
        fsm.act("WAIT_DONE",
            If(bist_unit.done,
                loop_counter_ce.eq(1),
                NextState("CHECK")
            )
        )

        cycles_counter       = Signal(32)
        cycles_counter_reset = Signal()
        cycles_counter_ce    = Signal()
        self.sync += \
            If(cycles_counter_reset,
                cycles_counter.eq(0)
            ).Elif(cycles_counter_ce,
                cycles_counter.eq(cycles_counter + 1)
            )

        self.sync += [
            cycles_counter_reset.eq(start),
            cycles_counter_ce.eq(~fsm.ongoing("IDLE")),
            self._cycles.status.eq(cycles_counter)
        ]

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

        self.fsm = fsm = FSM(reset_state="IDLE")
        self.submodules += fsm
        fsm.act("IDLE",
            self.done.eq(1),
            If(self.start,
                NextState("SEND_CMD")
            )
        )
        self.comb += [
            source.last.eq(1),
            source.identify.eq(1),
        ]
        fsm.act("SEND_CMD",
            fifo.reset.eq(1),
            source.valid.eq(1),
            If(source.valid & source.ready,
                NextState("WAIT_ACK")
            )
        )
        fsm.act("WAIT_ACK",
            If(sink.valid & sink.identify,
                NextState("RECEIVE_DATA")
            )
        )
        self.comb += fifo.sink.data.eq(sink.data)
        fsm.act("RECEIVE_DATA",
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
    def __init__(self, crossbar, with_csr=False, counter_width=32):
        generator = LiteSATABISTGenerator(crossbar.get_port(), counter_width)
        checker   = LiteSATABISTChecker(crossbar.get_port(), counter_width)
        identify  = LiteSATABISTIdentify(crossbar.get_port())
        if with_csr:
            generator = LiteSATABISTUnitCSR(generator)
            checker   = LiteSATABISTUnitCSR(checker)
            identify  = LiteSATABISTIdentifyCSR(identify)
        self.submodules.generator = generator
        self.submodules.checker   = checker
        self.submodules.identify  = identify
