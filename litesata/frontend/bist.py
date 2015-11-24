from litesata.common import *
from litesata.core.link import Scrambler

from litex.soc.interconnect.csr import *

class LiteSATABISTGenerator(Module):
    def __init__(self, user_port):
        self.start = Signal()
        self.sector = Signal(48)
        self.count = Signal(16)
        self.random = Signal()

        self.done = Signal()
        self.aborted = Signal()
        self.errors = Signal(32)  # Note: Not used for writes

        # # #

        n = user_port.dw//32
        count_mult = user_port.dw//user_port.controller_dw

        source, sink = user_port.sink, user_port.source

        counter = Signal(32)
        counter_reset = Signal()
        counter_ce = Signal()
        self.sync += \
        	If(counter_reset,
        		counter.eq(0)
        	).Elif(counter_ce,
        		counter.eq(counter + 1)
        	)

        scrambler = scrambler = ResetInserter()(Scrambler())
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
            source.sop.eq(counter == 0),
            source.eop.eq(counter == (logical_sector_size//4*self.count)-1),
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
            source.stb.eq(1),
            If(source.stb & source.ack,
                counter_ce.eq(1),
                If(source.eop,
                    NextState("WAIT_ACK")
                )
            )
        )
        fsm.act("WAIT_ACK",
            sink.ack.eq(1),
            If(sink.stb,
                NextState("IDLE")
            )
        )
        self.sync += If(sink.stb & sink.ack, self.aborted.eq(sink.failed))


class LiteSATABISTChecker(Module):
    def __init__(self, user_port):
        self.start = Signal()
        self.sector = Signal(48)
        self.count = Signal(16)
        self.random = Signal()

        self.done = Signal()
        self.aborted = Signal()
        self.errors = Signal(32)

        # # #

        n = user_port.dw//32
        count_mult = user_port.dw//user_port.controller_dw

        source, sink = user_port.sink, user_port.source

        counter = Signal(32)
        counter_ce = Signal()
        counter_reset = Signal()
        self.sync += \
            If(counter_reset,
                counter.eq(0)
            ).Elif(counter_ce,
                counter.eq(counter + 1)
            )

        error_counter = Signal(32)
        error_counter_ce = Signal()
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
            source.sop.eq(1),
            source.eop.eq(1),
            source.read.eq(1),
            source.sector.eq(self.sector),
            source.count.eq(self.count*count_mult),
        ]
        fsm.act("SEND_CMD",
            source.stb.eq(1),
            If(source.ack,
                counter_reset.eq(1),
                NextState("WAIT_ACK")
            )
        )
        fsm.act("WAIT_ACK",
            If(sink.stb & sink.read,
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
            sink.ack.eq(1),
            If(sink.stb,
                counter_ce.eq(1),
                If(sink.data != expected_data,
                    error_counter_ce.eq(~sink.last)
                ),
                If(sink.eop,
                    If(sink.last,
                        NextState("IDLE")
                    ).Else(
                        NextState("WAIT_ACK")
                    )
                )
            )
        )
        self.sync += If(sink.stb & sink.ack, self.aborted.eq(sink.failed))


class LiteSATABISTUnitCSR(Module, AutoCSR):
    def __init__(self, bist_unit):
        self._start = CSR()
        self._sector = CSRStorage(48)
        self._count = CSRStorage(16)
        self._loops = CSRStorage(8)
        self._random = CSRStorage()

        self._done = CSRStatus()
        self._aborted = CSRStatus()
        self._errors = CSRStatus(32)
        self._cycles = CSRStatus(32)

        # # #

        self.submodules += bist_unit

        start = self._start.r & self._start.re
        done = self._done.status
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
        loop_counter = Signal(8)
        loop_counter_reset = Signal()
        loop_counter_ce = Signal()
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

        cycles_counter = Signal(32)
        cycles_counter_reset = Signal()
        cycles_counter_ce = Signal()
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


class LiteSATABISTIdentify(Module):
    def __init__(self, user_port):
        self.start = Signal()
        self.done  = Signal()
        self.data_width = user_port.dw

        fifo = SyncFIFO([("data", 32)], 512, buffered=True)
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
            source.sop.eq(1),
            source.eop.eq(1),
            source.identify.eq(1),
        ]
        fsm.act("SEND_CMD",
            source.stb.eq(1),
            If(source.stb & source.ack,
                NextState("WAIT_ACK")
            )
        )
        fsm.act("WAIT_ACK",
            If(sink.stb & sink.identify,
                NextState("RECEIVE_DATA")
            )
        )
        self.comb += fifo.sink.data.eq(sink.data)
        fsm.act("RECEIVE_DATA",
            sink.ack.eq(fifo.sink.ack),
            If(sink.stb,
                fifo.sink.stb.eq(1),
                If(sink.eop,
                    NextState("IDLE")
                )
            )
        )


class LiteSATABISTIdentifyCSR(Module, AutoCSR):
    def __init__(self, bist_identify):
        self._start = CSR()
        self._done = CSRStatus()
        self._data_width = CSRStatus(16, reset=bist_identify.data_width)
        self._source_stb = CSRStatus()
        self._source_ack = CSR()
        self._source_data = CSRStatus(32)

        # # #

        self.submodules += bist_identify
        self.comb += [
            bist_identify.start.eq(self._start.r & self._start.re),
            self._done.status.eq(bist_identify.done),

            self._source_stb.status.eq(bist_identify.source.stb),
            self._source_data.status.eq(bist_identify.source.data),
            bist_identify.source.ack.eq(self._source_ack.r & self._source_ack.re)
        ]


class LiteSATABIST(Module, AutoCSR):
    def __init__(self, crossbar, with_csr=False):
        generator = LiteSATABISTGenerator(crossbar.get_port())
        checker = LiteSATABISTChecker(crossbar.get_port())
        identify = LiteSATABISTIdentify(crossbar.get_port())
        if with_csr:
            generator = LiteSATABISTUnitCSR(generator)
            checker = LiteSATABISTUnitCSR(checker)
            identify = LiteSATABISTIdentifyCSR(identify)
        self.submodules.generator = generator
        self.submodules.checker = checker
        self.submodules.identify = identify


class LiteSATABISTRobustness(Module):
    def __init__(self, crossbar, fifo_depth=512):
        self.crossbar = crossbar
        self.fifo_depth = fifo_depth

        # inputs
        self.sector = Signal(48)
        self.count = Signal(16)
        self.write_read_n = Signal()
        self.loop_prog_n = Signal()
        self.we = Signal()
        self.flush = Signal()
        self.start = Signal()

        # outputs
        self.done = Signal()
        self.loop_index = Signal(16)
        self.loop_count = Signal(16)

        # # #

        generator = LiteSATABISTGenerator(crossbar.get_port())
        checker = LiteSATABISTChecker(crossbar.get_port())
        self.submodules.generator = generator
        self.submodules.checker = checker

         # fifo
        fifo_layout = [("sector", 48),
                       ("count", 16),
                       ("write_read_n", 1),
                       ("start", 1)]
        fifo = SyncFIFO(fifo_layout, fifo_depth)
        self.submodules += ResetInserter()(fifo)
        self.comb += fifo.reset.eq(self.flush)

        # fifo write path
        self.sync += [
            # in "loop" mode, each data read from the fifo is
            # written back
            If(self.loop_prog_n,
                fifo.sink.sector.eq(fifo.source.sector),
                fifo.sink.count.eq(fifo.source.count),
                fifo.sink.write_read_n.eq(fifo.source.write_read_n),
                fifo.sink.start.eq(fifo.source.start),
                fifo.sink.stb.eq(fifo.source.ack)
            # in "program" mode, fifo input is connected
            # to inputs
            ).Else(
                fifo.sink.sector.eq(self.sector),
                fifo.sink.count.eq(self.count),
                fifo.sink.write_read_n.eq(self.write_read_n),
                fifo.sink.start.eq(~fifo.source.stb),
                fifo.sink.stb.eq(self.we)
            )
        ]

        # fifo read path
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            If(self.start,
                NextState("CHECK")
            )
        )
        fsm.act("CHECK",
            If(fifo.source.stb,
                If(fifo.source.write_read_n,
                    NextState("WRITE_START")
                ).Else(
                    NextState("READ_START")
                )
            ).Else(
                NextState("IDLE")
            )
        )
        self.comb += [
            generator.sector.eq(fifo.source.sector),
            generator.count.eq(fifo.source.count)
        ]
        fsm.act("WRITE_START",
            generator.start.eq(1),
            NextState("WRITE_WAIT")
        )
        fsm.act("WRITE_WAIT",
            If(generator.done,
                fifo.source.ack.eq(1),
                NextState("CHECK")
            )
        )
        self.comb += [
            checker.sector.eq(fifo.source.sector),
            checker.count.eq(fifo.source.count),
        ]
        fsm.act("READ_START",
            checker.start.eq(1),
            NextState("READ_WAIT")
        )
        fsm.act("READ_WAIT",
            If(checker.done,
                fifo.source.ack.eq(1),
                NextState("CHECK")
            )
        )

        # loop_index, loop_count
        # used by the for synchronization in "loop" mode
        self.sync += \
            If(self.flush,
                self.loop_index.eq(0),
                self.loop_count.eq(0),
            ).Elif(fifo.source.stb & fifo.source.ack,
                If(fifo.source.start,
                    self.loop_index.eq(0),
                    self.loop_count.eq(self.loop_count + 1)
                ).Else(
                    self.loop_index.eq(self.loop_index + 1)
                )
            )


class LiteSATABISTRobustnessCSR(Module, AutoCSR):
    def __init__(self, bist_robustness):
        self.sector = CSRStorage(48)
        self.count = CSRStorage(16)
        self.write_read_n = CSRStorage()
        self.loop_prog_n = CSRStorage()
        self.we = CSR()
        self.flush = CSR()
        self.start = CSR()

        self.done = CSRStatus()
        self.loop_index = CSRStatus(16)
        self.loop_count = CSRStatus(16)

        # # #

        self.submodules += bist_robustness

        identify = LiteSATABISTIdentify(bist_robustness.crossbar.get_port())
        self.submodules.identify = LiteSATABISTIdentifyCSR(identify)

        self.comb += [
            bist_robustness.sector.eq(self.sector.storage),
            bist_robustness.count.eq(self.count.storage),
            bist_robustness.write_read_n.eq(self.write_read_n.storage),
            bist_robustness.loop_prog_n.eq(self.loop_prog_n.storage),
            bist_robustness.we.eq(self.we.r & self.we.re),
            bist_robustness.flush.eq(self.flush.r & self.flush.re),
            bist_robustness.start.eq(self.start.r & self.start.re),

            self.done.status.eq(bist_robustness.done),
            self.loop_index.status.eq(bist_robustness.loop_index),
            self.loop_count.status.eq(bist_robustness.loop_count)
        ]
