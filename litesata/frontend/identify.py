#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *

from litex.soc.interconnect.csr import *

# LiteSATAIdentify ---------------------------------------------------------------------------------

class LiteSATAIdentify(Module):
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

# LiteSATAIdentifyCSR ------------------------------------------------------------------------------

class LiteSATAIdentifyCSR(Module, AutoCSR):
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
