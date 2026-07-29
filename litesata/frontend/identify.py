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

        # IDENTIFY returns exactly one 512-byte data block, but a drive can
        # deliver it as several DATA FISes, so a per-FIS last cannot be used
        # as the end of the transfer.
        ndwords = logical_sector_size*8//user_port.dw
        count   = Signal(max=ndwords)

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
            NextValue(count, 0),
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
                If(sink.ready,
                    NextValue(count, count + 1),
                    If(count == (ndwords - 1),
                        NextState("IDLE")
                    )
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

        self.bist_identify = bist_identify # exposed for analyzer probing
        self.submodules += bist_identify
        self.comb += [
            bist_identify.start.eq(self._start.wr_data & self._start.wr_stb),
            self._done.status.eq(bist_identify.done),

            self._source_valid.status.eq(bist_identify.source.valid),
            self._source_data.status.eq(bist_identify.source.data),
            bist_identify.source.ready.eq(self._source_ready.wr_data & self._source_ready.wr_stb)
        ]

# LiteSATASoftReset --------------------------------------------------------------------------------

class LiteSATASoftReset(Module):
    """Issue the two control FISes of the ATA software-reset protocol."""
    def __init__(self, user_port, reset_cycles):
        if reset_cycles < 1:
            raise ValueError("reset_cycles must be greater than zero")

        self.start = Signal()
        self.done  = Signal()

        # # #

        source, sink = user_port.sink, user_port.source
        count = Signal(max=reset_cycles)

        self.comb += [
            source.last.eq(1),
            source.soft_reset.eq(1),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            NextValue(count, 0),
            If(self.start,
                NextState("SEND-ASSERT")
            )
        )
        fsm.act("SEND-ASSERT",
            source.valid.eq(1),
            source.control.eq(0x0c),  # nIEN | SRST.
            If(source.valid & source.ready,
                NextState("WAIT-ASSERT-ACK")
            )
        )
        fsm.act("WAIT-ASSERT-ACK",
            sink.ready.eq(1),
            If(sink.valid & sink.last & sink.end,
                NextState("HOLD")
            )
        )
        fsm.act("HOLD",
            If(count == (reset_cycles - 1),
                NextState("SEND-DEASSERT")
            ).Else(
                NextValue(count, count + 1)
            )
        )
        fsm.act("SEND-DEASSERT",
            source.valid.eq(1),
            source.control.eq(0x08),  # nIEN, SRST cleared.
            If(source.valid & source.ready,
                NextState("WAIT-DEASSERT-ACK")
            )
        )
        fsm.act("WAIT-DEASSERT-ACK",
            sink.ready.eq(1),
            If(sink.valid & sink.last & sink.end,
                NextState("IDLE")
            )
        )

# LiteSATASoftResetCSR -----------------------------------------------------------------------------

class LiteSATASoftResetCSR(Module, AutoCSR):
    def __init__(self, soft_reset):
        self._start = CSR()
        self._done  = CSRStatus()

        # # #

        self.soft_reset = soft_reset
        self.submodules += soft_reset
        self.comb += [
            soft_reset.start.eq(self._start.wr_data & self._start.wr_stb),
            self._done.status.eq(soft_reset.done),
        ]
