#
# This file is part of LiteSATA.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from litex.soc.cores.dma import WishboneDMAWriter

from litesata.common import logical_sector_size

# SATA Block2Mem DMA ---------------------------------------------------------------------------------

class LiteSATABlock2MemDMA(Module, AutoCSR):
    """Block to Memory DMA

    Read a of block from the SATA drive and write it to memory through DMA.
    """
    def __init__(self, user_port, bus, endianness="little"):
        self.user_port = user_port
        self.bus       = bus
        assert user_port.dw   == 32 # FIXME: remove limitation.
        assert bus.data_width == 32 # FIXME: remove limitation.
        self.sector = CSRStorage(48)
        self.base   = CSRStorage(64)
        self.start  = CSR()
        self.done   = CSRStatus()
        self.error  = CSRStatus()

        # # #

        count = Signal(max=logical_sector_size//4)

        # DMA
        self.submodules.dma  = dma = WishboneDMAWriter(bus, with_csr=False, endianness=endianness)

        # FIFO
        self.submodules.fifo = fifo = stream.SyncFIFO([("data", 32), ("failed", 1)], logical_sector_size//4)
        self.comb += user_port.source.connect(fifo.sink, keep={"valid", "ready", "last", "data"})
        self.comb += fifo.sink.failed.eq(user_port.source.failed)

        # FSM
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.start.re,
                NextValue(count, 0),
                NextValue(self.error.status, 0),
                NextState("SEND-CMD")
            ).Else(
                self.done.status.eq(1)
            ),
            fifo.source.ready.eq(1),
        )
        fsm.act("SEND-CMD",
            user_port.sink.valid.eq(1),
            user_port.sink.last.eq(1),
            user_port.sink.read.eq(1),
            user_port.sink.sector.eq(self.sector.storage),
            user_port.sink.count.eq(1),
            If(user_port.sink.ready,
                NextState("RECEIVE_DATA")
            )
        )
        fsm.act("RECEIVE_DATA",
            dma.sink.valid.eq(fifo.source.valid),
            dma.sink.address.eq(self.base.storage[2:] + count),
            dma.sink.data.eq(reverse_bytes(fifo.source.data)),
            fifo.source.ready.eq(dma.sink.ready),
            If(fifo.source.valid & fifo.source.ready,
                NextValue(count, count + 1),
                If(fifo.source.last,
                     NextState("IDLE")
                ),
                If(fifo.source.failed,
                    NextValue(self.error.status, 1),
                    NextState("IDLE")
                )
            )
        )
