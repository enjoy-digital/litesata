#
# This file is part of LiteSATA.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from math import log2

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from litex.soc.cores.dma import WishboneDMAWriter

from litesata.common import logical_sector_size

# SATA Sector2Mem DMA ------------------------------------------------------------------------------

class LiteSATASector2MemDMA(Module, AutoCSR):
    """Sector to Memory DMA

    Read a sector from the SATA core and write it to memory through DMA.
    """
    def __init__(self, port, bus, endianness="little"):
        self.port   = port
        self.bus    = bus
        self.sector = CSRStorage(48)
        self.base   = CSRStorage(64)
        self.start  = CSR()
        self.done   = CSRStatus()
        self.error  = CSRStatus()

        # # #

        port_bytes = port.dw//8
        dma_bytes  = bus.data_width//8
        count      = Signal(max=logical_sector_size//dma_bytes)

        # Sector buffer
        buf = stream.SyncFIFO([("data", port.dw)], logical_sector_size//port_bytes)
        self.submodules.buf = buf

        # Converter
        conv = stream.Converter(nbits_from=port.dw, nbits_to=bus.data_width)
        self.submodules.conv = conv

        # Connect Port to Sector Buffer
        self.comb += port.source.connect(buf.sink, keep={"valid", "ready", "last", "data"})

        # Connect Sector Buffer to Converter
        self.comb += buf.source.connect(conv.sink)

        # DMA
        dma = WishboneDMAWriter(bus, with_csr=False, endianness=endianness)
        self.submodules.dma = dma

        # Control FSM
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.start.re,
                NextValue(count,             0),
                NextValue(self.error.status, 0),
                NextState("SEND-CMD")
            ).Else(
                self.done.status.eq(1)
            ),
            conv.source.ready.eq(1)
        )
        fsm.act("SEND-CMD",
            # Send read command for 1 Sector.
            port.sink.valid.eq(1),
            port.sink.last.eq(1),
            port.sink.read.eq(1),
            port.sink.sector.eq(self.sector.storage),
            port.sink.count.eq(1),
            If(port.sink.ready,
                NextState("RECEIVE-DATA-DMA")
            )
        )
        fsm.act("RECEIVE-DATA-DMA",
            # Connect Converter to DMA.
            dma.sink.valid.eq(conv.source.valid),
            dma.sink.address.eq(self.base.storage[int(log2(dma_bytes)):] + count),
            dma.sink.data.eq(reverse_bytes(conv.source.data)),
            conv.source.ready.eq(dma.sink.ready),
            If(dma.sink.valid & dma.sink.ready,
                NextValue(count, count + 1),
                If(dma.sink.last,
                    NextState("IDLE")
                )
            ),

            # Manage errors
            If(port.source.valid & port.source.ready,
                If(port.source.failed,
                    NextValue(self.error.status, 1),
                    NextState("IDLE"),
                )
            )
        )
