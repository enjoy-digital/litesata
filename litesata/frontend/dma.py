#
# This file is part of LiteSATA.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 Gabriel L. Somlo <gsomlo@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from math import log2

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from litex.soc.cores.dma import WishboneDMAWriter, WishboneDMAReader

from litesata.common import logical_sector_size

# SATA Sector2Mem DMA ------------------------------------------------------------------------------

class LiteSATASector2MemDMA(LiteXModule):
    """Sector to Memory DMA

    Read a sector from the SATA core and write it to memory through DMA.
    """
    def __init__(self, port, bus, endianness="little"):
        self.port     = port
        self.bus      = bus
        self.sector   = CSRStorage(48)
        self.nsectors = CSRStorage(16)
        self.base     = CSRStorage(64)
        self.start    = CSR()
        self.done     = CSRStatus()
        self.error    = CSRStatus()
        self.irq      = Signal()

        # # #

        port_bytes = port.dw//8
        dma_bytes  = bus.data_width//8
        count      = Signal(max=logical_sector_size//dma_bytes)
        crt_sec    = Signal(48)
        crt_base   = Signal(64)

        # Sector buffer
        self.buf = buf = stream.SyncFIFO([("data", port.dw)], logical_sector_size//port_bytes)

        # Converter
        self.conv = conv = stream.Converter(nbits_from=port.dw, nbits_to=bus.data_width)

        # Connect Port to Sector Buffer
        self.comb += port.source.connect(buf.sink, keep={"valid", "ready", "last", "data"})

        # Connect Sector Buffer to Converter
        self.comb += buf.source.connect(conv.sink)

        # DMA
        self.dma = dma = WishboneDMAWriter(bus, with_csr=False, endianness=endianness)

        # Control FSM
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.start.re,
                NextValue(count,             0),
                NextValue(crt_sec,           self.sector.storage),
                NextValue(crt_base,          self.base.storage),
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
            port.sink.sector.eq(crt_sec),
            port.sink.count.eq(1),
            If(port.sink.ready,
                NextState("RECEIVE-DATA-DMA")
            )
        )
        fsm.act("RECEIVE-DATA-DMA",
            # Connect Converter to DMA.
            dma.sink.valid.eq(conv.source.valid),
            dma.sink.last.eq(conv.source.last),
            dma.sink.address.eq(crt_base[int(log2(dma_bytes)):] + count),
            dma.sink.data.eq(reverse_bytes(conv.source.data)),
            conv.source.ready.eq(dma.sink.ready),
            If(dma.sink.valid & dma.sink.ready,
                NextValue(count, count + 1),
                If(dma.sink.last,
                    self.irq.eq(1),
                    NextState("SECTOR-LOOP")
                )
            ),

            # Monitor errors
            If(port.source.valid & port.source.ready & port.source.failed,
                self.irq.eq(1),
                NextValue(self.error.status, 1),
                NextState("IDLE"),
            )
        )
        fsm.act("SECTOR-LOOP",
            If(crt_sec == (self.sector.storage + self.nsectors.storage - 1),
                self.irq.eq(1),
                NextState("IDLE")
            ).Else(
                NextValue(count,    0),
                NextValue(crt_sec,  crt_sec + 1),
                NextValue(crt_base, crt_base + 512),
                NextValue(self.error.status, 0),
                conv.source.ready.eq(1),
                NextState("SEND-CMD")
            )
        )

# SATA Mem2Sector DMA ------------------------------------------------------------------------------

class LiteSATAMem2SectorDMA(LiteXModule):
    """Memory 2 Sector DMA

    Read memory through DMA and write it to a sector of the SATA core.
    """
    def __init__(self, bus, port, endianness="little"):
        self.bus      = bus
        self.port     = port
        self.sector   = CSRStorage(48)
        self.nsectors = CSRStorage(16)
        self.base     = CSRStorage(64)
        self.start    = CSR()
        self.done     = CSRStatus()
        self.error    = CSRStatus()
        self.irq      = Signal()

        # # #

        dma_bytes  = bus.data_width//8
        port_bytes = port.dw//8
        count      = Signal(max=logical_sector_size//min(dma_bytes, port_bytes))
        crt_sec    = Signal(48)
        crt_base   = Signal(64)

        # DMA
        self.dma = dma = WishboneDMAReader(bus, with_csr=False, endianness=endianness)

        # Sector buffer
        self.buf = buf = stream.SyncFIFO([("data", port.dw)], logical_sector_size//dma_bytes)

        # Converter
        self.conv = conv = stream.Converter(nbits_from=bus.data_width, nbits_to=port.dw)

        # Connect DMA to Sector Buffer
        self.comb += dma.source.connect(buf.sink)

        # Connect Sector Buffer to Converter
        self.comb += buf.source.connect(conv.sink)

        # Control FSM
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.start.re,
                NextValue(count,             0),
                NextValue(crt_sec,           self.sector.storage),
                NextValue(crt_base,          self.base.storage),
                NextValue(self.error.status, 0),
                NextState("READ-DATA-DMA")
            ).Else(
                self.done.status.eq(1)
            ),
            conv.source.ready.eq(1)
        )
        fsm.act("READ-DATA-DMA",
            # Read Sector data over DMA.
            dma.sink.valid.eq(1),
            dma.sink.address.eq(crt_base[int(log2(dma_bytes)):] + count),
            If(dma.sink.valid & dma.sink.ready,
                NextValue(count, count + 1),
                If(count == (logical_sector_size//dma_bytes - 1),
                    NextValue(count, 0),
                    NextState("SEND-CMD-AND-DATA")
                )
            )
        )
        fsm.act("SEND-CMD-AND-DATA",
            # Send write command/data for 1 Sector.
            port.sink.valid.eq(1),
            port.sink.last.eq(count == (logical_sector_size//port_bytes - 1)),
            port.sink.write.eq(1),
            port.sink.sector.eq(crt_sec),
            port.sink.count.eq(1),
            port.sink.data.eq(reverse_bytes(conv.source.data)),
            If(port.sink.ready,
                conv.source.ready.eq(1),
                NextValue(count, count + 1),
                If(port.sink.last,
                    NextState("WAIT-ACK")
                )
            ),

            # Monitor errors
            port.source.ready.eq(1),
            If(port.source.valid & port.source.ready & port.source.failed,
                self.irq.eq(1),
                NextValue(self.error.status, 1),
                NextState("IDLE"),
            )
        )
        fsm.act("WAIT-ACK",
            port.source.ready.eq(1),
            If(port.source.valid,
                If(port.source.failed,
                    NextValue(self.error.status, 1),
                    self.irq.eq(1),
                    NextState("IDLE")
                ).Elif(crt_sec == (self.sector.storage + self.nsectors.storage - 1),
                    self.irq.eq(1),
                    NextState("IDLE")
                ).Else(
                    NextValue(count,    0),
                    NextValue(crt_sec,  crt_sec + 1),
                    NextValue(crt_base, crt_base + 512),
                    NextState("READ-DATA-DMA")
                )
            )
        )
