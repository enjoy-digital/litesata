#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2025 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2023 Gabriel L. Somlo <gsomlo@gmail.com>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *

from litex.gen import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from litesata.common import logical_sector_size, reverse_bytes

# SATA Sectors2Stream ------------------------------------------------------------------------------

class LiteSATASectors2Stream(LiteXModule):
    """Sectors to Stream

    Read sectors from the SATA core and stream them out.
    """
    def __init__(self, port, data_width=64):
        self.port     = port
        self.sector   = CSRStorage(48)
        self.nsectors = CSRStorage(32)
        self.start    = CSR()
        self.done     = CSRStatus()
        self.error    = CSRStatus()
        self.irq      = Signal()

        self.source   = stream.Endpoint([("data", data_width)])

        # # #

        port_bytes   = port.dw//8
        stream_bytes = data_width//8

        # We stream whole sectors: 512B == 64 * 64-bit words.
        assert (logical_sector_size % stream_bytes) == 0

        crt_sec  = Signal(48)
        last_sec = Signal(48)

        # Sector buffer.
        self.buf = buf = stream.SyncFIFO([("data", port.dw)], logical_sector_size//port_bytes)

        # Converter.
        self.conv = conv = stream.Converter(nbits_from=port.dw, nbits_to=data_width)

        # Connect Port to Sector Buffer.
        self.comb += port.source.connect(buf.sink, keep={"valid", "ready", "last", "data"})

        # Connect Sector Buffer to Converter.
        self.comb += buf.source.connect(conv.sink)

        # Control FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.start.re,
                NextValue(crt_sec,           self.sector.storage),
                NextValue(last_sec,          self.sector.storage + self.nsectors.storage - 1),
                NextValue(self.error.status, 0),
                NextState("SEND-CMD")
            ).Else(
                self.done.status.eq(1)
            ),
            conv.source.ready.eq(1)
        )
        fsm.act("SEND-CMD",
            # Send read command for 1 sector.
            port.sink.valid.eq(1),
            port.sink.last.eq(1),
            port.sink.read.eq(1),
            port.sink.sector.eq(crt_sec),
            port.sink.count.eq(1),
            If(port.sink.ready,
                NextState("RECEIVE-DATA-STREAM")
            )
        )
        fsm.act("RECEIVE-DATA-STREAM",
            # Connect Converter to Stream.
            self.source.valid.eq(conv.source.valid),
            self.source.data.eq(reverse_bytes(conv.source.data)),

            # End-of-transfer framing:
            # Assert last only on the last word of the last sector.
            self.source.last.eq(conv.source.last & (crt_sec == last_sec)),

            conv.source.ready.eq(self.source.ready),

            If(self.source.valid & self.source.ready,
                If(conv.source.last,
                    If(crt_sec == last_sec,
                        self.irq.eq(1),
                        NextState("IDLE")
                    ).Else(
                        NextValue(crt_sec, crt_sec + 1),
                        NextState("SEND-CMD")
                    )
                )
            ),

            # Monitor errors.
            If(port.source.valid & port.source.ready & port.source.failed,
                self.irq.eq(1),
                NextValue(self.error.status, 1),
                NextState("IDLE")
            )
        )

# SATA Stream2Sectors ------------------------------------------------------------------------------

class LiteSATAStream2Sectors(LiteXModule):
    """Stream to Sectors

    Receive a stream and write it to SATA sectors.
    """
    def __init__(self, port, data_width=64):
        self.port     = port
        self.sector   = CSRStorage(48)
        self.nsectors = CSRStorage(32)
        self.start    = CSR()
        self.done     = CSRStatus()
        self.error    = CSRStatus()
        self.irq      = Signal()

        self.sink     = stream.Endpoint([("data", data_width)])

        # # #

        port_bytes   = port.dw//8
        stream_bytes = data_width//8

        # Full-sector writes: we require exact 512B chunks.
        assert (logical_sector_size % stream_bytes) == 0

        # Count in port-width words.
        words_per_sector = logical_sector_size//port_bytes

        fill_count = Signal(max=words_per_sector)
        send_count = Signal(max=words_per_sector)
        crt_sec    = Signal(48)

        # Converter.
        self.conv = conv = stream.Converter(nbits_from=data_width, nbits_to=port.dw)

        # Sector buffer (one sector).
        self.buf = buf = stream.SyncFIFO([("data", port.dw)], words_per_sector)

        # Connect Stream to Converter.
        self.comb += self.sink.connect(conv.sink, keep={"valid", "ready", "data", "last"})

        # Connect Converter to Sector Buffer.
        self.comb += conv.source.connect(buf.sink, keep={"valid", "ready", "data"})

        # Control FSM.
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.start.re,
                NextValue(fill_count,        0),
                NextValue(send_count,        0),
                NextValue(crt_sec,           self.sector.storage),
                NextValue(self.error.status, 0),
                NextState("FILL-SECTOR")
            ).Else(
                self.done.status.eq(1)
            ),
            conv.source.ready.eq(1)
        )
        fsm.act("FILL-SECTOR",
            # Fill one sector in buffer.
            buf.sink.valid.eq(conv.source.valid),
            buf.sink.data.eq(conv.source.data),
            conv.source.ready.eq(buf.sink.ready),
            If(buf.sink.valid & buf.sink.ready,
                NextValue(fill_count, fill_count + 1),
                If(fill_count == (words_per_sector - 1),
                    NextValue(fill_count, 0),
                    NextValue(send_count, 0),
                    NextState("SEND-CMD-AND-DATA")
                )
            )
        )
        fsm.act("SEND-CMD-AND-DATA",
            # Send write command/data for 1 sector.
            port.sink.valid.eq(buf.source.valid),
            port.sink.last.eq(send_count == (words_per_sector - 1)),
            port.sink.write.eq(1),
            port.sink.sector.eq(crt_sec),
            port.sink.count.eq(1),
            port.sink.data.eq(reverse_bytes(buf.source.data)),
            buf.source.ready.eq(port.sink.ready),
            If(port.sink.valid & port.sink.ready,
                NextValue(send_count, send_count + 1),
                If(port.sink.last,
                    NextState("WAIT-ACK")
                )
            ),

            # Monitor errors.
            port.source.ready.eq(1),
            If(port.source.valid & port.source.ready & port.source.failed,
                self.irq.eq(1),
                NextValue(self.error.status, 1),
                NextState("IDLE")
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
                    NextValue(send_count, 0),
                    NextValue(crt_sec, crt_sec + 1),
                    NextState("FILL-SECTOR")
                )
            )
        )
