#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import Module, passive, run_simulation

from litesata.common import logical_sector_size
from litesata.frontend.arbitration import LiteSATAUserPort
from litesata.frontend.stream import LiteSATAStream2Sectors, LiteSATASectors2Stream

# Helpers ------------------------------------------------------------------------------------------

def stream2sectors(data_width, words):
    """Write one sector of stream words with Stream2Sectors, return the port dwords."""
    dut    = Module()
    port   = LiteSATAUserPort(32)
    dut.submodules.s2s = s2s = LiteSATAStream2Sectors(port, data_width=data_width)
    dwords = []

    @passive
    def port_generator():
        yield port.sink.ready.eq(1)
        while True:
            if (yield port.sink.valid):
                dwords.append((yield port.sink.data))
                if (yield port.sink.last):
                    yield
                    yield port.source.valid.eq(1)
                    yield port.source.last.eq(1)
                    yield port.source.end.eq(1)
                    yield port.source.write.eq(1)
                    yield
                    yield port.source.valid.eq(0)
            yield

    def stream_generator():
        yield s2s.sector.storage.eq(0)
        yield s2s.nsectors.storage.eq(1)
        yield s2s.start.wr_stb.eq(1)
        yield
        yield s2s.start.wr_stb.eq(0)
        for i, word in enumerate(words):
            yield s2s.sink.valid.eq(1)
            yield s2s.sink.data.eq(word)
            yield s2s.sink.last.eq(i == len(words) - 1)
            yield
            while not (yield s2s.sink.ready):
                yield
        yield s2s.sink.valid.eq(0)
        yield
        for cycle in range(1024):
            if (yield s2s.done.status):
                break
            yield

    run_simulation(dut, [port_generator(), stream_generator()])
    return dwords

def sectors2stream(data_width, dwords):
    """Read one sector of port dwords with Sectors2Stream, return the stream words."""
    dut   = Module()
    port  = LiteSATAUserPort(32)
    dut.submodules.s2s = s2s = LiteSATASectors2Stream(port, data_width=data_width)
    words = []

    @passive
    def port_generator():
        yield port.sink.ready.eq(1)
        while not (yield port.sink.valid):
            yield
        yield
        yield port.sink.ready.eq(0)
        for i, dword in enumerate(dwords + [0]):
            yield port.source.valid.eq(1)
            yield port.source.read.eq(1)
            yield port.source.last.eq((i == len(dwords) - 1) or (i == len(dwords)))
            yield port.source.end.eq(i == len(dwords))
            yield port.source.data.eq(dword)
            yield
            while not (yield port.source.ready):
                yield
        yield port.source.valid.eq(0)

    def stream_generator():
        yield s2s.sector.storage.eq(0)
        yield s2s.nsectors.storage.eq(1)
        yield s2s.source.ready.eq(1)
        yield s2s.start.wr_stb.eq(1)
        yield
        yield s2s.start.wr_stb.eq(0)
        for cycle in range(4096):
            if (yield s2s.source.valid):
                words.append((yield s2s.source.data))
                if (yield s2s.source.last):
                    break
            yield

    run_simulation(dut, [port_generator(), stream_generator()])
    return words

# Test ---------------------------------------------------------------------------------------------

class TestStreamWordOrder(unittest.TestCase):
    def test_stream2sectors_sectors2stream_roundtrip(self):
        # Stream words are big-endian (first sector byte in the MSBs, as Sectors2Stream outputs them),
        # port dwords carry the first byte in the LSBs (as the DMAs). Words written with Stream2Sectors
        # must read back unchanged from Sectors2Stream (64-bit words had their 32-bit halves swapped).
        image = bytes((i % 251) + 1 for i in range(logical_sector_size))
        for data_width in [32, 64]:
            with self.subTest(data_width=data_width):
                stream_bytes = data_width//8
                words  = [int.from_bytes(image[i:i + stream_bytes], "big")
                    for i in range(0, len(image), stream_bytes)]
                dwords = stream2sectors(data_width, words)
                self.assertEqual(dwords, [int.from_bytes(image[i:i + 4], "little")
                    for i in range(0, len(image), 4)])
                self.assertEqual(sectors2stream(data_width, dwords), words)


if __name__ == "__main__":
    unittest.main()
