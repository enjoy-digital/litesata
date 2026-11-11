#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import Module, passive, run_simulation

from litesata.common import logical_sector_size
from litesata.frontend.arbitration import LiteSATAUserPort
from litesata.frontend.stream import LiteSATASectors2Stream

# Test ---------------------------------------------------------------------------------------------

class TestSectors2Stream(unittest.TestCase):
    def test_multi_sector_read(self):
        # Each read returns one sector of data then a response beat (end=1): with nsectors > 1 the
        # response beat was streamed as data and ended the transfer after the first sector.
        nsectors = 3
        ndwords  = logical_sector_size//4
        dut      = Module()
        port     = LiteSATAUserPort(32)
        dut.submodules.s2s = s2s = LiteSATASectors2Stream(port, data_width=32)
        received = []

        @passive
        def port_generator():
            for n in range(nsectors):
                yield port.sink.ready.eq(1)
                while not ((yield port.sink.valid) and (yield port.sink.ready)):
                    yield
                self.assertEqual((yield port.sink.read), 1)
                self.assertEqual((yield port.sink.sector), 5 + n)
                yield
                yield port.sink.ready.eq(0)
                # Data beats.
                for i in range(ndwords):
                    yield port.source.valid.eq(1)
                    yield port.source.read.eq(1)
                    yield port.source.last.eq(i == ndwords - 1)
                    yield port.source.end.eq(0)
                    yield port.source.data.eq((n << 16) | i)
                    yield
                    while not (yield port.source.ready):
                        yield
                # Response beat.
                yield port.source.last.eq(1)
                yield port.source.end.eq(1)
                yield port.source.data.eq(0)
                yield
                while not (yield port.source.ready):
                    yield
                yield port.source.valid.eq(0)
                yield port.source.end.eq(0)

        def stream_generator():
            yield s2s.sector.storage.eq(5)
            yield s2s.nsectors.storage.eq(nsectors)
            yield s2s.source.ready.eq(1)
            yield s2s.start.wr_stb.eq(1)
            yield
            yield s2s.start.wr_stb.eq(0)
            yield
            for cycle in range(20000):
                if (yield s2s.source.valid):
                    received.append(((yield s2s.source.data), (yield s2s.source.last)))
                    if (yield s2s.source.last):
                        break
                yield

        run_simulation(dut, [port_generator(), stream_generator()])
        expected = [(int.from_bytes(((n << 16) | i).to_bytes(4, "little"), "big"),
            int((n == nsectors - 1) and (i == ndwords - 1)))
            for n in range(nsectors) for i in range(ndwords)]
        self.assertEqual(len(received), len(expected))
        self.assertEqual(received, expected)


if __name__ == "__main__":
    unittest.main()
