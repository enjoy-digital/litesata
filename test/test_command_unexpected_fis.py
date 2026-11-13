#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.soc.interconnect import stream

from litesata.common import *
from litesata.core.command import LiteSATACommandRX


class TestCommandUnexpectedFIS(unittest.TestCase):
    def read_test(self, command, unexpected):
        # READ DMA EXT (1 sector) or IDENTIFY on a fake transport, with unexpected control FISes
        # before the data and before the end of the command.
        transport        = Module()
        transport.source = stream.Endpoint(transport_rx_description(32))
        dut   = LiteSATACommandRX(transport)
        beats = []
        ndwords = 128

        def ctrl(fis_type, status=0x50):
            yield transport.source.type.eq(fis_types[fis_type])
            yield transport.source.status.eq(status)
            yield transport.source.last.eq(1)
            yield transport.source.valid.eq(1)
            yield
            timeout = 0
            while not (yield transport.source.ready):
                timeout += 1
                self.assertLess(timeout, 64, "{} FIS not consumed".format(fis_type))
                yield
            yield transport.source.valid.eq(0)
            yield

        def data():
            yield transport.source.type.eq(fis_types["DATA"])
            yield transport.source.valid.eq(1)
            for i in range(ndwords):
                yield transport.source.data.eq(i)
                yield transport.source.last.eq(i == ndwords - 1)
                yield
                while not (yield transport.source.ready):
                    yield
            yield transport.source.valid.eq(0)
            yield transport.source.last.eq(0)
            yield

        def generator():
            yield getattr(dut.from_tx, command).eq(1)
            yield dut.from_tx.count.eq(1)
            yield
            yield getattr(dut.from_tx, command).eq(0)
            yield
            if command == "identify":
                yield from ctrl("PIO_SETUP_D2H", status=0x58)
            for fis_type in unexpected:
                yield from ctrl(fis_type)
            yield from data()
            if command == "read":
                for fis_type in unexpected:
                    yield from ctrl(fis_type)
                yield from ctrl("REG_D2H")
            for _ in range(8):
                yield

        @passive
        def checker():
            yield dut.source.ready.eq(1)
            while True:
                if (yield dut.source.valid):
                    beats.append((
                        (yield dut.source.last),
                        (yield dut.source.end),
                        (yield dut.source.failed),
                    ))
                yield

        run_simulation(dut, [generator(), checker()])
        return beats

    def test_read(self):
        # ndwords data beats then the response beat, no failure.
        reference = [(0, 0, 0)]*127 + [(1, 0, 0), (1, 1, 0)]
        self.assertEqual(self.read_test("read", []), reference)
        self.assertEqual(self.read_test("read", ["DMA_ACTIVATE_D2H"]), reference)
        self.assertEqual(self.read_test("read", ["PIO_SETUP_D2H", "DMA_ACTIVATE_D2H"]), reference)

    def test_identify(self):
        # ndwords data beats, end on the last one, no failure.
        reference = [(0, 0, 0)]*127 + [(1, 1, 0)]
        self.assertEqual(self.read_test("identify", []), reference)
        self.assertEqual(self.read_test("identify", ["DMA_ACTIVATE_D2H"]), reference)


if __name__ == "__main__":
    unittest.main()
