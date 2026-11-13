#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import *

from litex.soc.interconnect import stream

from litesata.common import *
from litesata.core.transport import LiteSATATransportRX


class TestTransportCmdDone(unittest.TestCase):
    def pio_setup_test(self, bubbles):
        # PIO Setup FIS (5 dwords) from a fake link, with bubbles (valid at 0, bus not held) before
        # the dwords listed in bubbles.
        link        = Module()
        link.source = stream.Endpoint(link_description(32))
        dut         = LiteSATATransportRX(link)
        dwords      = [0x0058605f, 0x00000000, 0x00000000, 0x50000001, 0x00000200]
        beats       = []

        def generator():
            for i, dword in enumerate(dwords):
                if i in bubbles:
                    yield link.source.valid.eq(0)
                    yield link.source.data.eq(0xdeadbeef)
                    yield
                yield link.source.valid.eq(1)
                yield link.source.last.eq(i == len(dwords) - 1)
                yield link.source.data.eq(dword)
                yield
                while not (yield link.source.ready):
                    yield
            yield link.source.valid.eq(0)
            yield link.source.last.eq(0)
            for _ in range(16):
                yield

        @passive
        def checker():
            yield dut.source.ready.eq(1)
            while True:
                if (yield dut.source.valid):
                    beats.append((
                        (yield dut.source.type),
                        (yield dut.source.status),
                        (yield dut.source.count),
                        (yield dut.source.transfer_count),
                    ))
                yield

        run_simulation(dut, [generator(), checker()])
        self.assertEqual(beats, [(fis_types["PIO_SETUP_D2H"], 0x58, 0x0001, 0x0200)])
        return beats

    def test_pio_setup(self):
        self.pio_setup_test(bubbles=[])

    def test_pio_setup_bubble_before_last_dword(self):
        self.pio_setup_test(bubbles=[4])

    def test_pio_setup_bubbles(self):
        self.pio_setup_test(bubbles=[1, 2, 3, 4])


if __name__ == "__main__":
    unittest.main()
