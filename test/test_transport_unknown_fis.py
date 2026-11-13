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


class TestTransportUnknownFIS(unittest.TestCase):
    def unknown_fis_test(self, frames, bubbles=False):
        # Frames from a fake link, followed by a Register D2H FIS: only the Register D2H FIS must
        # be presented.
        link        = Module()
        link.source = stream.Endpoint(link_description(32))
        dut         = LiteSATATransportRX(link)
        reg_d2h     = [0x00504034, 0x00000000, 0x00000000, 0x00000001, 0x00000000]
        beats       = []

        def generator():
            for frame in frames + [reg_d2h]:
                for i, dword in enumerate(frame):
                    if bubbles:
                        yield link.source.valid.eq(0)
                        yield
                    yield link.source.valid.eq(1)
                    yield link.source.last.eq(i == len(frame) - 1)
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
                        (yield dut.source.last),
                    ))
                yield

        run_simulation(dut, [generator(), checker()])
        self.assertEqual(beats, [(fis_types["REG_D2H"], 0x50, 0x0001, 1)])

    def test_set_device_bits(self):
        # Set Device Bits FIS (0xa1), 2 dwords.
        self.unknown_fis_test([[0x004040a1, 0x00000000]])

    def test_single_dword(self):
        # Unknown FIS of a single dword.
        self.unknown_fis_test([[0x000000a1]])

    def test_payload_with_known_type_bytes(self):
        # Unknown FIS whose payload dwords start with the type bytes of the supported FISes.
        frame = [0x004040a1, 0x00000034, 0x00000039, 0x0000005f, 0x00000046, 0x12345678]
        self.unknown_fis_test([frame])
        self.unknown_fis_test([frame], bubbles=True)
        self.unknown_fis_test([frame, frame])

    def test_dma_setup(self):
        # DMA Setup FIS (0x41), 7 dwords, with a count that starts with the Data FIS type byte.
        frame = [0x00000041, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000046, 0x00000000]
        self.unknown_fis_test([frame])


if __name__ == "__main__":
    unittest.main()
