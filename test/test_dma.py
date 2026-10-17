#
# This file is part of LiteSATA.
#
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from migen import Module, run_simulation

from litex.soc.interconnect import wishbone

from litesata.common import logical_sector_size
from litesata.frontend.arbitration import LiteSATAUserPort
from litesata.frontend.dma import LiteSATAMem2SectorDMA


class TestDMA(unittest.TestCase):
    def test_mem2sector(self):
        for data_width in [32, 64]:
            for endianness in ["little", "big"]:
                with self.subTest(data_width=data_width, endianness=endianness):
                    dut  = Module()
                    bus  = wishbone.Interface(data_width=data_width)
                    port = LiteSATAUserPort(32)
                    dut.submodules.dma = dma = LiteSATAMem2SectorDMA(bus, port, endianness)

                    # Distinct bytes expose truncation and word/byte reordering.
                    image = bytes((i % 251) + 1 for i in range(logical_sector_size))
                    bus_bytes = data_width//8
                    memory = [int.from_bytes(image[i:i + bus_bytes], endianness)
                        for i in range(0, len(image), bus_bytes)]
                    dut.submodules.ram = wishbone.SRAM(logical_sector_size, bus=bus, init=memory)
                    expected = [int.from_bytes(image[i:i + 4], "little")
                        for i in range(0, len(image), 4)]
                    received = []

                    def generator():
                        yield dma.sector.storage.eq(7)
                        yield dma.nsectors.storage.eq(1)
                        yield dma.base.storage.eq(0)
                        yield dma.start.wr_stb.eq(1)
                        yield
                        yield dma.start.wr_stb.eq(0)
                        for cycle in range(3000):
                            # Periodically stall the SATA output.
                            yield port.sink.ready.eq(cycle % 5 != 0)
                            yield
                            if (yield port.sink.valid) and (yield port.sink.ready):
                                self.assertEqual((yield port.sink.write), 1)
                                self.assertEqual((yield port.sink.sector), 7)
                                self.assertEqual((yield port.sink.count), 1)
                                self.assertEqual((yield port.sink.last), len(received) == len(expected) - 1)
                                received.append((yield port.sink.data))
                                if (yield port.sink.last):
                                    break
                        else:
                            self.fail("DMA did not emit a complete sector")
                        yield

                        # Acknowledge the sector write.
                        yield port.source.valid.eq(1)
                        yield port.source.last.eq(1)
                        yield port.source.write.eq(1)
                        yield port.source.end.eq(1)
                        yield port.source.failed.eq(0)
                        yield
                        self.assertEqual((yield port.source.ready), 1)
                        yield port.source.valid.eq(0)
                        for _ in range(10):
                            yield
                        self.assertEqual((yield dma.done.status), 1)
                        self.assertEqual((yield dma.error.status), 0)

                    run_simulation(dut, generator())
                    self.assertEqual(received, expected)


if __name__ == "__main__":
    unittest.main()
