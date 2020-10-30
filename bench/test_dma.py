#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import time
import argparse

from litex import RemoteClient

class DMADriver:
    def __init__(self, bus, base):
        self.bus  = bus
        self.base = base

    def read(self, sector):
        self.bus.regs.sata_sector2mem_sector.write(sector)
        self.bus.regs.sata_sector2mem_base.write(self.base)
        self.bus.regs.sata_sector2mem_start.write(1)
        while self.bus.regs.sata_sector2mem_done.read() == 0:
            time.sleep(0.1)
        return self.bus.regs.sata_sector2mem_error.read()

    def write(self, sector):
        self.bus.regs.sata_mem2sector_sector.write(sector)
        self.bus.regs.sata_mem2sector_base.write(self.base)
        self.bus.regs.sata_mem2sector_start.write(1)
        while self.bus.regs.sata_mem2sector_done.read() == 0:
            time.sleep(0.1)
        return self.bus.regs.sata_mem2sector_error.read()

    def dump(self):
        for addr in range(self.base, self.base + 512, 4):
            if (addr%16 == 0):
                if addr != self.base:
                    print("")
                print("0x{:08x}".format(addr), end="  ")
            data = self.bus.read(addr)
            for i in range(4):
                print("{:02x}".format((data >> (8*i)) & 0xff), end=" ")
        print("")

    def fill(self, datas):
        assert len(datas) == 512//4
        for i, addr in enumerate(range(self.base, self.base + 512, 4)):
            data = self.bus.write(addr, datas[i])

# DMA Test -----------------------------------------------------------------------------------------

def dma_test(port, mode, sector, count):
    assert mode in ["r", "r+w"]
    bus = RemoteClient(port=port)
    bus.open()

    dma = DMADriver(bus=bus, base=bus.mems.sram.base)
    for s in range(sector, sector + count):
        print("Sector {:d}:".format(s))
        error = dma.read(s)
        dma.dump()
        print("Error: {:d}".format(error))

    if mode == "r+w":
        s = 1000
        dma.fill([i for i in range(512//4)])
        error  = dma.write(s)
        print("Sector {:d}:".format(s))
        error += dma.read(s)
        dma.dump()
        print("Error: {:d}".format(error))

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA DMA test utility")
    parser.add_argument("--port",   default="1234", help="Host bind port")
    parser.add_argument("--mode",   default="r",    help="Mode (r: read (default) or r+w: read+write)")
    parser.add_argument("--sector", default="0",    help="SATA base sector")
    parser.add_argument("--count",  default="1",    help="SATA sector count")
    args = parser.parse_args()

    port   = int(args.port,   0)
    mode   = args.mode
    sector = int(args.sector, 0)
    count  = int(args.count,  0)

    dma_test(port, mode, sector, count)

if __name__ == "__main__":
    main()
