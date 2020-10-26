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

# Block2Mem Test -----------------------------------------------------------------------------------

def block2mem_test(port, sector, count):
    wb = RemoteClient(port=port)
    wb.open()

    class Block2MemDriver:
        def __init__(self, base):
            self.base = base

        def read(self, sector):
            wb.regs.sata_block2mem_sector.write(sector)
            wb.regs.sata_block2mem_base.write(self.base)
            wb.regs.sata_block2mem_start.write(1)
            while wb.regs.sata_block2mem_done.read() == 0:
                time.sleep(0.1)

        def dump(self):
            for addr in range(self.base, self.base + 512, 4):
                if (addr%16 == 0):
                    if addr != self.base:
                        print("")
                    print("0x{:08x}".format(addr), end="  ")
                data = wb.read(addr)
                for i in reversed(range(4)):
                    print("{:02x}".format((data >> (8*i)) & 0xff), end=" ")
            print("")

    block2mem = Block2MemDriver(base=wb.mems.sram.base)
    for s in range(sector, sector + count):
        print("Sector {:d}:".format(s))
        block2mem.read(s)
        block2mem.dump()

    wb.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA Block2Mem test utility")
    parser.add_argument("--port",   default="1234", help="Host bind port")
    parser.add_argument("--sector", default="0",    help="SATA base sector")
    parser.add_argument("--count",  default="1",    help="SATA sector count")
    args = parser.parse_args()

    port   = int(args.port,   0)
    sector = int(args.sector, 0)
    count  = int(args.count,  0)

    block2mem_test(port, sector, count)

if __name__ == "__main__":
    main()
