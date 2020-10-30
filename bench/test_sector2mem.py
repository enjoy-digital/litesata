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

# Sector2Mem Test -----------------------------------------------------------------------------------

def sector2mem_test(port, sector, count):
    wb = RemoteClient(port=port)
    wb.open()

    class Sector2MemDriver:
        def __init__(self, base):
            self.base = base

        def read(self, sector):
            wb.regs.sata_sector2mem_sector.write(sector)
            wb.regs.sata_sector2mem_base.write(self.base)
            wb.regs.sata_sector2mem_start.write(1)
            while wb.regs.sata_sector2mem_done.read() == 0:
                time.sleep(0.1)
            return wb.regs.sata_sector2mem_error.read()

        def dump(self):
            for addr in range(self.base, self.base + 512, 4):
                if (addr%16 == 0):
                    if addr != self.base:
                        print("")
                    print("0x{:08x}".format(addr), end="  ")
                data = wb.read(addr)
                for i in range(4):
                    print("{:02x}".format((data >> (8*i)) & 0xff), end=" ")
            print("")

    sector2mem = Sector2MemDriver(base=wb.mems.sram.base)
    for s in range(sector, sector + count):
        print("Sector {:d}:".format(s))
        error = sector2mem.read(s)
        sector2mem.dump()
        print("Error: {:d}".format(error))

    wb.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA Sector2Mem test utility")
    parser.add_argument("--port",   default="1234", help="Host bind port")
    parser.add_argument("--sector", default="0",    help="SATA base sector")
    parser.add_argument("--count",  default="1",    help="SATA sector count")
    args = parser.parse_args()

    port   = int(args.port,   0)
    sector = int(args.sector, 0)
    count  = int(args.count,  0)

    sector2mem_test(port, sector, count)

if __name__ == "__main__":
    main()
