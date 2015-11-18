import sys
import random
import time
from test_bist import *

from litex.soc.tools.remote import RemoteClient

# parameters
random.seed(0)
max_sector = 16*GB/logical_sector_size
max_count = 16*MB/logical_sector_size

class HDDAccess:
    def __init__(self, sector, count, write_read_n):
        self.sector = sector
        self.count = count
        self.write_read_n = write_read_n

class LiteSATABISTDriver:
    def __init__(self, regs, constants, name):
        self.regs = regs
        self.name = name
        self.frequency = constants.system_clock_frequency
        self.time = 0
        for s in ["sector", "count", "write_read_n", "loop_prog_n", "we", "flush", "start",
                  "done", "loop_index", "loop_count"]:
            setattr(self, s, getattr(regs, name + "_" + s))

    def program(self, accesses):
        self.loop_prog_n.write(0)
        self.flush.write(1)
        total_sectors = 0
        for access in accesses:
            self.sector.write(access.sector)
            self.count.write(access.count)
            self.write_read_n.write(access.write_read_n)
            self.we.write(1)
            total_sectors += access.count
        return total_sectors

    def run(self, loops=0, blocking=True, debug=True):
        if loops:
          self.loop_prog_n.write(1)
        self.start.write(1)
        if blocking:
            while (self.done.read() == 0):
                if loops:
                    if self.loop_count.read() > loops - 2:
                        self.loop_prog_n.write(0)
                if debug:
                  print("loop: {}, access {}".format(self.loop_count.read(), self.loop_index.read()))
                time.sleep(1)

wb = RemoteClient()
wb.open()

# # #

identify = LiteSATABISTIdentifyDriver(wb.regs, wb.constants, "sata_bist")
identify.run()
identify.hdd_info()

bist = LiteSATABISTDriver(wb.regs, wb.constants, "sata_bist")
bist_accesses = []
for i in range(256):
    bist_accesses.append(HDDAccess(random.randint(0, max_sector), # sector
                                   random.randint(0, max_count),  # count
                                   random.randint(0, 1)))         # read_write_n
bist_loops = 16
total_sectors = bist.program(bist_accesses)
bist.run(bist_loops)

print("total read/write transfer: {:d}GB".format(int(bist_loops*total_sectors*logical_sector_size/1e9)))

# # #

wb.close()
