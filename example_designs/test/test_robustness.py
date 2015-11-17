import sys
import random
from test_bist import *

from litex.soc.tools.remote import RemoteClient

# parameters
random.seed(0)
max_sector = 16*GB/logical_sector_size
max_count = 16*MB/logical_sector_size

wb = RemoteClient()
wb.open()

# # #

identify = LiteSATABISTIdentifyDriver(wb.regs, wb.constants, "sata_bist")
generator = LiteSATABISTGeneratorDriver(wb.regs, wb.constants, "sata_bist")
checker = LiteSATABISTCheckerDriver(wb.regs, wb.constants, "sata_bist")

identify.run()
identify.hdd_info()

try:
    write_errors = 0
    read_errors = 0
    while True:
        write_read_n = random.randint(0, 1)
        sector = random.randint(1, max_sector)
        count = random.randint(1, max_count)
        # write
        if write_read_n:
            aborted, dummy0, dummy1 = generator.run(sector, count, 1, True)
            write_errors += aborted
        # read
        else:
            aborted, dummy0, dummy1 = checker.run(sector, count, 1, True)
            read_errors += aborted
        print("write_read_n: {:d}, sector:{:d}, count: {:d}, writes errors: {:d}, reads errors: {:d}".format(
              write_read_n,
              sector,
              count,
              write_errors,
              read_errors))

except KeyboardInterrupt:
    pass

# # #

wb.close()
