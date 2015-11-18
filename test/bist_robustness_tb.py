from litesata.common import *
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABISTRobustness

from test.common import *
from test.model.hdd import *

class HDDAccess:
    def __init__(self, sector, count, write_read_n):
        self.sector = sector
        self.count = count
        self.write_read_n = write_read_n

class TB(Module):
    def __init__(self):
        self.submodules.hdd = HDD(
                link_debug=False, link_random_level=0,
                transport_debug=False, transport_loopback=False,
                hdd_debug=True)
        self.submodules.core = LiteSATACore(self.hdd.phy)
        self.submodules.crossbar = LiteSATACrossbar(self.core)
        self.submodules.bist = LiteSATABISTRobustness(self.crossbar, fifo_depth=16)

    def program_bist_fifo(self, bist, accesses):
        # clear fifo
        bist.loop_prog_n = 0
        bist.we = 0
        bist.flush = 1
        bist.start = 0
        yield
        bist.flush = 0
        yield

        # program accesses
        for access in accesses:
            bist.sector = access.sector
            bist.count = access.count
            bist.write_read_n = access.write_read_n
            bist.we = 1
            yield
            bist.we = 0
            yield

    def gen_simulation(self, selfp):
        hdd = self.hdd
        hdd.malloc(0, 64)
        bist = selfp.bist

        hdd_accesses = []
        for i in range(4):
            hdd_accesses.append(HDDAccess(i, 1, random.randint(0, 1)))
        yield from self.program_bist_fifo(bist, hdd_accesses)
        bist.start = 1
        yield
        yield
        while bist.done == 0:
            yield
        print("done")

if __name__ == "__main__":
    run_simulation(TB(), ncycles=2048, vcd_name="my.vcd", keep_files=True)