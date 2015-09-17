from litesata.common import *
from litesata.core import LiteSATACore
from litesata.frontend.crossbar import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABISTGenerator, LiteSATABISTChecker

from test.common import *
from test.model.hdd import *


class TB(Module):
    def __init__(self, dw=64):
        self.submodules.hdd = HDD(
                link_debug=False, link_random_level=0,
                transport_debug=False, transport_loopback=False,
                hdd_debug=True)
        self.submodules.core = LiteSATACore(self.hdd.phy)
        self.submodules.crossbar = LiteSATACrossbar(self.core)
        self.submodules.generator = LiteSATABISTGenerator(self.crossbar.get_port(dw))
        self.submodules.checker = LiteSATABISTChecker(self.crossbar.get_port(dw))

    def gen_simulation(self, selfp):
        hdd = self.hdd
        hdd.malloc(0, 64)
        sector = 0
        count = 17
        generator = selfp.generator
        checker = selfp.checker
        while True:
            # write data
            generator.sector = sector
            generator.count = count
            generator.start = 1
            yield
            generator.start = 0
            yield
            while generator.done == 0:
                yield

            # verify data
            checker.sector = sector
            checker.count = count
            checker.start = 1
            yield
            checker.start = 0
            yield
            while checker.done == 0:
                yield
            print("errors {}".format(checker.errors))

            # prepare next iteration
            sector += 1
            count = max((count + 1)%8, 1)

if __name__ == "__main__":
    run_simulation(TB(32), ncycles=8192*2, vcd_name="my.vcd", keep_files=True)
    run_simulation(TB(64), ncycles=8192*2, vcd_name="my.vcd", keep_files=True)