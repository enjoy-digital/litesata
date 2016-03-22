from litesata.common import *
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABISTGenerator, LiteSATABISTChecker

from common import *
from model.hdd import *


class TB(Module):
    def __init__(self, dw=32):
        self.submodules.hdd = HDD(
                link_debug=False, link_random_level=0,
                transport_debug=False, transport_loopback=False,
                hdd_debug=True)
        self.submodules.core = LiteSATACore(self.hdd.phy)
        self.submodules.crossbar = LiteSATACrossbar(self.core)
        self.submodules.generator = LiteSATABISTGenerator(self.crossbar.get_port(dw))
        self.submodules.checker = LiteSATABISTChecker(self.crossbar.get_port(dw))


def main_generator(dut):
    dut.hdd.malloc(0, 64)
    sector = 0
    count = 1
    generator = dut.generator
    checker = dut.checker
    for i in range(4):
        # write data
        yield dut.generator.sector.eq(sector)
        yield dut.generator.count.eq(count)
        yield dut.generator.start.eq(1)
        yield
        yield dut.generator.start.eq(0)
        yield
        while not (yield generator.done):
            yield

        # verify data
        yield dut.checker.sector.eq(sector)
        yield dut.checker.count.eq(count)
        yield dut.checker.start.eq(1)
        yield
        yield dut.checker.start.eq(0)
        yield
        while not (yield dut.checker.done):
            yield
        print("errors {}".format((yield dut.checker.errors)))

        # prepare next iteration
        sector += 1
        count = max((count + 1)%8, 1)

    # XXX: find a way to exit properly
    import sys
    sys.exit()

if __name__ == "__main__":
    tb = TB()
    generators = {
        "sys" :   [main_generator(tb),
                   tb.hdd.link.generator(),
                   tb.hdd.phy.rx.generator(),
                   tb.hdd.phy.tx.generator()]
    }
    clocks = {"sys": 10}
    run_simulation(tb, generators, clocks, vcd_name="sim.vcd")
