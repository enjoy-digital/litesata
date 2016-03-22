#!/usr/bin/env python3
from litesata.common import *
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABISTGenerator, LiteSATABISTChecker
from litesata.frontend.raid import LiteSATAStriping

from common import *
from model.hdd import *


class TB(Module):
    def __init__(self):
        self.submodules.hdd0 = HDD(n=0,
                link_debug=False, link_random_level=0,
                transport_debug=False, transport_loopback=False,
                hdd_debug=True)
        self.submodules.core0 = LiteSATACore(self.hdd0.phy)

        self.submodules.hdd1 = HDD(n=1,
                link_debug=False, link_random_level=0,
                transport_debug=False, transport_loopback=False,
                hdd_debug=True)
        self.submodules.core1 = LiteSATACore(self.hdd1.phy)

        self.submodules.striping = LiteSATAStriping([self.core0, self.core1])
        self.submodules.crossbar = LiteSATACrossbar(self.striping)

        self.submodules.generator = LiteSATABISTGenerator(self.crossbar.get_port())
        self.submodules.checker = LiteSATABISTChecker(self.crossbar.get_port())

def main_generator(dut):
    dut.hdd0.malloc(0, 64)
    dut.hdd1.malloc(0, 64)
    sector = 0
    count = 1
    for i in range(2):
        # write data
        yield dut.generator.sector.eq(sector)
        yield dut.generator.count.eq(count)
        yield dut.generator.start.eq(1)
        yield
        yield dut.generator.start.eq(0)
        yield
        while not (yield dut.generator.done):
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
        count = count + 1

    # XXX: find a way to exit properly
    import sys
    sys.exit()

if __name__ == "__main__":
    tb = TB()
    generators = {
        "sys" :   [main_generator(tb),
                   tb.hdd0.link.generator(),
                   tb.hdd0.phy.rx.generator(),
                   tb.hdd0.phy.tx.generator(),
                   tb.hdd1.link.generator(),
                   tb.hdd1.phy.rx.generator(),
                   tb.hdd1.phy.tx.generator()]
    }
    clocks = {"sys": 10}
    run_simulation(tb, generators, clocks, vcd_name="sim.vcd")
