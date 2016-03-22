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

    def generator_run(self, selfp, sector, count):
        selfp.generator.sector = sector
        selfp.generator.count = count
        selfp.generator.start = 1
        yield
        selfp.generator.start = 0
        yield
        while selfp.generator.done == 0:
            yield

    def checker_run(self, selfp, sector, count):
        selfp.checker.sector = sector
        selfp.checker.count = count
        selfp.checker.start = 1
        yield
        selfp.checker.start = 0
        yield
        while selfp.checker.done == 0:
            yield

    def gen_simulation(self, selfp):
        hdd = self.hdd
        hdd.malloc(0, 64)

        errors = 0

        # test write robustness
        # # # # # # # # # # # #

        # write some data (OK, will used for check)
        yield from self.generator_run(selfp, 0, 1)
        errors += selfp.generator.aborted != 0

        # write error (REG_D2H error set at the end)
        self.hdd.set_reg_d2h_status(1)
        yield from self.generator_run(selfp, 0, 1)
        self.hdd.set_reg_d2h_status(0)
        errors += selfp.generator.aborted != 1

        # write error (Device busy)
        self.hdd.set_busy(1)
        yield from self.generator_run(selfp, 0, 1)
        self.hdd.set_busy(0)
        errors += selfp.generator.aborted != 1

        # test read robustness
        # # # # # # # # # # # #

        # read data (OK)
        yield from self.checker_run(selfp, 0, 1)
        errors += selfp.checker.aborted != 0

        # read error (REG_D2H error set at the end)
        self.hdd.set_reg_d2h_status(1)
        yield from self.checker_run(selfp, 0, 1)
        self.hdd.set_reg_d2h_status(0)
        errors += selfp.checker.aborted != 1

        # read data (OK)
        yield from self.checker_run(selfp, 0, 1)
        errors += selfp.checker.aborted != 0

        # read error (CRC data error)
        self.hdd.set_data_error_injection(1)
        yield from self.checker_run(selfp, 0, 1)
        self.hdd.set_data_error_injection(0)
        errors += selfp.checker.aborted != 1

        # read data (OK)
        yield from self.checker_run(selfp, 0, 1)
        errors += selfp.checker.aborted != 0

        # read error (Device busy)
        self.hdd.set_busy(1)
        yield from self.checker_run(selfp, 0, 1)
        self.hdd.set_busy(0)
        errors += selfp.checker.aborted != 1

        # read data (OK)
        yield from self.checker_run(selfp, 0, 1)
        errors += selfp.checker.aborted != 0

        print("errors {}".format(errors))

        yield

if __name__ == "__main__":
    run_simulation(TB(), ncycles=4096, vcd_name="my.vcd", keep_files=True)