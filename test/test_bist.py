#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2016 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from litesata.common import *
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist import LiteSATABISTGenerator, LiteSATABISTChecker

from litex.soc.interconnect.stream_sim import *

from test.model.hdd import *

class TestBIST(unittest.TestCase):
    def test_bist(self):
        def main_generator(dut):
            dut.hdd.malloc(0, 64)
            sector    = 0
            count     = 1
            generator = dut.generator
            checker   = dut.checker
            for i in range(4):
                # Write data
                yield dut.generator.sector.eq(sector)
                yield dut.generator.count.eq(count)
                yield dut.generator.start.eq(1)
                yield
                yield dut.generator.start.eq(0)
                yield
                while not (yield generator.done):
                    yield

                # Verify data
                yield dut.checker.sector.eq(sector)
                yield dut.checker.count.eq(count)
                yield dut.checker.start.eq(1)
                yield
                yield dut.checker.start.eq(0)
                yield
                while not (yield dut.checker.done):
                    yield

                errors = (yield dut.checker.errors)
                print("errors {}".format(errors))
                self.assertEqual(errors, 0)

                # Prepare next iteration
                sector += 1
                count = max((count + 1)%8, 1)

        class DUT(Module):
            def __init__(self, dw=32):
                self.submodules.hdd = HDD(
                    link_debug         = False,
                    link_random_level  = 0,
                    transport_debug    = False,
                    transport_loopback = False,
                    hdd_debug          = True)
                self.submodules.core      = LiteSATACore(self.hdd.phy)
                self.submodules.crossbar  = LiteSATACrossbar(self.core)
                self.submodules.generator = LiteSATABISTGenerator(self.crossbar.get_port(dw))
                self.submodules.checker   = LiteSATABISTChecker(self.crossbar.get_port(dw))

        dut = DUT()
        generators = {
            "sys" : [main_generator(dut),
                       dut.hdd.link.generator(),
                       dut.hdd.phy.rx.generator(),
                       dut.hdd.phy.tx.generator()]
        }
        clocks = {"sys": 10}
        run_simulation(dut, generators, clocks)
