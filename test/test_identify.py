#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from litesata.common import *
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.identify import LiteSATAIdentify

from test.model.hdd import *


class TestIdentify(unittest.TestCase):
    def test_identify(self):
        def main_generator(dut):
            hdd = dut.hdd

            def identify(timeout=50000):
                yield dut.identify.start.eq(1)
                yield
                yield dut.identify.start.eq(0)
                yield
                for _ in range(timeout):
                    if (yield dut.identify.done):
                        break
                    yield
                else:
                    self.fail("Timeout waiting for identify to complete")
                error = (yield dut.identify.error)
                # Drain the FIFO: ready takes effect at the next edge, data is read before each pop.
                data  = []
                yield dut.identify.source.ready.eq(1)
                yield
                while (yield dut.identify.source.valid):
                    data.append((yield dut.identify.source.data))
                    yield
                yield dut.identify.source.ready.eq(0)
                yield
                return error, data

            # Success.
            error, data = yield from identify()
            self.assertEqual(error, 0)
            self.assertEqual(data, hdd.identify_data)

            # Aborted by the device: done with error, no data.
            hdd.set_busy(1)
            hdd.set_reg_d2h_status(0x41)
            error, data = yield from identify()
            self.assertEqual(error, 1)
            self.assertEqual(data, [])
            hdd.set_busy(0)
            hdd.set_reg_d2h_status(0)

            # Recovered: success again.
            error, data = yield from identify()
            self.assertEqual(error, 0)
            self.assertEqual(data, hdd.identify_data)

        class DUT(Module):
            def __init__(self):
                self.submodules.hdd = HDD(
                    link_debug         = False,
                    link_random_level  = 0,
                    transport_debug    = False,
                    transport_loopback = False,
                    hdd_debug          = True)
                self.submodules.core     = LiteSATACore(self.hdd.phy)
                self.submodules.crossbar = LiteSATACrossbar(self.core)
                self.submodules.identify = LiteSATAIdentify(self.crossbar.get_port())

        dut = DUT()
        generators = {
            "sys" : [main_generator(dut),
                     dut.hdd.link.generator(),
                     dut.hdd.phy.rx.generator(),
                     dut.hdd.phy.tx.generator()]
        }
        run_simulation(dut, generators, {"sys": 10})
