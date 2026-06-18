#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2016 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from litesata.common import *
from litesata.core.link import Scrambler

from test.stream_helpers import *
from test.model.common import scrambler_values


class TestLinkScrambler(unittest.TestCase):
    def test_link_scrambler(self):
        def generator(dut):
            # Init CRC
            yield dut.scrambler.ce.eq(1)
            yield dut.scrambler.reset.eq(1)
            yield
            yield dut.scrambler.reset.eq(0)

            # Log results
            yield
            sim_values = []
            for i in range(dut.length):
                sim_values.append((yield dut.scrambler.value))
                yield

            # Stop
            yield dut.scrambler.ce.eq(0)
            for i in range(32):
                yield

            # Get reference values
            ref_values = dut.get_ref_values(dut.length)

            # Check results
            s, l, e = check(ref_values, sim_values)
            print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))
            self.assertEqual(s, 0)
            self.assertEqual(e, 0)

        class DUT(Module):
            def __init__(self, length):
                self.submodules.scrambler = ResetInserter()(Scrambler())
                self.length = length

            def get_ref_values(self, length):
                return list(scrambler_values(length))

        dut = DUT(1024)
        generators = {
            "sys" :   [generator(dut)]
        }
        clocks = {"sys": 10}
        run_simulation(dut, generators, clocks)
