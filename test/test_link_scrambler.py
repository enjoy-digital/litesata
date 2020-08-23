#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2016 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest
import subprocess

from litesata.common import *
from litesata.core.link import Scrambler

from litex.soc.interconnect.stream_sim import *


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

            # Get C code reference
            c_values = dut.get_c_values(dut.length)

            # Check results
            s, l, e = check(c_values, sim_values)
            print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))
            self.assertEqual(s, 0)
            self.assertEqual(e, 0)

        class DUT(Module):
            def __init__(self, length):
                self.submodules.scrambler = ResetInserter()(Scrambler())
                self.length = length

            def get_c_values(self, length):
                stdin = "0x{:08x}".format(length)
                with subprocess.Popen("./test/model/scrambler",
                    stdin  = subprocess.PIPE,
                    stdout = subprocess.PIPE) as process:
                    process.stdin.write(stdin.encode("ASCII"))
                    out, err = process.communicate()
                return [int(e, 16) for e in out.decode("ASCII").split("\n")[:-1]]

        dut = DUT(1024)
        generators = {
            "sys" :   [generator(dut)]
        }
        clocks = {"sys": 10}
        run_simulation(dut, generators, clocks)
