#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

import subprocess

from litesata.common import *
from litesata.core.link import LiteSATACRC

from litex.soc.interconnect.stream_sim import *

class TestLinkCRC(unittest.TestCase):
    def test_link_crc(self):
        def generator(dut):
            dut.errors = 0
            # Init CRC
            yield dut.crc.data.eq(0)
            yield dut.crc.ce.eq(1)
            yield dut.crc.reset.eq(1)
            yield
            yield dut.crc.reset.eq(0)

            # Feed CRC with datas
            datas = []
            for i in range(dut.length):
                data = seed_to_data(i, dut.random)
                datas.append(data)
                yield dut.crc.data.eq(data)
                yield

            # Log results
            yield
            sim_crc = (yield dut.crc.value)

            # Stop
            yield dut.crc.ce.eq(0)
            for i in range(32):
                yield

            # Get C code reference
            c_crc = dut.get_c_crc(datas)

            # Check results
            s, l, e = check(c_crc, sim_crc)
            print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))
            self.assertEqual(s, 0)
            self.assertEqual(e, 0)

        class DUT(Module):
            def __init__(self, length, random):
                self.submodules.crc = LiteSATACRC()
                self.length = length
                self.random = random

            def get_c_crc(self, datas):
                stdin = ""
                for data in datas:
                    stdin += "0x{:08x} ".format(data)
                stdin += "exit"
                with subprocess.Popen("./test/model/crc", stdin=subprocess.PIPE, stdout=subprocess.PIPE) as process:
                    process.stdin.write(stdin.encode("ASCII"))
                    out, err = process.communicate()
                return int(out.decode("ASCII"), 16)

        dut = DUT(1024, False)
        run_simulation(dut, generator(dut))
