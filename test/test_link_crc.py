#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from litesata.common import *
from litesata.core.link import LiteSATACRC

from test.stream_helpers import *
from test.model.common import sata_crc

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

            # Get reference value
            ref_crc = dut.get_ref_crc(datas)

            # Check results
            s, l, e = check(ref_crc, sim_crc)
            print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))
            self.assertEqual(s, 0)
            self.assertEqual(e, 0)

        class DUT(Module):
            def __init__(self, length, random):
                self.submodules.crc = LiteSATACRC()
                self.length = length
                self.random = random

            def get_ref_crc(self, datas):
                return sata_crc(datas)

        dut = DUT(1024, False)
        run_simulation(dut, generator(dut))
