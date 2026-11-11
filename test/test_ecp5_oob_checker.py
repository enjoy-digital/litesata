#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from litesata.common import *
from litesata.phy.ecp5sataphy import ECP5SATAOOBChecker

# Helpers ------------------------------------------------------------------------------------------

def check_oob(clk_freq, gap_ns, burst_ns=106.7):
    """Send 6 bursts separated by gap_ns idles to the checker, return (cominit, comwake) seen."""
    dut = ECP5SATAOOBChecker(clk_freq)
    det = {"cominit": 0, "comwake": 0}
    def wait(ns):
        for i in range(round(ns*1e-9*clk_freq)):
            yield
            det["cominit"] |= (yield dut.cominit_det)
            det["comwake"] |= (yield dut.comwake_det)
    def generator():
        yield dut.rx_idle.eq(1)
        yield from wait(2000)
        for burst in range(6):
            yield dut.rx_idle.eq(0)
            yield from wait(burst_ns)
            yield dut.rx_idle.eq(1)
            yield from wait(gap_ns)
        yield from wait(2000)
    run_simulation(dut, generator())
    return det["cominit"], det["comwake"]

# Test ---------------------------------------------------------------------------------------------

class TestECP5OOBChecker(unittest.TestCase):
    def test_cominit_comwake_detection(self):
        # COMINIT: 320ns gaps, COMWAKE: 106.7ns gaps, over the usable sys clock range.
        for clk_freq in [75e6, 90e6, 100e6, 125e6, 150e6]:
            with self.subTest(clk_freq=clk_freq):
                self.assertEqual(check_oob(clk_freq, gap_ns=320.0), (1, 0))
                self.assertEqual(check_oob(clk_freq, gap_ns=106.7), (0, 1))


if __name__ == "__main__":
    unittest.main()
