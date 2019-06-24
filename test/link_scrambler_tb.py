#!/usr/bin/env python3

# This file is Copyright (c) 2015-2016 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

import subprocess

from litesata.common import *
from litesata.core.link import Scrambler

from litex.soc.interconnect.stream_sim import *


class TB(Module):
    def __init__(self, length):
        self.submodules.scrambler = ResetInserter()(Scrambler())
        self.length = length

    def get_c_values(self, length):
        stdin = "0x{:08x}".format(length)
        with subprocess.Popen("./scrambler",
                              stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE) as process:
            process.stdin.write(stdin.encode("ASCII"))
            out, err = process.communicate()
        return [int(e, 16) for e in out.decode("ASCII").split("\n")[:-1]]

def main_generator(dut):
    # init CRC
    yield dut.scrambler.ce.eq(1)
    yield dut.scrambler.reset.eq(1)
    yield
    yield dut.scrambler.reset.eq(0)

    # log results
    yield
    sim_values = []
    for i in range(dut.length):
        sim_values.append((yield dut.scrambler.value))
        yield

    # stop
    yield dut.scrambler.ce.eq(0)
    for i in range(32):
        yield

    # get C code reference
    c_values = dut.get_c_values(dut.length)

    # check results
    s, l, e = check(c_values, sim_values)
    print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))


if __name__ == "__main__":
    tb = TB(1024)
    generators = {
        "sys" :   [main_generator(tb)]
    }
    clocks = {"sys": 10}
    run_simulation(tb, generators, clocks, vcd_name="sim.vcd")
