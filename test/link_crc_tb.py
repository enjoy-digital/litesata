#!/usr/bin/env python3
import subprocess

from litesata.common import *
from litesata.core.link import LiteSATACRC

from common import *


class TB(Module):
    def __init__(self, length, random):
        self.submodules.crc = LiteSATACRC()
        self.length = length
        self.random = random

    def get_c_crc(self, datas):
        stdin = ""
        for data in datas:
            stdin += "0x{:08x} ".format(data)
        stdin += "exit"
        with subprocess.Popen("./crc",
                              stdin=subprocess.PIPE,
                              stdout=subprocess.PIPE) as process:
            process.stdin.write(stdin.encode("ASCII"))
            out, err = process.communicate()
        return int(out.decode("ASCII"), 16)

def main_generator(dut):
    # init CRC
    yield dut.crc.data.eq(0)
    yield dut.crc.ce.eq(1)
    yield dut.crc.reset.eq(1)
    yield
    yield dut.crc.reset.eq(0)

    # feed CRC with datas
    datas = []
    for i in range(dut.length):
        data = seed_to_data(i, dut.random)
        datas.append(data)
        yield dut.crc.data.eq(data)
        yield

    # log results
    yield
    sim_crc = (yield dut.crc.value)

    # stop
    yield dut.crc.ce.eq(0)
    for i in range(32):
        yield

    # get C core reference
    c_crc = dut.get_c_crc(datas)

    # check results
    s, l, e = check(c_crc, sim_crc)
    print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))

if __name__ == "__main__":
    tb = TB(1024, False)
    generators = {
        "sys" :   [main_generator(tb)]
    }
    clocks = {"sys": 10}
    run_simulation(tb, generators, clocks, vcd_name="sim.vcd")
