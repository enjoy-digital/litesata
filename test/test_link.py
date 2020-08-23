#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2016 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest

from litesata.common import *
from litesata.core.link import LiteSATALink

from litex.soc.interconnect.stream_sim import *

from test.model.hdd import *


class LinkStreamer(PacketStreamer):
    def __init__(self):
        PacketStreamer.__init__(self, link_description(32), packet_cls=LinkTXPacket)


class LinkLogger(PacketLogger):
    def __init__(self):
        PacketLogger.__init__(self, link_description(32), packet_cls=LinkRXPacket)


class TestLink(unittest.TestCase):
    def test_link(self):
        def generator(dut):
            for i in range(2):
                streamer_packet = LinkTXPacket([i for i in range(64)])
                yield from dut.streamer.send_blocking(streamer_packet)
                yield from dut.logger.receive()

                # check results
                s, l, e = check(streamer_packet, dut.logger.packet)
                print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))
                self.assertEqual(s, 0)
                self.assertEqual(e, 0)

        class DUT(Module):
            def __init__(self):
                self.submodules.hdd = HDD(
                        link_debug         = False,
                        link_random_level  = 50,
                        transport_debug    = False,
                        transport_loopback = True)
                self.submodules.link = LiteSATALink(self.hdd.phy)

                self.submodules.streamer = LinkStreamer()
                self.submodules.streamer_randomizer = Randomizer(link_description(32), level=50)

                self.submodules.logger_randomizer = Randomizer(link_description(32), level=50)
                self.submodules.logger = LinkLogger()

                self.submodules.pipeline = Pipeline(
                    self.streamer,
                    self.streamer_randomizer,
                    self.link,
                    self.logger_randomizer,
                    self.logger
                )

        dut = DUT()
        generators = {
            "sys" :   [generator(dut),
                       dut.hdd.link.generator(),
                       dut.streamer.generator(),
                       dut.streamer_randomizer.generator(),
                       dut.logger.generator(),
                       dut.logger_randomizer.generator(),
                       dut.hdd.phy.rx.generator(),
                       dut.hdd.phy.tx.generator()]
        }
        clocks = {"sys": 10}
        run_simulation(dut, generators, clocks)
