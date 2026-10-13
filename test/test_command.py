#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2016 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import unittest
from copy import deepcopy

from litesata.common import *
from litesata.core import LiteSATACore
from litesata.core.command import LiteSATACommandRX

from test.stream_helpers import *

from test.model.hdd import *


class CommandTXPacket(list):
    def __init__(self, write=0, read=0, sector=0, count=0, data=[]):
        self.ongoing = False
        self.done    = False
        self.write   = write
        self.read    = read
        self.sector  = sector
        self.count   = count
        for d in data:
            self.append(d)


class CommandStreamer(PacketStreamer):
    def __init__(self):
        self.source = stream.Endpoint(command_tx_description(32))

        # # #

        self.packets = []
        self.packet  = CommandTXPacket()
        self.packet.done = True

    def send(self, packet):
        packet = deepcopy(packet)
        self.packets.append(packet)
        return packet

    def send_blocking(self, packet):
        packet = self.send(packet)
        while not packet.done:
            yield

    @passive
    def generator(self):
        while True:
            if len(self.packets) and self.packet.done:
                self.packet = self.packets.pop(0)
            yield self.source.write.eq(self.packet.write)
            yield self.source.read.eq(self.packet.read)
            yield self.source.sector.eq(self.packet.sector)
            yield self.source.count.eq(self.packet.count)
            if not self.packet.ongoing and not self.packet.done:
                yield self.source.valid.eq(1)
                if len(self.packet) > 0:
                    yield self.source.data.eq(self.packet.pop(0))
                self.packet.ongoing = True
            elif (yield self.source.valid) and (yield self.source.ready):
                yield self.source.last.eq(len(self.packet) == 1)
                if len(self.packet) > 0:
                    yield self.source.valid.eq(1)
                    yield self.source.data.eq(self.packet.pop(0))
                else:
                    self.packet.done = True
                    yield self.source.valid.eq(0)

            yield


class CommandRXPacket(list):
    def __init__(self):
        self.ongoing = False
        self.done    = False
        self.write   = 0
        self.read    = 0
        self.failed  = 0


class CommandLogger(PacketLogger):
    def __init__(self):
        PacketLogger.__init__(self, command_rx_description(32), CommandRXPacket)
        self.first = True

    @passive
    def generator(self):
        while True:
            yield self.sink.ready.eq(1)
            if (yield self.sink.valid) and self.first:
                self.packet = CommandRXPacket()
                self.packet.write  = (yield self.sink.write)
                self.packet.read   = (yield self.sink.read)
                self.packet.failed = (yield self.sink.failed)
                self.packet.append((yield self.sink.data))
                self.first = False
            elif (yield self.sink.valid):
                self.packet.append((yield self.sink.data))
            if (yield self.sink.valid) and (yield self.sink.last):
                self.packet.done = True
                self.first = True
            yield


class TestCommand(unittest.TestCase):
    def test_write_device_fault(self):
        transport = Module()
        transport.source = stream.Endpoint(transport_rx_description(32))
        dut = LiteSATACommandRX(transport)
        results = []

        def generator():
            # ERR and DF independently fail a write; a later success clears the error.
            for status in (0x40, 0x41, 0x60, 0x61, 0x40):
                yield dut.from_tx.write.eq(1)
                yield
                yield dut.from_tx.write.eq(0)
                yield

                yield transport.source.type.eq(fis_types["REG_D2H"])
                yield transport.source.status.eq(status)
                yield transport.source.errors.eq(4 if status & 1 else 0)
                yield transport.source.i.eq(1)
                yield transport.source.last.eq(1)
                yield transport.source.valid.eq(1)
                yield
                self.assertEqual((yield transport.source.ready), 1)
                yield transport.source.valid.eq(0)
                yield

                failed = (yield dut.source.failed)
                results.append(failed)
                # Hold the completion under backpressure before accepting it.
                for _ in range(3):
                    self.assertEqual((yield dut.source.valid), 1)
                    self.assertEqual((yield dut.source.write), 1)
                    self.assertEqual((yield dut.source.end), 1)
                    self.assertEqual((yield dut.source.last), 1)
                    self.assertEqual((yield dut.source.failed), failed)
                    yield
                yield dut.source.ready.eq(1)
                yield
                yield dut.source.ready.eq(0)
                yield

        run_simulation(dut, generator())
        self.assertEqual(results, [0, 1, 1, 1, 0])

    def test_command(self):
        def generator(dut):
            hdd = dut.hdd
            hdd.malloc(0, 64)
            write_data   = [i for i in range(sectors2dwords(2))]
            write_len    = dwords2sectors(len(write_data))
            write_packet = CommandTXPacket(write=1, sector=2, count=write_len, data=write_data)
            yield from dut.streamer.send_blocking(write_packet)
            yield from dut.logger.receive()

            read_packet = CommandTXPacket(read=1, sector=2, count=write_len)
            yield from dut.streamer.send_blocking(read_packet)
            yield from dut.logger.receive()
            read_data = dut.logger.packet

            # Check results
            s, l, e = check(write_data, read_data)
            print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))
            self.assertEqual(s, 0)
            self.assertEqual(e, 0)

        class DUT(Module):
            def __init__(self):
                self.submodules.hdd = HDD(
                    link_debug         = False,
                    link_random_level  = 50,
                    transport_debug    = False,
                    transport_loopback = False,
                    hdd_debug          = True)
                self.submodules.core = LiteSATACore(self.hdd.phy)

                self.submodules.streamer = CommandStreamer()
                self.submodules.streamer_randomizer = Randomizer(command_tx_description(32), level=50)

                self.submodules.logger = CommandLogger()
                self.submodules.logger_randomizer = Randomizer(command_rx_description(32), level=50)

                self.submodules.pipeline = Pipeline(
                    self.streamer,
                    self.streamer_randomizer,
                    self.core,
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
