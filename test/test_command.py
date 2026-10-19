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
    def __init__(self, write=0, read=0, identify=0, sector=0, count=0, data=[]):
        self.ongoing  = False
        self.done     = False
        self.write    = write
        self.read     = read
        self.identify = identify
        self.sector   = sector
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
            yield self.source.identify.eq(self.packet.identify)
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
        self.ongoing  = False
        self.done     = False
        self.write    = 0
        self.read     = 0
        self.identify = 0
        self.end      = 0
        self.failed   = 0


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
                self.packet.write    = (yield self.sink.write)
                self.packet.read     = (yield self.sink.read)
                self.packet.identify = (yield self.sink.identify)
                self.packet.append((yield self.sink.data))
                self.first = False
            elif (yield self.sink.valid):
                self.packet.append((yield self.sink.data))
            if (yield self.sink.valid):
                # failed can be flagged on any beat, end on the final one.
                self.packet.failed |= (yield self.sink.failed)
                self.packet.end     = (yield self.sink.end)
            if (yield self.sink.valid) and (yield self.sink.last):
                self.packet.done = True
                self.first = True
            yield


class CommandDUT(Module):
    def __init__(self, random_level=50):
        self.submodules.hdd = HDD(
            link_debug         = False,
            link_random_level  = random_level,
            transport_debug    = False,
            transport_loopback = False,
            hdd_debug          = True)
        self.submodules.core = LiteSATACore(self.hdd.phy)

        self.submodules.streamer = CommandStreamer()
        self.submodules.streamer_randomizer = Randomizer(command_tx_description(32), level=random_level)

        self.submodules.logger = CommandLogger()
        self.submodules.logger_randomizer = Randomizer(command_rx_description(32), level=random_level)

        self.submodules.pipeline = Pipeline(
            self.streamer,
            self.streamer_randomizer,
            self.core,
            self.logger_randomizer,
            self.logger
        )

    def run(self, generator):
        generators = {
            "sys" :   [generator,
                       self.hdd.link.generator(),
                       self.streamer.generator(),
                       self.streamer_randomizer.generator(),
                       self.logger.generator(),
                       self.logger_randomizer.generator(),
                       self.hdd.phy.rx.generator(),
                       self.hdd.phy.tx.generator()]
        }
        run_simulation(self, generators, {"sys": 10})


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

        dut = CommandDUT(random_level=50)
        dut.run(generator(dut))

    def receive(self, dut, timeout=50000):
        """Wait for the next response packet, failing instead of hanging on a regression."""
        dut.logger.packet.done = False
        for _ in range(timeout):
            if dut.logger.packet.done:
                return
            yield
        self.fail("Timeout waiting for a command response")

    def test_read_device_fault(self):
        def generator(dut):
            hdd = dut.hdd
            hdd.malloc(0, 8)
            data = [i for i in range(sectors2dwords(1))]

            # A write completed with DF (and ERR clear) fails.
            hdd.set_reg_d2h_status(1 << reg_d2h_status["df"])
            yield from dut.streamer.send_blocking(CommandTXPacket(write=1, sector=1, count=1, data=data))
            yield from self.receive(dut)
            self.assertEqual(dut.logger.packet.write,  1)
            self.assertEqual(dut.logger.packet.end,    1)
            self.assertEqual(dut.logger.packet.failed, 1)

            # A read completed with DF fails: data packet, then a failed response.
            yield from dut.streamer.send_blocking(CommandTXPacket(read=1, sector=1, count=1))
            yield from self.receive(dut)
            self.assertEqual(dut.logger.packet.read, 1)
            self.assertEqual(dut.logger.packet.end,  0)
            self.assertEqual(len(dut.logger.packet), sectors2dwords(1))
            yield from self.receive(dut)
            self.assertEqual(dut.logger.packet.read,   1)
            self.assertEqual(dut.logger.packet.end,    1)
            self.assertEqual(dut.logger.packet.failed, 1)

            # A clean completion succeeds and returns the data.
            hdd.set_reg_d2h_status(0)
            yield from dut.streamer.send_blocking(CommandTXPacket(read=1, sector=1, count=1))
            yield from self.receive(dut)
            read_data = list(dut.logger.packet)
            yield from self.receive(dut)
            self.assertEqual(dut.logger.packet.end,    1)
            self.assertEqual(dut.logger.packet.failed, 0)
            self.assertEqual(read_data, data)

        dut = CommandDUT(random_level=0)
        dut.run(generator(dut))

    def test_identify(self):
        def generator(dut):
            hdd = dut.hdd

            def identify():
                yield from dut.streamer.send_blocking(CommandTXPacket(identify=1))
                yield from self.receive(dut)
                self.assertEqual(dut.logger.packet.identify, 1)
                self.assertEqual(dut.logger.packet.read,     0)
                self.assertEqual(dut.logger.packet.end,      1)

            # Success: one 128-dword block, end on the final dword.
            yield from identify()
            self.assertEqual(dut.logger.packet.failed, 0)
            self.assertEqual(list(dut.logger.packet), hdd.identify_data)

            # Device Fault flagged in the PIO Setup FIS: a single failed response.
            hdd.set_pio_setup_status(0x58 | (1 << reg_d2h_status["df"]))
            yield from identify()
            self.assertEqual(dut.logger.packet.failed, 1)
            self.assertEqual(len(dut.logger.packet), 1)
            hdd.set_pio_setup_status(0x58)

            # Command aborted with a Register D2H (ERR) instead of a PIO Setup.
            hdd.set_busy(1)
            hdd.set_reg_d2h_status(0x41)
            yield from identify()
            self.assertEqual(dut.logger.packet.failed, 1)
            self.assertEqual(len(dut.logger.packet), 1)
            hdd.set_busy(0)
            hdd.set_reg_d2h_status(0)

            # The port is not stuck: a following identify succeeds.
            yield from identify()
            self.assertEqual(dut.logger.packet.failed, 0)
            self.assertEqual(list(dut.logger.packet), hdd.identify_data)

        dut = CommandDUT(random_level=0)
        dut.run(generator(dut))
