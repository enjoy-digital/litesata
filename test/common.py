import random
from copy import deepcopy

from litesata.common import *

def print_with_prefix(s, prefix=""):
    if not isinstance(s, str):
        s = s.__repr__()
    s = s.split("\n")
    for l in s:
        print(prefix + l)


def seed_to_data(seed, random=True):
    if random:
        return (seed * 0x31415979 + 1) & 0xffffffff
    else:
        return seed


def check(p1, p2):
    p1 = deepcopy(p1)
    p2 = deepcopy(p2)
    if isinstance(p1, int):
        return 0, 1, int(p1 != p2)
    else:
        if len(p1) >= len(p2):
            ref, res = p1, p2
        else:
            ref, res = p2, p1
        shift = 0
        while((ref[0] != res[0]) and (len(res) > 1)):
            res.pop(0)
            shift += 1
        length = min(len(ref), len(res))
        errors = 0
        for i in range(length):
            if ref.pop(0) != res.pop(0):
                errors += 1
        return shift, length, errors


def randn(max_n):
    return random.randint(0, max_n-1)


class PacketStreamer(Module):
    def __init__(self, description, packet_class):
        self.source = stream.Endpoint(description)

        # # #

        self.packets = []
        self.packet = packet_class()
        self.packet.done = True

    def send(self, packet):
        packet = deepcopy(packet)
        self.packets.append(packet)
        return packet

    def send_blocking(self, packet):
        packet = self.send(packet)
        while not packet.done:
            yield

    def generator(self):
        while True:
            if len(self.packets) and self.packet.done:
                self.packet = self.packets.pop(0)
            if not self.packet.ongoing and not self.packet.done:
                yield self.source.valid.eq(1)
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


class PacketLogger(Module):
    def __init__(self, description, packet_class):
        self.sink = stream.Endpoint(description)

        # # #

        self.packet_class = packet_class
        self.packet = packet_class()
        self.first = True

    def receive(self, length=None):
        self.packet.done = 0
        if length is None:
            while self.packet.done == 0:
                yield
        else:
            while length > len(self.packet):
                yield

    def generator(self):
        while True:
            yield self.sink.ready.eq(1)
            if (yield self.sink.valid) and self.first:
                self.packet = self.packet_class()
                self.first = False
            if (yield self.sink.valid):
                if hasattr(self.sink, "data"):
                    self.packet.append((yield self.sink.data))
                else:
                    self.packet.append((yield self.sink.d))
            if (yield self.sink.valid) and (yield self.sink.last):
                self.packet.done = True
                self.first = True
            yield


class Randomizer(Module):
    def __init__(self, description, level=0):
        self.level = level

        self.sink = stream.Endpoint(description)
        self.source = stream.Endpoint(description)

        self.ce = Signal(reset=1)

        self.comb += \
            If(self.ce,
                Record.connect(self.sink, self.source)
            ).Else(
                self.source.valid.eq(0),
                self.sink.ready.eq(0),
            )

    def generator(self):
        while True:
            n = randn(100)
            if n < self.level:
                yield self.ce.eq(0)
            else:
                yield self.ce.eq(1)
            yield
