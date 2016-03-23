from litex.soc.interconnect.stream_sim import *

from litesata.common import *


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

    @passive
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
                if len(self.packet):
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
            while not self.packet.done:
                yield
        else:
            while length > len(self.packet):
                yield

    @passive
    def generator(self):
        while True:
            yield self.sink.ready.eq(1)
            if (yield self.sink.valid):
                if self.first:
                    self.packet = self.packet_class()
                    self.first = False
                self.packet.append((yield self.sink.data))
                if (yield self.sink.last):
                    self.packet.done = True
                    self.first = True
            yield
