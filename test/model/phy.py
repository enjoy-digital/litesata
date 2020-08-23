# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *

# PHYDword -----------------------------------------------------------------------------------------

class PHYDword:
    def __init__(self, dat=0):
        self.dat   = dat
        self.start = 1
        self.done  = 0

# PHYSource ----------------------------------------------------------------------------------------

class PHYSource(Module):
    def __init__(self):
        self.source = stream.Endpoint(phy_description(32))

        # # #

        self.dword = PHYDword()

    def send(self, dword):
        self.dword = dword

    @passive
    def generator(self):
        while True:
            yield self.source.valid.eq(1)
            yield self.source.charisk.eq(0b0000)
            for k, v in primitives.items():
                if v == self.dword.dat:
                    yield self.source.charisk.eq(0b0001)
            yield self.source.data.eq(self.dword.dat)
            yield

# PHYSink ------------------------------------------------------------------------------------------

class PHYSink(Module):
    def __init__(self):
        self.sink = stream.Endpoint(phy_description(32))

        # # #

        self.dword = PHYDword()

    def receive(self):
        self.dword.done = 0
        while self.dword.done == 0:
            yield

    @passive
    def generator(self):
        while True:
            self.dword.done = 0
            yield self.sink.ready.eq(1)
            if (yield self.sink.valid):
                self.dword.done = 1
                self.dword.dat = (yield self.sink.data)
            yield

# PHY Layer model ----------------------------------------------------------------------------------

class PHYLayer(Module):
    def __init__(self):

        self.submodules.rx = PHYSink()
        self.submodules.tx = PHYSource()

        self.source = self.tx.source
        self.sink   = self.rx.sink

    def send(self, dword):
        packet = PHYDword(dword)
        self.tx.send(packet)

    def receive(self):
        yield from self.rx.receive()

    def __repr__(self):
        receiving = "{:08x} ".format(self.rx.dword.dat)
        receiving += decode_primitive(self.rx.dword.dat)
        receiving += " "*(16-len(receiving))

        sending = "{:08x} ".format(self.tx.dword.dat)
        sending += decode_primitive(self.tx.dword.dat)
        sending += " "*(16-len(sending))

        return receiving + sending
