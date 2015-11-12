from litesata.common import *

from migen.genlib.roundrobin import *


class LiteSATAMasterPort:
    def __init__(self, dw):
        self.dw = dw
        self.source = Source(command_tx_description(dw))
        self.sink = Sink(command_rx_description(dw))

    def connect(self, slave):
        return [
            Record.connect(self.source, slave.sink),
            Record.connect(slave.source, self.sink)
        ]


class LiteSATASlavePort:
    def __init__(self, dw):
        self.dw = dw
        self.sink = Sink(command_tx_description(dw))
        self.source = Source(command_rx_description(dw))

    def connect(self, master):
        return [
            Record.connect(self.sink, master.source),
            Record.connect(master.sink, self.source)
        ]


class LiteSATAUserPort(LiteSATASlavePort):
    def __init__(self, dw, controller_dw=None):
        self.controller_dw = dw if controller_dw is None else controller_dw
        LiteSATASlavePort.__init__(self, dw)


class LiteSATAArbiter(Module):
    def __init__(self, users, master):
        self.rr = RoundRobin(len(users))
        self.submodules += self.rr
        self.grant = self.rr.grant
        cases = {}
        for i, slave in enumerate(users):
            sink, source = slave.sink, slave.source
            start = Signal()
            done = Signal()
            ongoing = Signal()
            self.comb += [
                start.eq(sink.stb & sink.sop),
                done.eq(source.stb & source.last & source.eop & source.ack)
            ]
            self.sync += \
                If(start,
                    ongoing.eq(1)
                ).Elif(done,
                    ongoing.eq(0)
                )
            self.comb += self.rr.request[i].eq((start | ongoing) & ~done)
            cases[i] = [users[i].connect(master)]
        self.comb += Case(self.grant, cases)


class LiteSATACrossbar(Module):
    def __init__(self, controller):
        self.dw = len(controller.sink.data)
        self.users = []
        self.master = LiteSATAMasterPort(self.dw)
        self.comb += [
            self.master.source.connect(controller.sink),
            controller.source.connect(self.master.sink)
        ]

    def get_port(self, dw=32):
        user_port = LiteSATAUserPort(dw, self.dw)
        internal_port = LiteSATAUserPort(self.dw, self.dw)

        if dw != self.dw:
            converter = Converter(command_tx_description(user_port.dw),
                                  command_tx_description(self.dw))
            self.submodules += converter
            self.comb += [
                Record.connect(user_port.sink, converter.sink),
                Record.connect(converter.source, internal_port.sink)
            ]

            converter = Converter(command_rx_description(self.dw),
                                  command_rx_description(user_port.dw))
            self.submodules += converter
            self.comb += [
                Record.connect(internal_port.source, converter.sink),
                Record.connect(converter.source, user_port.source)
            ]

            self.users += [internal_port]
        else:
            self.users += [user_port]

        return user_port

    def get_ports(self, n, dw=32):
        ports = []
        for i in range(n):
            ports.append(self.get_port(dw))
        return ports

    def do_finalize(self):
        arbiter = LiteSATAArbiter(self.users, self.master)
        self.submodules += arbiter
