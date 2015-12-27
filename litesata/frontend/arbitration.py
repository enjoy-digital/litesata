from litesata.common import *

from litex.gen.genlib.roundrobin import *


class LiteSATAMasterPort:
    def __init__(self, dw):
        self.dw = dw
        self.source = Source(command_tx_description(dw))
        self.sink = Sink(command_rx_description(dw))

    def connect(self, slave):
        return [self.source.connect(slave.sink),
                slave.source.connect(self.sink)]


class LiteSATASlavePort:
    def __init__(self, dw):
        self.dw = dw
        self.sink = Sink(command_tx_description(dw))
        self.source = Source(command_rx_description(dw))

    def connect(self, master):
        return [self.sink.connect(master.source),
                master.sink.connect(self.source)]


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
                user_port.sink.connect(converter.sink),
                converter.source.connect(internal_port.sink)
            ]

            converter = Converter(command_rx_description(self.dw),
                                  command_rx_description(user_port.dw))
            self.submodules += converter
            self.comb += [
                internal_port.source.connect(converter.sink),
                converter.source.connect(user_port.source)
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
