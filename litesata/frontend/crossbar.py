from litesata.common import *
from litesata.frontend.common import *
from litesata.frontend.arbiter import LiteSATAArbiter


class LiteSATACrossbar(Module):
    def __init__(self, controller):
        self.dw = flen(controller.sink.data)
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
