#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *

from migen.genlib.roundrobin import *

# LiteSATAMasterPort -------------------------------------------------------------------------------

class LiteSATAMasterPort:
    """SATA master port

    A port encapsulates 2 endpoints: A sink and a source.
    The sink if used to send command to the device an write data.
    The source is used to receive commands and data from the device.

    A master port drive command_tx and receive on command_rx.
    """
    def __init__(self, dw):
        self.dw     = dw
        self.source = stream.Endpoint(command_tx_description(dw))
        self.sink   = stream.Endpoint(command_rx_description(dw))

    def connect(self, slave):
        return [self.source.connect(slave.sink), slave.source.connect(self.sink)]

# LiteSATASlavePort --------------------------------------------------------------------------------

class LiteSATASlavePort:
    """SATA slave port

    A port encapsulates 2 endpoints: A sink and a source.
    The sink if used to send command to the device an write data.
    The source is used to receive commands and data from the device.

    A master port drive command_rx and receive on command_tx.
    """
    def __init__(self, dw):
        self.dw     = dw
        self.sink   = stream.Endpoint(command_tx_description(dw))
        self.source = stream.Endpoint(command_rx_description(dw))

    def connect(self, master):
        return [self.sink.connect(master.source), master.sink.connect(self.source)]

# LiteSATAUserPort ---------------------------------------------------------------------------------

class LiteSATAUserPort(LiteSATASlavePort):
    """SATA user port

    A user port is simply a slave port exposed to the user.
    """
    def __init__(self, dw, controller_dw=None):
        self.controller_dw = dw if controller_dw is None else controller_dw
        LiteSATASlavePort.__init__(self, dw)

# LiteSATAArbiter ----------------------------------------------------------------------------------

class LiteSATAArbiter(Module):
    """SATA arbiter

    Round robin arbiter between SATA user ports.
    """
    def __init__(self, users, master):
        self.rr = RoundRobin(len(users))
        self.submodules += self.rr
        self.grant = self.rr.grant
        cases = {}
        for i, slave in enumerate(users):
            sink, source = slave.sink, slave.source
            done    = Signal()
            ongoing = Signal()
            self.comb += done.eq(source.valid & source.last & source.end & source.ready)
            self.sync += \
                If(done,
                    ongoing.eq(0)
                ).Elif(sink.valid,
                    ongoing.eq(1)
                )
            self.comb += self.rr.request[i].eq((sink.valid | ongoing) & ~done)
            cases[i] = [users[i].connect(master)]
        self.comb += Case(self.grant, cases)

# LiteSATACrossbar ---------------------------------------------------------------------------------

class LiteSATACrossbar(Module):
    """SATA Crossbar

    This module provides a crossbar between a SATA master port from a controller
    and user ports requested with the get_port method.

    The controller be a PHY but also a RAID (Striping, Mirroring) module.
    """
    def __init__(self, controller):
        self.dw     = len(controller.sink.data)
        self.users  = []
        self.master = LiteSATAMasterPort(self.dw)
        self.comb += [
            self.master.source.connect(controller.sink),
            controller.source.connect(self.master.sink)
        ]

    def get_port(self, dw=None):
        dw = self.dw if dw is None else dw
        user_port     = LiteSATAUserPort(dw, self.dw)
        internal_port = LiteSATAUserPort(self.dw, self.dw)

        if dw != self.dw:
            converter = StrideConverter(
                command_tx_description(user_port.dw),
                command_tx_description(self.dw))
            self.submodules += converter
            self.comb += [
                user_port.sink.connect(converter.sink),
                converter.source.connect(internal_port.sink)
            ]

            converter = StrideConverter(
                command_rx_description(self.dw),
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

    def get_ports(self, n, dw=None):
        dw    = self.dw if dw is None else dw
        ports = []
        for i in range(n):
            ports.append(self.get_port(dw))
        return ports

    def do_finalize(self):
        arbiter = LiteSATAArbiter(self.users, self.master)
        self.submodules += arbiter
