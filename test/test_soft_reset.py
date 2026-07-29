#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.sim import run_simulation

from litex.soc.interconnect import stream

from litesata.common import command_rx_description, command_tx_description
from litesata.frontend.identify import LiteSATASoftReset


class UserPort:
    def __init__(self):
        self.dw = 32
        self.sink = stream.Endpoint(command_tx_description(32))
        self.source = stream.Endpoint(command_rx_description(32))


def test_soft_reset_sequences_assert_hold_and_deassert():
    port = UserPort()
    dut = LiteSATASoftReset(port, reset_cycles=5)
    commands = []

    def stimulus():
        pending_response = False
        yield port.sink.ready.eq(1)
        yield dut.start.eq(1)
        yield
        yield dut.start.eq(0)

        for cycle in range(64):
            command_fire = (
                (yield port.sink.valid) and
                (yield port.sink.ready)
            )
            response_fire = (
                (yield port.source.valid) and
                (yield port.source.ready)
            )
            if response_fire:
                pending_response = False
            if command_fire:
                commands.append((
                    cycle,
                    (yield port.sink.soft_reset),
                    (yield port.sink.control),
                ))
                pending_response = True

            yield port.source.valid.eq(pending_response)
            yield port.source.last.eq(1)
            yield port.source.end.eq(1)

            if len(commands) == 2 and (yield dut.done):
                break
            yield
        else:
            raise AssertionError("soft-reset sequencer did not complete")

    run_simulation(dut, stimulus())

    assert [command[1:] for command in commands] == [
        (1, 0x0c),
        (1, 0x08),
    ]
    assert commands[1][0] - commands[0][0] >= 5
