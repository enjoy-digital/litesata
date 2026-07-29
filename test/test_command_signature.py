#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.sim import run_simulation

from litex.soc.interconnect import stream

from litesata.common import fis_types, transport_rx_description
from litesata.core.command import LiteSATACommandRX


class TransportStub(Module):
    def __init__(self):
        self.source = stream.Endpoint(transport_rx_description(32))


def test_identify_consumes_multibeat_signature_and_keeps_waiting_for_pio():
    class DUT(Module):
        def __init__(self):
            self.submodules.transport = TransportStub()
            self.submodules.command = LiteSATACommandRX(self.transport)

    dut = DUT()
    received = []

    def wait_state(name, timeout=32):
        expected = dut.command.fsm.encoding[name]
        for _ in range(timeout):
            if (yield dut.command.fsm.state) == expected:
                return
            yield
        state = (yield dut.command.fsm.state)
        actual = next(
            state_name for state_name, encoding in dut.command.fsm.encoding.items()
            if encoding == state
        )
        raise AssertionError(f"command RX did not enter {name} (stopped in {actual})")

    def drive_source(fis_type, data, last, status=0):
        source = dut.transport.source
        yield source.valid.eq(1)
        yield source.type.eq(fis_type)
        yield source.status.eq(status)
        yield source.error.eq(0)
        yield source.data.eq(data)
        yield source.last.eq(last)

    def stop_source():
        yield dut.transport.source.valid.eq(0)
        yield dut.transport.source.last.eq(0)
        yield

    @passive
    def monitor():
        while True:
            yield dut.command.source.ready.eq(1)
            if (yield dut.command.source.valid):
                received.append((
                    (yield dut.command.source.data),
                    (yield dut.command.source.last),
                    (yield dut.command.source.failed),
                ))
            yield

    def stimulus():
        yield dut.command.from_tx.identify.eq(1)
        yield
        yield dut.command.from_tx.identify.eq(0)
        yield from wait_state("WAIT_PIO_SETUP_D2H")

        # A buffered signature FIS is legal before the command's PIO Setup FIS.
        yield from drive_source(fis_types["REG_D2H"], 0x34, last=0)
        yield
        yield from wait_state("EAT_REG_D2H")
        yield from drive_source(fis_types["REG_D2H"], 0x00000101, last=1)
        yield
        yield from stop_source()
        yield from wait_state("WAIT_PIO_SETUP_D2H")

        for _ in range(8):
            assert (yield dut.command.fsm.state) == dut.command.fsm.encoding["WAIT_PIO_SETUP_D2H"]
            assert (yield dut.transport.source.ready)
            yield
        assert received == []

    run_simulation(dut, [stimulus(), monitor()])
