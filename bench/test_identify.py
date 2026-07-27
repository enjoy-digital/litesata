#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Standalone IDENTIFY DEVICE test.

Unlike bench/test_bist.py this never spins unbounded: LiteSATAIdentify has no
timeout and no error path (litesata/frontend/identify.py: `done` is asserted only
in IDLE), and a hung identify keeps the crossbar round-robin grant
(litesata/frontend/arbitration.py), which wedges every other port until the FPGA
is reloaded. So every wait here is bounded and reports what it saw.
"""

import sys
import time
import argparse

from collections import OrderedDict

from litex import RemoteClient

logical_sector_size = 512

# Identify -----------------------------------------------------------------------------------------

class Identify:
    def __init__(self, bus, timeout=1.0):
        self.bus     = bus
        self.timeout = timeout
        self.data    = []

    def drain_fifo(self, limit=4096):
        """Pop everything currently in the identify FIFO (3 transactions per dword)."""
        n = 0
        while self.bus.regs.sata_bist_identify_source_valid.read() and n < limit:
            dword = self.bus.regs.sata_bist_identify_source_data.read()
            self.data += [dword & 0xffff, (dword >> 16) & 0xffff]
            self.bus.regs.sata_bist_identify_source_ready.write(1)
            n += 1
        return n

    def run(self):
        # Flush anything stale from a previous (possibly failed) attempt.
        self.data = []
        flushed = self.drain_fifo()
        if flushed:
            print(f"  (flushed {flushed} stale dwords)")
        self.data = []

        self.bus.regs.sata_bist_identify_start.write(1)  # pulse CSR: write(0) would be a no-op

        t0 = time.time()
        while self.bus.regs.sata_bist_identify_done.read() == 0:
            if time.time() - t0 > self.timeout:
                return False
            time.sleep(1e-3)

        self.drain_fifo()
        return True

    def decode(self):
        def string(words):
            out = ""
            for w in words:
                out += w.to_bytes(2, byteorder="big").decode("utf-8", errors="replace")
            return out.strip()

        self.serial_number     = string(self.data[10:20])
        self.firmware_revision = string(self.data[23:27])
        self.model_number      = string(self.data[27:46])

        self.total_sectors  = self.data[100]
        self.total_sectors += (self.data[101] << 16)
        self.total_sectors += (self.data[102] << 32)
        self.total_sectors += (self.data[103] << 48)

        self.capabilities = OrderedDict()
        self.capabilities["SATA Gen1"]             = (self.data[76] >> 1) & 0x1
        self.capabilities["SATA Gen2"]             = (self.data[76] >> 2) & 0x1
        self.capabilities["SATA Gen3"]             = (self.data[76] >> 3) & 0x1
        self.capabilities["48 bits LBA supported"] = (self.data[83] >> 10) & 0x1

    def info(self):
        gb   = 1024**3
        out  = f"Serial Number:     {self.serial_number}\n"
        out += f"Firmware Revision: {self.firmware_revision}\n"
        out += f"Model Number:      {self.model_number}\n"
        out += "Capacity:          {:3.2f} GiB\n".format(
            (self.total_sectors*logical_sector_size)/gb)
        for k, v in self.capabilities.items():
            out += f"{k}: {v}\n"
        return out

# Identify Test ------------------------------------------------------------------------------------

def identify_test(port, link_timeout, identify_timeout, reset):
    bus = RemoteClient(port=port)
    bus.open()

    if reset:
        bus.regs.sata_phy_enable.write(0)
        time.sleep(1e-3)
        bus.regs.sata_phy_enable.write(1)

    # 1. Wait for the PHY to report a link, bounded.
    t0     = time.time()
    status = 0
    while time.time() - t0 < link_timeout:
        status = bus.regs.sata_phy_status.read()
        if status & 0x1:
            break
        time.sleep(10e-3)

    if not (status & 0x1):
        print("Link not ready (status: 0x{:08x}, tx_ready: {:d}, rx_ready: {:d}, ctrl_ready: {:d})".format(
            status, (status >> 1) & 0x1, (status >> 2) & 0x1, (status >> 3) & 0x1))
        bus.close()
        return 1
    print(f"Link ready after {time.time()-t0:.2f}s (status: 0x{status:08x})")

    # 2. IDENTIFY, bounded.
    ident = Identify(bus, timeout=identify_timeout)
    ok    = ident.run()
    status_after = bus.regs.sata_phy_status.read()

    if not ok:
        print(f"IDENTIFY timed out after {identify_timeout}s "
              f"(phy status now 0x{status_after:08x}).")
        print("The identify FSM has no error path: it is now parked off-IDLE and holds the")
        print("crossbar grant. Reload the bitstream before retrying.")
        bus.close()
        return 1

    # 3. A short drain means a partial transfer - report it rather than crashing in decode().
    if len(ident.data) != 256:
        print(f"IDENTIFY returned {len(ident.data)} words, expected 256 (partial transfer).")
        bus.close()
        return 1

    ident.decode()
    print(ident.info(), end="")
    print(f"phy status after IDENTIFY: 0x{status_after:08x}")
    bus.close()
    return 0

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA IDENTIFY test utility")
    parser.add_argument("--port",             default="1234", help="Host bind port.")
    parser.add_argument("--link-timeout",     default=20.0, type=float,
        help="Seconds to wait for PHY link-up (default: 20).")
    parser.add_argument("--identify-timeout", default=1.0, type=float,
        help="Seconds to wait for IDENTIFY to complete (default: 1).")
    parser.add_argument("--reset",            action="store_true",
        help="Reset the PHY before waiting for the link.")
    args = parser.parse_args()

    sys.exit(identify_test(
        port             = args.port,
        link_timeout     = args.link_timeout,
        identify_timeout = args.identify_timeout,
        reset            = args.reset,
    ))

if __name__ == "__main__":
    main()
