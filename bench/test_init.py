#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import time
import argparse

from litex import RemoteClient

# Init Test ----------------------------------------------------------------------------------------

def init_test(port, retries, interval):
    bus = RemoteClient(port=port)
    bus.open()

    # Reset once and let the PHY controller perform its internal OOB retries.
    bus.regs.sata_phy_enable.write(0)
    time.sleep(1e-3)
    bus.regs.sata_phy_enable.write(1)

    init_done = False
    status    = 0
    for poll in range(1, retries + 1):
        time.sleep(interval)

        status = bus.regs.sata_phy_status.read()
        if status & 0x1:
            init_done = True
            break

        print(".", end="")
        sys.stdout.flush()
    print("")

    if init_done:
        print(f"Success (polls: {poll:d}, status: 0x{status:08x})")
    else:
        print("Failed (status: 0x{:08x}, tx_ready: {:d}, rx_ready: {:d}, ctrl_ready: {:d})".format(
            status,
            (status >> 1) & 0x1,
            (status >> 2) & 0x1,
            (status >> 3) & 0x1,
        ))

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA Init test utility")
    parser.add_argument("--port",     default="1234", help="Host bind port")
    parser.add_argument("--retries",  default="80",   help="Number of status polls")
    parser.add_argument("--interval", default=0.1, type=float,
        help="Status poll interval in seconds (default: 0.1)")
    args = parser.parse_args()

    port    = int(args.port,    0)
    retries = int(args.retries, 0)

    init_test(port, retries, args.interval)

if __name__ == "__main__":
    main()
