#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import sys
import time
import argparse

from litex import RemoteClient

# Init Test ----------------------------------------------------------------------------------------

def init_test(port, retries):
    bus = RemoteClient(port=port)
    bus.open()

    init_done = False
    for i in range(retries):
        # Reset PHY
        bus.regs.sata_phy_enable.write(0)
        bus.regs.sata_phy_enable.write(1)

        # Wait
        time.sleep(10e-3)

        # Check Status
        if (bus.regs.sata_phy_status.read() & 0x1) != 0:
            init_done = True
            break

        print(".", end="")
        sys.stdout.flush()
    print("")

    print("Success (retries: {:d})".format(i) if init_done else "Failed")

    bus.close()

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteSATA Init test utility")
    parser.add_argument("--port",    default="1234", help="Host bind port")
    parser.add_argument("--retries", default="80",   help="Init retries")
    args = parser.parse_args()

    port    = int(args.port,    0)
    retries = int(args.retries, 0)

    init_test(port, retries)

if __name__ == "__main__":
    main()
