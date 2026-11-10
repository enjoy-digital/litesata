#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# How the command layer fills the FISes from a user port beat: flags to the ATA command code,
# sector to the LBA, count to the sector count, constants for the other fields, data to Data FISes.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 335)
    d.add(Container(10, 10, 250, 310, title="User port sink", title_pos="tl"))
    d.add(Container(400, 10, 350, 240, title="Register H2D FIS", title_pos="tr"))
    rows = [55, 105, 155, 205]
    lw, rw, h = 210, 310, 36
    left = [d.add(Box(30, y, lw, h, label=l, size=14)) for y, l in zip(rows, [
        "write, read or identify", "sector (48 bits)", "count (16 bits)"])]
    right = [d.add(Box(420, y, rw, h, label=l, size=14, fill=COLOR_ACCENT)) for y, l in zip(rows, [
        "command: 0x35, 0x25 or 0xEC", "LBA (48 bits)", "count (16 bits)"])]
    const = d.add(Box(420, rows[3], rw, h, label="device 0xE0, C bit 1, others 0", size=14, fill=COLOR_SHADE))
    data  = d.add(Box(30, 275, lw, h, label="data", size=14))
    dfis  = d.add(Box(420, 275, rw, h, label="Data FIS payload (writes)", size=14, fill=COLOR_ACCENT))
    for a, b in zip(left, right):
        d.add(Arrow((a, "r"), (b, "l"), kind="data"))
    d.add(Arrow((data, "r"), (dfis, "l"), kind="data"))
    return d
