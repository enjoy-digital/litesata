#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Register H2D FIS: five dwords, bit 31 on the left.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 215)
    x0, b = 100, 20
    A, L, S = COLOR_ACCENT, COLOR_ACCENT_LIGHT, COLOR_SHADE
    d.add(Text(x0, 12, "bit 31", size=12, anchor="start"))
    d.add(Text(x0 + 32*b, 12, "bit 0", size=12, anchor="end"))
    rows = [
        ("Dword 0", [(8*b, "Features 7:0", L), (8*b, "Command", A), (b, "C", L), (3*b, "", S),
                     (4*b, "PM port", L), (8*b, "Type 0x27", A)]),
        ("Dword 1", [(8*b, "Device", L), (24*b, "LBA 23:0", A)]),
        ("Dword 2", [(8*b, "Features 15:8", L), (24*b, "LBA 47:24", A)]),
        ("Dword 3", [(8*b, "Control", L), (8*b, "ICC", L), (16*b, "Count 15:0", A)]),
        ("Dword 4", [(32*b, "Reserved", S)]),
    ]
    y = 25
    for name, items in rows:
        d.add(Text(x0 - 12, y + 15, name, size=13, anchor="end"))
        cells(d, x0, y, items)
        y += 36
    return d
