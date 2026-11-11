#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# User port beats: a one-sector write (command fields on the data beats, one response beat) and a
# one-sector read (one command beat, data beats, one response beat).

from core_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    x0, resp = 110, 590
    lane = lambda y, name: d.add(Text(x0 - 14, y + 17, name, size=13, anchor="end"))
    # Write.
    d.add(Text(20, 22, "WRITE DMA EXT, one sector", size=14, anchor="start", bold=True))
    lane(40, "sink")
    cells(d, x0, 40, [(150, "write, sector,\ncount, D0", COLOR_ACCENT), (60, "D1", COLOR_ACCENT),
        (60, "...", COLOR_ACCENT), (80, "D127, last", COLOR_ACCENT)], h=40)
    lane(90, "source")
    cells(d, resp, 90, [(140, "end, last,\nfailed", COLOR_ACCENT_LIGHT)], h=40)
    # Read.
    d.add(Text(20, 162, "READ DMA EXT, one sector", size=14, anchor="start", bold=True))
    lane(180, "sink")
    cells(d, x0, 180, [(150, "read, sector,\ncount, last", COLOR_ACCENT)], h=40)
    lane(230, "source")
    cells(d, 280, 230, [(60, "D0", COLOR_ACCENT_LIGHT), (60, "D1", COLOR_ACCENT_LIGHT),
        (60, "...", COLOR_ACCENT_LIGHT), (80, "D127, last", COLOR_ACCENT_LIGHT)], h=40)
    cells(d, resp, 230, [(140, "end, last,\nfailed", COLOR_ACCENT_LIGHT)], h=40)
    d.add(Arrow((x0, 300), (730, 300), kind="control", label="time", label_side="below"))
    return d
