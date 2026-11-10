#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Striping over two drives: one 64-bit port, each 64-bit word split in two dwords, one per core.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    port = d.add(Box(20, 20, 110, 190, label="User port\n64-bit", fill=COLOR_ACCENT_LIGHT))
    st   = d.add(Box(200, 20, 130, 190, label="Striping\nsplit, combine", fill=COLOR_ACCENT))
    c0   = d.add(Box(450, 30, 120, 50, label="Core 0, PHY 0"))
    c1   = d.add(Box(450, 150, 120, 50, label="Core 1, PHY 1"))
    d0   = d.add(Box(630, 30, 110, 50, label="Drive 0", fill=COLOR_HARDWARE))
    d1   = d.add(Box(630, 150, 110, 50, label="Drive 1", fill=COLOR_HARDWARE))
    d.add(Arrow((port.x + port.w, st.cy), (st.x, st.cy), kind="data", heads="both"))
    d.add(Arrow((st.x + st.w, c0.cy), (c0, "l"), kind="data", heads="both", label="data[31:0]"))
    d.add(Arrow((st.x + st.w, c1.cy), (c1, "l"), kind="data", heads="both", label="data[63:32]"))
    d.add(Arrow((c0, "r"), (d0, "l"), kind="data", heads="both"))
    d.add(Arrow((c1, "r"), (d1, "l"), kind="data", heads="both"))
    return d
