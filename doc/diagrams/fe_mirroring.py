#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Mirroring over two drives: one port per drive; reads go to the port's own core, writes from any
# port go to every core.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    p0 = d.add(Box(20, 30, 110, 50, label="Port 0", fill=COLOR_ACCENT_LIGHT))
    p1 = d.add(Box(20, 150, 110, 50, label="Port 1", fill=COLOR_ACCENT_LIGHT))
    mi = d.add(Box(200, 20, 160, 190, label="Mirroring\nreads: own core\nwrites: all cores",
        fill=COLOR_ACCENT))
    c0 = d.add(Box(450, 30, 120, 50, label="Core 0, PHY 0"))
    c1 = d.add(Box(450, 150, 120, 50, label="Core 1, PHY 1"))
    d0 = d.add(Box(630, 30, 110, 50, label="Drive 0", fill=COLOR_HARDWARE))
    d1 = d.add(Box(630, 150, 110, 50, label="Drive 1", fill=COLOR_HARDWARE))
    d.add(Arrow((p0, "r"), (mi.x, p0.cy), kind="data", heads="both"))
    d.add(Arrow((p1, "r"), (mi.x, p1.cy), kind="data", heads="both"))
    d.add(Arrow((mi.x + mi.w, c0.cy), (c0, "l"), kind="data", heads="both"))
    d.add(Arrow((mi.x + mi.w, c1.cy), (c1, "l"), kind="data", heads="both"))
    d.add(Arrow((c0, "r"), (d0, "l"), kind="data", heads="both"))
    d.add(Arrow((c1, "r"), (d1, "l"), kind="data", heads="both"))
    return d
