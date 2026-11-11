#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# One core, many PHYs: the same core and frontends on every transceiver family.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 190)
    core = d.add(Box(255, 15, 250, 50, label="Core and frontends", fill=COLOR_ACCENT))
    names = ["Artix-7\nGTP", "Kintex-7\nGTX", "UltraScale\nGTH", "UltraScale+\nGTH, GTY", "ECP5\nDCU"]
    w, gap = 128, 25
    x0 = (CANVAS_WIDTH - 5*w - 4*gap)/2
    ys = 125
    d.add(Bus([(x0 + w/2, 95), (x0 + 4*(w + gap) + w/2, 95)]))
    d.add(Bus([(core.cx, core.y + core.h), (core.cx, 95)]))
    for i, name in enumerate(names):
        x = x0 + i*(w + gap)
        b = d.add(Box(x, ys, w, 50, label=name, size=14, fill=COLOR_ACCENT_LIGHT))
        d.add(Arrow((b.cx, 95), (b, "t"), kind="control"))
    return d
