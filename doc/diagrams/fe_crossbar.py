#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Crossbar: frontends on user ports, width converters for 64-bit ports, round-robin arbiter in front
# of the core.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 290)
    d.add(Container(240, 10, 330, 270, title="LiteSATACrossbar", title_pos="tl"))
    ys = [30, 90, 150, 210]
    names = ["Identify", "BIST", "DMAs", "User logic"]
    fes = [d.add(Box(20, y, 170, 44, label=n, fill=COLOR_ACCENT_LIGHT)) for y, n in zip(ys, names)]
    arb  = d.add(Box(430, 30, 120, 224, label="Round-robin\narbiter", fill=COLOR_ACCENT))
    conv = d.add(Box(270, ys[3], 120, 44, label="Converter\n64 to 32"))
    core = d.add(Box(620, 30, 120, 224, label="Core\n32-bit"))
    for f in fes[:3]:
        d.add(Arrow((f, "r"), (arb.x, f.cy), kind="data", heads="both", label="32-bit",
            label_pos=(215, f.cy - 10)))
    d.add(Arrow((fes[3], "r"), (conv, "l"), kind="data", heads="both", label="64-bit",
        label_pos=(215, fes[3].cy - 10)))
    d.add(Arrow((conv, "r"), (arb.x, conv.cy), kind="data", heads="both"))
    d.add(Arrow((arb, "r"), (core, "l"), kind="data", heads="both"))
    return d
