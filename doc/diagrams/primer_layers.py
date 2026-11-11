#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# The four layers of SATA on both sides of the link, and the unit each pair of layers exchanges.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 360)
    d.add(Container(20, 10, 230, 340, title="Host (LiteSATA)", title_pos="bl"))
    d.add(Container(510, 10, 230, 340, title="Device (drive)", title_pos="br"))
    rows = [("Command layer",   "ATA commands"),
            ("Transport layer", "FIS"),
            ("Link layer",      "frames, primitives"),
            ("Physical layer",  "characters, OOB")]
    y = 25
    for name, what in rows:
        h = d.add(Box(45,  y, 180, 50, label=name, fill=COLOR_ACCENT))
        v = d.add(Box(535, y, 180, 50, label=name, fill=COLOR_HARDWARE))
        d.add(Arrow((h, "r"), (v, "l"), kind="feedback", heads="both", label=what, label_size=13))
        y += 72
    return d
