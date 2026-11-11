#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Striping over two drives: one crossbar on top of LiteSATAStriping, one core and one PHY per drive.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 260)
    xbar = d.add(Box(15, 40, 110, 180, label="Crossbar", fill=COLOR_ACCENT_LIGHT))
    strp = d.add(Box(170, 40, 110, 180, label="Striping", fill=COLOR_ACCENT))
    rows = []
    for i, y in enumerate((50, 166)):
        core  = d.add(Box(330, y, 120, 44, label=f"Core {i}", fill=COLOR_ACCENT_LIGHT))
        phy   = d.add(Box(490, y, 100, 44, label=f"PHY {i}"))
        drive = d.add(Box(630, y, 115, 44, label=f"Drive {i}", fill=COLOR_HARDWARE))
        d.add(Arrow((280, y + 22), (core, "l"), kind="data", heads="both"))
        d.add(Arrow((core, "r"), (phy, "l"), kind="data", heads="both"))
        d.add(Arrow((phy, "r"), (drive, "l"), kind="data", heads="both"))
    d.add(Arrow((xbar, "r"), (strp, "l"), kind="accent", heads="both", label="64 bits",
        label_side="above"))
    return d
