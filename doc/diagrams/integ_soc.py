#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# LiteSATA in a LiteX SoC (add_sata): the DMAs and Identify take crossbar ports, the core and the
# PHY reach the drive; the PHY is created by the target and passed to add_sata.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(135, 20, 380, 260, title="add_sata", title_pos="bl"))
    bus   = d.add(Box(15, 50, 95, 174, label="SoC bus,\nCPU, RAM"))
    s2m   = d.add(Box(155, 50, 115, 44, label="Sector2Mem", fill=COLOR_ACCENT))
    m2s   = d.add(Box(155, 115, 115, 44, label="Mem2Sector", fill=COLOR_ACCENT))
    idf   = d.add(Box(155, 180, 115, 44, label="Identify", fill=COLOR_ACCENT_LIGHT))
    xbar  = d.add(Box(300, 50, 90, 174, label="Crossbar", fill=COLOR_ACCENT_LIGHT))
    core  = d.add(Box(415, 50, 80, 174, label="Core", fill=COLOR_ACCENT_LIGHT))
    phy   = d.add(Box(545, 110, 75, 54, label="PHY"))
    drive = d.add(Box(650, 110, 95, 54, label="Drive", fill=COLOR_HARDWARE))
    d.add(Arrow((s2m, "l"), (110, 72), kind="data"))
    d.add(Arrow((110, 137), (m2s, "l"), kind="data"))
    d.add(Arrow((110, 202), (idf, "l"), kind="control", heads="both"))
    for b in (s2m, m2s, idf):
        y = b.y + b.h/2
        d.add(Arrow((270, y), (300, y), kind="data", heads="both"))
    d.add(Arrow((390, 137), (415, 137), kind="data", heads="both"))
    d.add(Arrow((core, "r"), (phy, "l"), kind="data", heads="both"))
    d.add(Arrow((phy, "r"), (drive, "l"), kind="accent", heads="both"))
    return d
