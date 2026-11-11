#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# ECP5 PHY blocks: the controller, the OOB generator and checker in the fabric, the SYNC filler
# and the DCU driven through its raw TX path and loss-of-signal output.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    ctrl   = d.add(Box(15, 95, 110, 100, label="ECP5\ncontroller", fill=COLOR_ACCENT_LIGHT))
    d.add(Container(160, 15, 410, 270, title="ECP5LiteSATAPHY", title_pos="br"))
    gen    = d.add(Box(175, 40, 140, 60, label="OOB generator\ntx domain", fill=COLOR_ACCENT))
    fill   = d.add(Box(175, 120, 140, 50, label="SYNC filler"))
    chk    = d.add(Box(175, 190, 140, 60, label="OOB checker\nsys domain", fill=COLOR_ACCENT))
    serdes = d.add(Box(395, 40, 150, 210, label="SerDesECP5\nraw TX\n8b/10b\nword aligner"))
    dcu    = d.add(Box(615, 40, 130, 210, label="DCU", fill=COLOR_HARDWARE))
    d.add(Arrow((ctrl, "r", 0.2), (gen, "l"), via=[(142, ctrl.y + 0.2*ctrl.h), (142, gen.cy)],
        kind="control"))
    d.add(Arrow((gen, "r"), (serdes, "l", (gen.cy - serdes.y)/serdes.h), kind="accent",
        label="raw words"))
    d.add(Arrow((ctrl, "r", 0.5), (fill, "l", (ctrl.cy - fill.y)/fill.h), kind="data"))
    d.add(Arrow((fill, "r"), (serdes, "l", (fill.cy - serdes.y)/serdes.h), kind="data",
        label="SYNC, data"))
    d.add(Arrow((serdes, "l", (chk.cy - serdes.y)/serdes.h), (chk, "r"), kind="control",
        label="RLOS"))
    d.add(Arrow((chk, "l"), (ctrl, "r", 0.8), via=[(142, chk.cy), (142, ctrl.y + 0.8*ctrl.h)],
        kind="control"))
    d.add(Arrow((serdes, "r", 0.3), (dcu, "l", 0.3), kind="data"))
    d.add(Arrow((dcu, "l", 0.7), (serdes, "r", 0.7), kind="data"))
    return d
