#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# ECP5 OOB receive: the gap measured on the loss-of-signal output is classified in the COMWAKE
# window (55-175 ns) or the COMINIT window (175-525 ns); nominal gaps 106.7 and 320 ns. A silence
# of max(32, init_max + 1) sys cycles (48 cycles, 533 ns at 90 MHz) clears the detections.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 200)
    x0, scale = 60, 1.2  # px per ns
    X = lambda ns: x0 + ns*scale
    y_axis = 140
    d.add(Arrow((x0, y_axis), (X(560), y_axis), kind="control"))
    for ns in (0, 55, 175, 525):
        d.add(Bus([(X(ns), y_axis - 4), (X(ns), y_axis + 4)]))
        d.add(Text(X(ns), y_axis + 18, f"{ns}", size=FONT_SIZE_SM))
    d.add(Text(X(560) + 5, y_axis + 18, "ns", size=FONT_SIZE_SM, anchor="end"))
    d.add(Text(25, 85, "gap", anchor="middle", size=13))
    wake = d.add(Box(X(55), 60, (175 - 55)*scale, 50, label="COMWAKE", radius=0, fill=COLOR_ACCENT))
    init = d.add(Box(X(175), 60, (525 - 175)*scale, 50, label="COMINIT", radius=0, fill=COLOR_ACCENT_LIGHT))
    d.add(Bus([(X(533), 60), (X(533), y_axis)], dash="4,3"))
    d.add(Text(X(533), y_axis + 38, "quiet interval: 48 cycles, 533 ns at 90 MHz", size=FONT_SIZE_SM, anchor="end"))
    d.add(Box(X(0), 60, 55*scale, 50, label="", radius=0, fill=COLOR_SHADE))
    d.add(Box(X(525), 60, 35*scale, 50, label="", radius=0, fill=COLOR_SHADE))
    for ns, t in ((106.7, "106.7"), (320, "320")):
        d.add(Bus([(X(ns), 40), (X(ns), 60)], color=COLOR_CYAN))
        d.add(Text(X(ns), 30, f"nominal {t}", size=FONT_SIZE_SM))
    return d
