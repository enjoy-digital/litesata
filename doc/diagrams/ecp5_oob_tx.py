#
# This file is part of LiteSATA.
#
# Copyright (c) 2020-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# ECP5 OOB transmit: bursts are a square wave of raw words, gaps are a constant word (data without
# transitions), electrical idle only before and after the sequence.

from svgdiag import *


def _square(x0, x1, hi, lo, half=5):
    pts = [(x0, (hi + lo)/2), (x0, hi)]
    x, level = x0, hi
    while x + half < x1:
        x += half
        pts.append((x, level))
        level = lo if level == hi else hi
        pts.append((x, level))
    pts.append((x1, level))
    pts.append((x1, (hi + lo)/2))
    return pts


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    hi, lo, mid = 75, 115, 95
    bursts = [(110, 190), (290, 370), (470, 550)]
    gaps   = [(190, 290), (370, 470), (550, 640)]
    d.add(Bus([(20, mid), (110, mid)], kind="data", dash="6,4"))
    for x0, x1 in bursts:
        d.add(Bus(_square(x0, x1, hi, lo), kind="data", color=COLOR_CYAN))
    for x0, x1 in gaps:
        d.add(Bus([(x0, lo), (x1, lo)], kind="data"))
    d.add(Bus([(640, lo), (640, mid), (740, mid)], kind="data", dash="6,4"))
    d.add(Text(65, 55, "electrical\nidle", size=FONT_SIZE_SM))
    d.add(Text(690, 55, "electrical\nidle", size=FONT_SIZE_SM))
    d.add(Text(150, 55, "burst", size=FONT_SIZE_SM))
    d.add(Text(240, 55, "gap", size=FONT_SIZE_SM))
    d.add(Text(420, 55, "6 bursts, 6 gaps", size=FONT_SIZE_SM))
    for x in (110, 190, 290):
        d.add(Bus([(x, 125), (x, 215)], dash="2,3"))
    d.add(Arrow((110, 150), (190, 150), heads="both", label="160 UI", label_side="below"))
    d.add(Arrow((190, 150), (290, 150), heads="both", label="160 or 480 UI", label_side="below"))
    d.add(Text(150, 200, "square wave", size=FONT_SIZE_SM))
    d.add(Text(240, 200, "constant word", size=FONT_SIZE_SM))
    return d
