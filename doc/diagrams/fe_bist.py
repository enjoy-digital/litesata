#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# BIST: register units (loops, cycle counter) drive a generator and a checker, each with its own
# pattern (counter or scrambler); an Identify on a third user port.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 290)
    d.add(Container(10, 10, 410, 270, title="LiteSATABIST", title_pos="bl"))
    ug  = d.add(Box(30, 30, 140, 70, label="Unit registers\nloops, cycles"))
    uc  = d.add(Box(30, 126, 140, 70, label="Unit registers\nloops, cycles"))
    gen = d.add(Box(250, 30, 150, 70, label="Generator\nWRITE DMA EXT\nown pattern", fill=COLOR_ACCENT))
    chk = d.add(Box(250, 126, 150, 70, label="Checker\nREAD DMA EXT\nown pattern", fill=COLOR_ACCENT))
    idf = d.add(Box(250, 216, 150, 40, label="Identify"))
    xb  = d.add(Box(470, 30, 110, 226, label="Crossbar"))
    core = d.add(Box(630, 30, 110, 226, label="Core"))
    d.add(Arrow((ug, "r"), (gen, "l"), kind="control", label="start", label_side="above"))
    d.add(Arrow((uc, "r", 0.3), (chk, "l", 0.3), kind="control", label="start", label_side="above"))
    d.add(Arrow((chk, "l", 0.75), (uc, "r", 0.75), kind="control", label="errors", label_side="below"))
    d.add(Arrow((gen, "r"), (xb.x, gen.cy), kind="data", heads="both"))
    d.add(Arrow((chk, "r"), (xb.x, chk.cy), kind="data", heads="both"))
    d.add(Arrow((idf, "r"), (xb.x, idf.cy), kind="data", heads="both"))
    d.add(Arrow((xb.x + xb.w, xb.cy), (core.x, xb.cy), kind="data", heads="both"))
    return d
