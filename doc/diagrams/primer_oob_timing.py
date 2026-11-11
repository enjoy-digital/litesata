#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# OOB signals: six bursts of 160 Gen1 UI separated by gaps of 480 UI (COMRESET, COMINIT) or
# 160 UI (COMWAKE). Bursts are drawn as dense square waves, gaps as a dashed idle line.

from svgdiag import *


def burst(d, x, w, y):
    hi, lo = y - 14, y + 14
    pts = [(x, y)]
    n = 6
    for k in range(n):
        xa = x + k*w/n
        pts += [(xa, hi), (xa + w/(2*n), hi), (xa + w/(2*n), lo), (xa + w/n, lo)]
    pts += [(x + w, y)]
    d.add(Bus(pts, color=COLOR_CYAN))


def signal(d, x0, y, burst_w, gap_w, name):
    d.add(Text(x0 - 12, y, name, size=14, anchor="end"))
    x = x0
    for i in range(6):
        burst(d, x, burst_w, y)
        x += burst_w
        if i < 5:
            d.add(Bus([(x, y), (x + gap_w, y)], dash="4,3"))
            x += gap_w
    return x


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    x0, b = 120, 30
    signal(d, x0, 45, b, 3*b, "COMRESET\nCOMINIT")
    d.add(Arrow((x0, 82), (x0 + b, 82), kind="control", heads="both", label="160 UI",
        label_pos=(x0 - 8, 82), label_anchor="end"))
    d.add(Arrow((x0 + b, 82), (x0 + 4*b, 82), kind="control", heads="both", label="480 UI",
        label_side="below"))
    signal(d, x0, 155, b, b, "COMWAKE")
    d.add(Arrow((x0, 192), (x0 + b, 192), kind="control", heads="both", label="160 UI",
        label_pos=(x0 - 8, 192), label_anchor="end"))
    d.add(Arrow((x0 + b, 192), (x0 + 2*b, 192), kind="control", heads="both", label="160 UI",
        label_pos=(x0 + 2*b + 8, 192), label_anchor="start"))
    return d
