#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Shared helpers of the core and frontend figures (no build(): not a figure).
#
# Exchange (sequence) diagrams: one vertical lane per side (a titled box on top and a dashed line
# below), horizontal arrows between the lanes for the FISes, labels above the arrows, and notes on
# the left (host state machine states) or on the right.

from svgdiag import *


class Exchange:
    """Two-lane exchange diagram: left and right lanes at x_left/x_right."""
    def __init__(self, d, left, right, x_left=290, x_right=640, top=20, bottom=None, fill_left=COLOR_ACCENT,
        fill_right=COLOR_HARDWARE):
        self.d, self.xl, self.xr = d, x_left, x_right
        bottom = bottom or d.height - 10
        for x, name, fill in ((x_left, left, fill_left), (x_right, right, fill_right)):
            d.add(Box(x - 60, top, 120, 36, label=name, fill=fill))
            d.add(Bus([(x, top + 36), (x, bottom)], dash="4,3"))
        self.y = top + 36 + 30

    def msg(self, label, to="right", kind="control", step=40, note=None):
        """Arrow between the lanes at the current row, label above; note on the left; returns y."""
        y = self.y
        a, b = ((self.xl, y), (self.xr, y)) if to == "right" else ((self.xr, y), (self.xl, y))
        self.d.add(Arrow(a, b, kind=kind, label=label, label_side="above", label_size=14))
        if note:
            self.d.add(Text(self.xl - 12, y - 8, note, size=13, anchor="end", color="#444444"))
        self.y += step
        return y


def cells(d, x, y, items, h=34, size=13):
    """Row of adjoining square cells: items is a list of (width, label, fill)."""
    out = []
    for w, label, fill in items:
        out.append(d.add(Box(x, y, w, h, label=label, radius=0, fill=fill, size=size)))
        x += w
    return out
