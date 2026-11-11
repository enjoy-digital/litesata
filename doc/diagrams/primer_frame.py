#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A frame: SOF, FIS dwords, CRC, EOF, then WTRM; an ALIGN pair and a HOLD pause inside the frame.
# Scrambled dwords are tinted, primitives are grey.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 170)
    A, P = COLOR_ACCENT, COLOR_SHADE
    w = 56
    items = [("SOF", P), ("FIS 0", A), ("FIS 1", A), ("ALIGN", P), ("ALIGN", P), ("FIS 2", A),
             ("HOLD", P), ("HOLD", P), ("FIS n", A), ("CRC", A), ("EOF", P), ("WTRM", P)]
    x0 = (CANVAS_WIDTH - len(items)*w)/2
    cells(d, x0, 55, [(w, label, fill) for label, fill in items])
    x = lambda i: x0 + i*w
    d.add(Arrow((x(0), 35), (x(11), 35), kind="control", heads="both", label="frame",
        label_side="above"))
    d.add(Arrow((x(3), 105), (x(5), 105), kind="control", heads="both", label="clock",
        label_side="below"))
    d.add(Arrow((x(6), 105), (x(8), 105), kind="control", heads="both", label="pause",
        label_side="below"))
    d.add(Arrow((x(1), 145), (x(10), 145), kind="accent", heads="both", label="scrambled",
        label_side="below"))
    return d
