#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# RX byte realignment at 16 bits: a two-word window, a one-byte shift that puts the K character on
# byte 0, then the pairing of two 16-bit words into a dword with the K character on byte 0.

from svgdiag import *


def _strip(d, x, y, cells, w=95, h=38):
    boxes = []
    for label, fill in cells:
        boxes.append(d.add(Box(x, y, w, h, label=label, radius=0, size=13, fill=fill)))
        x += w
    return boxes


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    x0, w = 250, 95
    # Byte positions of the two-word window (byte 0 on the left).
    for i, t in enumerate(["byte 0", "byte 1", "byte 0", "byte 1"]):
        d.add(Text(x0 + w*i + w/2, 30, t, size=FONT_SIZE_SM))
    d.add(Text(25, 65, "Two received\nwords", anchor="start", size=13))
    win = _strip(d, x0, 46, [("D", COLOR_FILL), ("K28.5", COLOR_ACCENT),
                             ("D10.2", COLOR_FILL), ("D10.2", COLOR_FILL)])
    d.add(Text(x0 + w, 100, "previous word", size=FONT_SIZE_SM))
    d.add(Text(x0 + 3*w, 100, "new word", size=FONT_SIZE_SM))
    d.add(Text(25, 165, "Shifted\n16-bit word", anchor="start", size=13))
    sel = _strip(d, x0 + w, 146, [("K28.5", COLOR_ACCENT), ("D10.2", COLOR_FILL)])
    d.add(Arrow((win[1], "b"), (sel[0], "t"), kind="data", label="shift one byte",
        label_pos=(x0 + w - 15, 128)))
    d.add(Arrow((win[2], "b"), (sel[1], "t"), kind="data"))
    d.add(Text(25, 265, "Dword\nto sys", anchor="start", size=13))
    dw = _strip(d, x0, 246, [("K28.5", COLOR_ACCENT), ("D10.2", COLOR_FILL),
                             ("D10.2", COLOR_FILL), ("D27.3", COLOR_FILL)])
    d.add(Arrow((sel[0], "b"), (dw[0], "t"), via=[(sel[0].cx, 215), (dw[0].cx, 215)], kind="data",
        label="pair two words", label_pos=(345, 203)))
    for i, t in enumerate(["byte 0", "byte 1", "byte 2", "byte 3"]):
        d.add(Text(x0 + w*i + w/2, 300, t, size=FONT_SIZE_SM))
    return d
