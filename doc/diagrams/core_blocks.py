#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Core architecture: frontends on user ports, crossbar, command, transport and link layers, PHY,
# drive; what each layer exchanges with the next one.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 486)
    x, w, h, step = 270, 230, 46, 70
    d.add(Container(40, 10 + 2*step - 12, x + w + 20 - 40, 2*step + h + 24, title="LiteSATACore", title_pos="tl"))
    names = [
        ("Frontends, user logic", COLOR_ACCENT_LIGHT),
        ("Crossbar", COLOR_ACCENT_LIGHT),
        ("Command layer\nATA commands", COLOR_ACCENT),
        ("Transport layer\nFISes", COLOR_ACCENT),
        ("Link layer\nframes, primitives", COLOR_ACCENT),
        ("PHY\nOOB, alignment", COLOR_FILL),
        ("Drive", COLOR_HARDWARE),
    ]
    labels = [
        "user ports: commands, sectors",
        "command stream, 32-bit",
        "FIS fields and payload",
        "frames: dwords, error flag",
        "dwords with K flags",
        "serial link, 8b/10b",
    ]
    boxes = []
    for i, (n, fill) in enumerate(names):
        boxes.append(d.add(Box(x, 10 + i*step, w, h, label=n, fill=fill)))
    for a, b, l in zip(boxes, boxes[1:], labels):
        d.add(Arrow((a.x + 80, a.y + h), (b.x + 80, b.y), kind="data"))
        d.add(Arrow((b.x + w - 80, b.y), (a.x + w - 80, a.y + h), kind="data"))
        d.add(Text(x + w + 36, (a.y + h + b.y)/2, l, size=13, anchor="start"))
    d.add(Text(x + 80 - 10, 10 + h + 12, "TX", size=12, anchor="end"))
    d.add(Text(x + w - 80 + 10, 10 + h + 12, "RX", size=12, anchor="start"))
    return d
