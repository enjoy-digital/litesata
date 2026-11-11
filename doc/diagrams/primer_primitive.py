#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# A primitive: four 8b/10b characters, byte 0 first on the wire, a control character in byte 0.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 200)
    x0, w = 160, 140
    for i in range(4):
        d.add(Text(x0 + i*w + w/2, 18, f"byte {i}", size=13))
        d.add(Text(x0 + i*w + w/2, 36, ["bits 7:0", "bits 15:8", "bits 23:16", "bits 31:24"][i],
            size=12, color="#444444"))
    rows = [("ALIGN", ["K28.5", "D10.2", "D10.2", "D27.3"]),
            ("SYNC",  ["K28.3", "D21.4", "D21.5", "D21.5"])]
    y = 55
    for name, chars in rows:
        d.add(Text(x0 - 12, y + 15, name, size=14, anchor="end"))
        cells(d, x0, y, [(w, c, COLOR_ACCENT if i == 0 else COLOR_ACCENT_LIGHT)
            for i, c in enumerate(chars)])
        y += 45
    d.add(Arrow((x0, 165), (x0 + 4*w, 165), kind="control", label="order on the wire",
        label_side="below"))
    return d
