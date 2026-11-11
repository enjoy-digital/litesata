#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Wishbone DMAs: Sector2Mem (read a sector, write it to memory) and Mem2Sector (read memory, write a
# sector), each through a one-sector buffer and a width/byte-order converter.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 260)
    d.add(Container(120, 10, 520, 110, title="LiteSATASector2MemDMA", title_pos="tl"))
    d.add(Container(120, 140, 520, 110, title="LiteSATAMem2SectorDMA", title_pos="tl"))
    port = d.add(Box(10, 20, 90, 220, label="User\nports", fill=COLOR_ACCENT_LIGHT))
    bus  = d.add(Box(660, 20, 90, 220, label="Wishbone\nbus", fill=COLOR_FILL))
    w, xs = 130, [140, 310, 480]
    y1, y2 = 58, 188
    r1 = [d.add(Box(x, y1, w, 46, label=l)) for x, l in zip(xs, [
        "Sector buffer\n512 bytes", "Width, byte\norder", "DMA writer"])]
    r2 = [d.add(Box(x, y2, w, 46, label=l)) for x, l in zip(xs, [
        "Width, byte\norder", "Sector buffer\n512 bytes", "DMA reader"])]
    d.add(Arrow((port.x + port.w, r1[0].cy), (r1[0], "l"), kind="data"))
    for a, b in zip(r1, r1[1:]):
        d.add(Arrow((a, "r"), (b, "l"), kind="data"))
    d.add(Arrow((r1[2], "r"), (bus.x, r1[2].cy), kind="data"))
    d.add(Arrow((bus.x, r2[2].cy), (r2[2], "r"), kind="data"))
    for a, b in zip(r2[::-1], r2[::-1][1:]):
        d.add(Arrow((a, "l"), (b, "r"), kind="data"))
    d.add(Arrow((r2[0], "l"), (port.x + port.w, r2[0].cy), kind="data"))
    return d
