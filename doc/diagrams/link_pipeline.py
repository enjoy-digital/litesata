#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Link layer pipelines: TX (CRC and scrambler, TX state machine, ALIGN inserter) and RX (ALIGN and
# CONT removers, RX state machine, descrambler and CRC check, receive buffer); the RX state machine
# hands its primitives to the TX side, the buffer level asks for HOLD.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 260)
    d.add(Container(110, 10, 540, 240, title="LiteSATALink", title_pos="tl"))
    tr  = d.add(Box(10, 40, 80, 180, label="Transport\nlayer", fill=COLOR_ACCENT_LIGHT))
    phy = d.add(Box(670, 40, 80, 180, label="PHY", fill=COLOR_HARDWARE))
    w = 105
    # TX row.
    y = 50
    crc = d.add(Box(240, y, w, 50, label="CRC insert,\nscrambler"))
    tx  = d.add(Box(390, y, w, 50, label="TX state\nmachine", fill=COLOR_ACCENT))
    ali = d.add(Box(530, y, w, 50, label="ALIGN\ninserter"))
    # RX row.
    y = 170
    buf = d.add(Box(125, y, w, 50, label="RX buffer\n256 dwords"))
    dsc = d.add(Box(255, y, w, 50, label="descrambler,\nCRC check"))
    rx  = d.add(Box(390, y, w, 50, label="RX state\nmachine", fill=COLOR_ACCENT))
    rem = d.add(Box(530, y, w, 50, label="ALIGN, CONT\nremovers"))
    d.add(Arrow((tr.x + tr.w, crc.cy), (crc, "l"), kind="data"))
    d.add(Arrow((crc, "r"), (tx, "l"), kind="data"))
    d.add(Arrow((tx, "r"), (ali, "l"), kind="data"))
    d.add(Arrow((ali, "r"), (phy.x, ali.cy), kind="data"))
    d.add(Arrow((phy.x, rem.cy), (rem, "r"), kind="data"))
    d.add(Arrow((rem, "l"), (rx, "r"), kind="data"))
    d.add(Arrow((rx, "l"), (dsc, "r"), kind="data"))
    d.add(Arrow((dsc, "l"), (buf, "r"), kind="data"))
    d.add(Arrow((buf, "l"), (tr.x + tr.w, buf.cy), kind="data"))
    hy = 140
    d.add(Arrow((buf.cx, buf.y), (rx.x + 25, rx.y), via=[(buf.cx, hy), (rx.x + 25, hy)], kind="control",
        label="level > 128", label_pos=(300, hy - 8)))
    d.add(Arrow((rx.x + 80, rx.y), (tx.x + 80, tx.y + tx.h), kind="control", label="primitives",
        label_pos=(rx.x + 88, hy), label_anchor="start"))
    return d
