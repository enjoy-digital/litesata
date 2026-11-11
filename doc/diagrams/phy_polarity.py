#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# RX polarity detection: the first ALIGN of the drive decodes as ALIGN or as ALIGN_N, which sets
# the RX polarity before SEND-ALIGN.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 230)
    drive = d.add(Box(15, 75, 95, 70, label="Drive\nALIGN", fill=COLOR_HARDWARE))
    trx   = d.add(Box(145, 75, 120, 70, label="Transceiver\nRX, 8b/10b", fill=COLOR_HARDWARE))
    cmp_  = d.add(Box(300, 75, 120, 70, label="AWAIT-ALIGN\ncompare", fill=COLOR_ACCENT))
    pol0  = d.add(Box(470, 20, 135, 60, label="RX polarity 0", fill=COLOR_ACCENT_LIGHT))
    pol1  = d.add(Box(470, 140, 135, 60, label="RX polarity 1", fill=COLOR_ACCENT_LIGHT))
    send  = d.add(Box(645, 75, 100, 70, label="SEND-ALIGN"))
    d.add(Arrow((drive, "r"), (trx, "l"), kind="accent"))
    d.add(Arrow((trx, "r"), (cmp_, "l"), kind="data"))
    d.add(Arrow((cmp_, "t"), (pol0, "l"), kind="control", label="0x7b4a4abc",
        label_pos=(400, 42)))
    d.add(Arrow((cmp_, "b"), (pol1, "l"), kind="control", label="0x7bb5b5bc",
        label_pos=(400, 184)))
    d.add(Arrow((pol0, "r"), (send, "t"), kind="control"))
    d.add(Arrow((pol1, "r"), (send, "b"), kind="control"))
    return d
