#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# PHY blocks: the core, the TX and RX datapaths, the controller, the transceiver wrapper and the
# drive.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    core  = d.add(Box(15, 40, 100, 220, label="Link layer\n(core)", fill=COLOR_ACCENT_LIGHT))
    d.add(Container(140, 15, 470, 270, title="LiteSATAPHY", title_pos="br"))
    tx    = d.add(Box(165, 45, 150, 60, label="Datapath TX\nsys to sata_tx", fill=COLOR_ACCENT))
    rx    = d.add(Box(165, 190, 150, 60, label="Datapath RX\nsata_rx to sys", fill=COLOR_ACCENT))
    ctrl  = d.add(Box(345, 118, 120, 60, label="Controller\nOOB, ALIGN", fill=COLOR_ACCENT))
    trx   = d.add(Box(495, 45, 100, 205, label="Transceiver\nand CRG", fill=COLOR_HARDWARE))
    drive = d.add(Box(640, 40, 105, 220, label="Drive", fill=COLOR_HARDWARE))
    ytx, yrx = 65, 230
    d.add(Arrow((core, "r", (ytx - core.y)/core.h), (tx, "l", (ytx - tx.y)/tx.h), kind="data"))
    d.add(Arrow((tx, "r", (ytx - tx.y)/tx.h), (trx, "l", (ytx - trx.y)/trx.h), kind="data",
        label="16 or 32 bits", label_side="above"))
    d.add(Arrow((trx, "r", (ytx - trx.y)/trx.h), (drive, "l", (ytx - drive.y)/drive.h), kind="accent"))
    d.add(Arrow((drive, "l", (yrx - drive.y)/drive.h), (trx, "r", (yrx - trx.y)/trx.h), kind="accent"))
    d.add(Arrow((trx, "l", (yrx - trx.y)/trx.h), (rx, "r", (yrx - rx.y)/rx.h), kind="data"))
    d.add(Arrow((rx, "l", (yrx - rx.y)/rx.h), (core, "r", (yrx - core.y)/core.h), kind="data"))
    # Controller: drives the TX mux before ready, reads the RX stream and its idle/misalign flags.
    d.add(Arrow((ctrl, "t"), (tx, "r", 40/60), via=[(ctrl.cx, 85)], kind="control"))
    d.add(Arrow((rx, "r", 20/60), (ctrl, "b"), via=[(ctrl.cx, 210)], kind="control"))
    d.add(Arrow((ctrl, "r"), (trx, "l", (ctrl.cy - trx.y)/trx.h), kind="control", heads="both",
        label="OOB", label_side="above"))
    return d
