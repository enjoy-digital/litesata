#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Link RX state machine (LiteSATALinkRX): each state with the primitive it asks the TX side to send.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 280)
    h, y1, y2, y3 = 46, 20, 140, 220
    st = lambda x, y, w, n, p, fill=COLOR_FILL: d.add(Box(x, y, w, h, label=f"{n}\n{p}", fill=fill))
    idle  = st(20,  y1, 100, "IDLE", "(TX side)")
    rdy   = st(190, y1, 100, "RDY", "R_RDY")
    first = st(360, y1, 120, "WAIT_FIRST", "R_IP")
    copy  = st(550, y1, 180, "COPY", "R_IP, HOLD, HOLDA", COLOR_ACCENT)
    wtrm  = st(580, y2, 120, "WTRM", "R_IP")
    rok   = st(360, y2, 120, "R_OK", "R_OK")
    rerr  = st(360, y3, 120, "R_ERR", "R_ERR")
    d.add(Arrow((idle, "r"), (rdy, "l"), kind="data", label="X_RDY", label_size=12))
    d.add(Arrow((rdy, "r"), (first, "l"), kind="data", label="SOF", label_size=12))
    d.add(Arrow((first, "r"), (copy, "l"), kind="data", label="data", label_size=12))
    d.add(Arrow((wtrm.cx, copy.y + h), (wtrm, "t"), kind="data", label="EOF", label_side="right"))
    d.add(Arrow((wtrm, "l"), (rok, "r"), kind="control", label="CRC ok", label_side="above"))
    d.add(Arrow((wtrm, "b"), (rerr, "r"), kind="control", label="CRC error", label_pos=(570, rerr.cy - 9)))
    d.add(Arrow((rok, "l"), (idle.x + 30, idle.y + h), kind="control", label="SYNC", label_pos=(230, rok.cy - 9)))
    d.add(Arrow((rerr, "l"), (idle.x + 70, idle.y + h), kind="control", label="SYNC", label_pos=(230, rerr.cy - 9)))
    return d
