#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Link TX state machine (LiteSATALinkTX): each state with the primitive it sends.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 240)
    w, h, y1, y2 = 90, 46, 60, 180
    st = lambda x, y, n, p, fill=COLOR_FILL: d.add(Box(x, y, w, h, label=f"{n}\n{p}", fill=fill))
    idle  = st(20,  y1, "IDLE", "SYNC")
    rdy   = st(180, y1, "RDY", "X_RDY")
    sof   = st(340, y1, "SOF", "SOF")
    copy  = st(500, y1, "COPY", "data, HOLD", COLOR_ACCENT)
    holda = st(660, y1, "HOLDA", "HOLDA")
    eof   = st(500, y2, "EOF", "EOF")
    wtrm  = st(340, y2, "WTRM", "WTRM")
    d.add(Arrow((idle, "r"), (rdy, "l"), kind="data", label="data, SYNC", label_size=12))
    d.add(Arrow((rdy, "r"), (sof, "l"), kind="data", label="R_RDY", label_size=12))
    d.add(Arrow((sof, "r"), (copy, "l"), kind="data"))
    d.add(Arrow((copy, "r", 0.3), (holda, "l", 0.3), kind="control", label="HOLD", label_side="above"))
    d.add(Arrow((holda, "l", 0.7), (copy, "r", 0.7), kind="control", label="no HOLD", label_side="below"))
    d.add(Arrow((copy, "b"), (eof, "t"), kind="data", label="last dword", label_side="right"))
    d.add(Arrow((eof, "l"), (wtrm, "r"), kind="data"))
    d.add(Arrow((wtrm, "l"), (idle.x + 30, idle.y + h), kind="control", label="R_OK, R_ERR, SYNC",
        label_pos=(190, wtrm.cy - 9)))
    d.add(Arrow((rdy.cx, rdy.y), (idle.cx, idle.y), via=[(rdy.cx, 22), (idle.cx, 22)], kind="control",
        label="device X_RDY", label_pos=(110, 13)))
    return d
