#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# PHY controller state machine: OOB states on the left, ALIGN states and READY on the right, the
# 10 ms and 873 us timer resets and the RX idle return to RESET.

from svgdiag import *


def build():
    step, h = 75, 50
    lx, rx, w = 95, 440, 205
    ys = [25 + i*step for i in range(5)]
    d = Diagram(CANVAS_WIDTH, ys[-1] + h + 25)
    left = [
        d.add(Box(lx, ys[0], w, h, label="RESET\nAWAIT-CRG-RESET")),
        d.add(Box(lx, ys[1], w, h, label="COMINIT\n(send COMRESET)")),
        d.add(Box(lx, ys[2], w, h, label="AWAIT-COMINIT\nAWAIT-NO-COMINIT")),
        d.add(Box(lx, ys[3], w, h, label="CALIBRATE\nCOMWAKE")),
        d.add(Box(lx, ys[4], w, h, label="AWAIT-COMWAKE\nAWAIT-NO-COMWAKE")),
    ]
    rst_rx = d.add(Box(rx, ys[1], w, h, label="RESET_RX"))
    ready  = d.add(Box(rx, ys[2], w, h, label="READY\n(send SYNC)", fill=COLOR_ACCENT))
    send   = d.add(Box(rx, ys[3], w, h, label="SEND-ALIGN"))
    wait   = d.add(Box(rx, ys[4], w, h, label="AWAIT-ALIGN\n(send D10.2)"))
    conds = ["transceiver ready", "COMRESET sent", "COMINIT received", "COMWAKE sent"]
    for a, b, c in zip(left, left[1:], conds):
        d.add(Arrow((a, "b"), (b, "t"), kind="data", label=c, label_side="right"))
    d.add(Arrow((left[4], "r"), (wait, "l"), kind="data", label="COMWAKE received", label_side="above"))
    d.add(Arrow((wait, "t"), (send, "b"), kind="data", label="ALIGN received", label_side="right"))
    d.add(Arrow((send, "t"), (ready, "b"), kind="data", label="3 primitives", label_side="right"))
    d.add(Arrow((ready, "t", 0.3), (rst_rx, "b", 0.3), kind="control", label="misalign",
        label_side="left"))
    d.add(Arrow((rst_rx, "b", 0.7), (ready, "t", 0.7), kind="control", label="ready",
        label_side="right"))
    # 10 ms retry timer: AWAIT-COMINIT and AWAIT-COMWAKE back to RESET (left trunk).
    lt = 40
    for b in (left[2], left[4]):
        d.add(Bus([(lx, b.cy), (lt, b.cy)], dash=DASH_FEEDBACK))
    d.add(Text((lx + lt)/2, left[2].cy - 10, "10 ms", size=FONT_SIZE_SM))
    d.add(Text((lx + lt)/2, left[4].cy - 10, "10 ms", size=FONT_SIZE_SM))
    d.add(Arrow((lt, left[4].cy), (left[0], "l"), via=[(lt, left[0].cy)], kind="feedback"))
    # 873 us ALIGN timer and RX idle: right trunk back to RESET.
    rt = 715
    for b, t in ((wait, "873 µs"), (send, "873 µs"), (ready, "RX idle")):
        d.add(Bus([(rx + w, b.cy), (rt, b.cy)], dash=DASH_FEEDBACK))
        d.add(Text((rx + w + rt)/2, b.cy - 10, t, size=FONT_SIZE_SM))
    d.add(Arrow((rt, wait.cy), (left[0], "r"), via=[(rt, left[0].cy)], kind="feedback"))
    return d
