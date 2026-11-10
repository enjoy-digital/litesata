#
# This file is part of LiteSATA project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Migen unit tests: a test driver sends commands through the LiteSATA core; the Python drive model
# (test/model/hdd.py) stands for the PHY and the drive and answers at the dword level.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 420)
    tb   = d.add(Container(20, 120, 170, 110, title="Test", title_pos="bl"))
    core = d.add(Container(210, 120, 200, 280, title="LiteSATACore", title_pos="bl"))
    hdd  = d.add(Container(470, 20, 270, 380, title="HDD model", title_pos="bl"))
    drv  = d.add(Box(35, 140, 140, 50, label="Streamer,\nlogger", fill=COLOR_ACCENT_LIGHT))
    cmd  = d.add(Box(225, 140, 170, 50, label="Command"))
    trn  = d.add(Box(225, 220, 170, 50, label="Transport"))
    lnk  = d.add(Box(225, 300, 170, 50, label="Link"))
    hcmd = d.add(Box(485, 60, 130, 50, label="CommandLayer", fill=COLOR_ACCENT))
    htrn = d.add(Box(485, 140, 130, 50, label="TransportLayer", fill=COLOR_ACCENT))
    hlnk = d.add(Box(485, 220, 130, 50, label="LinkLayer", fill=COLOR_ACCENT))
    hphy = d.add(Box(485, 300, 130, 50, label="PHYLayer", fill=COLOR_ACCENT))
    mem  = d.add(Box(640, 60, 85, 50, label="Sector\nmemory"))
    d.add(Arrow((drv, "r"), (cmd, "l"), kind="data", heads="both"))
    d.add(Arrow((cmd, "b"), (trn, "t"), kind="data", heads="both"))
    d.add(Arrow((trn, "b"), (lnk, "t"), kind="data", heads="both"))
    d.add(Arrow((lnk, "r"), (hphy, "l"), kind="accent", heads="both", label="dwords", label_pos=(440, 314)))
    d.add(Arrow((hphy, "t"), (hlnk, "b"), kind="data", heads="both"))
    d.add(Arrow((hlnk, "t"), (htrn, "b"), kind="data", heads="both"))
    d.add(Arrow((htrn, "t"), (hcmd, "b"), kind="data", heads="both"))
    d.add(Arrow((hcmd, "r"), (mem, "l"), kind="data", heads="both"))
    return d
