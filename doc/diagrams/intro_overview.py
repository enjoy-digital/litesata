#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# LiteSATA at a glance: user logic, frontends, crossbar, core and PHY in the FPGA, the drive
# outside; a CPU and the BIOS reach the frontends through registers.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    d.add(Container(105, 10, 535, 155, title="LiteSATA", title_pos="tl"))
    user = d.add(Box(10,  55, 75, 70, label="User\nlogic", fill=COLOR_FILL))
    fe   = d.add(Box(120, 55, 95, 70, label="Frontends", stack=2, size=14, fill=COLOR_ACCENT_LIGHT))
    xbar = d.add(Box(250, 55, 75, 70, label="Cross-\nbar", fill=COLOR_ACCENT))
    core = d.add(Box(355, 45, 135, 90, label="Core\ncommand\ntransport\nlink", fill=COLOR_ACCENT))
    phy  = d.add(Box(515, 55, 105, 70, label="PHY", fill=COLOR_ACCENT_LIGHT))
    hdd  = d.add(Box(680, 55, 70, 70, label="Drive", fill=COLOR_HARDWARE))
    cpu  = d.add(Box(105, 195, 130, 44, label="CPU and BIOS", fill=COLOR_FILL))
    d.add(Arrow((user, "r"), (fe, "l"), kind="data", heads="both"))
    d.add(Arrow((fe.x + fe.w + 2*STACK_DX, fe.cy), (xbar, "l"), kind="data", heads="both"))
    d.add(Arrow((xbar, "r"), (core, "l"), kind="data", heads="both"))
    d.add(Arrow((core, "r"), (phy, "l"), kind="data", heads="both"))
    d.add(Arrow((phy, "r"), (hdd, "l"), kind="data", heads="both", label="SATA", label_pos=(660, 79)))
    d.add(Arrow((cpu.cx, 195), (cpu.cx, fe.y + fe.h), kind="control", label="registers",
        label_side="right", label_pos=(178, 178), label_anchor="start"))
    return d
