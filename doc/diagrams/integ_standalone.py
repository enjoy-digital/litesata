#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Standalone core (litesata_gen): the port groups of the generated litesata_core module. The
# control and DMA buses exist only in the control/DMA flavour.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 320)
    d.add(Container(200, 20, 340, 280, title="litesata_core", title_pos="bl"))
    core  = d.add(Box(240, 45, 260, 130, label="Core, crossbar,\nfrontends", fill=COLOR_ACCENT))
    phy   = d.add(Box(240, 215, 260, 50, label="PHY", fill=COLOR_ACCENT_LIGHT))
    clk   = d.add(Box(20, 40, 150, 36, label="clk, rst"))
    ctl   = d.add(Box(20, 105, 150, 50, label="wb_ctrl or\naxil_ctrl"))
    dma   = d.add(Box(20, 180, 150, 36, label="wb_dma"))
    port  = d.add(Box(590, 50, 150, 36, label="user ports"))
    st    = d.add(Box(590, 110, 150, 50, label="enable, ready,\nlink_up, irq"))
    drive = d.add(Box(590, 222, 150, 36, label="Drive", fill=COLOR_HARDWARE))
    d.add(Arrow((clk, "r"), (240, 58), kind="control"))
    d.add(Arrow((ctl, "r"), (240, 130), kind="control", heads="both"))
    d.add(Arrow((255, 175), (dma, "r"), via=[(255, 198)], kind="data", heads="both"))
    d.add(Arrow((500, 68), (port, "l"), kind="accent", heads="both"))
    d.add(Arrow((500, 135), (st, "l"), kind="control", heads="both"))
    d.add(Arrow((370, 175), (370, 215), kind="data", heads="both"))
    d.add(Arrow((phy, "r"), (drive, "l"), kind="data", heads="both"))
    return d
