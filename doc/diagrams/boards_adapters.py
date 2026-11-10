#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# From the FPGA transceiver to the drive: the ways the boards of this manual bring a SATA lane out
# (onboard connector or M.2 slot, FMC RAID adapter, SFP-to-SATA adapter, PCIe riser mod).

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(15, 20, 200, 260, title="FPGA", title_pos="bl"))
    trx = d.add(Box(35, 55, 160, 60, label="Transceiver", fill=COLOR_HARDWARE))
    pll = d.add(Box(35, 165, 160, 50, label="PLL, 150 MHz", fill=COLOR_ACCENT_LIGHT))
    adapters = [
        d.add(Box(300, 30,  190, 44, label="Onboard, M.2", fill=COLOR_ACCENT)),
        d.add(Box(300, 95,  190, 44, label="FMC RAID adapter",  fill=COLOR_ACCENT)),
        d.add(Box(300, 160, 190, 44, label="SFP-to-SATA",       fill=COLOR_ACCENT)),
        d.add(Box(300, 225, 190, 44, label="PCIe riser mod",    fill=COLOR_ACCENT)),
    ]
    drive = d.add(Box(600, 115, 140, 70, label="Drive", fill=COLOR_HARDWARE))
    d.add(Arrow((pll, "t"), (trx, "b"), kind="control", label="refclk", label_side="right"))
    for a in adapters:
        y = a.y + a.h/2
        d.add(Arrow((195, 85), (300, y), via=[(250, 85), (250, y)], kind="data", heads="both"))
        d.add(Arrow((490, y), (600, 150), via=[(545, y), (545, 150)], kind="data", heads="both"))
    fmc = adapters[1]
    d.add(Arrow((fmc.x, fmc.y + fmc.h - 8), (195, 105), via=[(292, fmc.y + fmc.h - 8), (292, 105)],
        kind="feedback", label="refclk", label_pos=(255, 101), label_anchor="start"))
    return d
