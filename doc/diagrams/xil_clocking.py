#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Xilinx transceiver clocking: 150 MHz reference clock, channel or quad PLL at 3 GHz, TXOUTCLK
# and RXOUTCLK buffered into the sata_tx and sata_rx clock domains, sys for the core.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 280)
    ref  = d.add(Box(15, 45, 110, 60, label="Refclk\n150 MHz", fill=COLOR_HARDWARE))
    d.add(Container(150, 15, 300, 250, title="Transceiver", title_pos="br"))
    pll  = d.add(Box(170, 45, 120, 60, label="PLL\nVCO 3 GHz", fill=COLOR_HARDWARE))
    tdiv = d.add(Box(315, 45, 115, 60, label="TX divider\nTXOUTCLK", fill=COLOR_HARDWARE))
    cdr  = d.add(Box(315, 150, 115, 60, label="CDR\nRXOUTCLK", fill=COLOR_HARDWARE))
    tbuf = d.add(Box(480, 45, 120, 60, label="BUFG, MMCM\nor BUFG_GT"))
    rbuf = d.add(Box(480, 150, 120, 60, label="BUFG\nor BUFG_GT"))
    txd  = d.add(Box(640, 45, 105, 60, label="sata_tx", fill=COLOR_ACCENT))
    rxd  = d.add(Box(640, 150, 105, 60, label="sata_rx", fill=COLOR_ACCENT))
    d.add(Arrow((ref, "r"), (pll, "l"), kind="control"))
    d.add(Arrow((pll, "r"), (tdiv, "l"), kind="control"))
    d.add(Arrow((pll, "b"), (cdr, "l"), kind="control"))
    d.add(Arrow((tdiv, "r"), (tbuf, "l"), kind="control"))
    d.add(Arrow((cdr, "r"), (rbuf, "l"), kind="control"))
    d.add(Arrow((tbuf, "r"), (txd, "l"), kind="control"))
    d.add(Arrow((rbuf, "r"), (rxd, "l"), kind="control"))
    return d
