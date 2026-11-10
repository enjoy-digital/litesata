#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Clock domains: the core runs in sys; the PHY datapath crosses to sata_tx and from sata_rx through
# asynchronous FIFOs and converts 32-bit words to/from the transceiver width.

from svgdiag import *


def build():
    d = Diagram(CANVAS_WIDTH, 300)
    d.add(Container(10, 10, 250, 280, title="sys", title_pos="bl"))
    d.add(Container(420, 10, 330, 130, title="sata_tx", title_pos="tr"))
    d.add(Container(420, 160, 330, 130, title="sata_rx", title_pos="br"))
    core = d.add(Box(30, 40, 175, 190, label="Frontends\ncore\nPHY controller", fill=COLOR_ACCENT))
    txf  = d.add(Box(290, 60, 100, 50, label="Async\nFIFO"))
    rxf  = d.add(Box(290, 190, 100, 50, label="Async\nFIFO"))
    txc  = d.add(Box(440, 60, 120, 50, label="32 to 16/32\nconversion"))
    rxc  = d.add(Box(440, 190, 120, 50, label="Realign, to\n32-bit"))
    gtx  = d.add(Box(600, 60, 120, 50, label="Transceiver\nTX", fill=COLOR_HARDWARE))
    grx  = d.add(Box(600, 190, 120, 50, label="Transceiver\nRX", fill=COLOR_HARDWARE))
    d.add(Arrow((core.x + core.w, txf.cy), (txf, "l"), kind="data", label="32-bit", label_pos=(233, txf.cy - 10)))
    d.add(Arrow((txf, "r"), (txc, "l"), kind="data"))
    d.add(Arrow((txc, "r"), (gtx, "l"), kind="data"))
    d.add(Arrow((grx, "l"), (rxc, "r"), kind="data"))
    d.add(Arrow((rxc, "l"), (rxf, "r"), kind="data"))
    d.add(Arrow((rxf, "l"), (core.x + core.w, rxf.cy), kind="data", label="32-bit", label_pos=(233, rxf.cy - 10)))
    return d
