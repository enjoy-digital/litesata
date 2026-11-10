#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# WRITE DMA EXT: Register H2D, then DMA Activate and Data FIS pairs, Register D2H with the status.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 335)
    e = Exchange(d)
    e.msg("Register H2D FIS", to="right", note="WRITE DMA EXT")
    e.msg("DMA Activate FIS", to="left", note="ready for data")
    e.msg("Data FIS", to="right", kind="accent", note="up to 8 KiB")
    e.msg("DMA Activate FIS", to="left", note="ready for data")
    e.msg("Data FIS", to="right", kind="accent", note="up to 8 KiB")
    e.msg("Register D2H FIS", to="left", note="status")
    return d
