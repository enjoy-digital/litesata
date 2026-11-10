#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# WRITE DMA EXT as the command layer runs it: one Data FIS of at most 2048 dwords per DMA Activate,
# the Register D2H FIS closes the command.

from core_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 330)
    e = Exchange(d, "Host", "Drive")
    e.msg("Register H2D: WRITE DMA EXT", to="right", note="TX: SEND_CMD")
    e.msg("DMA Activate", to="left", note="TX: WAIT_DMA_ACTIVATE")
    e.msg("Data FIS, 2048 dwords", to="right", kind="accent", note="TX: SEND_DATA")
    e.msg("DMA Activate", to="left", note="TX: WAIT_DMA_ACTIVATE")
    e.msg("Data FIS, remaining dwords", to="right", kind="accent", note="TX: SEND_DATA")
    e.msg("Register D2H: status", to="left", note="RX: PRESENT_WRITE_RESPONSE")
    return d
