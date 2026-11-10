#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# READ DMA EXT as the command layer runs it: Data FISes come back until the Register D2H FIS, which
# closes the command.

from core_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 250)
    e = Exchange(d, "Host", "Drive")
    e.msg("Register H2D: READ DMA EXT", to="right", note="TX: SEND_CMD")
    e.msg("Data FIS", to="left", kind="accent", note="RX: PRESENT_READ_DATA")
    e.msg("Data FIS", to="left", kind="accent", note="RX: PRESENT_READ_DATA")
    e.msg("Register D2H: status", to="left", note="RX: PRESENT_READ_RESPONSE")
    return d
