#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# READ DMA EXT: Register H2D, Data FISes from the device, Register D2H with the status.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 255)
    e = Exchange(d)
    e.msg("Register H2D FIS", to="right", note="READ DMA EXT")
    e.msg("Data FIS", to="left", kind="accent", note="up to 8 KiB")
    e.msg("Data FIS", to="left", kind="accent", note="up to 8 KiB")
    e.msg("Register D2H FIS", to="left", note="status")
    return d
