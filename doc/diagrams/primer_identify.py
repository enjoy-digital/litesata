#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# IDENTIFY DEVICE (PIO data-in): Register H2D, PIO Setup with length and status, one Data FIS.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 215)
    e = Exchange(d)
    e.msg("Register H2D FIS", to="right", note="IDENTIFY DEVICE")
    e.msg("PIO Setup FIS", to="left", note="512 bytes, status")
    e.msg("Data FIS", to="left", kind="accent", note="512 bytes")
    return d
