#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Link bring-up: COMRESET, COMINIT, COMWAKE (OOB), then D10.2 from the host, ALIGN from the
# device, ALIGN back from the host, and SYNC from the device.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 405)
    e = Exchange(d)
    e.msg("COMRESET", to="right", kind="accent", note="OOB")
    e.msg("COMINIT", to="left", kind="accent", note="OOB")
    e.msg("COMWAKE", to="right", kind="accent", note="OOB")
    e.msg("COMWAKE", to="left", kind="accent", note="OOB")
    e.gap()
    e.msg("D10.2", to="right", note="until ALIGN", note_at="src")
    e.msg("ALIGN", to="left", note="fastest rate first", note_at="src")
    e.msg("ALIGN", to="right", note="same rate")
    y = e.msg("SYNC", to="left", note="link up")
    d.add(Mark(e.xl - 12 - text_width("link up", 13) - 14, y, color=COLOR_OK))
    return d
