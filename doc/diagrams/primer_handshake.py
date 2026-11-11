#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Link handshake of one frame from the host: X_RDY, R_RDY, SOF, FIS, R_IP, EOF, WTRM, R_OK, SYNC.

from primer_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 470)
    e = Exchange(d, left="Transmitter (host)", right="Receiver (device)")
    e.msg("X_RDY", to="right", note="frame to send")
    e.msg("R_RDY", to="left", note="ready")
    e.msg("SOF", to="right")
    e.msg("FIS and CRC", to="right", kind="accent", note="scrambled")
    e.msg("R_IP", to="left", note="receiving")
    e.msg("EOF", to="right")
    e.msg("WTRM", to="right", note="waiting")
    y = e.msg("R_OK", to="left", note="CRC good")
    d.add(Mark(e.xl - 12 - text_width("CRC good", 13) - 14, y, color=COLOR_OK))
    e.msg("SYNC", to="right")
    e.msg("SYNC", to="left", note="idle")
    return d
