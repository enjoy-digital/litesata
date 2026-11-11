#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Receive flow control: the host sends HOLD when its receive buffer passes 128 dwords and R_IP again
# when it drains; the drive answers HOLD with HOLDA and pauses the frame.

from core_helpers import *


def build():
    d = Diagram(CANVAS_WIDTH, 420)
    e = Exchange(d, "Host", "Drive")
    e.msg("X_RDY", to="left", step=36)
    e.msg("R_RDY", to="right", note="RX: RDY", step=36)
    e.msg("SOF, data dwords", to="left", kind="accent", note="RX: COPY, sends R_IP", step=40)
    e.msg("HOLD", to="right", note="buffer above 128 dwords", step=36)
    e.msg("HOLDA", to="left", note="frame paused", step=40)
    e.msg("R_IP", to="right", note="buffer at 128 or below", step=36)
    e.msg("data dwords, EOF", to="left", kind="accent", step=36)
    e.msg("WTRM", to="left", step=36)
    e.msg("R_OK", to="right", note="CRC checked", step=36)
    return d
