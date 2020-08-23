#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from litesata.common import *
from litesata.core.link import LiteSATALink
from litesata.core.transport import LiteSATATransport
from litesata.core.command import LiteSATACommand

# LiteSATA Core ------------------------------------------------------------------------------------

class LiteSATACore(Module):
    def __init__(self, phy):
        self.submodules.link      = LiteSATALink(phy)
        self.submodules.transport = LiteSATATransport(self.link)
        self.submodules.command   = LiteSATACommand(self.transport)
        self.sink, self.source = self.command.sink, self.command.source
