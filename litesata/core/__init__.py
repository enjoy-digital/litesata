from litesata.common import *
from litesata.core.link import LiteSATALink
from litesata.core.transport import LiteSATATransport
from litesata.core.command import LiteSATACommand


class LiteSATACore(Module):
    def __init__(self, phy):
        self.submodules.link = LiteSATALink(phy)
        self.submodules.transport = LiteSATATransport(self.link)
        self.submodules.command = LiteSATACommand(self.transport)
        self.sink, self.source = self.command.sink, self.command.source
