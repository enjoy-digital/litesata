from litesata.common import *
from litesata.phy.ctrl import *
from litesata.phy.datapath import *


class LiteSATAPHY(Module):
    def __init__(self, device, clock_pads_or_refclk, pads, revision, clk_freq, trx_dw=16):
        self.clock_pads = clock_pads_or_refclk
        self.pads = pads
        self.revision = revision

        # Transceiver / Clocks
        if device[:3] == "xc7": # Kintex 7
            from litesata.phy.k7.trx import K7LiteSATAPHYTRX
            from litesata.phy.k7.crg import K7LiteSATAPHYCRG
            self.submodules.trx = K7LiteSATAPHYTRX(pads, revision, trx_dw)
            self.submodules.crg = K7LiteSATAPHYCRG(clock_pads_or_refclk, pads, self.trx, revision, clk_freq)
        else:
            raise NotImplementedError

        # Control
        self.submodules.ctrl = LiteSATAPHYCtrl(self.trx, self.crg, clk_freq)

        # Datapath
        self.submodules.datapath = LiteSATAPHYDatapath(self.trx, self.ctrl)
        self.comb += [
            self.ctrl.rx_idle.eq(self.datapath.rx_idle),
            self.ctrl.misalign.eq(self.datapath.misalign)
        ]
        self.sink, self.source = self.datapath.sink, self.datapath.source
