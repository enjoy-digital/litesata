from litesata.common import *
from litesata.phy.ctrl import *
from litesata.phy.datapath import *


class LiteSATAPHY(Module):
    """SATA PHY

    Manages the low level interface between the SATA core and the device.

    This modules use FPGA transceivers (high speed serializers/deserializer) to
    communicates with the SATA devices. Since transceivers are primitives inside
    the FPGA, device passed as parameter is used to select the right PHY. PHY is
    composed of 3 main modules:
    - Transceiver and clocking (vendor specific)
    - Control (vendor agnostic)
    - Datapath (vendor agnostic)

    For now, the Kintex7/Zynq(with PL based on K7) PHY is the only one available,
    but the achitecture is modular enough to accept others PHYs.
    """
    def __init__(self, device, clock_pads_or_refclk, pads, revision, clk_freq, data_width=16):
        self.clock_pads = clock_pads_or_refclk
        self.pads = pads
        self.revision = revision

        # Transceiver / Clocks
        if device[:4] == "xc7k": # Kintex 7
            from litesata.phy.k7sataphy import K7LiteSATAPHYCRG, K7LiteSATAPHY
            self.submodules.phy = K7LiteSATAPHY(pads, revision, data_width)
            self.submodules.crg = K7LiteSATAPHYCRG(clock_pads_or_refclk, pads, self.phy, revision, clk_freq)
        elif device[:4] == "xc7a": # Artix 7
            from litesata.phy.a7sataphy import A7LiteSATAPHYCRG, A7LiteSATAPHY
            self.submodules.phy = A7LiteSATAPHY(pads, revision, data_width)
            self.submodules.crg = A7LiteSATAPHYCRG(clock_pads_or_refclk, pads, self.phy, revision, clk_freq)
        else:
            raise NotImplementedError

        # Control
        self.submodules.ctrl = LiteSATAPHYCtrl(self.phy, self.crg, clk_freq)

        # Datapath
        self.submodules.datapath = LiteSATAPHYDatapath(self.phy, self.ctrl)
        self.comb += [
            self.ctrl.rx_idle.eq(self.datapath.rx_idle),
            self.ctrl.misalign.eq(self.datapath.misalign)
        ]
        self.sink, self.source = self.datapath.sink, self.datapath.source
