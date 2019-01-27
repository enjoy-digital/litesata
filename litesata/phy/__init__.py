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
        if device[:3] == "xc7": # Kintex 7
            from litesata.phy.k7sataphy import K7LiteSATAPHYCRG, K7LiteSATAPHYTRX
            self.submodules.trx = K7LiteSATAPHYTRX(pads, revision, data_width)
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
