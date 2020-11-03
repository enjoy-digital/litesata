from litesata.common import *
from litesata.phy.ctrl import *
from litesata.phy.datapath import *


class LiteSATAPHY(Module, AutoCSR):
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
    def __init__(self, device, pads, gen, clk_freq, refclk=None, data_width=16):
        self.pads   = pads
        self.gen    = gen
        self.refclk = refclk

        # Registers
        self.enable = CSRStorage(reset=1)
        self.status = CSRStatus(fields=[
            CSRField("ready", size=1, values=[
                    ("``0b0``", "PHY not initialized."),
                    ("``0b1``", "PHY initialized and ready.")
            ]),
            CSRField("tx_ready", size=1, values=[
                    ("``0b0``", "TX not initialized."),
                    ("``0b1``", "TX initialized and ready.")
            ]),
            CSRField("rx_ready", size=1, values=[
                    ("``0b0``", "RX not initialized."),
                    ("``0b1``", "RX initialized and ready.")
            ]),
            CSRField("ctrl_ready", size=1, values=[
                    ("``0b0``", "Ctrl/OOB not initialized."),
                    ("``0b1``", "Ctrl/OOB initialized and ready.")
            ]),
        ])

        # Transceiver / Clocks
        if device[:4] == "xc7k": # Kintex 7
            from litesata.phy.k7sataphy import K7LiteSATAPHYCRG, K7LiteSATAPHY
            self.submodules.phy = K7LiteSATAPHY(pads, gen, clk_freq, data_width)
            self.submodules.crg = K7LiteSATAPHYCRG(refclk, pads, self.phy, gen)
        elif device[:4] == "xc7a": # Artix 7
            from litesata.phy.a7sataphy import A7LiteSATAPHYCRG, A7LiteSATAPHY
            self.submodules.phy = A7LiteSATAPHY(pads, gen, clk_freq, data_width, tx_buffer_enable=True)
            self.submodules.crg = A7LiteSATAPHYCRG(refclk, pads, self.phy, gen,  tx_buffer_enable=True)
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

        # Restart/Status
        self.comb += [
            self.phy.tx_init.restart.eq(~self.enable.storage),
            self.phy.rx_init.restart.eq(~self.enable.storage | self.ctrl.rx_reset),
            self.status.fields.ctrl_ready.eq(self.ctrl.ready),
            self.status.fields.ready.eq(self.phy.ready & self.ctrl.ready),
            self.status.fields.tx_ready.eq(self.phy.tx_init.done),
            self.status.fields.rx_ready.eq(self.phy.rx_init.done),
        ]