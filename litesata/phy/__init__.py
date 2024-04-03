#
# This file is part of LiteSATA.
#
# Copyright (c) 2019-2024 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import re

from litex.gen import *

from litesata.common import *
from litesata.phy.ctrl import *
from litesata.phy.datapath import *

# LiteSATAPHY --------------------------------------------------------------------------------------

class LiteSATAPHY(LiteXModule):
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
    def __init__(self, device, pads, gen, clk_freq, refclk=None, data_width=16,
                 qpll=None, gt_type="GTY", use_gtgrefclk=True, with_csr=True):
        self.pads   = pads
        self.gen    = gen
        self.refclk = refclk

        # Control/Status.
        self.enable = Signal()
        self.ready  = Signal()

        # Transceiver / Clocks.
        # ---------------------

        # Kintex7.
        if re.match("^xc7k", device):
            from litesata.phy.k7sataphy import K7LiteSATAPHYCRG, K7LiteSATAPHY
            self.phy = K7LiteSATAPHY(pads, gen, clk_freq, data_width)
            self.crg = K7LiteSATAPHYCRG(refclk, pads, self.phy, gen)

        # Artix7.
        elif re.match("^xc7a", device):
            from litesata.phy.a7sataphy import A7LiteSATAPHYCRG, A7LiteSATAPHY
            self.phy = A7LiteSATAPHY(pads, gen, clk_freq, data_width, tx_buffer_enable=True, qpll=qpll)
            self.crg = A7LiteSATAPHYCRG(refclk, pads, self.phy, gen,  tx_buffer_enable=True)

        # Kintex/Virtex Ultrascale.
        elif re.match("^xc[kv]u[0-9]+-", device):
            from litesata.phy.ussataphy import USLiteSATAPHYCRG, USLiteSATAPHY
            self.phy = USLiteSATAPHY(pads, gen, clk_freq, data_width)
            self.crg = USLiteSATAPHYCRG(refclk, pads, self.phy, gen)

        # Kintex/Virtex/Zynq Ultrascale+.
        elif re.match("^xc([kv]u[0-9]+p-|zu[0-9])", device):
            if gt_type == "GTY":
                # GTY transceiver for Virtex/Kintex Ultrascale+
                from litesata.phy.uspsataphy import USPLiteSATAPHYCRG, USPLiteSATAPHY
                self.phy = USPLiteSATAPHY(pads, gen, clk_freq, data_width, use_gtgrefclk=use_gtgrefclk)
                self.crg = USPLiteSATAPHYCRG(refclk, pads, self.phy, gen)
            elif gt_type == "GTH":
                # GTH transceiver for Kintex/Zynq Ultrascale+
                from litesata.phy.gthe4sataphy import GTHE4LiteSATAPHYCRG, GTHE4LiteSATAPHY
                self.phy = GTHE4LiteSATAPHY(pads, gen, clk_freq, data_width, use_gtgrefclk=use_gtgrefclk)
                self.crg = GTHE4LiteSATAPHYCRG(refclk, pads, self.phy, gen)
            else:
                raise NotImplementedError(f"Unsupported GT type. : {gt_type}")

        # ECP5.
        elif re.match("^LFE5UM5G-", device):
            from litesata.phy.ecp5sataphy import ECP5LiteSATAPHYCRG, ECP5LiteSATAPHY
            self.phy = ECP5LiteSATAPHY(refclk, pads, gen, clk_freq, data_width)
            self.crg = ECP5LiteSATAPHYCRG(self.phy)

        # Unknown.
        else:
            raise NotImplementedError(f"Unsupported {device} Device.")

        # Control.
        # --------
        self.ctrl = LiteSATAPHYCtrl(self.phy, self.crg, clk_freq)

        # Datapath.
        # ---------
        self.datapath = LiteSATAPHYDatapath(self.phy, self.ctrl)
        self.comb += [
            self.ctrl.rx_idle.eq(self.datapath.rx_idle),
            self.ctrl.misalign.eq(self.datapath.misalign)
        ]
        self.sink, self.source = self.datapath.sink, self.datapath.source

        # Restart/Status.
        # ---------------
        if hasattr(self.phy, "tx_init") and hasattr(self.phy, "rx_init"):
            self.comb += self.phy.tx_init.restart.eq(~self.enable)
            self.comb += self.phy.rx_init.restart.eq(~self.enable | self.ctrl.rx_reset)
        self.comb += self.ready.eq(self.phy.ready & self.ctrl.ready)

        # CSRs.
        # -----
        if with_csr:
            self.add_csr()

    def  add_csr(self):
        self._enable = CSRStorage(reset=1)
        self._status = CSRStatus(fields=[
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

        self.comb += self.enable.eq(self._enable.storage)
        self.comb += self._status.fields.ready.eq(self.phy.ready & self.ctrl.ready)
        if hasattr(self.phy, "tx_init") and hasattr(self.phy, "rx_init"):
            self.comb += self._status.fields.tx_ready.eq(self.phy.tx_init.done)
            self.comb += self._status.fields.rx_ready.eq(self.phy.rx_init.done)
        else:
            self.comb += self._status.fields.tx_ready.eq(self.phy.ready)
            self.comb += self._status.fields.rx_ready.eq(self.phy.ready)
        self.comb += self._status.fields.ctrl_ready.eq(self.ctrl.ready)
