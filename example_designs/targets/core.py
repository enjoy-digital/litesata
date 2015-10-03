from migen.genlib.resetsync import AsyncResetSynchronizer

from mibuild.generic_platform import *
from mibuild.xilinx.platform import XilinxPlatform

from targets import *

from misoclib.soc import SoC

from litesata.common import *
from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.raid import LiteSATAStriping
from litesata.frontend.bist import LiteSATABIST


_io = [
    ("sys_clk", 0, Pins("X")),
    ("sys_rst", 1, Pins("X")),
    ("sata_clocks", 0,
        Subsignal("refclk_p", Pins("X")),
        Subsignal("refclk_n", Pins("X"))
    ),
]
for i in range(4):
    _io.append(("sata", i,
                   Subsignal("txp", Pins("X")),
                   Subsignal("txn", Pins("X")),
                   Subsignal("rxp", Pins("X")),
                   Subsignal("rxn", Pins("X"))
                )
    )


class CorePlatform(XilinxPlatform):
    name = "core"
    def __init__(self):
        XilinxPlatform.__init__(self, "xc7", _io)

    def do_finalize(self, *args, **kwargs):
        pass


class Core(Module):
    platform = CorePlatform()
    def __init__(self, platform, design="base", clk_freq=200*1000000, nports=2, ports_dw=32):
        self.clk_freq = clk_freq

        if design == "base" or design == "bist":
            # SATA PHY/Core/frontend
            self.submodules.sata_phy = LiteSATAPHY(platform.device, platform.request("sata_clocks"), platform.request("sata"), "sata_gen3", clk_freq)
            self.sata_phys = [self.sata_phy]
            self.submodules.sata_core = LiteSATACore(self.sata_phy)
            self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_core)

            if design == "bist":
                # BIST
                self.submodules.sata_bist = LiteSATABIST(self.sata_crossbar)

        elif design == "striping":
            self.nphys = 4
            # SATA PHYs
            sata_phy0 = LiteSATAPHY(platform.device, platform.request("sata_clocks"), platform.request("sata", 0), "sata_gen3", clk_freq)
            sata_phy1 = LiteSATAPHY(platform.device, sata_phy0.crg.refclk, platform.request("sata", 1), "sata_gen3", clk_freq)
            sata_phy2 = LiteSATAPHY(platform.device, sata_phy0.crg.refclk, platform.request("sata", 2), "sata_gen3", clk_freq)
            sata_phy3 = LiteSATAPHY(platform.device, sata_phy0.crg.refclk, platform.request("sata", 3), "sata_gen3", clk_freq)
            self.sata_phys = [sata_phy0, sata_phy1, sata_phy2, sata_phy3]
            for i, sata_phy in enumerate(self.sata_phys):
                sata_phy = RenameClockDomains(sata_phy, {"sata_rx": "sata_rx{}".format(str(i)),
                                                         "sata_tx": "sata_tx{}".format(str(i))})
                setattr(self.submodules, "sata_phy{}".format(str(i)), sata_phy)

            # SATA Cores
            self.submodules.sata_core0 = LiteSATACore(self.sata_phy0)
            self.submodules.sata_core1 = LiteSATACore(self.sata_phy1)
            self.submodules.sata_core2 = LiteSATACore(self.sata_phy2)
            self.submodules.sata_core3 = LiteSATACore(self.sata_phy3)
            sata_cores = [self.sata_core0, self.sata_core1, self.sata_core2, self.sata_core3]

            # SATA Frontend
            self.submodules.sata_striping = LiteSATAStriping(sata_cores)
            self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_striping)

        else:
            ValueError("Unknown design " + design)

        # Get user ports from crossbar
        self.user_ports = self.sata_crossbar.get_ports(nports, ports_dw)

    def get_ios(self):
        ios = set()

        for sata_phy in self.sata_phys:
            # Transceiver
            for e in dir(sata_phy.clock_pads):
                obj = getattr(sata_phy.clock_pads, e)
                if isinstance(obj, Signal):
                    ios = ios.union({obj})
            for e in dir(sata_phy.pads):
                obj = getattr(sata_phy.pads, e)
                if isinstance(obj, Signal):
                    ios = ios.union({obj})

            # Status
            ios = ios.union({
                sata_phy.crg.ready,
                sata_phy.ctrl.ready
            })

        # BIST
        if hasattr(self, "sata_bist"):
            for bist_unit in ["generator", "checker"]:
                for signal in ["start", "sector", "count", "random", "done", "aborted", "errors"]:
                    ios = ios.union({getattr(getattr(self.sata_bist, bist_unit), signal)})
            ios = ios.union({
                self.sata_bist.identify.start,
                self.sata_bist.identify.done,
                self.sata_bist.identify.source.stb,
                self.sata_bist.identify.source.data,
                self.sata_bist.identify.source.ack
            })

        # User ports
        def _iter_layout(layout):
            for e in layout:
                if isinstance(e[1], list):
                    yield from _iter_layout(e[1])
                else:
                    yield e

        for port in self.user_ports:
            for endpoint in [port.sink, port.source]:
                for e in _iter_layout(endpoint.layout):
                    obj = getattr(endpoint, e[0])
                    ios = ios.union({obj})
        return ios

default_subtarget = Core
