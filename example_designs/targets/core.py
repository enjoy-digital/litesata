from litex.gen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *
from litex.build.xilinx.platform import XilinxPlatform

from targets import *

from litesata.common import *
from litesata.phy import LiteSATAPHY
from litesata.core import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.raid import LiteSATAStriping
from litesata.frontend.bist import LiteSATABIST


_io = [
    ("sys_clock", 0, Pins(1)),
    ("sys_reset", 1, Pins(1)),
    ("sata_clocks", 0,
        Subsignal("refclk_p", Pins(1)),
        Subsignal("refclk_n", Pins(1))
    ),
]
for i in range(4):
    _io.append(("sata", i,
                   Subsignal("txp", Pins(1)),
                   Subsignal("txn", Pins(1)),
                   Subsignal("rxp", Pins(1)),
                   Subsignal("rxn", Pins(1))
                )
    )
_io += [
    # Ready pins
    ("crg_ready",  0, Pins(4)), # FIXME
    ("ctrl_ready", 0, Pins(4)), # FIXME

    # Generator pins
    ("generator", 0,
        Subsignal("start",   Pins(1)),
        Subsignal("sector",  Pins(48)),
        Subsignal("count",   Pins(16)),
        Subsignal("random",  Pins(1)),
        Subsignal("done",    Pins(1)),
        Subsignal("aborted", Pins(1)),
        Subsignal("errors", Pins(32))
    ),

    # Checker pins
    ("checker", 0,
        Subsignal("start",   Pins(1)),
        Subsignal("sector",  Pins(48)),
        Subsignal("count",   Pins(16)),
        Subsignal("random",  Pins(1)),
        Subsignal("done",    Pins(1)),
        Subsignal("aborted", Pins(1)),
        Subsignal("errors",  Pins(32))
    ),

    # Identify pins
    ("identify", 0,
        Subsignal("start",        Pins(1)),
        Subsignal("done",         Pins(1)),
        Subsignal("source_valid", Pins(1)),
        Subsignal("source_ready", Pins(1)),
        Subsignal("source_data",  Pins(32))
    ),
]

# User Ports pins
for i in range(2):
    _io += [
        ("user_port", i+1,
            Subsignal("sink_valid",    Pins(1)),
            Subsignal("sink_last",     Pins(1)),
            Subsignal("sink_ready",    Pins(1)),
            Subsignal("sink_write",    Pins(1)),
            Subsignal("sink_read",     Pins(1)),
            Subsignal("sink_identify", Pins(1)),
            Subsignal("sink_sector",   Pins(48)),
            Subsignal("sink_count",    Pins(16)),
            Subsignal("sink_data",     Pins(128)), # FIXME

            Subsignal("source_valid",    Pins(1)),
            Subsignal("source_last",     Pins(1)),
            Subsignal("source_ready",    Pins(1)),
            Subsignal("source_write",    Pins(1)),
            Subsignal("source_read",     Pins(1)),
            Subsignal("source_identify", Pins(1)),
            Subsignal("source_end",       Pins(1)),
            Subsignal("source_failed",   Pins(1)),
            Subsignal("source_data",     Pins(128)), #FIXME
        ),
    ]

class CorePlatform(XilinxPlatform):
    name = "core"
    def __init__(self):
        XilinxPlatform.__init__(self, "xc7", _io)

    def do_finalize(self, *args, **kwargs):
        pass


class Core(Module):
    platform = CorePlatform()
    def __init__(self, platform, design="base", clk_freq=200*1000000, nports=1, ports_dw=32):
        self.clk_freq = clk_freq

        self.clock_domains.cd_sys = ClockDomain()
        self.comb += [
            self.cd_sys.clk.eq(platform.request("sys_clock")),
            self.cd_sys.rst.eq(platform.request("sys_reset"))
        ]

        if design == "base" or design == "bist":
            # SATA PHY/Core/frontend
            self.submodules.sata_phy = LiteSATAPHY(platform.device, platform.request("sata_clocks"), platform.request("sata"), "sata_gen3", clk_freq)
            self.sata_phys = [self.sata_phy]
            self.submodules.sata_core = LiteSATACore(self.sata_phy)
            self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_core)

            if design == "bist":
                # BIST
                self.submodules.sata_bist = LiteSATABIST(self.sata_crossbar)
                generator = self.sata_bist.generator
                generator_pads = platform.request("generator")
                self.comb += [
                    generator.start.eq(generator_pads.start),
                    generator.sector.eq(generator_pads.sector),
                    generator.count.eq(generator_pads.count),
                    generator.random.eq(generator_pads.random),

                    generator_pads.done.eq(generator.done),
                    generator_pads.aborted.eq(generator.aborted),
                    generator_pads.errors.eq(generator.errors)
                ]

                checker = self.sata_bist.checker
                checker_pads = platform.request("checker")
                self.comb += [
                    checker.start.eq(checker_pads.start),
                    checker.sector.eq(checker_pads.sector),
                    checker.count.eq(checker_pads.count),
                    checker.random.eq(checker_pads.random),

                    checker_pads.done.eq(checker.done),
                    checker_pads.aborted.eq(checker.aborted),
                    checker_pads.errors.eq(checker.errors),
                ]

                identify = self.sata_bist.identify
                identify_pads = platform.request("identify")
                self.comb += [
                    identify.start.eq(identify_pads.start),
                    identify_pads.done.eq(identify.done),

                    identify_pads.source_valid.eq(identify.source.valid),
                    identify_pads.source_data.eq(identify.source.data),
                    identify.source.ready.eq(identify_pads.source_ready)
                ]

            self.sata_phy.crg.cd_sata_rx.clk.attr.add("keep")
            self.sata_phy.crg.cd_sata_tx.clk.attr.add("keep")

        elif design == "striping":
            self.nphys = 4
            # SATA PHYs
            self.sata_phys = []
            for i in range(self.nphys):
                sata_phy = LiteSATAPHY(platform.device,
                                       platform.request("sata_clocks") if i == 0 else self.sata_phys[0].crg.refclk,
                                       platform.request("sata", i),
                                       "sata_gen3",
                                       clk_freq)
                sata_phy = ClockDomainsRenamer({"sata_rx": "sata_rx{}".format(str(i)),
                                                "sata_tx": "sata_tx{}".format(str(i))})(sata_phy)
                setattr(self.submodules, "sata_phy{}".format(str(i)), sata_phy)
                self.sata_phys.append(sata_phy)

            # SATA Cores
            self.sata_cores = []
            for i in range(self.nphys):
                sata_core = LiteSATACore(self.sata_phys[i])
                setattr(self.submodules, "sata_core{}".format(str(i)), sata_core)
                self.sata_cores.append(sata_core)

            # SATA Frontend
            self.submodules.sata_striping = LiteSATAStriping(self.sata_cores)
            self.submodules.sata_crossbar = LiteSATACrossbar(self.sata_striping)

            for i in range(len(self.sata_phys)):
                self.sata_phys[i].crg.cd_sata_rx.clk.attr.add("keep")
                self.sata_phys[i].crg.cd_sata_tx.clk.attr.add("keep")

        else:
            ValueError("Unknown design " + design)


        # CRG / Ctrl ready
        crg_ready_pads = platform.request("crg_ready")
        ctrl_ready_pads = platform.request("ctrl_ready")
        for i, sata_phy in enumerate(self.sata_phys):
            self.comb += [
                crg_ready_pads[i].eq(sata_phy.crg.ready),
                ctrl_ready_pads[i].eq(sata_phy.ctrl.ready)
            ]

        # Get user ports from crossbar
        self.user_ports = self.sata_crossbar.get_ports(nports, ports_dw)
        for i, user_port in enumerate(self.user_ports):
            user_port_pads = platform.request("user_port", i+1)
            self.comb += [
                user_port.sink.valid.eq(user_port_pads.sink_valid),
                user_port.sink.last.eq(user_port_pads.sink_last),
                user_port.sink.write.eq(user_port_pads.sink_write),
                user_port.sink.read.eq(user_port_pads.sink_read),
                user_port.sink.identify.eq(user_port_pads.sink_identify),
                user_port.sink.sector.eq(user_port_pads.sink_sector),
                user_port.sink.count.eq(user_port_pads.sink_count),

                user_port_pads.sink_ready.eq(user_port.sink.ready),
            ]
            self.comb += [
                user_port_pads.source_valid.eq(user_port.source.valid),
                user_port_pads.source_last.eq(user_port.source.last),
                user_port_pads.source_write.eq(user_port.source.write),
                user_port_pads.source_read.eq(user_port.source.read),
                user_port_pads.source_identify.eq(user_port.source.identify),
                user_port_pads.source_end.eq(user_port.source.end),
                user_port_pads.source_failed.eq(user_port.source.failed),
                user_port_pads.source_data.eq(user_port.source.data),

                user_port.source.ready.eq(user_port_pads.source_ready),
            ]

default_subtarget = Core
