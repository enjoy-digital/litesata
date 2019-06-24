# This file is Copyright (c) 2018-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# License: BSD

from litex.build.generic_platform import *
from litex.boards.platforms import nexys_video

_sata_io = [
    ("sata_clocks", 0,
        Subsignal("refclk_p", Pins("LPC:GBTCLK0_M2C_P")),
        Subsignal("refclk_n", Pins("LPC:GBTCLK0_M2C_N"))
    ),
    ("sata", 0,
        Subsignal("txp", Pins("LPC:DP0_C2M_P")),
        Subsignal("txn", Pins("LPC:DP0_C2M_N")),
        Subsignal("rxp", Pins("LPC:DP0_M2C_P")),
        Subsignal("rxn", Pins("LPC:DP0_M2C_N"))
    ),
]


class Platform(nexys_video.Platform):
    def __init__(self, *args, **kwargs):
        nexys_video.Platform.__init__(self, *args, **kwargs)
        self.add_extension(_sata_io)

    def do_finalize(self, fragment):
            try:
                self.add_period_constraint(self.lookup_request("clk100"), 10.0)
            except ConstraintError:
                pass
            self.add_platform_command("""
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 2.5 [current_design]
""")
