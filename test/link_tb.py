from litesata.common import *
from litesata.core.link import LiteSATALink

from model.hdd import *
from common import *


class LinkStreamer(PacketStreamer):
    def __init__(self):
        PacketStreamer.__init__(self, link_description(32), LinkTXPacket)


class LinkLogger(PacketLogger):
    def __init__(self):
        PacketLogger.__init__(self, link_description(32), LinkRXPacket)


class TB(Module):
    def __init__(self):
        self.submodules.hdd = HDD(
                link_debug=False, link_random_level=50,
                transport_debug=False, transport_loopback=True)
        self.submodules.link = LiteSATALink(self.hdd.phy)

        self.submodules.streamer = LinkStreamer()
        self.submodules.streamer_randomizer = Randomizer(link_description(32), level=50)

        self.submodules.logger_randomizer = Randomizer(link_description(32), level=50)
        self.submodules.logger = LinkLogger()

        self.submodules.pipeline = Pipeline(
            self.streamer,
            self.streamer_randomizer,
            self.link,
            self.logger_randomizer,
            self.logger
        )

def main_generator(dut):
    for i in range(2):
        streamer_packet = LinkTXPacket([i for i in range(64)])
        yield from dut.streamer.send_blocking(streamer_packet)
        yield from dut.logger.receive()

        # check results
        s, l, e = check(streamer_packet, dut.logger.packet)
        print("shift " + str(s) + " / length " + str(l) + " / errors " + str(e))

    # XXX: find a way to exit properly
    import sys
    sys.exit()

if __name__ == "__main__":
    tb = TB()
    generators = {
        "sys" :   [main_generator(tb),
                   tb.hdd.link.generator(),
                   tb.streamer.generator(),
                   tb.streamer_randomizer.generator(),
                   tb.logger.generator(),
                   tb.logger_randomizer.generator(),
                   tb.hdd.phy.rx.generator(),
                   tb.hdd.phy.tx.generator()]
    }
    clocks = {"sys": 10}
    run_simulation(tb, generators, clocks, vcd_name="sim.vcd")
