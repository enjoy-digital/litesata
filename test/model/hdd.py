#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import math

from litesata.common import *

from test.model.phy import *
from test.model.link import *
from test.model.transport import *
from test.model.command import *

# Helpers ------------------------------------------------------------------------------------------

def print_hdd(s, n=None):
    print( "[HDD{}]: {}".format("" if n is None else str(n), s))

# HDD Mem Region -----------------------------------------------------------------------------------

class HDDMemRegion:
    def __init__(self, base, count, sector_size):
        self.base  = base
        self.count = count
        self.data  = [0]*(count*sector_size//4)

# HDD model ----------------------------------------------------------------------------------------

class HDD(Module):
    def __init__(self, n=None,
            link_debug=False, link_random_level=0,
            transport_debug=False, transport_loopback=False,
            hdd_debug=False):
        self.n = n
        self.submodules.phy       = PHYLayer()
        self.submodules.link      = LinkLayer(self.phy, link_debug, link_random_level)
        self.submodules.transport = TransportLayer(self.link, transport_debug, transport_loopback)
        self.submodules.command   = CommandLayer(self.transport)

        self.command.set_hdd(self)

        self.debug         = hdd_debug
        self.mem           = None
        self.wr_sector     = 0
        self.wr_end_sector = 0
        self.rd_sector     = 0
        self.rx_end_sector = 0

        self.reg_d2h_status       = 0
        self.data_error_injection = 0
        self.busy                 = 0

    def malloc(self, sector, count):
        if self.debug:
            s = "Allocating {n} sectors: {s} to {e}".format(n=count, s=sector, e=sector+count-1)
            s += " ({} KB)".format(count*logical_sector_size//1024)
            print_hdd(s, self.n)
        self.mem = HDDMemRegion(sector, count, logical_sector_size)

    def write(self, sector, data):
        n = math.ceil(dwords2sectors(len(data)))
        if self.debug:
            if n == 1:
                s = "{}".format(sector)
            else:
                s = "{s} to {e}".format(s=sector, e=sector+n-1)
            print_hdd("Writing sector " + s, self.n)
        for i in range(len(data)):
            offset = sectors2dwords(sector)
            self.mem.data[offset+i] = data[i]

    def read(self, sector, count):
        if self.debug:
            if count == 1:
                s = "{}".format(sector)
            else:
                s = "{s} to {e}".format(s=sector, e=sector+count-1)
            print_hdd("Reading sector " + s, self.n)
        data = []
        for i in range(sectors2dwords(count)):
            data.append(self.mem.data[sectors2dwords(sector)+i])
        return data

    def set_reg_d2h_status(self, value):
        self.reg_d2h_status = value & 0xff

    def get_reg_d2h(self):
        reg_d2h = FIS_REG_D2H()
        reg_d2h.status = self.reg_d2h_status or self.busy
        return reg_d2h

    def set_data_error_injection(self, value):
        self.data_error_injection = value

    def set_busy(self, value):
        self.busy = value & 0x1

    def write_dma_callback(self, fis):
        self.wr_sector = fis.lba_lsb + (fis.lba_msb << 32)
        self.wr_end_sector = self.wr_sector + fis.count
        return [FIS_DMA_ACTIVATE_D2H()] if not self.busy else [self.get_reg_d2h()]

    def read_dma_callback(self, fis):
        self.rd_sector = fis.lba_lsb + (fis.lba_msb << 32)
        self.rd_end_sector = self.rd_sector + fis.count
        packets = []
        if not self.busy:
            while self.rd_sector != self.rd_end_sector:
                count = min(self.rd_end_sector-self.rd_sector, (fis_max_dwords*4)//logical_sector_size)
                packet = self.read(self.rd_sector, count)
                packet.insert(0, 0)
                packets.append(FIS_DATA(packet, direction="D2H"))
                self.rd_sector += count
            if self.data_error_injection:
                for packet in packets:
                    packet.data_error_injection = True
        packets.append(self.get_reg_d2h())
        return packets

    def data_callback(self, fis):
        self.write(self.wr_sector, fis.packet[1:])
        self.wr_sector += dwords2sectors(len(fis.packet[1:]))
        if self.wr_sector == self.wr_end_sector or self.busy:
            return [self.get_reg_d2h()]
        else:
            return [FIS_DMA_ACTIVATE_D2H()]
