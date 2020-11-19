#
# This file is part of LiteSATA.
#
# Copyright (c) 2015-2019 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from math import ceil

from migen import *
from migen.genlib.cdc import PulseSynchronizer

from litex.soc.interconnect import stream
from litex.soc.interconnect.stream import *
from litex.soc.interconnect.packet import Header, HeaderField

# Bitrates/Frequencies -----------------------------------------------------------------------------

bitrates = {
    "gen3": 6.0,
    "gen2": 3.0,
    "gen1": 1.5,
}

frequencies = {
    "gen3": 150.0,
    "gen2": 75.0,
    "gen1": 37.5,
}


# PHY / Link Layers --------------------------------------------------------------------------------

primitives = {
    "ALIGN":   0x7b4a4abc,
    "ALIGN_N": 0x7bb5b5bc,
    "CONT":    0x9999aa7c,
    "SYNC":    0xb5b5957c,
    "R_RDY":   0x4a4a957c,
    "R_OK":    0x3535b57c,
    "R_ERR":   0x5656b57c,
    "R_IP":    0x5555b57c,
    "X_RDY":   0x5757b57c,
    "CONT":    0x9999aa7c,
    "WTRM":    0x5858b57c,
    "SOF":     0x3737b57c,
    "EOF":     0xd5d5b57c,
    "HOLD":    0xd5d5aa7c,
    "HOLDA":   0x9595aa7c
}

def is_primitive(dword):
    for k, v in primitives.items():
        if dword == v:
            return True
    return False


def decode_primitive(dword):
    for k, v in primitives.items():
        if dword == v:
            return k
    return ""


def phy_description(dw):
    layout = [
        ("data", dw),
        ("charisk", dw//8),
    ]
    return EndpointDescription(layout)


def link_description(dw):
    layout = [
        ("data", dw),
        ("error", 1)
    ]
    return EndpointDescription(layout)

class _PulseSynchronizer(PulseSynchronizer):
    def __init__(self, i, idomain, o, odomain):
        PulseSynchronizer.__init__(self, idomain, odomain)
        self.comb += [
            self.i.eq(i),
            o.eq(self.o)
        ]

class _RisingEdge(Module):
    def __init__(self, i, o):
        i_d = Signal()
        self.sync += i_d.eq(i)
        self.comb += o.eq(i & ~i_d)

# Transport Layer ----------------------------------------------------------------------------------

fis_max_dwords = 2048

fis_types = {
    "REG_H2D":          0x27,
    "REG_D2H":          0x34,
    "DMA_ACTIVATE_D2H": 0x39,
    "PIO_SETUP_D2H":    0x5f,
    "DATA":             0x46
}

fis_reg_h2d_header_length = 5
fis_reg_h2d_header_fields = {
    "type":         HeaderField(0*4,  0, 8),
    "pm_port":      HeaderField(0*4,  8, 4),
    "c":            HeaderField(0*4, 15, 1),
    "command":      HeaderField(0*4, 16, 8),
    "features_lsb": HeaderField(0*4, 24, 8),

    "lba_lsb":      HeaderField(1*4, 0, 24),
    "device":       HeaderField(1*4, 24, 8),

    "lba_msb":      HeaderField(2*4, 0, 24),
    "features_msb": HeaderField(2*4, 24, 8),

    "count":        HeaderField(3*4, 0, 16),
    "icc":          HeaderField(3*4, 16, 8),
    "control":      HeaderField(3*4, 24, 8)
}
fis_reg_h2d_header = Header(fis_reg_h2d_header_fields,
                            fis_reg_h2d_header_length,
                            swap_field_bytes=False)

fis_reg_d2h_header_length = 5
fis_reg_d2h_header_fields = {
    "type":    HeaderField(0*4,  0, 8),
    "pm_port": HeaderField(0*4,  8, 4),
    "i":       HeaderField(0*4, 14, 1),
    "status":  HeaderField(0*4, 16, 8),
    "errors":  HeaderField(0*4, 24, 8),

    "lba_lsb": HeaderField(1*4, 0, 24),
    "device":  HeaderField(1*4, 24, 8),

    "lba_msb": HeaderField(2*4, 0, 24),

    "count":   HeaderField(3*4, 0, 16)
}
fis_reg_d2h_header = Header(fis_reg_d2h_header_fields,
                            fis_reg_d2h_header_length,
                            swap_field_bytes=False)

fis_dma_activate_d2h_header_length = 1
fis_dma_activate_d2h_header_fields = {
    "type":    HeaderField(0*4,  0, 8),
    "pm_port": HeaderField(0*4,  8, 4)
}
fis_dma_activate_d2h_header = Header(fis_dma_activate_d2h_header_fields,
                                     fis_dma_activate_d2h_header_length,
                                     swap_field_bytes=False)

fis_pio_setup_d2h_header_length = 5
fis_pio_setup_d2h_header_fields = {
    "type":           HeaderField(0*4,  0, 8),
    "pm_port":        HeaderField(0*4,  8, 4),
    "d":              HeaderField(0*4, 13, 1),
    "i":              HeaderField(0*4, 14, 1),
    "status":         HeaderField(0*4, 16, 8),
    "errors":         HeaderField(0*4, 24, 8),

    "lba_lsb":        HeaderField(1*4, 0, 24),

    "lba_msb":        HeaderField(2*4, 0, 24),

    "count":          HeaderField(3*4, 0, 16),

    "transfer_count": HeaderField(4*4, 0, 16),
}
fis_pio_setup_d2h_header = Header(fis_pio_setup_d2h_header_fields,
                                  fis_pio_setup_d2h_header_length,
                                  swap_field_bytes=False)

fis_data_header_length = 1
fis_data_header_fields = {
    "type": HeaderField(0,  0, 8)
}
fis_data_header = Header(fis_data_header_fields,
                         fis_data_header_length,
                         swap_field_bytes=False)

def transport_tx_description(dw):
    param_layout = [
        ("type",      8),
        ("pm_port",   4),
        ("c",         1),
        ("command",   8),
        ("features", 16),
        ("lba",      48),
        ("device",    8),
        ("count",    16),
        ("icc",       8),
        ("control",   8)
    ]
    payload_layout = [("data", dw)]
    return EndpointDescription(payload_layout, param_layout)


def transport_rx_description(dw):
    param_layout = [
        ("type",            8),
        ("pm_port",         4),
        ("r",               1),
        ("d",               1),
        ("i",               1),
        ("status",          8),
        ("errors",          8),
        ("lba",            48),
        ("device",          8),
        ("count",          16),
        ("transfer_count", 16),
        ("error",           1)
    ]
    payload_layout = [("data", dw)]
    return EndpointDescription(payload_layout, param_layout)


# Command Layer ------------------------------------------------------------------------------------

regs = {
    "WRITE_DMA_EXT":   0x35,
    "READ_DMA_EXT":    0x25,
    "IDENTIFY_DEVICE": 0xec
}

reg_d2h_status = {
    "bsy":  7,
    "drdy": 6,
    "df":   5,
    "se":   5,
    "dwe":  4,
    "drq":  3,
    "ae":   2,
    "sns":  1,
    "cc":   0,
    "err":  0
}

def command_tx_description(dw):
    param_layout = [
        ("write",    1),
        ("read",     1),
        ("identify", 1),
        ("sector",  48),
        ("count",   16)
    ]
    payload_layout = [("data", dw)]
    return EndpointDescription(payload_layout, param_layout)


def command_rx_description(dw):
    param_layout = [
        ("write",    1),
        ("read",     1),
        ("identify", 1),
        ("end",      1),
        ("failed",   1)
    ]
    payload_layout = [("data", dw)]
    return EndpointDescription(payload_layout, param_layout)


def command_rx_cmd_description(dw):
    param_layout = [
        ("write",    1),
        ("read",     1),
        ("identify", 1),
        ("end",      1),
        ("failed",   1)
    ]
    payload_layout = [("dummy", 1)]
    return EndpointDescription(payload_layout, param_layout)


def command_rx_data_description(dw):
    payload_layout = [("data", dw)]
    return EndpointDescription(payload_layout)


# HDD ----------------------------------------------------------------------------------------------

logical_sector_size = 512  # constant since all HDDs use this

def dwords2sectors(n):
    return ceil(n*4/logical_sector_size)


def sectors2dwords(n):
    return n*logical_sector_size//4
