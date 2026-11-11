# Xilinx transceiver PHYs {#xilinx-phys}

LiteSATA runs on the multi-gigabit transceivers of the Xilinx 7-Series, UltraScale and UltraScale+
families. This chapter describes the five wrappers: how the PHY picks one, how each one is
clocked, how it makes OOB in hardware and what differs between families. Read it to choose a
generation and a data width for a board, or to feed the reference clock correctly.

The parts shared with the ECP5 PHY (controller, datapath, registers) are in
[PHY architecture and OOB controller](#phy-architecture).

## At a glance {#xil-glance}

| FPGA family | Class | Primitive | PLL | Status |
|-------------------------|----------------------------|-------------------------|-----|-------------|
| Artix-7 | `A7LiteSATAPHY` | `GTPE2_CHANNEL` | QPLL | Validated |
| Kintex-7 | `K7LiteSATAPHY` | `GTXE2_CHANNEL` | CPLL | Validated |
| Kintex/Virtex UltraScale | `GTH3LiteSATAPHY` | `GTHE3_CHANNEL` | CPLL | Validated |
| Artix/Kintex/Zynq UltraScale+ | `GTH4LiteSATAPHY` | `GTHE4_CHANNEL` | CPLL | Validated |
| Kintex/Virtex UltraScale+ | `GTY4LiteSATAPHY` | `GTYE4_CHANNEL` | CPLL | Validated |

Every wrapper accepts Gen1, Gen2 and Gen3 and a 16-bit or 32-bit transceiver width, with a
150 MHz reference clock. *Validated* means run on hardware with a drive at the generations the
code allows. On Artix-7, the speed grade limits the GTP line rate: a -1 part stops at 3.75 Gbps,
so the Nexys Video (`xc7a200t-sbg484-1`) runs Gen1 or Gen2, as its targets allow.

Each wrapper class comes with a CRG class of the same prefix (`A7LiteSATAPHYCRG` and so on), which
creates the `sata_tx` and `sata_rx` clock domains.

## Device selection {#xil-selection}

`LiteSATAPHY` matches the device name of the platform against a list of patterns and builds the
first wrapper that matches. The device name is the part number as LiteX gives it, such as
`xc7k325t-ffg900-2`.

| Device name starts with | Wrapper | Notes |
|-------------------------------|--------------------------|-----------------------------------|
| `xc7k` | `K7LiteSATAPHY` | Kintex-7 |
| `xc7a` | `A7LiteSATAPHY` | Artix-7, TX buffer enabled |
| `xcku`, `xcvu` plus digits | `GTH3LiteSATAPHY` | UltraScale, no `p` after the digits |
| `xcau`, `xcku`, `xcvu` plus `p` | GTY4 or GTH4 | UltraScale+, from `gt_type` |
| `xczu` | GTY4 or GTH4 | Zynq UltraScale+, from `gt_type` |
| `LFE5UM5G-` | `ECP5LiteSATAPHY` | [Lattice ECP5 PHY](#ecp5-phy) |

On UltraScale+, `gt_type` picks the transceiver: `"GTY"` (the default) or `"GTH"`. Artix
UltraScale+ only has GTH transceivers, so the PHY forces `"GTH"` for `xcau` devices. A Zynq
UltraScale+ MPSoC with GTH transceivers, such as the one of the ZCU102, needs `gt_type="GTH"`. Any
other device name raises `NotImplementedError`.

## Reference clock {#xil-refclk}

Every wrapper expects a 150 MHz reference clock. The PHY takes it in one of two ways, decided by
the `refclk` parameter.

| `refclk` | Source | Buffer |
|-------------------------|-----------------------------------------|------------------------------------|
| `None` | `clk_p`, `clk_n` of the pads | `IBUFDS_GTE2`, `IBUFDS_GTE3` or `IBUFDS_GTE4` |
| A `Signal` or `ClockSignal` | Any 150 MHz clock of the design | None, used as given |

Dedicated pins are the clean source: a crystal oscillator on the reference clock input of the
transceiver quad. Many boards have no 150 MHz oscillator there, so most bench targets generate it
with a fabric PLL instead and pass it as `refclk`. The KC705 and KCU105 targets do this for their
SFP and PCIe adapters and lower a Vivado design rule check to a warning for it.

On UltraScale+, `use_gtgrefclk` decides which transceiver input takes the clock. With `True`, the
default, the clock enters through `GTGREFCLK`, the input for a clock from the fabric. With `False`,
it enters through `GTREFCLK0`, the input for a dedicated pin. Set it to `False` with an
`IBUFDS_GTE4` reference; the ZCU102 target does so for its external reference clock, and the
standalone generator for the `sata_clk_p`/`sata_clk_n` pins of its cores.

On Artix-7, the GTP channel gets its clock from a quad PLL (`GTPQuadPLL` of LiteICLink). The PHY
creates one unless the `qpll` parameter passes an existing one, to share the quad PLL with another
core of the same quad, Ethernet for example. A shared PLL must run its VCO at 3 GHz, as the PHY
would configure it, since the channel dividers are fixed per generation.

## Clocking {#xil-clocking}

Every wrapper runs its PLL VCO at 3 GHz from the 150 MHz reference and sets the line rate with an
output divider per generation. The UltraScale and UltraScale+ wrappers also derive `TXOUTCLK` from
the VCO with the programmable divider of the channel (PROGDIV). The figure below shows how the
clocks reach the fabric.

![Xilinx transceiver clocking. The PLL runs at 3 GHz on every generation; the TX and RX output
clocks are buffered into the `sata_tx` and `sata_rx` clock domains, where the transceiver words
are exchanged.](xil_clocking.svg)

| Generation | Line rate | Output divider | TX PROGDIV, 16 bits | 32 bits |
|------------|-----------|----------------|---------------------|---------|
| Gen1 | 1.5 Gbps | 4 | 40 | 80 |
| Gen2 | 3 Gbps | 2 | 20 | 40 |
| Gen3 | 6 Gbps | 1 | 10 | 20 |

The resulting `sata_tx` and `sata_rx` frequencies are in
[Clocks and the system clock](#phy-clocks).

The receive clock is always the recovered clock of the transceiver (`RXOUTCLK`), buffered by a
BUFG on 7-Series and a `BUFG_GT` on UltraScale and UltraScale+. The transmit clock differs per
family, as the next sections show. The resets of both domains follow the PLL lock and the
`tx_reset` and `rx_reset` of the controller.

The transceiver width sets the fabric clock. At 16 bits, Gen3 puts `sata_tx` and `sata_rx` at
300 MHz; the 32-bit width halves them at the cost of wider words. The bench targets use 16 bits,
except the ZCU102 target, which takes the width as an option (`--dw`).

## OOB in hardware {#xil-oob}

The Xilinx transceivers have SATA OOB support: they send the COMRESET and COMWAKE burst sequences
on request and detect the COMINIT and COMWAKE of the drive. The wrapper maps them onto the
transceiver contract of the controller.

| Contract signal | Transceiver port | Crossing |
|----------------------|-------------------------------|-----------------------------------|
| `tx_cominit_stb` | `TXCOMINIT` | Rising edge, pulse into `sata_tx` |
| `tx_comwake_stb` | `TXCOMWAKE` | Rising edge, pulse into `sata_tx` |
| `tx_cominit_ack`, `tx_comwake_ack` | `TXCOMFINISH` | Pulse into `sys` |
| `rx_cominit_stb` | `RXCOMINITDET` | Synchronized into `sys` |
| `rx_comwake_stb` | `RXCOMWAKEDET` | Synchronized into `sys` |
| `tx_idle` | `TXELECIDLE` | Synchronized into `sata_tx` |

The `SATA_*` attributes of the transceiver instance set the burst and gap windows of the
detectors. On the GTX, a toggle flip-flop divides the reference clock by two and feeds the 75 MHz
result to the OOB clock input (`CLKRSVD`). The GTP wrapper drives `SIGVALIDCLK` the same way and
sets `RXOOB_CLK_CFG` to `"PMA"`.

On UltraScale and UltraScale+, the wrappers select the PMA clock for OOB (`RXOOB_CLK_CFG`) and feed
no extra clock.

The `RXELECIDLE` output of the transceivers is left unconnected and the `rx_idle` of the wrappers
is never driven. The datapath detects a silent drive from the missing ALIGN primitives instead
([RX idle and misalignment](#phy-idle)).

## Buffers and alignment {#xil-buffers}

The wrappers take `tx_buffer_enable` and `rx_buffer_enable`, both off by default. Without the
buffers, the transceiver aligns the phase of its user clocks itself, and the reset sequences of
LiteICLink run that alignment (`GTXTXInit`, `GTXRXInit` and their GTP, GTH and GTY versions).
`LiteSATAPHY` turns the TX buffer on for Artix-7 only and leaves the RX buffer off everywhere.

| Setting | Value | Effect |
|---------------------|---------------------------|------------------------------------------|
| Comma alignment | `ALIGN_COMMA_WORD` 2 or 4 | Comma on byte 0 of a 16-bit or 32-bit word |
| Comma detection | K28.5, both disparities | The transceiver aligns on every ALIGN |
| Clock correction | Off (`CLK_CORRECT_USE`) | No ALIGN removal in the transceiver |
| 8b/10b | In the transceiver | `charisk` per byte on both sides |

Clock correction is off because the RX side has no elastic buffer: `sata_rx` runs on the recovered
clock, and the asynchronous FIFO of the datapath absorbs the difference with `sys`. The link layer
removes the ALIGN primitives ([Link layer](#link-layer)).

## Per family {#xil-families}

**Artix-7 GTP.** The channel takes its clock from the quad PLL, PLL0 or PLL1 of the
`GTPE2_COMMON` as the PLL object selects. The TX buffer is on, so `TXOUTCLK` goes through a BUFG
straight into `sata_tx`. The Acorn target defaults to Gen1 with a 150 MHz reference from a fabric
PLL; the Nexys Video target runs Gen1 or Gen2.

**Kintex-7 GTX.** The channel PLL (CPLL) multiplies the reference by 20. Without a TX buffer, the
wrapper builds `sata_tx` with an MMCM from `TXOUTCLK`. The KC705 and Genesys2 targets default to
Gen3 at 16 bits, with `sata_tx` and `sata_rx` at 300 MHz.

**UltraScale GTH.** The CPLL runs in its SATA configuration at 3 GHz. Without a TX buffer,
`TXOUTCLK` is the programmable divider of the channel, which divides the 3 GHz VCO by the values of
the table in [Clocking](#xil-clocking). A `BUFG_GT` turns it into `sata_tx`. The KCU105
target uses it.

**UltraScale+ GTH and GTY.** Same scheme as the UltraScale GTH, with the reference clock input
chosen by `use_gtgrefclk` ([Reference clock](#xil-refclk)). The GTH4 wrapper serves Artix, Kintex
and Zynq UltraScale+; the GTY4 wrapper serves Kintex and Virtex UltraScale+. The ZCU102 target uses
the GTH and the XCU1525 target the GTY.

## Legacy names {#xil-legacy}

Three modules of `litesata/phy/` keep the class names of earlier versions as aliases, so that old
designs build unchanged.

| Module | Aliases | Class |
|----------------------------|----------------------------|--------------------------|
| `ussataphy.py` | `USLiteSATAPHY` | `GTH3LiteSATAPHY` |
| `uspsataphy.py` | `USPLiteSATAPHY` | `GTY4LiteSATAPHY` |
| `gthe4sataphy.py` | `GTHE4LiteSATAPHY` | `GTH4LiteSATAPHY` |

Each module also aliases the CRG class with the same prefix. New designs use `LiteSATAPHY` and
let it select the wrapper.

## Limitations {#xil-limitations}

- No rate fallback: the generation is fixed at build time ([Limitations](#phy-limitations)).
- The transceiver electrical idle detectors are unused.

## In the code {#xil-code}

| File | Classes |
|---------------------------------------|-------------------------------------------------|
| `litesata/phy/__init__.py` | `LiteSATAPHY`: device selection |
| `litesata/phy/a7sataphy.py` | `A7LiteSATAPHY`, `A7LiteSATAPHYCRG` |
| `litesata/phy/k7sataphy.py` | `K7LiteSATAPHY`, `K7LiteSATAPHYCRG` |
| `litesata/phy/gth3sataphy.py` | `GTH3LiteSATAPHY`, `GTH3LiteSATAPHYCRG` |
| `litesata/phy/gth4sataphy.py` | `GTH4LiteSATAPHY`, `GTH4LiteSATAPHYCRG` |
| `litesata/phy/gty4sataphy.py` | `GTY4LiteSATAPHY`, `GTY4LiteSATAPHYCRG` |
| `test/test_phy.py` | Selection per family, legacy aliases, comma word |

Where to go next: [Lattice ECP5 PHY with OOB in fabric](#ecp5-phy) for the ECP5 DCU,
[Supported FPGAs and boards](#boards) for the boards and adapters, [Integration](#integration) for
the clock rules in a SoC.
