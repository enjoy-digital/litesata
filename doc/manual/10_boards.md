```{=latex}
\part{Use it}
```

```{=html}
<h1 class="part">Part IV. Use it</h1>
```

# Supported FPGAs and boards {#boards}

This chapter lists the FPGA families that LiteSATA supports, the boards that run it, and the ways
a board brings a SATA lane out to a drive. Read it before choosing hardware, before buying an
adapter, or before routing a SATA connector on a board of your own.

## At a glance {#boards-glance}

LiteSATA has one PHY per transceiver family. `LiteSATAPHY` picks it from the FPGA device name, so
a design names the device and never the PHY class. The table below gives the transceiver, the
generations and the PHY data widths of each family; [Xilinx transceiver PHYs](#xilinx-phys) and
[Lattice ECP5 PHY with OOB in fabric](#ecp5-phy) describe each PHY.

| FPGA family | Transceiver | PHY class | Gens | Status |
|--------------------------|---------------|----------------------------|-------------|-------------|
| Artix-7 | GTP | `A7LiteSATAPHY` | Gen1-Gen3 | Validated |
| Kintex-7 | GTX | `K7LiteSATAPHY` | Gen1-Gen3 | Validated |
| Kintex/Virtex UltraScale | GTH | `GTH3LiteSATAPHY` | Gen1-Gen3 | Validated |
| Artix/Kintex/Zynq UltraScale+ | GTH | `GTH4LiteSATAPHY` | Gen1-Gen3 | Validated |
| Kintex/Virtex UltraScale+ | GTY | `GTY4LiteSATAPHY` | Gen1-Gen3 | Validated |
| Lattice ECP5-5G | DCU | `ECP5LiteSATAPHY` | Gen2 | Validated |

"Validated" means that the PHY has run on hardware with a real drive, at the generations of its
row. On Artix-7, Gen3 also needs a speed grade whose GTP reaches 6 Gbps (see [Bench
targets](#boards-bench)). Every Xilinx PHY has a 16-bit or a 32-bit transceiver interface, 16 bits
by default, 32 bits to halve the transceiver clocks; the ECP5 PHY is 16-bit only. The core itself
always works on 32-bit dwords.

The Xilinx PHYs use the out-of-band (OOB) signalling logic of their transceiver. The ECP5 DCU has
none usable for SATA, so its PHY makes and detects the OOB signals in the fabric, at Gen2 only.
Intel (Altera), Gowin, Efinix and the other Lattice families have no PHY.

## How the PHY is selected {#boards-selection}

`LiteSATAPHY` matches the device string of the platform against the patterns below and builds
the matching transceiver wrapper. Any other device raises `NotImplementedError`.

| Device name starts with | Family | Transceiver |
|-----------------------------------------|---------------------------|------------------------------|
| `xc7k` | Kintex-7 | GTX |
| `xc7a` | Artix-7 | GTP, TX buffer enabled |
| `xcku`, `xcvu` and digits | Kintex/Virtex UltraScale | GTH (GTHE3) |
| `xcku`, `xcvu`, `xcau` with `p`; `xczu` | UltraScale+ | GTY, or GTH with `gt_type` |
| `LFE5UM5G-` | ECP5-5G | DCU, Gen2 only |

On UltraScale+, `gt_type` selects the GTY (the default) or the GTH wrapper. Artix UltraScale+
devices only have GTH transceivers, and the PHY forces `gt_type="GTH"` for them; a Zynq
UltraScale+ board such as the ZCU102 passes `gt_type="GTH"` itself. On ECP5, the PHY accepts the
`LFE5UM5G` devices only, and raises an error for any generation other than Gen2.

## From the transceiver to the drive {#boards-adapters}

A SATA link is one transceiver lane: a TX pair and an RX pair, plus a 150 MHz reference clock for
the transceiver PLL. The boards of this manual bring the lane out in five ways: an onboard SATA
connector, an M.2 SATA slot, an FMC adapter, an SFP adapter or a PCIe riser mod. The figure below
groups the two onboard paths. The reference clock comes from the adapter when it has one,
otherwise from a PLL of the FPGA.

![From the FPGA transceiver to the drive. Each board uses one of the paths in the middle (the
onboard box stands for a connector or an M.2 slot); the FMC RAID adapter can also provide the
150 MHz reference clock, the other paths take it from a PLL of the FPGA.](boards_adapters.svg)

| Path | Example boards | Reference clock |
|----------------------------|-----------------------------------------|-----------------------|
| Onboard SATA connector | ECPIX-5, Acorn Baseboard Mini | PLL of the FPGA |
| M.2 SATA slot | Acorn Baseboard, standard variant | PLL of the FPGA |
| AB09-FMCRAID FMC adapter | KC705, Genesys2, KCU105, Nexys Video, ZCU102 | FMC clock or PLL |
| TE0424 SFP-to-SATA adapter | KC705, KCU105, KCU116, XCU1525 | PLL of the FPGA |
| PCIe x1 riser mod | Acorn CLE-215+, KC705, KCU105, XCU1525 | PLL of the FPGA |

**FMC RAID adapter.** The AB09-FMCRAID is an FMC card with SATA connectors on the multi-gigabit
lanes of the FMC. The targets use lane `DP0` and take the reference clock from the `GBTCLK0` pins
of the FMC, a dedicated transceiver clock input. The ZCU102 target describes five lanes of the
card and uses the first one.

**SFP-to-SATA adapter.** The TE0424 fits in an SFP cage and has a SATA connector at its back. It
carries the lane only, so the target makes the 150 MHz reference clock with a PLL of the FPGA.

**PCIe riser mod.** A small PCIe x1 riser board, modified with a SATA plug, connects lane 0 of a
PCIe connector to a drive. The figure below shows which pins of the riser carry the two pairs and
their grounds, and the pinout of the SATA plug. Get the pair order right: the PHY corrects a
swapped P/N on the RX pair, which it detects from the received ALIGN. It cannot detect the
polarity of the TX pair, only alternate it from one attempt to the next
([Polarity](#phy-polarity)), and it cannot fix swapped TX and RX pairs.

![PCIe x1 riser to SATA adapter: the two pairs A and B on the riser, their grounds, and the pinout
of the SATA plug (pins 2 and 3 are pair A, pins 5 and 6 are pair
B).](pcie_riser_sata_adapter.jpg){width=90%}

**Onboard connectors.** Some boards route a transceiver lane to a SATA connector or to an M.2 slot
wired for SATA. The Acorn Baseboard, shown below in its standard variant, has an M.2 SATA slot on
a lane of the Acorn module, and two SATA connectors on the transceivers of its own ECP5. No
target of this repository or of LiteX-Boards uses these two connectors.

![The LiteX Acorn Baseboard, standard variant, with an Acorn CLE-215+ module, the two SATA
connectors of its ECP5, an SFP cage and an RJ45 jack.](acorn_baseboard.jpg){width=75%}

![An M.2 SATA SSD in the M.2 slot of the Acorn Baseboard, next to the Acorn module. The two SATA
connectors at the back belong to the ECP5 of the baseboard.](acorn_baseboard_m2_ssd.jpg){width=75%}

## Bench targets {#boards-bench}

The `bench/` directory of the repository has one target per test board. Each target is a small
SoC without a CPU: the PHY, the core, the crossbar and the BIST, with a bridge that gives a host
computer access to the registers. [Bench targets](#ex-bench) of the Examples chapter lists them,
with their board, SATA path, generations and system clock; this section gives the hardware points
that the table leaves out.

All targets use a 16-bit PHY, except `zcu102.py` which defaults to 32 bits (`--dw`).
`nexys_video.py` accepts Gen1 and Gen2 only. On Artix-7, the speed grade limits the GTP line rate:
a `-1` part stops at 3.75 Gbps, so the Nexys Video (`xc7a200t-sbg484-1`) runs Gen1 or Gen2, as its
targets allow.

The reference clock follows the SATA path. With an FMC adapter, it comes from the `GBTCLK0` pins
of the FMC, except on the Nexys Video with `--pll-refclk` and on the ZCU102 with
`--refclk internal`. The other paths take it from a 150 MHz PLL output.

The `xcu1525.py` target names its first path `qsfp` and puts the lane on pins of a QSFP cage;
its comment refers to the TE0424 SFP-to-SATA adapter. The `ecpix5.py` target uses the open-source
toolchain (Yosys, nextpnr, Trellis) and takes `--device` for the 85F or 45F version of the board.

## LiteX-Boards targets with SATA {#boards-litex-boards}

Most users meet LiteSATA through LiteX-Boards, the collection of LiteX platforms and targets. The
targets below create a SATA PHY when given `--with-sata`, then call `add_sata` of the LiteX SoC,
which adds the core, the DMAs and the BIOS support. [Integration in a LiteX SoC](#integ-litex)
describes `add_sata`.

| Target | FPGA | SATA path | Gen |
|-----------------------------------------------|----------------|----------------|-------------------|
| `sqrl_acorn` | Artix-7 | PCIe riser mod | Gen1 |
| `litex_acorn_baseboard_mini` | Artix-7 | Onboard connector | Gen2, `--sata-gen` 1 or 2 |
| `digilent_nexys_video` | Artix-7 | FMC adapter | Gen2, `--sata-gen` 1 or 2 |
| `decklink_mini_4k` | Artix-7 | PCIe riser mod | Gen2 |
| `xilinx_kc705` | Kintex-7 | SFP-to-SATA | Gen2 |
| `sitlinv_stlv7325_v1` | Kintex-7 | Onboard `sata` pins | Gen2 |
| `sitlinv_stlv7325_v2` | Kintex-7 | Onboard `sata` pins | Gen2, `--sata-gen` 1 to 3 |
| `sitlinv_xc7k420t` | Kintex-7 | Onboard `sata` pins | Gen2 |
| `xilinx_kcu105` | Kintex UltraScale | SFP-to-SATA | Gen2 |
| `xilinx_kcu116` | Kintex UltraScale+ | SFP-to-SATA | Gen2 |
| `sqrl_xcu1525` | Virtex UltraScale+ | SFP-to-SATA, QSFP pins | Gen2 |
| `lambdaconcept_ecpix5` | ECP5-5G | Onboard connector | Gen2 |

All of them use a 16-bit PHY and a 150 MHz reference clock from a PLL. On the Acorn Baseboard
Mini, the SATA PHY shares the GTP quad PLL with the Ethernet PHY, so that both run in the same
bitstream; SATA and PCIe exclude each other on that board. The command below builds and loads the
KC705 target with SATA; [Getting started](#getting-started) continues from there.

```sh
python3 -m litex_boards.targets.xilinx_kc705 --with-sata --build --load
```

## Drives {#boards-drives}

LiteSATA talks to any SATA drive that accepts the three commands of the core: IDENTIFY DEVICE,
READ DMA EXT and WRITE DMA EXT, on 512-byte logical sectors with 48-bit addressing. SSDs and hard
disks both work. A hard disk may need time to leave a power-down state before it answers a
command, and the command layer has no timeout.

Drives differ in how long they take to answer and in the write rate they sustain. Keep one
known-good drive as a reference, and write down which drive a result was measured with. The drives
below appear in the public captures and logs of LiteSATA.

| Drive | Kind | Where it was used |
|---------------------------|----------------|--------------------------------------------|
| Samsung 850 EVO 250GB | 2.5" SSD | Artix-7 Gen2, LiteX-M2SDR recording |
| Kingston SA400S37240G | 2.5" SSD | BIOS `sata_init` capture |
| WDC WDS120G1G0A | 2.5" SSD | Acorn Baseboard Mini, BIOS `sata_init` |
| PNY M.2 SATA SSD | M.2 SATA SSD | Acorn Baseboard, M.2 slot (photo) |
| Toshiba DT01ACA100 | 3.5" hard disk | ECPIX-5 at Gen2, IDENTIFY |

The sustained write rate is a property of the drive, not only of the link. The LiteX-M2SDR
project records RF samples to a SATA SSD with LiteSATA, on an Acorn Baseboard Mini (Artix-7 GTP,
Gen2) with a Samsung 850 EVO 250GB; [Measured throughput](#perf-measured) quotes its figure. The
same project reports much lower rates with the WDC WDS120G1G0A on the same path. Choose the drive
for the rate it sustains once its write cache is full. [Performance and resources](#performance)
gives the ceilings of the link itself.

## Requirements for a custom board {#boards-custom}

A board of your own needs four things for LiteSATA. Start from a LiteX-Boards platform of the
same FPGA family when there is one.

1. **A transceiver lane** routed as two 100 ohm differential pairs to the SATA connector, in the
   pair order of the connector. In the platform, the lane is a record with the subsignals `tx_p`,
   `tx_n`, `rx_p` and `rx_n`.
2. **A 150 MHz reference clock** for the transceiver: either an oscillator on dedicated
   transceiver clock pins, given in the same record as `clk_p` and `clk_n`, or a PLL output of
   the FPGA passed to `LiteSATAPHY` as `refclk`. [Reference clock](#integ-refclk) shows both.
3. **A system clock** fast enough for the generation: 37.5 MHz for Gen1, 75 MHz for Gen2 and
   150 MHz for Gen3, as [Generation, width and system clock](#integ-clocks) explains.
4. **A transceiver that reaches the line rate**: 6 Gbps for Gen3, 3 Gbps for Gen2. Check the
   speed grade of the FPGA in its data sheet; on ECP5, use an `LFE5UM5G` device.

On Artix-7, the four channels of a GTP quad share two PLLs. When Ethernet uses the same quad, the
target builds the quad PLL once and passes the SATA channel of it to `LiteSATAPHY` as `qpll`, as
the Acorn Baseboard Mini target does.

Where to go next: [Getting started](#getting-started) to bring up a first board,
[Integration](#integration) to add LiteSATA to a design, [Debugging](#debugging) when the link
does not come up.
