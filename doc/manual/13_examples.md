# Example designs and targets {#examples}

This chapter describes the ready-made designs of the repository: the generator examples of
`examples/`, the bench targets of `bench/` and the host scripts that drive them. Use them to check a
board and a drive, to start a configuration of your own, or to measure a link.

## At a glance {#ex-glance}

Each example is a YAML file for the generator (`litesata_gen`): one PHY, one generation, one system
clock and the ports of the core. The generator turns it into a standalone Verilog core.

| Example | PHY, generation, clock | Flavour and ports |
|------------------------------------------------|------------------------|----------------------------|
| `artix7` | `A7SATAPHY`, Gen2, 100 MHz | Native, one 32-bit port |
| `kintex7` | `K7SATAPHY`, Gen3, 100 MHz | Native, one 32-bit port |
| `ultrascale_plus` | `GTH4SATAPHY`, Gen3, 150 MHz | Native, one 32-bit port |
| `artix7_wishbone_dma` | `A7SATAPHY`, Gen2, 100 MHz | Wishbone, DMA, one 32-bit port |
| `ultrascale_plus_axi_lite_dma` | `GTH4SATAPHY`, Gen3, 150 MHz | AXI-Lite, DMA, BIST |

The clock is the system clock (`clk_freq`). The *native* flavour exposes the user ports of the
crossbar as streaming ports and needs no CPU. The *control/DMA* flavour adds a control bus to the
registers, a Wishbone DMA master and an interrupt output. [Integration](#integration) describes
both flavours and their ports, and [Build options](#build-options) lists every key of the YAML
files.

## Generating an example {#ex-generate}

The generator reads the YAML file, elaborates the core and writes the Verilog and the register map.
It never runs the FPGA toolchain: the project that integrates the core synthesizes it, so no vendor
tool is needed to generate it.

```sh
# Native core of examples/artix7.yml in build/
litesata_gen examples/artix7.yml
# Control/DMA core in its own directory, with another module name
litesata_gen examples/artix7_wishbone_dma.yml --output-dir build/a7_dma --name sata_core
```

| Output | Content |
|----------------------------------------------|----------------------------------------------------|
| `gateware/litesata_core.v` | The core: PHY, link, transport, command, crossbar and frontends |
| `gateware/` constraint and Tcl files | Periods of the SATA clocks, false paths to the system clock |
| `csr.csv` | Register map of the core |

The constraints cover the clocks only. The location of the transceiver and the pins of its
reference clock are added by the integrator, as for any transceiver design.

The test `test/test_gen.py` runs the generator on the five examples and checks the transceiver
primitive and the pads of each one. It is the quickest check that a change of the core still
generates with every PHY of the generator.

## What each example shows {#ex-details}

The examples share their structure: PHY, generation, system clock, then the control bus and the
ports. They differ in the points below.

**Native cores.** `artix7.yml`, `kintex7.yml` and `ultrascale_plus.yml` have one native port of 32
bits and nothing else. Their comments show where `data_width` (16 or 32) and `gt_type` go, both
left at their defaults.

**System clock.** The Artix-7 examples run Gen2 at 100 MHz. `kintex7.yml` runs Gen3 at 100 MHz,
below the 150 MHz that the 32-bit datapath needs at Gen3 ([System clock](#perf-sysclk)). The native
flavour does not check the system clock, so this example generates; raise `clk_freq` to 150 MHz
or more before using it on a Gen3 link.

**Control/DMA cores.** `artix7_wishbone_dma.yml` adds a Wishbone control bus, a 32-bit DMA bus,
Identify, both DMAs and the interrupt, and keeps one native port beside them.
`ultrascale_plus_axi_lite_dma.yml` uses an AXI-Lite control bus and a 64-bit DMA bus, adds the
BIST and has no native port. The register appendix is generated from it
([Register reference](#registers)).

**UltraScale+ transceivers.** `ultrascale_plus.yml` lists the three PHY names of the UltraScale
families in its comments. `GTH4SATAPHY` selects the GTH of the Artix, Kintex and Zynq UltraScale+
devices, `GTY4SATAPHY` the GTY of the Kintex and Virtex UltraScale+ devices.

## Bench targets {#ex-bench}

The targets of `bench/` build a small LiteX SoC around the core for one board, without a CPU. They
exist to bring up a PHY on hardware: the host reads the registers through a bridge and runs the
host scripts below.

| Target | Board, FPGA | PHY | SATA path (`--connector`) | Gen | sys_clk |
|----------------------|-----------------------|--------|-----------------------|-------|-----------|
| `acorn` | Acorn CLE-215+, Artix-7 | GTP | PCIe riser mod | 1 (1-3) | 100 MHz |
| `ecpix5` | ECPIX-5, ECP5-5G | ECP5 DCU | Onboard connector | 2 | 90 MHz |
| `genesys2` | Genesys2, Kintex-7 | GTX | FMC RAID adapter (HPC) | 3 (1-3) | 200 MHz |
| `kc705` | KC705, Kintex-7 | GTX | `fmc`, `sfp`, `pcie` | 3 (1-3) | 200 MHz |
| `kcu105` | KCU105, Kintex UltraScale | GTH | `fmc`, `sfp`, `pcie` | 3 (1-3) | 187.5 MHz |
| `nexys_video` | Nexys Video, Artix-7 | GTP | FMC RAID adapter (LPC) | 2 (1-2) | 100 MHz |
| `xcu1525` | XCU1525, Virtex UltraScale+ | GTY | `qsfp`, `pcie` | 3 (1-3) | 187.5 MHz |
| `zcu102` | ZCU102, Zynq UltraScale+ | GTH | FMC RAID adapter (HPC1) | 3 (1-3) | 175 MHz |

The Gen column gives the default of the command line (`--gen`) and the accepted values. Every
target uses a 16-bit PHY datapath, except `zcu102`, which defaults to 32 bits (`--dw`). The host
reaches the registers through a JTAGBone bridge on `acorn` and a UARTBone bridge on the other
targets, at 1 Mbaud on `ecpix5`. [Bench targets](#boards-bench) gives the reference clock of each
path, and [Supported FPGAs and boards](#boards) describes the boards and the adapters.

The design is the same on every target:

- A PLL makes the system clock and, on most boards, the 150 MHz reference clock of the
  transceiver. The FMC targets take it from the adapter instead, except `zcu102` with
  `--refclk internal` and `nexys_video` with `--pll-refclk`.
- The PHY, the core and the crossbar, with the BIST on two user ports and its own Identify on a
  third.
- `kc705` adds the Sector2Mem and Mem2Sector DMAs: it is the only target with DMAs.
- The LEDs blink from the system clock and the SATA clocks (RX only on `xcu1525` and `zcu102`,
  none on `ecpix5`); another LED shows that the link is up. A blinking SATA clock LED shows that
  the transceiver PLL runs.
- `--with-analyzer` adds a LiteScope analyzer on the state machines of the PHY and the core
  ([Debugging](#debugging)), on every Xilinx target except `kc705`, which has three analyzers
  instead: `--with-global-analyzer`, `--with-sector2mem-analyzer` and
  `--with-mem2sector-analyzer`. `ecpix5` has none.

```sh
# From the build directory of the target.
# Elaborate only: no toolchain, writes csr.csv for the host scripts
python3 ../bench/kc705.py --connector sfp
# Build and load the bitstream (volatile: the flash is not written)
python3 ../bench/kc705.py --connector sfp --gen 2 --build --load
```

`acorn.py` checks its system clock: it refuses a generation that needs more than the system clock
gives through the 32-bit core, so Gen3 needs `--sys-clk-freq 150e6`. `--variant` selects the
CLE-101, CLE-215 or CLE-215+ device. `ecpix5.py` builds with the open-source toolchain (Yosys,
nextpnr, Trellis), runs Gen2 only, and takes `--device` (85F or 45F) and the placement `--seed`.
`zcu102.py` takes the usual LiteX target options plus `--gen`, `--dw` and `--refclk`.

**Limitation.** `zcu102.py` does not elaborate with the LiteX of this manual: it asks for UARTBone
with `no_uart` set, and the LiteX SoC refuses UARTBone on the default serial UART, with
`--refclk external` as with `internal`.

## Host scripts {#ex-scripts}

The host scripts open a `RemoteClient` on a running `litex_server` and use the register names of
the `csr.csv` written by the target. Start the server for the bridge of the target first.

```sh
# UARTBone targets (ECPIX-5: add --uart-baudrate 1000000)
litex_server --uart --uart-port /dev/ttyUSB1
# JTAGBone target (acorn), with the JTAG cable configuration of your setup
litex_server --jtag --jtag-config <openocd_config>
```

| Script | Does | Options |
|----------------------|--------------------------------------|------------------------------------------|
| `test_init.py` | Restarts the PHY, polls its status | `--retries` (80), `--interval` (0.1 s), `--port` |
| `test_bist.py` | IDENTIFY, then BIST writes and reads | see the table below |
| `test_dma.py` | Reads sectors to memory and dumps them | `--mode` (`r`, `r+w`), `--sector`, `--count`, `--port` |

`test_init.py` writes 0 then 1 to `sata_phy_enable` and polls `sata_phy_status` until its `ready`
bit is set. It prints `Success` with the number of polls, or `Failed` with the `tx_ready`,
`rx_ready` and `ctrl_ready` bits; [Debugging](#debugging) explains how to read them.

`test_bist.py` first runs IDENTIFY DEVICE through the BIST and prints the serial number, firmware
revision, model, capacity in GiB, the generations the drive supports and its 48-bit LBA support.
It then writes and reads back the drive from sector 0, one transfer at a time, and prints one line
per transfer with the write and read throughputs in MiB/s, the errors and the retries.

| Option | Default | Effect |
|------------------------------|--------|------------------------------------------------------|
| `-s`, `--transfer_size` | 1024 | KiB per transfer: one BIST command of that many sectors |
| `-l`, `--total_length` | 256 | MiB to test, from sector 0 |
| `-n`, `--loops` | 1 | Commands per transfer, for a more precise rate |
| `-r`, `--random` | off | Pseudo-random data instead of a counter |
| `-c`, `--continuous` | off | Run until interrupted |
| `-i`, `--identify` | off | IDENTIFY only |
| `-t`, `--software_timer` | off | Time on the host instead of the `cycles` register |
| `-a`, `--random_addressing` | off | Random start sectors |
| `-d`, `--delayed_read` | off | Write the whole length, then read it |

A BIST command carries at most 65535 sectors, since `count` is 16 bits, so a transfer stays under
32 MiB. The rate is computed from the `cycles` register of the BIST, in system clock cycles, so it
does not include the register accesses of the host. A command that the drive aborts is run again
and counted in `retry`.

`test_dma.py` reads sectors one by one into the `sram` memory region of the SoC and dumps them.
With `--mode r+w` it also writes a counter pattern to sector 1000 and reads it back.

**Limitation.** `test_dma.py` does not write `nsectors`, and a DMA started with `nsectors` at 0 does
not end. The only bench target with DMAs, `kc705`, has no `sram` region either. Use the script as a
starting point on a SoC that has both DMAs and an SRAM, and write `nsectors` first.

## LiteX-Boards targets {#ex-litex-boards}

The LiteX-Boards targets with a SATA connector or adapter take `--with-sata`. They add the PHY and
call `add_sata` of the SoC, with the BIOS commands of [Getting started](#getting-started).

```sh
python3 -m litex_boards.targets.lambdaconcept_ecpix5 --with-sata --build --load
```

They are the right starting point for a SoC with a CPU, while the bench targets suit the first
bring-up of a PHY. [Supported FPGAs and boards](#boards) lists them.

## Where to go next {#ex-next}

[Getting started](#getting-started) for a first build, [Integration](#integration) for the generator
flavours and `add_sata`, [Simulation and verification](#simulation) for the tests, and
[Debugging](#debugging) when a link does not come up.
