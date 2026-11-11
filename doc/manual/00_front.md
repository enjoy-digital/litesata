```{=latex}
\frontmatter
```

# About this document {.unnumbered}

This manual is the reference of the LiteSATA project: a SATA host core for FPGAs, its PHYs, its
frontends and the software that uses it. It is written for FPGA engineers who need to read or
write a SATA hard disk or SSD from their design, and it teaches SATA along the way.

No prior knowledge of SATA is assumed. Part I explains how a host and a drive wake up the link,
exchange frames and run commands, and points at the module that implements each mechanism. Part II
then describes the core: its architecture, the link layer, the transport and command layers and
the frontends.

Part III describes the PHYs: their shared out-of-band controller, the Xilinx transceiver PHYs and
the Lattice ECP5 PHY. Part IV is the user guide: boards, first build, integration, examples,
simulation and debugging. The appendices hold the register reference, the build options, the
glossary, the performance figures and the maintenance notes of this manual.

| If you want to | Read |
|------------------------------------------|----------------------------------------------|
| Know what the project does and does not do | [Introduction](#introduction) |
| Learn how SATA works | [SATA in a nutshell](#sata-primer) |
| Understand the layers of the core | [Core architecture](#core-architecture) |
| Check that your FPGA or board is supported | [Supported FPGAs and boards](#boards) |
| Get a first drive identified | [Getting started](#getting-started) |
| Add SATA to your own design | [Integration](#integration) |
| Understand a link that does not come up | [Debugging](#debugging) |

The manual follows the repository: every name, number and default is taken from the source code,
and the manual is generated from it on the date printed on the title page. When the code and the
manual disagree, the code is right and the manual has a bug.

## Conventions {.unnumbered}

- Numbers: `0x` prefix for hexadecimal, `0b` for binary.
- Rates: SATA line rates in Gbps (Gen1 `1.5 Gbps`, Gen2 `3 Gbps`, Gen3 `6 Gbps`), clock
  frequencies in MHz. Throughputs in MiB/s, as the bench script `test_bist.py` and the tools quoted
  in this manual print them. Byte rates derived from a line rate are in MB/s (10^6 bytes per
  second). Sizes in bytes, KiB, MiB and GiB.
- A *dword* is 32 bits, the unit of the SATA link. A *sector* is 512 bytes, and an *LBA* is the
  48-bit address of a sector.
- Primitives are written in capitals as the SATA specification names them (ALIGN, SYNC, X_RDY,
  R_OK, HOLD), and so are the out-of-band signals (COMRESET, COMINIT, COMWAKE). 8b/10b characters
  are written K28.5 or D10.2.
- Frames carry a FIS (Frame Information Structure), named with its direction: "Register H2D FIS"
  goes from host to device, "Register D2H FIS" from device to host. ATA commands are written as
  ATA/ACS names them: READ DMA EXT, WRITE DMA EXT, IDENTIFY DEVICE.
- *The host* is LiteSATA in the FPGA; *the device* or *the drive* is the hard disk or SSD.
- *The PHY* is the transceiver, its clocking, *the controller* that brings the link up, and the
  datapath; in this manual "controller" always means this PHY controller. *The core* is the link,
  transport and command layers. *A user port* is a port of the crossbar; *a frontend* is a module
  on a user port, such as the BIST or a DMA.
- *The BIOS* is the LiteX BIOS and its SATA library, `liblitesata`.
- `<target>` stands for the name of a target script of `bench/` without its extension, such as
  `kc705` or `acorn`.
- `<config>` stands for a YAML configuration file of the standalone generator `litesata_gen`, such
  as `examples/artix7.yml`.
- Paths are relative to the repository root. Shell commands are given for a Linux host.
- When the text names a source file, it is the file to open to check or extend the fact.
