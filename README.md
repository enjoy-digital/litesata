```
                                    __   _ __      _______ _________
                                   / /  (_) /____ / __/ _ /_  __/ _ |
                                  / /__/ / __/ -_)\ \/ __ |/ / / __ |
                                 /____/_/\__/\__/___/_/ |_/_/ /_/ |_|

                                  Copyright 2014-2026 / EnjoyDigital
                                     Copyright 2014-2015 / HKU

                              A small footprint and configurable SATA core
                                      powered by Migen & LiteX
```

[![](https://github.com/enjoy-digital/litesata/workflows/ci/badge.svg)](https://github.com/enjoy-digital/litesata/actions) ![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg) [![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/enjoy-digital/litesata)


[> Intro
--------
LiteSATA provides a small footprint and configurable SATA core.

LiteSATA is part of LiteX libraries whose aims are to lower the entry level of
complex FPGA cores by providing simple, elegant and efficient implementations
of components used in today's SoC such as Ethernet, SATA, PCIe, SDRAM Controller...

Using Migen to describe the HDL allows the core to be highly and easily configurable.

LiteSATA can be used as LiteX library or can be integrated with your standard
design flow by generating the Verilog RTL that you will use as a standard core.

<p align="center"><img src="https://github.com/enjoy-digital/litesata/raw/master/doc/architecture.png" width="800"></p>

[> Features
-----------
PHY:
  - Xilinx Artix-7 GTP (`GTPE2_CHANNEL`)
  - Xilinx Kintex-7 GTX (`GTXE2_CHANNEL`)
  - Xilinx UltraScale GTH (`GTHE3_CHANNEL`)
  - Xilinx UltraScale+ GTH/GTY (`GTHE4_CHANNEL`/`GTYE4_CHANNEL`, selected with `gt_type`; Artix UltraScale+ is GTH only)
  - Lattice ECP5 SERDES (`DCUA`, Gen2 only, OOB generated/detected in fabric)
  - OOB, COMWAKE, COMINIT
  - ALIGN inserter/remover and bytes alignment on K28.5
  - 8B/10B encoding/decoding in transceiver
  - Automatic TX/RX P/N polarity detection and swap.
  - Error detection and reporting
  - 16/32-bit PHY datapath
  - 1.5/3.0/6.0Gbps supported speeds (respectively 37.5/75/150MHz system clock)

Core:

 - Link:
    - CONT inserter/remover
    - Scrambling/Descrambling of data
    - CRC inserter/checker
    - HOLD insertion/detection
    - Error detection and reporting
  - Transport/Command:
    - Easy to use user interfaces (Can be used with or without CPU)
    - 48 bits sector addressing
    - 3 supported commands: READ_DMA(_EXT), WRITE_DMA(_EXT), IDENTIFY_DEVICE
    - Error detection and reporting

Frontend:
  - Configurable crossbar (simply declare your crossbar and use crossbar.get_port() to add a new port!)
  - Standalone Verilog core generator (`litesata_gen`) with native user ports and/or a Wishbone/AXI-Lite control bus + Wishbone DMA master
  - Ports arbitration transparent to the user
  - Synthesizable BIST
  - Striping module to segment data on multiple HDDs and increase write/read speed and capacity. (RAID0 equivalent)
  - Mirroring module for data redundancy and increased read speed. (RAID1 equivalent)

[> FPGA Proven
--------------
LiteSATA is already used in commercial and open-source designs:
- High End 50Mpixels CMOS Camera using CFAST & SD cards.
- Low latency database research.

[> Possible improvements
------------------------
- add AXI-Stream user ports / AXI DMA master to the standalone core
- add host-side link rate negotiation (Gen3 -> Gen2 -> Gen1 fallback)
- add FLUSH CACHE / TRIM commands and 4Kn sector support
- add NCQ support
- add AES hardware encryption
- add on-the-fly compression/decompression
- add support for Altera/Intel PHYs.
- add ECP5 Gen1 (1.5Gbps) support and Linux integration.
- add Zynq Linux drivers.
- ... See below Support and consulting :)

If you want to support these features, please contact us at florent [AT]
enjoy-digital.fr.

[> Getting started
------------------
1. Install Python 3.9+ and FPGA vendor's development tools.
2. Install LiteX and the cores by following the LiteX's wiki [installation guide](https://github.com/enjoy-digital/litex/wiki/Installation).
3. You can find examples of integration of the core with LiteX in LiteX-Boards and in the examples directory.

[> Standalone core generation
-----------------------------
LiteSATA can also be generated as a standalone Verilog core from a YAML description, to integrate it
in a traditional flow without Migen/LiteX at build time:
```sh
$ litesata_gen examples/artix7.yml [--output-dir build] [--name litesata_core]
```
The outputs are `build/gateware/litesata_core.v`, the Vivado constraints/`.tcl` and `build/csr.csv`
(register map, for the control/DMA flavour).

YAML keys:

| Key              | Values                                                     | Default      |
|------------------|------------------------------------------------------------|--------------|
| `phy`            | `A7SATAPHY`, `K7SATAPHY`, `GTH3SATAPHY`, `GTH4SATAPHY`, `GTY4SATAPHY` | required |
| `gen`            | `gen1`, `gen2`, `gen3` (fixed link rate)                   | required     |
| `clk_freq`       | system clock frequency (Hz)                                | required     |
| `data_width`     | PHY datapath width, 16 or 32                               | 16           |
| `gt_type`        | `GTH`/`GTY` override (UltraScale+ only)                    | from `phy`   |
| `core`           | `wishbone` or `axi-lite` control bus (CSRs at 0x00000000)  | none         |
| `dma_data_width` | Wishbone DMA master width                                  | 32           |
| `dma_mode`       | `read`, `write`, `read+write`                              | `read+write` |
| `with_identify`, `with_bist`, `with_irq` | frontends/IRQ of the control/DMA flavour | True/False/True |
| `user_ports`     | `{name: {"type": "native", "data_width": 32 or 64}}`       | none         |

PHY names map to transceivers: `A7SATAPHY` Artix-7 GTP, `K7SATAPHY` Kintex-7 GTX, `GTH3SATAPHY`
UltraScale GTH, `GTH4SATAPHY` Artix/Kintex/Zynq UltraScale+ GTH, `GTY4SATAPHY` Kintex/Virtex
UltraScale+ GTY. Without `core`, only native streaming ports are exposed (no CPU needed). With `core`,
the control bus gives access to the PHY status, Identify, optional BIST and the Sector2Mem/Mem2Sector
DMAs (which master the `wb_dma` bus), and `irq` signals DMA completion. The system clock must satisfy
`clk_freq >= linerate / dma_data_width` for the DMA flavour (e.g. 75MHz for Gen3 on a 64-bit bus).
Transceiver location and reference clock pin constraints are added by the integrator. See
`examples/` for native and control/DMA configurations.

[> Tests
--------
Unit tests are available in ./test/.
To run all the unit tests:
```sh
$ python3 -m pytest -v
```

Tests can also be run individually:
```sh
$ python3 -m pytest -v test/test_name.py
```

[> License
----------
LiteSATA is released under the very permissive two-clause BSD license. Under the
terms of this license, you are authorized to use LiteSATA for closed-source
proprietary designs.
Even though we do not require you to do so, those things are awesome, so please
do them if possible:
 - tell us that you are using LiteSATA
 - cite LiteSATA in publications related to research it has helped
 - send us feedback and suggestions for improvements
 - send us bug reports when something goes wrong
 - send us the modifications and improvements you have done to LiteSATA.

[> Support and consulting
-------------------------
We love open-source hardware and like sharing our designs with others.

LiteSATA is developed and maintained by EnjoyDigital.

If you would like to know more about LiteSATA or if you are already a happy
user and would like to extend it for your needs, EnjoyDigital can provide standard
commercial support as well as consulting services.

So feel free to contact us, we'd love to work with you! (and eventually shorten
the list of the possible improvements :)

[> Contact
----------
E-mail: florent [AT] enjoy-digital.fr
