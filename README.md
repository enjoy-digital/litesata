```
                                    __   _ __      _______ _________
                                   / /  (_) /____ / __/ _ /_  __/ _ |
                                  / /__/ / __/ -_)\ \/ __ |/ / / __ |
                                 /____/_/\__/\__/___/_/ |_/_/ /_/ |_|

                                  Copyright 2014-2020 / EnjoyDigital
                                     Copyright 2014-2015 / HKU

                              A small footprint and configurable SATA core
                                      powered by Migen & LiteX
```

[![](https://github.com/enjoy-digital/litesata/workflows/ci/badge.svg)](https://github.com/enjoy-digital/litesata/actions) ![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)


[> Intro
--------
LiteSATA provides a small footprint and configurable SATA core.

LiteSATA is part of LiteX libraries whose aims are to lower entry level of
complex FPGA cores by providing simple, elegant and efficient implementations
of components used in today's SoC such as Ethernet, SATA, PCIe, SDRAM Controller...

Using Migen to describe the HDL allows the core to be highly and easily configurable.

LiteSATA can be used as LiteX library or can be integrated with your standard
design flow by generating the verilog rtl that you will use as a standard core.

<p align="center"><img src="https://github.com/enjoy-digital/litesata/raw/master/doc/architecture.png" width="800"></p>

[> Features
-----------
PHY:
  - Xilinx 7-Series (Kintex7, Artix7)
  - Xilinx Ultrascale
  - Xilinx Ultrascale+
  - OOB, COMWAKE, COMINIT
  - ALIGN inserter/remover and bytes alignment on K28.5
  - 8B/10B encoding/decoding in transceiver
  - Automatic TX/RX P/N polarity detection and swap.
  - Errors detection and reporting
  - 32 bits interface
  - 1.5/3.0/6.0GBps supported speeds (respectively 37.5/75/150MHz system clk)

Core:

 - Link:
    - CONT inserter/remover
    - Scrambling/Descrambling of data
    - CRC inserter/checker
    - HOLD insertion/detection
    - Errors detection and reporting
  - Transport/Command:
    - Easy to use user interfaces (Can be used with or without CPU)
    - 48 bits sector addressing
    - 3 supported commands: READ_DMA(_EXT), WRITE_DMA(_EXT), IDENTIFY_DEVICE
    - Errors detection and reporting

Frontend:
  - Configurable crossbar (simply declare your crossbar and use crossbar.get_port() to add a new port!)
  - Ports arbitration transparent to the user
  - Synthetizable BIST
  - Striping module to segment data on multiple HDDs and increase write/read speed and capacity. (RAID0 equivalent)
  - Mirroring module for data redundancy and increase read speeds. (RAID1 equivalent)

[> FPGA Proven
--------------
LiteSATA is already used in commercial and open-source designs:
- High End 50Mpixels CMOS Camera using CFAST & SD cards.
- Low latency database research.

[> Possible improvements
------------------------
- add standardized interfaces (AXI, Avalon-ST)
- add NCQ support
- add AES hardware encryption
- add on-the-flow compression/decompression
- add support for Altera PHYs.
- add support for Lattice PHYs.
- add Zynq Linux drivers.
- ... See below Support and consulting :)

If you want to support these features, please contact us at florent [AT]
enjoy-digital.fr.

[> Getting started
------------------
1. Install Python 3.6+ and FPGA vendor's development tools.
2. Install LiteX and the cores by following the LiteX's wiki [installation guide](https://github.com/enjoy-digital/litex/wiki/Installation).
3. You can find examples of integration of the core with LiteX in LiteX-Boards and in the examples directory.

[> Tests
--------
Unit tests are available in ./test/.
To run all the unit tests:
```sh
$ ./setup.py test
```

Tests can also be run individually:
```sh
$ python3 -m unittest test.test_name
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
