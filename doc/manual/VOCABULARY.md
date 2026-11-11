# Vocabulary of the LiteSATA manual

Project-specific writing rules, completing the generic `STYLE.md` of the manual kit.

## Names of the parts

- *the host* is LiteSATA in the FPGA; *the device* or *the drive* is the HDD or SSD at the other end
  of the cable. LiteSATA is always the host.
- *the PHY* is `LiteSATAPHY`: the transceiver wrapper and its clocking (CRG), *the controller* (the
  OOB and ALIGN state machine of `litesata/phy/ctrl.py`, and its ECP5 subclass in
  `litesata/phy/ecp5sataphy.py`) and *the datapath* (realignment, width conversion, clock domain
  crossing). In this manual "controller" means that PHY controller only,
  never a host controller in the AHCI sense.
- *the core* is `LiteSATACore`: the *link layer*, the *transport layer* and the *command layer*.
- *a user port* is a port of the crossbar (`LiteSATACrossbar.get_port()`); *a frontend* is a module
  on a user port (Identify, BIST, DMA, stream, striping, mirroring).
- *the BIOS* is the LiteX BIOS and its `liblitesata`; *the generator* is `litesata_gen`; *target* is
  a script of `bench/`; *example* is a YAML file of `examples/`.
- *register* for a CSR of the SoC (the PHY `enable` and `status`, the BIST `start`), never "CSR" in
  prose, except in names of files and tools (`csr.csv`).

## Protocol names

- Generations as Gen1, Gen2, Gen3 (no space), with line rates `1.5 Gbps`, `3 Gbps`, `6 Gbps`; byte
  rates in MiB/s or MB/s exactly as the tool that printed them; clock frequencies in MHz.
- OOB signals in capitals: COMRESET, COMINIT, COMWAKE; *OOB* for out-of-band signalling, *burst* and
  *gap* for its two states; *UI* for a unit interval (one bit time), in Gen1 UI unless stated.
- Primitives in capitals, with underscores as the specification writes them: ALIGN, SYNC, CONT,
  X_RDY, R_RDY, R_IP, R_OK, R_ERR, HOLD, HOLDA, SOF, EOF, WTRM. Write "an ALIGN primitive" or
  "ALIGN", not "ALIGN_P".
- 8b/10b characters as K28.5, K28.3, D10.2, D21.5; *comma* for K28.5.
- *dword* is 32 bits (one primitive or one data word on the link); *word* is 16 bits, used when
  quoting ATA (IDENTIFY words); *sector* is 512 bytes; *LBA* is a 48-bit logical block address.
- FIS names with their direction: "Register H2D FIS", "Register D2H FIS", "DMA Activate FIS",
  "PIO Setup FIS", "Data FIS", "Set Device Bits FIS"; *H2D* is host to device, *D2H* device to host.
- ATA commands in capitals as ATA/ACS names them: READ DMA EXT, WRITE DMA EXT, IDENTIFY DEVICE;
  status bits BSY, DRDY, DF, DRQ, ERR. Cite SATA or ATA/ACS sections only when sure.

## Placeholders

- `<target>` is a target script name of `bench/` without `.py`; `<config>` is a generator YAML file
  of `examples/`. Both are defined once in the Conventions of the front matter.

## Hardware names

- Transceivers as AMD/Xilinx name them: GTP (Artix-7), GTX (Kintex-7), GTH (UltraScale, the
  `gth3` PHY), GTH and GTY (UltraScale+, the `gth4` and `gty4` PHYs); Lattice ECP5 *DCU* (DCUA
  primitive).
- FPGA families: Artix-7, Kintex-7, Kintex UltraScale, Kintex/Virtex/Zynq UltraScale+, Lattice ECP5.
- Boards as LiteX-Boards names them (Acorn CLE-215+, ECPIX-5, Genesys2, KC705, KCU105, Nexys Video,
  XCU1525, ZCU102).
