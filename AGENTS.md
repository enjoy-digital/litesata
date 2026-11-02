# AGENTS.md: LiteSATA

SATA host core for LiteX: PHYs on transceivers (Xilinx GTP/GTX/GTH/GTY, Lattice ECP5), link,
transport and command layers, BIST, DMA (Block2Mem/Mem2Block) and a standalone generator.

Read the LiteX agent page first: https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents (framework map, conventions, how to
verify a change, contribute back). Index of the whole wiki for agents: https://github.com/enjoy-digital/litex/wiki/llms.txt.

## Where things are

- `litesata/phy/`: transceiver PHYs and OOB (COMINIT/COMWAKE) handling; `litesata/core/`: link/transport/command layers.
- `litesata/frontend/`: arbitration, BIST, DMA; `litesata/gen.py` (`litesata_gen`): standalone core.
- `bench/`: hardware benches (Acorn, ECPIX-5, Genesys2, KC705, KCU105, Nexys Video); `test/`.

## Verify a change

```sh
python3 -m pytest -v                                                # what CI runs
python3 -m litex_boards.targets.<board> --with-sata --build --no-compile
```

- OOB depends on the transceiver: Xilinx GTP/GTX have hardware COMINIT/COMWAKE; on ECP5 the OOB
  gaps are made of data patterns (no usable electrical idle). Test both families when touching OOB.
- A plain TX→RX loopback can never complete an OOB handshake: test the post-OOB datapath with the
  OOB bypass control.
- Drives differ: keep a known-good drive as reference and say which drive was used.

## Rules

- Style: https://github.com/enjoy-digital/litex/blob/master/doc/coding_style.md (match the
  surrounding file; `module: Description of the change.` commit titles; one change per commit).
- Never edit generated files (`build/`, `csr.h`, `csr.csv`, the Verilog): change the Python.
- One topic per pull request, from the default development branch, with what was tested and on
  which hardware; say that an agent wrote it and who supervised. Do not rename options or
  reformat unrelated code.
- Hardware: never flash a board or write its SPI flash unless the user asked; `--load` (volatile)
  is the default for experiments.

## Debug on hardware

Method: https://github.com/enjoy-digital/litex/wiki/Debug-A-SoC-On-Hardware (check the link and `--ident` first, registers before
traces, narrow LiteScope captures). `litex_mcp` gives the same loop as MCP tools (CSRs by name,
LiteScope captures, builds with parsed timing, simulation): https://github.com/enjoy-digital/litex/wiki/LiteX-for-AI-Agents#drive-a-soc-through-mcp.
