# Standalone core generation

LiteSATA is normally used as a Python package inside a LiteX SoC. For a traditional flow (no
Migen/LiteX at build time, versioned/packaged core, ...), a standalone Verilog core can be generated
from a YAML description with `litesata_gen` (`python3 -m litesata.gen`):
```sh
$ litesata_gen examples/artix7.yml [--output-dir build] [--name litesata_core]
```
The outputs are `build/gateware/litesata_core.v`, the Vivado constraints/`.tcl` and `build/csr.csv`
(register map, for the control/DMA flavour).

## YAML keys

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

## PHYs and flavours

PHY names map to transceivers: `A7SATAPHY` Artix-7 GTP, `K7SATAPHY` Kintex-7 GTX, `GTH3SATAPHY`
UltraScale GTH, `GTH4SATAPHY` Artix/Kintex/Zynq UltraScale+ GTH, `GTY4SATAPHY` Kintex/Virtex
UltraScale+ GTY. Without `core`, only native streaming ports are exposed (no CPU needed). With `core`,
the control bus gives access to the PHY status, Identify, optional BIST and the Sector2Mem/Mem2Sector
DMAs (which master the `wb_dma` bus), and `irq` signals DMA completion. The system clock must satisfy
`clk_freq >= linerate / dma_data_width` for the DMA flavour (e.g. 75MHz for Gen3 on a 64-bit bus).
Transceiver location and reference clock pin constraints are added by the integrator. See
`examples/` for native and control/DMA configurations.
