# Build and generator options {#build-options}

This appendix lists the parameters and options of every part of the project: the PHY and its
transceiver classes, the core and the frontends, the `add_sata` method of LiteX, the generator,
the bench targets and the constants of the BIOS. The chapters explain when to use them; this
appendix gives their names and defaults.

## PHY {#opt-phy}

`LiteSATAPHY` (`litesata/phy/__init__.py`) selects the transceiver PHY from the device name and
adds the controller, the datapath, the link monitor and the registers.

| Parameter | Default | Meaning |
|----------------------------|------------|--------------------------------------------------------|
| `device` | required | FPGA device name, as `platform.device`; selects the PHY |
| `pads` | required | Transceiver pads: `tx_p`, `tx_n`, `rx_p`, `rx_n`, and `clk_p`, `clk_n` without `refclk` |
| `gen` | required | `"gen1"`, `"gen2"` or `"gen3"`: the fixed line rate |
| `clk_freq` | required | System clock frequency in Hz, for the timers of the controller |
| `refclk` | `None` | 150 MHz reference clock signal; `None` takes it from `clk_p`, `clk_n` |
| `data_width` | `16` | Transceiver datapath: 16 or 32 bits (ECP5: 16 only) |
| `qpll` | `None` | Artix-7: an external `GTPQuadPLL` to share with another core |
| `gt_type` | `"GTY"` | UltraScale+: `"GTY"` or `"GTH"`; forced to `"GTH"` on Artix UltraScale+; pass `"GTH"` on Zynq UltraScale+ and on Kintex UltraScale+ GTH quads |
| `use_gtgrefclk` | `True` | UltraScale+: reference clock from the fabric (`True`) or a dedicated pin (`False`) |
| `with_csr` | `True` | Adds the `enable` and `status` registers |
| `dual`, `channel` | `0`, `0` | ECP5: DCU and channel of the transceiver |
| `link_down_timeout` | `100e-3` | Seconds without `ready` before `link_up` falls |

The device name selects the PHY as below; any other device raises `NotImplementedError`.

| Device name pattern | PHY class | Transceiver |
|--------------------------------------------|----------------------------|----------------------------|
| `^xc7a` | `A7LiteSATAPHY` | GTP, TX buffer on |
| `^xc7k` | `K7LiteSATAPHY` | GTX |
| `^xc[kv]u[0-9]+`, no `p` after | `GTH3LiteSATAPHY` | GTH (UltraScale) |
| `^xc[akv]u[0-9]+p`, `^xczu[0-9]` | `GTY4LiteSATAPHY`, `GTH4LiteSATAPHY` | GTY or GTH, from `gt_type` |
| `^LFE5UM5G-` | `ECP5LiteSATAPHY` | DCU, Gen2 only |

The patterns follow the regular expressions of `LiteSATAPHY`, matched on the start of the device
name.

The transceiver classes take `pads`, `gen`, `clk_freq` and `data_width`. The Xilinx classes add
`tx_buffer_enable` and `rx_buffer_enable` (default `False`), which `LiteSATAPHY` leaves at their
defaults except for the TX buffer of the Artix-7. `ECP5LiteSATAPHY` adds `refclk_freq` (150 MHz).
The legacy names `USLiteSATAPHY`, `USPLiteSATAPHY` and `GTHE4LiteSATAPHY` remain as aliases of the
GTH3, GTY4 and GTH4 classes.

## Core and frontends {#opt-core}

| Class | Parameters | Notes |
|--------------------------------------|---------------------------------|-----------------------------|
| `LiteSATACore` | `phy` | Link, transport and command layers |
| `LiteSATACrossbar` | `controller` (a core) | Gives the user ports |
| `LiteSATAIdentify` | `user_port` | Registers in a separate class |
| `LiteSATABIST` | `crossbar`, `with_csr=False`, `count_width=32` | Generator, checker and Identify, three ports |
| `LiteSATASector2MemDMA` | `port`, `bus`, `endianness="little"` | Wishbone master, registers, `irq` |
| `LiteSATAMem2SectorDMA` | `bus`, `port`, `endianness="little"` | Wishbone master, registers, `irq` |
| `LiteSATASectors2Stream` | `port`, `data_width=64` | Not exported by the package |
| `LiteSATAStream2Sectors` | `port`, `data_width=64` | Not exported by the package |
| `LiteSATAStriping` | `controllers` (list of cores) | Acts as one core of N times 32 bits |
| `LiteSATAMirroring` | `controllers` (list of cores) | One port per core, in `ports` |

`get_port(dw=None)` of the crossbar returns a user port of the core width, 32 bits, or of a
multiple of it such as 64 bits, and converts the width; `get_ports(n, dw=None)` returns `n` of
them. `LiteSATAIdentifyCSR` adds the registers of an Identify frontend. The stream frontends are
imported from `litesata.frontend.stream`. [Frontends](#frontends) describes each class.

## LiteX integration {#opt-add-sata}

`add_sata` of the LiteX SoC (`litex/soc/integration/soc.py`) adds the core and its frontends
around a PHY created by the caller.

| Parameter | Default | Meaning |
|---------------------|---------------------|------------------------------------------------------|
| `name` | `"sata"` | Prefix of the modules and register banks; the BIOS expects `sata` |
| `phy` | required | The `LiteSATAPHY`, named `sata_phy` for the BIOS |
| `mode` | `"read+write"` | `"read"` adds Sector2Mem, `"write"` adds Mem2Sector |
| `with_identify` | `True` | Identify frontend and its registers |
| `with_bist` | `False` | BIST frontend and its registers |
| `with_irq` | `True` | Event manager with one event per DMA |

`add_sata` requires a system clock of at least the clock of a 32-bit PHY, half the 16-bit SATA
clock: 37.5 MHz, 75 MHz and 150 MHz for Gen1, Gen2 and Gen3, whatever the width of the bus and of
the PHY ([System clock](#perf-sysclk)). The DMAs master the DMA bus of the SoC when it has one,
else the main bus.

## Generator {#opt-gen}

`litesata_gen` (`litesata/gen.py`, or `python3 -m litesata.gen`) takes one YAML file.

| Option | Default | Effect |
|------------------|--------------------|--------------------------------------------|
| `config` | required | YAML file of the core |
| `--output-dir` | `build` | Output directory |
| `--name` | `litesata_core` | Name of the generated module and files |

The keys of the YAML file are below. The strings `"True"`, `"False"` and `"None"` are converted to
their Python values; unknown keys are ignored.

| Key | Default | Meaning |
|------------------------|-----------------|-------------------------------------------------------|
| `phy` | required | `A7SATAPHY`, `K7SATAPHY`, `GTH3SATAPHY`, `GTH4SATAPHY` or `GTY4SATAPHY` |
| `gen` | required | `gen1`, `gen2` or `gen3` |
| `clk_freq` | required | System clock frequency in Hz |
| `data_width` | `16` | Transceiver datapath, 16 or 32 |
| `gt_type` | from `phy` | Fixed by the PHY name: GTH for `GTH3SATAPHY`, `GTH4SATAPHY`, GTY for `GTY4SATAPHY`; a different value is an error. Unused with `A7SATAPHY`, `K7SATAPHY` |
| `core` | none | `wishbone` or `axi-lite`: control/DMA flavour; none: native only |
| `dma_data_width` | `32` | Width of the Wishbone DMA master |
| `dma_mode` | `read+write` | `read`, `write` or `read+write`, as `mode` of `add_sata` |
| `with_identify` | `True` | Identify frontend (control/DMA flavour) |
| `with_bist` | `False` | BIST frontend (control/DMA flavour) |
| `with_irq` | `True` | `irq` output (control/DMA flavour) |
| `user_ports` | none | Native user ports, by name, as in the block below |

```yaml
"user_ports": {
    "native_0" : {
        "type": "native",
        "data_width": 32,
    },
},
```

Each port has `type: native`, the only type, and a `data_width` of 32 or 64 bits. Its signals
are named `user_port_<name>_sink_*` and `user_port_<name>_source_*` in the generated core.

The PHY names stand for a device of each family: `xc7a`, `xc7k`, `xcku040`, `xcau25p` and `xcvu9p`.
The generated core does not depend on the exact device. ECP5 is not offered by the generator. The
control/DMA flavour goes through `add_sata` and checks the system clock as it does; the native
flavour does not check it. Both feed the reference clock of the `sata` pins to the dedicated
transceiver input: the generator builds the PHY with `use_gtgrefclk=False`.

## Bench targets {#opt-bench}

Every target of `bench/` takes `--build` (run the toolchain) and `--load` (load the bitstream into
the FPGA, not the flash). Without `--build`, it elaborates the SoC and writes `csr.csv`, except
`zcu102`, which does not elaborate with the LiteX of this manual ([Bench targets](#ex-bench)).

| Target | Options |
|--------------------|------------------------------------------------------------------------------|
| `acorn` | `--variant` (`cle-101`, `cle-215`, `cle-215+`), `--sys-clk-freq` (100e6), `--gen` (1), `--with-analyzer` |
| `ecpix5` | `--device` (`85F`, `45F`), `--seed` (3) |
| `genesys2` | `--gen` (3), `--with-analyzer` |
| `kc705` | `--gen` (3), `--connector` (`fmc`, `sfp`, `pcie`), `--with-global-analyzer`, `--with-sector2mem-analyzer`, `--with-mem2sector-analyzer` |
| `kcu105` | `--gen` (3), `--connector` (`fmc`, `sfp`, `pcie`), `--with-analyzer` |
| `nexys_video` | `--gen` (2, max 2), `--pll-refclk`, `--with-analyzer` |
| `xcu1525` | `--gen` (3), `--connector` (`qsfp`, `pcie`), `--with-analyzer` |
| `zcu102` | `--gen` (3), `--dw` (32), `--refclk` (`external`, `internal`), `--with-analyzer`, `--analyzer-csv`, and the LiteX target options |

The options of the host scripts are in [Host scripts](#ex-scripts).

## BIOS constants {#opt-bios}

The SATA code of the LiteX BIOS has C macros with a default, overridden by defining them when the
BIOS is compiled.

| Macro | Default | Effect |
|-----------------------------|-----------|-----------------------------------------------------|
| `SATA_OP_RETRIES` | 8 | Attempts of each `sata_read` and `sata_write` call of `liblitesata` |
| `SATA_OP_TIMEOUT_US` | 100000 | Polls of the DMA `done` register per attempt, about 1 µs each |
| `SATA_BOOT_PRIORITY` | 40 | Priority of the SATA boot method among the BIOS boot methods |
| `SATA_BOOT_DISABLE` | undefined | Removes the SATA boot method |

The commands and the boot method exist only when the SoC has the matching register banks: the PHY
for `sata_init`, Sector2Mem for `sata_read`, `sata_sec2mem` and the boot from SATA, Mem2Sector for
`sata_write` and `sata_mem2sec`, both DMAs for `sata_rwtest`.
