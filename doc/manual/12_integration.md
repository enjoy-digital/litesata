# Integration {#integration}

This chapter explains how to add LiteSATA to your own design. It starts with a LiteX SoC, the
native way, then covers the clocks, the constraints, the BIOS support and the ports for user
logic. It ends with the standalone Verilog core for designs built without LiteX, and a checklist
to port LiteSATA to a new board.

## At a glance {#integ-glance}

| Item | Value |
|---------------------------|-----------------------------------------------------------------|
| Language | Python (Migen, LiteX); Verilog through `litesata_gen` |
| LiteX helper | `add_sata` of the LiteX SoC class |
| Built by the design | `LiteSATAPHY`: the PHY, its reference clock and its pins |
| Built by `add_sata` | Core, crossbar, Identify, two DMAs, interrupts, constraints; BIST on request |
| User interface | User ports of the crossbar; DMAs and Identify through registers |
| Software | LiteX BIOS commands, `liblitesata`, FatFs boot from the drive |
| Reference designs | LiteX-Boards targets, `bench/`, `examples/` |

## Integration in a LiteX SoC {#integ-litex}

A LiteX SoC with SATA is a chain of parts, shown in the figure below. The two DMAs move sectors
between the drive and the memory of the SoC, and the Identify module reads the IDENTIFY DEVICE
answer for the software. All three take a user port of the crossbar, which shares the core
between them. The core and the PHY carry the commands to the drive.

![LiteSATA in a LiteX SoC. add_sata builds everything inside the frame from a PHY that the target
creates; the DMAs master the bus of the SoC, Identify is read through registers. The optional
BIST takes three more crossbar ports: generator, checker and its own Identify.](integ_soc.svg)

The `add_sata` method of the LiteX SoC class builds this chain from a PHY. The example below is
the SATA part of the KC705 target of LiteX-Boards: a Kintex-7 GTX at Gen2, on the TE0424
SFP-to-SATA adapter, with the reference clock from the PLL of the clock generator.

```python
from litex.soc.integration.soc_core import SoCCore

from litesata.phy import LiteSATAPHY

class MySoC(SoCCore):
    def __init__(self, platform, sys_clk_freq=125e6, **kwargs):
        # Your clock generator, with its PLL in self.crg.pll.
        self.crg = _CRG(platform, sys_clk_freq)
        SoCCore.__init__(self, platform, sys_clk_freq, ident="My SoC", **kwargs)

        # 150 MHz reference clock of the transceiver, from the PLL.
        self.cd_sata_refclk = ClockDomain()
        self.crg.pll.create_clkout(self.cd_sata_refclk, 150e6)
        platform.add_platform_command(
            "set_property SEVERITY {{Warning}} [get_drc_checks REQP-52]")

        # PHY: selected from the device name, Gen2, 16-bit transceiver interface.
        self.sata_phy = LiteSATAPHY(platform.device,
            refclk     = ClockSignal("sata_refclk"),
            pads       = platform.request("sfp2sata"),
            gen        = "gen2",
            clk_freq   = sys_clk_freq,
            data_width = 16)

        # Core, crossbar, Identify, DMAs, interrupts and timing constraints.
        self.add_sata(phy=self.sata_phy, mode="read+write")
```

Name the PHY `sata_phy` and keep the default `name` of `add_sata`: the BIOS looks for the
`sata_phy`, `sata_identify` and DMA register banks by these names. The parameters of `add_sata`
are below.

| Parameter | Default | Use |
|--------------------|-------------------|-----------------------------------------------|
| `name` | `"sata"` | Prefix of the modules and register banks |
| `phy` | required | The `LiteSATAPHY` instance |
| `mode` | `"read+write"` | `"read"`: Sector2Mem only; `"write"`: Mem2Sector only |
| `with_identify` | `True` | Identify module and its registers |
| `with_bist` | `False` | BIST generator, checker and their own Identify |
| `with_irq` | `True` | Interrupt controller of the DMAs |

`add_sata` checks the generation of the PHY, the mode and the system clock, then adds the modules
below. Each one with registers is a register bank of the same name; [Register
reference](#registers) lists their registers.

| Module | Class | Role |
|----------------------|-------------------------------|---------------------------------|
| `sata_core` | `LiteSATACore` | Link, transport and command layers |
| `sata_crossbar` | `LiteSATACrossbar` | User ports of the core |
| `sata_identify` | `LiteSATAIdentifyCSR` | IDENTIFY DEVICE answer, through a FIFO |
| `sata_bist` | `LiteSATABIST` | Write and read-check test |
| `sata_sector2mem` | `LiteSATASector2MemDMA` | Drive to memory, one sector per command |
| `sata_mem2sector` | `LiteSATAMem2SectorDMA` | Memory to drive, one sector per command |
| `sata_irq` | `EventManager` | Events `sector2mem_dma`, `mem2sector_dma` |

The DMAs are Wishbone masters as wide as the bus of the SoC, in the endianness of the CPU. They
go on the DMA bus of the SoC when it has one, a bus into the memory of the CPU, and on the main
bus otherwise. Each DMA interrupt is a pulse at the end of a transfer, on completion or on
error; the CPU reads `done` and `error` of the DMA to tell them apart. [Frontends](#frontends)
describes each module.

A few things are left to the design. `add_sata` does not connect the link events of the PHY
(`connect_irq`, `disconnect_irq`) to an interrupt; the `link_up` bit of the `status` register of
the PHY gives the same information by polling.

The system clock check of `add_sata` takes the clock of a 16-bit PHY times 16 and divides it by
the width of the bus, while the core path is 32 bits wide whatever the bus. On a 64-bit bus it
accepts half the floor of [Generation, width and system clock](#integ-clocks), on a 16-bit bus it
asks for twice that floor. Keep the system clock at that floor whatever the bus width. The
control/DMA flavour of `litesata_gen` makes the same check with `dma_data_width` as the bus width.

## Without add_sata: the explicit chain {#integ-explicit}

The bench targets build the chain by hand, without a CPU, to keep the SoC small and to choose
each frontend. The pattern below comes from `bench/kc705.py`; [Example designs and
targets](#examples) describes the other targets.

```python
from litesata.phy                  import LiteSATAPHY
from litesata.core                 import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar
from litesata.frontend.bist        import LiteSATABIST

# PHY, then the core on top of it, then the crossbar on top of the core.
self.sata_phy      = LiteSATAPHY(platform.device, pads=platform.request("fmc2sata"),
    gen="gen3", clk_freq=sys_clk_freq, data_width=16)
self.sata_core     = LiteSATACore(self.sata_phy)
self.sata_crossbar = LiteSATACrossbar(self.sata_core)

# Frontends, each on ports of the crossbar.
self.sata_bist = LiteSATABIST(self.sata_crossbar, with_csr=True)

# Timing constraints: SATA clocks of a 16-bit PHY at Gen3, and false paths to sys.
platform.add_period_constraint(self.sata_phy.crg.cd_sata_tx.clk, 1e9/300e6)
platform.add_period_constraint(self.sata_phy.crg.cd_sata_rx.clk, 1e9/300e6)
platform.add_false_path_constraints(self.crg.cd_sys.clk,
    self.sata_phy.crg.cd_sata_tx.clk, self.sata_phy.crg.cd_sata_rx.clk)
```

`LiteSATABIST` requests three ports of the crossbar: one for its generator, one for its checker
and one for its Identify. A DMA takes one port and a Wishbone interface that the target adds to
the bus as a master. With `with_csr=True`, the BIST registers are in the `sata_bist` bank that
`bench/test_bist.py` drives.

## Generation, width and system clock {#integ-clocks}

The generation sets the line rate, and the line rate sets every clock of the PHY. With 8b/10b
coding, each byte takes 10 bits on the line, so a 32-bit dword takes 40: the core path, 32 bits
wide in the system clock domain, needs a dword every 40 bits of line rate. The table below gives
the clocks; [Core architecture](#core-architecture) shows the clock domains.

| Gen | Line rate | `sata_tx`, `sata_rx`, 16-bit PHY | 32-bit PHY | Minimum system clock |
|------|-----------|-------------------|------------|-----------|
| Gen1 | 1.5 Gbps | 75 MHz | 37.5 MHz | 37.5 MHz |
| Gen2 | 3 Gbps | 150 MHz | 75 MHz | 75 MHz |
| Gen3 | 6 Gbps | 300 MHz | 150 MHz | 150 MHz |

The system clock must be at least the clock of a 32-bit PHY, whatever the width of the PHY. The
datapath crosses from the transceiver clocks to the system clock through small asynchronous
FIFOs, and the link sends a dword in every slot, primitives included: a slower system clock
overflows them. The bench targets run with margin, from 90 MHz at Gen2 to 200 MHz at Gen3.

The PHY width, `data_width`, trades transceiver clock speed for fabric width. A 16-bit PHY at Gen3
runs its user clocks at 300 MHz, which the slower speed grades may not reach; a 32-bit PHY
halves them. The ECP5 PHY is 16-bit only. The generation is fixed when the design is
built: the core does not fall back to a lower rate when the drive or the cable cannot follow.

## Reference clock {#integ-refclk}

Every PHY needs a 150 MHz reference clock. It comes in one of two ways, selected by the `refclk`
argument of `LiteSATAPHY`.

- **Dedicated pins**, `refclk=None`: the PHY takes the clock from the `clk_p` and `clk_n`
  subsignals of the pads, through the differential transceiver clock buffer of the family. This
  is the path of an FMC adapter that drives `GBTCLK0`.
- **Fabric clock**, `refclk=ClockSignal("sata_refclk")`: a PLL of the FPGA makes the clock, and
  the PHY routes it to the transceiver. This is the path of every board without a 150 MHz
  oscillator on transceiver clock pins.

On 7-Series, a fabric reference clock trips a design rule check of Vivado, which the targets
downgrade to a warning: `REQP-49` for the Artix-7 GTP, `REQP-52` for the Kintex-7 GTX. On
UltraScale+, `use_gtgrefclk` selects the transceiver input. Its default, `True`, takes the clock
on `GTGREFCLK`, the fabric input; set it to `False` for a clock on the dedicated pins, as
`bench/zcu102.py` does for its FMC clock.

On Artix-7, `qpll` gives the PHY a channel of a quad PLL built by the design, instead of a PLL of
its own: this is how a SATA PHY and another GTP user share a quad. On ECP5, the target passes the
150 MHz clock of a PLL, and `dual` and `channel` select the DCU and its channel.

## Timing constraints {#integ-constraints}

A SATA design needs a period constraint on the TX and RX clocks of the PHY and false paths between
them and the system clock: the datapath crosses between these domains through asynchronous FIFOs.
`add_sata` adds both, with the clock of a 16-bit PHY; the bench targets add them by hand.

With a 32-bit PHY, `add_sata` therefore constrains the SATA clocks at twice their frequency, and
LiteX refuses a second, different period on the same clock. Use a 16-bit PHY with `add_sata`, or
build the explicit chain with the constraints of a 32-bit PHY, as `bench/zcu102.py` does. The
control/DMA flavour of `litesata_gen` with `data_width: 32` has the same over-constraint. The pins
of the transceiver need no IO constraint beyond their location, which the platform gives.

## BIOS commands {#integ-bios}

The LiteX BIOS has commands for SATA, compiled in when the SoC has the register banks they use.
`add_sata` with its defaults gives them all.

| Command | Needs | Action |
|------------------|------------|----------------------------------------------------|
| `sata_init` | `sata_phy` | Reset the PHY, wait for the link, run IDENTIFY DEVICE |
| `sata_read` | Sector2Mem | Read one sector and dump it |
| `sata_sec2mem` | Sector2Mem | Read sectors into memory |
| `sata_write` | Mem2Sector | Fill one sector with a string and write it |
| `sata_mem2sec` | Mem2Sector | Write sectors from memory |
| `sata_rwtest` | both DMAs | Write sectors, read them back and compare |
| `sataboot` | Sector2Mem | Boot from a FAT file system on the drive |

Their arguments, as the BIOS prints them when one is missing:

```text
sata_read <sector>
sata_sec2mem <sector> <dst_addr> [count]
sata_write <sector> <str>
sata_mem2sec <src_addr> <sector> [count]
sata_rwtest <sector> <address> <count> <str>
sataboot [manifest]
```

`sata_write` and `sata_rwtest` overwrite sectors of the drive, and `sata_rwtest` also uses
`2*512*count` bytes of memory at `address`.

`sata_init` makes up to 16 attempts. Each one toggles the `enable` register of the PHY, waits
100 ms, checks the `ready` bit of its `status` register, starts Identify and waits another
100 ms. On success, it prints the model of the drive and its capacity in GB; [Step 8 of Getting
started](#gs-bios) shows real captures. It also reads the registers of Identify, so a BIOS with
SATA needs `with_identify=True`.

The commands rest on `liblitesata`, which a firmware of your own can call too. `sata_init(show)`
brings the drive up; `sata_read(sector, count, buf)` and `sata_write(sector, count, buf)` run a DMA
and return 0 on success. Each transfer is retried up to `SATA_OP_RETRIES` times (8) and each
attempt waits at most `SATA_OP_TIMEOUT_US` (100 ms), so a missing drive cannot hang the BIOS.
A retry helps after an error, not after a timeout: a DMA that has not finished ignores a new
`start`, so the BIOS gives up and the core stays blocked until it is reset. Without a DMA bus,
`sata_read` flushes the data caches of the CPU after the transfer.

## Booting from SATA {#integ-boot}

`sataboot` mounts the FAT file system of the drive with FatFs and loads from it, like the SD card
boot. It first looks for `boot.json`, a manifest that lists the files to load and their
addresses, then for `boot.bin`, which it copies to the start of the main RAM and runs.
`sataboot <manifest>` names another manifest.

At power-up, the BIOS tries its boot methods in order of priority. SATA comes after serial,
flash, ROM and SD card, and before Ethernet; a method that finds nothing to load hands over to
the next one. Two compile-time defines change this, set as SoC constants with `add_constant`:

| Define | Effect |
|-------------------------|-------------------------------------------------------------|
| `SATA_BOOT_PRIORITY` | Priority of the SATA boot, 40 by default; lower runs first |
| `SATA_BOOT_DISABLE` | Remove SATA from the automatic boot sequence |

The FatFs access is read-only: the BIOS reads files from the drive but never writes to its file
system. Prepare the partition and the files on a host computer.

## User ports in user logic {#integ-user-ports}

User logic reaches the drive through a port of the crossbar. `get_port()` returns a port as wide
as the core, 32 bits; `get_port(64)` returns a 64-bit port, converted to 32 bits inside the
crossbar. Ports are served in turn, and a port keeps the core from its command to the last beat
of its response. The command layer does not enforce one command at a time: user logic waits for
the response beat with `end` before it sends its next command, as every frontend does. [Core
architecture](#core-architecture) gives the full contract of a port.

A port has a `sink` for commands and a `source` for responses. A write command carries `write`,
the first `sector` and the sector `count` on every beat, with the data of the sectors and `last`
on the final beat. The response is one beat with `write`, `end` and `failed`. The module below
writes one sector of a counter on a 32-bit port.

```python
from migen import *

from litex.gen import *

class SectorWriter(LiteXModule):
    def __init__(self, port):
        self.start  = Signal()
        self.sector = Signal(48)
        self.done   = Signal()
        self.failed = Signal()

        # # #

        count = Signal(7) # 128 dwords in a 512-byte sector.
        self.comb += [
            port.sink.write.eq(1),
            port.sink.sector.eq(self.sector),
            port.sink.count.eq(1),
            port.sink.data.eq(count),
            port.sink.last.eq(count == 127),
        ]
        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            If(self.start, NextValue(count, 0), NextState("SEND"))
        )
        fsm.act("SEND",
            port.sink.valid.eq(1),
            If(port.sink.ready,
                NextValue(count, count + 1),
                If(port.sink.last, NextState("WAIT"))
            )
        )
        fsm.act("WAIT",
            port.source.ready.eq(1),
            If(port.source.valid & port.source.last & port.source.end,
                NextValue(self.failed, port.source.failed),
                NextState("IDLE")
            )
        )

# In the SoC, after add_sata:
self.writer = SectorWriter(self.sata_crossbar.get_port())
```

A read command is one beat with `read`, `sector`, `count` and `last`. The data comes back on the
`source` in beats with `read`, with `last` at the end of each Data FIS. A final beat with `last`
and `end` closes the response; `failed` on it means that the drive reported an error, that a
received Data FIS had a CRC error or that the data was short. The command layer has no timeout: a
design that must not wait forever adds its own.

## Several drives: striping and mirroring {#integ-raid}

Each drive has its own PHY and its own core. The striping and mirroring modules sit between the
cores and the crossbar, as the figure below shows for striping over two drives. The crossbar then
works on the striping port, as wide as all the cores together: 64 bits for two drives.

![Striping over two drives: one PHY and one core per drive, the striping module on the two cores,
and one crossbar on top of it, 64 bits wide.](integ_raid.svg)

```python
from litesata.frontend.raid import LiteSATAStriping

self.sata_core0    = LiteSATACore(self.sata_phy0)
self.sata_core1    = LiteSATACore(self.sata_phy1)
self.sata_striping = LiteSATAStriping([self.sata_core0, self.sata_core1])
self.sata_crossbar = LiteSATACrossbar(self.sata_striping)
```

`LiteSATAMirroring` takes the same list of cores and gives one port per drive, in `ports`; give
each port a crossbar of its own. [Frontends](#frontends) explains the capacity and the throughput
of both modules, and their limits. `add_sata` builds a single drive only, so a multi-drive SoC
uses the explicit chain.

## Standalone core {#integ-standalone}

`litesata_gen` generates a Verilog core from a YAML file, for designs built without LiteX. It
elaborates the core without running any vendor tool, so it needs only Python, LiteX and
LiteSATA. It covers the Xilinx PHYs; the ECP5 PHY is not offered.

```sh
litesata_gen examples/artix7_wishbone_dma.yml --output-dir build --name litesata_core
```

The core comes in two flavours, chosen by the `core` key. Without it, the core is **native**: the
PHY, the core, the crossbar and the user ports listed in `user_ports`, with no CPU and no
registers. With `core` set to `wishbone` or `axi-lite`, it is **control/DMA**: a control bus gives
access to the PHY status, Identify, the optional BIST and the two DMAs, which master a Wishbone
DMA bus. This flavour goes through `add_sata`, and can have user ports too.

The main keys are below; [Build and generator options](#build-options) lists them all.

| Key | Values | Default |
|--------------------|-----------------------------------------------|---------------|
| `phy` | `A7SATAPHY`, `K7SATAPHY`, `GTH3SATAPHY`, `GTH4SATAPHY`, `GTY4SATAPHY` | required |
| `gen` | `gen1`, `gen2`, `gen3` | required |
| `clk_freq` | System clock in Hz | required |
| `data_width` | PHY width, 16 or 32 | 16 |
| `core` | `wishbone`, `axi-lite`, or absent for native | absent |
| `dma_data_width` | Width of the DMA bus | 32 |
| `dma_mode` | `read`, `write`, `read+write` | `read+write` |
| `user_ports` | Name, type (`native`) and width (32 or 64) of each port | none |

The PHY name selects the transceiver family, and the generator elaborates for a stand-in device of
that family: the Verilog does not depend on the exact part. The `gt_type` key of the YAML is
overwritten by the PHY name, so choose `GTH4SATAPHY` or `GTY4SATAPHY` to pick the UltraScale+
transceiver.

### Outputs and ports {#integ-standalone-ports}

The generator writes `gateware/litesata_core.v` with its constraints file and build script in the
output directory, and `csr.csv`, the register map, at its top. The figure below shows the port
groups of the generated module.

![Ports of the generated litesata_core module. The control and DMA buses exist in the
control/DMA flavour only; the user ports are those of the YAML file.](integ_standalone.svg)

| Ports | Direction | Role |
|----------------------------------|----------|------------------------------------------|
| `clk`, `rst` | in | System clock and reset |
| `sata_clk_p`, `sata_clk_n` | in | 150 MHz reference clock, dedicated pins |
| `sata_tx_p`, `sata_tx_n`, `sata_rx_p`, `sata_rx_n` | both | The SATA lane |
| `enable`, `ready` | in, out | PHY enable; PHY ready |
| `link_up`, `connect`, `disconnect` | out | Link status, debounced; one-cycle events |
| `wb_ctrl_*` or `axil_ctrl_*` | slave | Control bus, registers from address 0 |
| `wb_dma_*` | master | DMA bus, `dma_data_width` bits |
| `irq` | out | Interrupt of the DMAs, with `with_irq` |
| `user_port_<name>_sink_*`, `_source_*` | both | Native user port |

The registers start at address 0 of the control bus; Wishbone addresses 32-bit words, AXI-Lite
bytes. In the register map, each bank takes a 2 KiB page: `ctrl` (reset, scratch) at 0 and the
identifier at `0x800`, then `sata_identify`, `sata_irq`, `sata_mem2sector`, `sata_phy` and
`sata_sector2mem` for the example above, plus `sata_bist` with `with_bist`. Read the bases from
`csr.csv`. The interrupt follows the LiteX event manager: enable
the events in `sata_irq_enable`, and clear them by writing `sata_irq_pending`.

In the control/DMA flavour, the PHY takes its enable from the `sata_phy_enable` register, which
resets to 1, and the `enable` input is not used. In the native flavour, `enable` drives the PHY
and must be held high for the link to come up.

### What the integrator adds {#integ-standalone-integrator}

The generated core has no pin locations and no clock generator. The design that instantiates it
provides:

- the location of the transceiver: a `set_property LOC` on the channel, or pins of the lane;
- the reference clock on the pins of the transceiver quad, at 150 MHz. On UltraScale+, the
  generated core routes this `sata_clk_p`/`sata_clk_n` clock to the fabric input `GTGREFCLK`
  (see [Known issues](#dbg-known));
- a system clock at or above the minimum of [Generation, width and system
  clock](#integ-clocks), with its reset;
- for the control/DMA flavour, a memory behind the DMA bus and a CPU or a state machine on the
  control bus.

The generator does not check the system clock of the native flavour. The example
`examples/kintex7.yml` asks for Gen3 with a 100 MHz system clock, below the 150 MHz minimum:
raise `clk_freq` before using it at Gen3.

## Porting to a new board {#integ-porting}

The steps below add SATA to a board that LiteX already supports. [Requirements for a custom
board](#boards-custom) gives the hardware side.

1. **Describe the lane.** Add a record with `tx_p`, `tx_n`, `rx_p`, `rx_n` to the platform, plus
   `clk_p` and `clk_n` when a 150 MHz oscillator drives transceiver clock pins.
2. **Make the clocks.** Choose the generation and the PHY width, then a system clock at or above
   the minimum. Without a reference clock on the pins, add a 150 MHz output to a PLL and pass it
   as `refclk`.
3. **Build a bench first.** Copy the bench target of the closest board, change the platform and
   the pads, and run [Getting started](#getting-started) up to the BIST: it checks the link
   without a CPU or software in the way.
4. **Add SATA to the SoC.** Create `sata_phy` and call `add_sata`, then check `sata_init` in the
   BIOS.
5. **Start low.** Bring the link up at Gen1 or Gen2 first, then raise the generation. A link that
   comes up at a low rate and not at a higher one points to signal integrity or to the speed grade.

Where to go next: [Example designs and targets](#examples) for the targets of this repository,
[Register reference](#registers) for the registers, [Debugging](#debugging) when the link or a
command fails.
