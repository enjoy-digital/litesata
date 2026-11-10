# Frontends {#frontends}

This chapter describes the crossbar that shares the core between user ports, and the frontends on
these ports: Identify, BIST, DMAs, stream frontends, striping and mirroring. Read it to pick a
frontend, to program its registers, or to wire several drives. The contract of a user port is in
[Core architecture](#core-architecture).

## At a glance {#fe-glance}

| Frontend | Ports | Registers | Use |
|------------------|----------------|-------------|------------------------------------------------|
| Crossbar | n user ports | none | Shares one core, 32-bit or 64-bit ports |
| Identify | 1 | yes | Reads the IDENTIFY data |
| BIST | 3 | yes | Writes and checks a pattern, measures the rate |
| Sector2Mem DMA | 1 | yes, `irq` | Sectors to memory over Wishbone |
| Mem2Sector DMA | 1 | yes, `irq` | Memory to sectors over Wishbone |
| Stream | 1 | yes, `irq` | Sectors to a LiteX stream, and back |
| Striping | 1 wide | none | N drives as one, N times the capacity |
| Mirroring | N | none | Same data on N drives, reads in parallel |

The classes are in `litesata/frontend/` ([In the code](#fe-code)). `add_sata` in LiteX builds the
crossbar, the Identify unit, the DMAs and optionally the BIST ([Integration](#integration)).

## Crossbar {#fe-crossbar}

The crossbar lets several frontends share one core, as the figure below shows. Each call to
`get_port()` returns a new user port; a round-robin arbiter gives the core to one port at a time.

![Crossbar. Each frontend has its own user port; a 64-bit port goes through a width converter,
and the arbiter connects one port at a time to the core.](fe_crossbar.svg)

A port keeps the core from its first command beat until its response beat with `end` and `last`.
The other ports wait, then the arbiter moves to the next port that has a request. A port that
issues a command and does not read its response therefore blocks every port. The arbiter does not
stop the granted port from sending a second command before the first one has ended: every
frontend waits for the response beat, and user logic must do the same.

`get_port(dw)` returns a port as wide as the core, 32 bits, or a multiple of it. A 64-bit port goes
through two stride converters: the first dword of a beat is `data[31:0]`. `get_ports(n, dw)` returns
`n` ports at once. The crossbar can also sit on a striping module or on a mirroring port, whose
width it takes as its own.

```python
from litesata.core                 import LiteSATACore
from litesata.frontend.arbitration import LiteSATACrossbar

self.sata_core     = LiteSATACore(self.sata_phy)
self.sata_crossbar = LiteSATACrossbar(self.sata_core)
port32 = self.sata_crossbar.get_port()   # 32-bit user port.
port64 = self.sata_crossbar.get_port(64) # 64-bit user port.
```

## Identify {#fe-identify}

`LiteSATAIdentify` sends IDENTIFY DEVICE and stores the 512 bytes of the answer in a 32-bit FIFO
(512 entries, 128 used). `LiteSATAIdentifyCSR` exposes it to software with the registers below. The
LiteX BIOS uses it in `sata_init` to print the model and the capacity of the drive.

| Register | Access | Meaning |
|------------------|--------|--------------------------------------------------------------|
| `start` | Write | Write 1 to send IDENTIFY DEVICE; empties the FIFO |
| `done` | Read | 1 when idle: the command is over or not started |
| `error` | Read | The drive aborted the command, or a Data FIS had a CRC error |
| `data_width` | Read | Width of the user port, in bits |
| `source_valid` | Read | A word is available in the FIFO |
| `source_data` | Read | The word at the head of the FIFO |
| `source_ready` | Write | Write 1 to pop the word |

Software writes `start`, waits for `done`, then reads `source_data` and writes `source_ready` while
`source_valid` is set. A 32-bit port gives 128 words; each holds two 16-bit IDENTIFY words, the
first one in bits 15:0. The FIFO is 32 bits wide, so the unit needs a 32-bit port: on a 64-bit
port it would keep only the low half of each beat.

## BIST {#fe-bist}

The BIST writes a known pattern to the drive, reads it back and counts the differences, as the
figure below shows. It needs no memory and no processor, which makes it the first test of a new
board and the tool to measure the throughput of a drive.

![BIST. The generator writes the pattern, the checker reads it back and compares it with the same
pattern, which each one regenerates on its own; each one has its unit of registers, which repeats
the transfer and counts the cycles.](fe_bist.svg)

`LiteSATABIST(crossbar, with_csr)` takes three ports of the crossbar: one for the generator, one for
the checker and one for an Identify unit of its own. With `with_csr=True` it adds the registers.

- **Generator** (`LiteSATABISTGenerator`): sends one WRITE DMA EXT of `count` sectors from `sector`.
  The data is the index of each dword in the transfer, or, with `random` set, the output of the
  scrambler of the link layer.
- **Checker** (`LiteSATABISTChecker`): sends one READ DMA EXT of the same sectors, regenerates the
  same pattern and counts the received beats that differ from it in `errors` (dwords on a 32-bit
  port).
- **Unit** (`LiteSATABISTUnitCSR`): runs its generator or checker `loops` times, and counts the
  `sys` cycles from `start` to the end of the last loop in `cycles`.

Each loop restarts the pattern, so all the loops write the same data to the same sectors. Each loop
also clears `aborted` and `errors`, which therefore report the last loop only. The
registers of each unit are listed below; with `add_sata`, they are named `sata_bist_generator_*` and
`sata_bist_checker_*`, and the Identify unit `sata_bist_identify_*`.

| Register | Width | Meaning |
|-----------|-------|---------------------------------------------------------------|
| `start` | 1 | Write 1 to run |
| `sector` | 48 | First sector |
| `count` | 16 | Number of sectors per command |
| `loops` | 8 | Number of commands to run |
| `random` | 1 | Scrambler pattern instead of the dword index |
| `done` | 1 | 1 when idle |
| `aborted` | 1 | The command of the last loop failed |
| `errors` | 32 | Mismatching beats of the last loop (checker only) |
| `cycles` | 32 | `sys` cycles for the whole run |

The throughput follows from the cycle count. The host script `bench/test_bist.py` computes it as
below and prints it in MiB/s, with `count` the sectors per command, the clock frequency of the SoC
and the width ratio of the port:

```text
throughput = loops * count * 512 * (port width / 32) / (cycles / sys_clk_freq)   bytes per second
```

`cycles` is a 32-bit counter, so a run longer than 2^32 cycles (about 28 s at 150 MHz) wraps.

Each loop is a single command of `count` sectors, so a large `count` hides the command overhead;
[Performance and resources](#performance) discusses the overheads. On a port wider than the core,
the generator and the checker multiply the ATA count by the width ratio: `count` then counts units
of 1024 bytes on a 64-bit port.

## DMAs {#fe-dma}

The two DMAs move sectors between the drive and the memory of a SoC over a Wishbone bus, as the
figure below shows. They are the path of the LiteX BIOS: `sata_read`, `sata_write` and the boot
from a FAT file system all go through them.

![Wishbone DMAs. Sector2Mem reads a sector into its buffer and writes it to memory; Mem2Sector
reads 512 bytes of memory into its buffer and writes them to a sector.](fe_dma.svg)

Both DMAs work one sector at a time: each sector is one ATA command of `count` 1, with a 512-byte
buffer in between. `LiteSATASector2MemDMA` reads a sector, then writes it to memory at `base`;
`LiteSATAMem2SectorDMA` reads 512 bytes at `base`, then writes them to the sector. Both then go on
with the next sector and the next 512 bytes, `nsectors` times.

| Register | Width | Meaning |
|------------|-------|-------------------------------------------------------------------|
| `sector` | 48 | First sector |
| `nsectors` | 16 | Number of sectors, 1 or more |
| `base` | 64 | Byte address in memory, aligned to the bus width |
| `start` | 1 | Write to run |
| `done` | 1 | 1 when idle |
| `error` | 1 | The last run stopped on a failed command |

The DMA stops at the first failed command and sets `error`. Its `irq` output pulses for one cycle
when a run ends, whether it completed or failed; `add_sata` connects the two pulses to the
`sector2mem_dma` and `mem2sector_dma` events of the `sata_irq` interrupt controller.

The buffer in memory holds the bytes of the sectors in their order on the drive, whatever the
endianness of the processor: the DMAs take the endianness of the CPU as a parameter and reorder the
bytes accordingly. `nsectors` must be at least 1: with 0, the DMA does not stop after the first
sector.

One command per sector costs a Register H2D FIS, the data and a Register D2H FIS for every 512
bytes. The DMAs suit the BIOS and moderate rates; a design that needs the full rate of the drive
sends commands of many sectors from its own logic or uses the BIST pattern of access.

## Stream frontends {#fe-stream}

`LiteSATASectors2Stream` reads sectors and sends their bytes on a LiteX stream;
`LiteSATAStream2Sectors` takes the bytes of a stream and writes them to sectors. Both use the same
registers as the DMAs, with `nsectors` 32 bits wide, and a 64-bit stream by default (`data_width`).
They are in `litesata/frontend/stream.py`, not exported by `litesata.frontend`, and no target, test
or example uses them.

Like the DMAs, they work one sector per command. `LiteSATAStream2Sectors` waits for 512 bytes of
the stream before each write; it ignores the `last` of the stream, which must deliver exactly
`nsectors` times 512 bytes. `LiteSATASectors2Stream` sets `last` on the final word of the final
sector and pulses `irq` at the end.

The code as it is has two limits:

- **Several sectors**: in `LiteSATASectors2Stream`, the response beat of a sector read enters the
  sector buffer with the data. In a simulation with the drive model, a read of two sectors on a
  32-bit stream gives the 128 dwords of the first sector, then one word of value 0 with `last`, and
  ends with `done` and no error; the second sector never comes out. Reads of one sector work: run
  one sector per start.
- **Byte order**: on a 64-bit stream, the two classes do not order the bytes the same way. A word
  written by `LiteSATAStream2Sectors` reads back from `LiteSATASectors2Stream` with its two 32-bit
  halves swapped. With `data_width=32`, both put the first byte of the sector in bits 31:24.

## Striping {#fe-striping}

`LiteSATAStriping` joins N drives into one larger and faster one, like RAID 0, as the figure below
shows. Each drive has its own PHY and core; the striping module offers one port N times as wide as
a core.

![Striping over two drives. Each 64-bit word of the port is split: the low dword goes to the first
core, the high dword to the second; reads put the two dwords back
together.](fe_striping.svg)

All the cores receive the same command, with the same `sector` and `count`. On writes, core `i`
gets bits `32*i` to `32*i+31` of each word. On reads, the module waits for a beat from every core,
joins the dwords, and sets `failed` if any core failed.

A sector of the port is therefore N times 512 bytes, spread over the same sector of every drive.

The port sees N times the capacity of one drive and up to N times its throughput, limited by the
slowest drive. A crossbar sits on top of the striping module like on a core:

```python
from litesata.frontend.raid import LiteSATAStriping

self.striping = LiteSATAStriping([self.sata_core0, self.sata_core1])
self.crossbar = LiteSATACrossbar(self.striping)
port = self.crossbar.get_port() # 64-bit port: 2 x 32-bit cores.
```

## Mirroring {#fe-mirroring}

`LiteSATAMirroring` keeps the same data on N drives, like RAID 1, as the figure below shows. It
offers one port per drive. A read or an IDENTIFY on port `i` goes to drive `i` only, so N reads run
in parallel; a write on any port goes to all the drives.

![Mirroring over two drives. Reads stay on the port's own core; a write from either port goes to
both cores with the same data.](fe_mirroring.svg)

Writes have priority over reads. When a write arrives on a port, the module stops accepting new
reads, waits for the commands in progress on every core to end, then switches to write mode. When
no write is left, it switches back to reads.

In write mode, a round-robin arbiter sends the writes of the ports one after another to every core.
The response reaches the port that wrote, with `failed` set if any core failed.

The ports see the capacity of one drive. Reads reach up to N times the throughput of one drive;
writes reach the throughput of the slowest drive. Each port takes a crossbar of its own:

```python
from litesata.frontend.raid import LiteSATAMirroring

self.mirroring = LiteSATAMirroring([self.sata_core0, self.sata_core1])
self.crossbar0 = LiteSATACrossbar(self.mirroring.ports[0])
self.crossbar1 = LiteSATACrossbar(self.mirroring.ports[1])
```

Neither module checks that the drives match in size, nor rebuilds a drive after a failure: they
spread or copy the commands, and leave the management of the drives to the system.

## The drive as a block device {#fe-block-device}

LiteSATA has no block device frontend of its own. In a LiteX SoC, the BIOS uses the Identify unit
and the two DMAs as one: `sata_init`, `sata_read` and `sata_write` reach any sector, and the SATA
boot reads files from a FAT file system through them. [Integration](#integration) describes the
commands and the boot.

## In the code {#fe-code}

| Class | File and role |
|---------------------------------|------------------------------------------------------------|
| `LiteSATACrossbar` | `arbitration.py`: user ports, width converters |
| `LiteSATAArbiter` | `arbitration.py`: round-robin grant until `end` |
| `LiteSATAIdentify` | `identify.py`: IDENTIFY into a FIFO; `LiteSATAIdentifyCSR` |
| `LiteSATABIST` | `bist.py`: generator, checker and Identify on three ports |
| `LiteSATABISTUnitCSR` | `bist.py`: registers, loops and cycle counter of one unit |
| `LiteSATASector2MemDMA` | `dma.py`: sectors to memory, one command per sector |
| `LiteSATAMem2SectorDMA` | `dma.py`: memory to sectors, one command per sector |
| `LiteSATASectors2Stream` | `stream.py`: sectors to a stream; `LiteSATAStream2Sectors` back |
| `LiteSATAStriping` | `raid.py`: one wide port over N cores |
| `LiteSATAMirroring` | `raid.py`: N ports, reads per drive, writes to all |

The files are in `litesata/frontend/`.

Where to go next: [Integration](#integration) to add the frontends to a SoC, [Getting
started](#getting-started) to run the BIST on a board, [Register reference](#registers) for the
register addresses.
