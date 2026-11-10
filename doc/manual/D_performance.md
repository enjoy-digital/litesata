# Performance and resources {#performance}

This appendix gathers the performance figures of the project: the payload ceilings of each
generation, computed from the line rate and the overheads of the protocol, the limits that the
system clock and the frontends add, how the BIST measures a rate, and the throughput recorded on a
public design. Computed values are labelled as such; the measured value names its source.

## Line rate and payload ceiling {#perf-ceiling}

The line rate of a generation carries 8b/10b characters: ten bits per byte. The link therefore
moves one dword per 40 bits, and the transceiver clock gives one dword per cycle of the 32-bit
SATA clock.

| Generation | Line rate | Bytes on the line | Dwords | 32-bit SATA clock |
|--------------|--------------|----------------------|-----------------|-------------------|
| Gen1 | 1.5 Gbps | 150 MB/s | 37.5 M/s | 37.5 MHz |
| Gen2 | 3 Gbps | 300 MB/s | 75 M/s | 75 MHz |
| Gen3 | 6 Gbps | 600 MB/s | 150 M/s | 150 MHz |

Not every dword carries payload. The host and the drive insert two ALIGN primitives every 256
dwords, and every frame adds a SOF, the FIS header, the CRC and an EOF around its data. A Data FIS
carries at most 2048 dwords, 8 KiB, so a long transfer loses 4 dwords per 2048. The table computes
the ceiling with these two overheads only, in MB/s (10^6 bytes per second) and in MiB/s
(2^20 bytes per second, as `test_bist.py` prints).

| Generation | After ALIGN (254/256) | After framing (2048/2052) | Ceiling in MiB/s |
|--------------|------------------------|---------------------------|------------------|
| Gen1 | 148.8 MB/s | 148.5 MB/s | 141.7 MiB/s |
| Gen2 | 297.7 MB/s | 297.1 MB/s | 283.3 MiB/s |
| Gen3 | 595.3 MB/s | 594.2 MB/s | 566.6 MiB/s |

These ceilings leave out the frame handshakes (X_RDY, R_RDY, WTRM, R_OK), the HOLD pauses, the
command FISes and, above all, the time the drive takes to answer. A write also waits for a DMA
Activate FIS before each Data FIS. The rate of a real transfer is set by the drive: its media, its
cache and its state.

## System clock {#perf-sysclk}

The PHY datapath and the core are 32 bits wide and run in the system clock domain. The transceiver
delivers one dword per cycle of the 32-bit SATA clock and cannot be paused, so the system clock
must be at least that clock: 37.5 MHz for Gen1, 75 MHz for Gen2 and 150 MHz for Gen3. Below it,
the receive clock domain crossing of the datapath overflows and the link loses dwords
(`litesata/phy/datapath.py`). This is a requirement of the link, not only of its rate.

| Configuration | Generation | sys_clk | Rule (computed) |
|---------------------------------------|-------------|------------|------------------------------|
| `kintex7` example | Gen3 | 100 MHz | Not met: 150 MHz needed |
| `artix7`, `artix7_wishbone_dma` | Gen2 | 100 MHz | Met |
| `ultrascale_plus*` examples | Gen3 | 150 MHz | Met, at the limit |
| `acorn` target (default) | Gen1 | 100 MHz | Met |
| `ecpix5` target | Gen2 | 90 MHz | Met |
| `kc705`, `genesys2` targets | Gen3 | 200 MHz | Met |

`add_sata` and the control/DMA flavour of the generator refuse a system clock below a rule scaled by
the width of their bus ([Build options](#build-options)); the `acorn` target applies the rule above.
On a 32-bit bus the scaled rule is the rule above. On a 64-bit bus, the check of `add_sata` accepts
half of it, 75 MHz at Gen3, which the datapath does not support: keep the system clock at the rule
above whatever the bus width. The native flavour of the generator does not check the clock.

## Frontends {#perf-frontends}

A frontend adds its own overhead per command. The BIST sends one command for its whole `count`, up
to 65535 sectors, so its overhead per command is small for large transfers. The DMAs and the
stream frontends send one command per sector, and pay the command FISes and the latency of the
drive every 512 bytes.

For a one-sector write, the link carries a Register H2D FIS, a DMA Activate FIS, a Data FIS and a
Register D2H FIS: 152 dwords with their SOF, CRC and EOF, for 128 dwords of payload. Without any
handshake or drive latency, this caps the one-sector write at 128/152 of the framing ceiling, 84%
(computed). In practice the drive latency per command dominates, and the DMAs reach a fraction of
the link rate. Use the BIST or a frontend with multi-sector commands when the rate matters.

The striping frontend spreads each transfer over several drives and the mirroring frontend reads
from either drive, so their rates scale with the number of drives, within the limit of the system
clock of the shared datapath ([Frontends](#frontends)).

## Measuring with the BIST {#perf-bist}

The BIST counts the system clock cycles of a run in its `cycles` register. `test_bist.py` computes
the rate of each transfer as

```text
rate = loops * count * 512 / (cycles / clk_freq)
```

in bytes per second, then prints it in MiB/s ([Host scripts](#ex-scripts)). The count includes the
command FISes and the drive latency of each command, so it is the rate a user would see with
commands of that size. Use transfers of 1 MiB or more (`-s 1024`, the default) to approach the
link ceiling, and `-n` to average several commands.

The first transfers of a write test often run faster than the next ones: an SSD writes into a
cache first, then at the rate of its media. Run a test longer than the cache of the drive to
measure its sustained rate.

## Measured throughput {#perf-measured}

No throughput measurement is recorded in the LiteSATA repository. The public LiteX-M2SDR project,
which records radio samples to an SSD through LiteSATA, publishes the figure below, quoted as its
page prints it (one row of its table, in the units of its tool).

```text
| RF -> SATA | 30.72 MS/s SC16 2T2R, 15 s (3.43 GiB) | 232.94 MiB/s, elapsed 15.092 s | real-time, past the drive's TurboWrite cache |
```

Source: `doc/sata-validation.md` of the LiteX-M2SDR repository, a LiteX-M2SDR board in an Acorn
Baseboard Mini (Artix-7 GTP at Gen2) with a Samsung 850 EVO 250GB, write cache enabled. The
recording goes through a stream frontend of that project, which sends multi-sector write commands,
not the one-sector frontends of LiteSATA. About 233 MiB/s sustained, past the write cache of the
drive, is 82% of the Gen2 ceiling of 283.3 MiB/s (computed). The same page records lower rates with
another drive: the drive, not the core, sets the sustained rate.

## Resources {#perf-resources}

No resource figures are recorded in the repository. To get the resources of a configuration, build
it with the toolchain of the FPGA and read the utilization report, or generate a standalone core
with `litesata_gen` and synthesize it in the integrating project.

## Where to go next {#perf-next}

[Example designs and targets](#examples) for the BIST script, [Frontends](#frontends) for the
command size of each frontend, and [Integration](#integration) for the system clock rules.
