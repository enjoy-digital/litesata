```{=latex}
\part{The core}
```

```{=html}
<h1 class="part">Part II. The core</h1>
```

# Core architecture {#core-architecture}

This chapter describes the core, `LiteSATACore`: the three layers between the PHY and the user
ports, the streams that connect them, their clock domain, and the contract of a user port. Read it
before connecting user logic to the core or choosing a system clock. The protocol concepts it uses
are explained in [SATA in a nutshell](#sata-primer).

## At a glance {#core-glance}

| Item | Value |
|-----------------------|----------------------------------------------------------------------|
| Top-level class | `LiteSATACore(phy)` in `litesata/core/__init__.py` |
| Role | Host only, one drive per core |
| Layers | Link, transport, command |
| Datapath | 32-bit dwords, one per `sys` cycle at most |
| Clock domain | `sys`; the PHY crosses to `sata_tx` and from `sata_rx` |
| System clock | At least 37.5, 75 or 150 MHz for Gen1, Gen2 or Gen3 |
| ATA commands | READ DMA EXT, WRITE DMA EXT, IDENTIFY DEVICE |
| Addressing | 48-bit LBA, 16-bit sector count, 512-byte sectors |
| User side | One command stream (`sink`) and one response stream (`source`) |
| Sharing | Several user ports through the crossbar ([Frontends](#frontends)) |

## Layers {#core-layers}

The core is a stack of three layers, each with a transmit half and a receive half, as the figure
below shows. The command layer turns a request into the FISes of an ATA command. The transport
layer builds and parses the FISes. The link layer frames them with primitives, a CRC and
scrambling, and runs the handshake with the drive.

![Core layers. Follow a command down the TX arrows and its response up the RX arrows; the right
column names what each pair of neighbours exchanges. The crossbar and the frontends sit on top of
the core, the PHY below it.](core_blocks.svg)

`LiteSATACore` only chains the three layers: its `sink` and `source` are those of the command
layer. The two halves of a layer talk through small side channels. The link RX half tells the TX
half which primitive to send; the command RX half tells the TX half when a DMA Activate FIS or a
failed status arrives.

The core knows nothing about transceivers. It needs a PHY object with a 32-bit `sink` and `source`
in the `sys` clock domain, which `LiteSATAPHY` provides ([PHY architecture and OOB
controller](#phy-architecture)). The drive model of the tests is such an object, so the whole core
runs against a simulated drive ([Simulation and verification](#simulation)).

## Stream layouts {#core-layouts}

Every interface between two layers is a LiteX stream: `valid`, `ready` and `last`, plus the fields
of the table below. Parameters stay constant over a packet; payload fields change on each beat. All
layouts are defined in `litesata/common.py`.

| Interface | Layout | Parameters | Payload |
|--------------|-------------------------------------------|---------------------------|--------------|
| PHY and link | `phy_description` | none | `data`, `charisk` |
| Link and transport | `link_description` | none | `data`, `error` |
| Transport TX | `transport_tx_description` | FIS type, command, LBA, count, others | `data` |
| Transport RX | `transport_rx_description` | FIS type, status, errors, LBA, count, transfer count, `error` | `data` |
| Command TX | `command_tx_description` | `write`, `read`, `identify`, `sector`, `count` | `data` |
| Command RX | `command_rx_description` | `write`, `read`, `identify`, `end`, `failed` | `data` |

On the PHY interface, `charisk` has one bit per byte and flags the K characters. A primitive has
`charisk` at `0b0001`: its first byte, on `data[7:0]`, is K28.3 or K28.5. A data dword has
`charisk` at 0. The first byte on the cable is always on `data[7:0]`.

On the link interface, `error` is set on the last dword of a received frame whose CRC is wrong. The
transport layer carries it with the Data FIS payload, and the command layer reports it as `failed`.

## Clock domains {#core-clocks}

The core and the frontends run in the `sys` clock domain. The PHY datapath moves the dwords to the
transmit clock `sata_tx`, and from the recovered receive clock `sata_rx`, through asynchronous
FIFOs of 8 entries, as the figure below shows. It also converts between the 32-bit core words and
the 16-bit or 32-bit transceiver words.

![Clock domains. Look at the two asynchronous FIFOs: everything on their left runs on the system
clock and sees 32-bit words; the transceiver side runs on clocks derived from the line
rate.](core_clocks.svg)

The transceiver clocks follow the generation and the transceiver width. The table below gives them
with the dword rate of the link: the line rate divided by 40, since 8b/10b coding turns 32 data
bits into 40 line bits.

| Gen | Line rate | `sata_tx`, `sata_rx`, 16-bit | `sata_tx`, `sata_rx`, 32-bit | Dword rate |
|--------|---------------|------------------------------|------------------------------|----------------|
| Gen1 | `1.5 Gbps` | 75 MHz | 37.5 MHz | 37.5 million/s |
| Gen2 | `3 Gbps` | 150 MHz | 75 MHz | 75 million/s |
| Gen3 | `6 Gbps` | 300 MHz | 150 MHz | 150 million/s |

### System clock rule {#core-sys-clock}

The core moves at most one dword per `sys` cycle, and the host cannot slow the drive down: every
received dword, primitives included, must leave the receive FIFO in time. The system clock must
therefore be at least the dword rate, as the project README states: 37.5 MHz for Gen1, 75 MHz for
Gen2, 150 MHz for Gen3. There is no slack for a slower clock: the ALIGN pairs the core drops are
only 2 dwords in 256, and the receive FIFO holds 8 entries.

Keep a margin above it; [Integration](#integration) gives the clocks of the targets.

`add_sata` in LiteX, and so the control/DMA flavour of the generator, refuses a slower clock, as
the Acorn target does. The native flavour of the generator does not check it. The core path stays
32-bit whatever the bus width: follow the rule of this section.

### Pace of the link {#core-pace}

The transmit side of the link layer always has something to send: data, or a primitive such as SYNC
when it is idle. The transmit FIFO of the PHY sets its pace, at the dword rate. The receive side is
always ready: dwords arrive at the dword rate, with idle `sys` cycles between them when the system
clock is faster.

Until the PHY reports the link ready, its datapath connects the transceiver to the PHY controller
only. The core then sends and receives nothing, and its transmit stream waits. The core has no reset
tied to the link state: a link lost in the middle of a command leaves the layers where they were
([Transport and command layers](#transport-command) lists this limit).

## User port contract {#core-user-port}

The command layer stream, reached through a crossbar port, is the user port. User logic sends a
command and its write data on `sink`, and receives read data and the command status on `source`.
The core runs one command at a time, but it does not enforce it: the command layer accepts a new
command beat as soon as the previous command is sent. User logic must wait for the response beat
(`end`) before it sends the next command; the crossbar only keeps the other ports out
([Frontends](#frontends)).

| `sink` field | Width | Meaning |
|--------------|-------|-------------------------------------------------------------|
| `write` | 1 | WRITE DMA EXT: the data beats follow |
| `read` | 1 | READ DMA EXT: one beat, no data |
| `identify` | 1 | IDENTIFY DEVICE: one beat, no data |
| `sector` | 48 | First LBA of the command |
| `count` | 16 | Number of sectors, 1 to 65535 |
| `data` | 32 or 64 | Write data, first byte on `data[7:0]` |
| `last` | 1 | Last beat of the command |

| `source` field | Width | Meaning |
|-------------------|-------|-----------------------------------------------------------|
| `write`, `read`, `identify` | 1 | Command the beat belongs to |
| `data` | 32 or 64 | Read or IDENTIFY data |
| `last` | 1 | Last beat of a Data FIS, or the response beat |
| `end` | 1 | The command is over |
| `failed` | 1 | The command failed (with `end`), or, on the last beat of a Data FIS, that this Data FIS had a CRC error |

The figure below shows the beats of a one-sector write and of a one-sector read on a 32-bit port.

![User port beats. A write carries its command fields on the data beats and gets one response
beat; a read is one command beat, then the data beats and one response beat.](core_port.svg)

The rules of the port are:

- **Command fields**: the core samples `write`, `read`, `identify`, `sector` and `count` on the
  first beat. Frontends keep them constant over the packet, as the crossbar converters and the RAID
  modules expect.
- **Write**: the packet holds exactly `count` sectors, 128 dwords each, with `last` on the final
  beat. A shorter packet ends the Data FIS early and the command layer returns to idle; the DMA
  Activate with which the drive asks for the rest is ignored, and the command does not end. A
  longer packet starts a new command.
- **Read**: one beat with `last` set. The data comes back in packets, one per Data FIS, each ended
  by `last` with `end` at 0. A response beat follows with `end` and `last` set; `failed` gives the
  result and `data` has no meaning.
- **IDENTIFY**: one beat with `last` set. The 128 dwords of the IDENTIFY data come back with
  `identify` set, and `end` set on the 128th dword. No response beat follows, unless the drive
  aborts the command: then a single beat with `end` and `failed` arrives instead of the data.
- **Failure**: `failed` with `end` means that the drive reported an error, that a received Data FIS
  had a CRC error, or that a read returned fewer dwords than asked. [Transport and command
  layers](#transport-command) lists the cases.
- **One command at a time**: wait for the beat with `end` before sending the next command. The
  command layer does not check it, and the frontends of the repository all wait.
- **Count**: `count` is 16 bits. A count of 0 means 65536 sectors, as in ATA: the command layer
  sends it to the drive as is, and on a READ its dword counter wraps to that length.

A 64-bit port carries two dwords per beat, the first one on `data[31:0]`. `count` counts
512-byte sectors, so a sector takes 64 beats. The command layer has no timeout: user logic that
must not wait forever adds its own.

## In the code {#core-code}

| Class | File and role |
|----------------------------|-----------------------------------------------------------------|
| `LiteSATACore` | `litesata/core/__init__.py`: chains link, transport and command |
| `LiteSATALink` | `litesata/core/link.py`: framing, CRC, scrambling, handshake |
| `LiteSATATransport` | `litesata/core/transport.py`: FIS encoding and decoding |
| `LiteSATACommand` | `litesata/core/command.py`: ATA command sequences |
| Layouts, constants | `litesata/common.py`: primitives, FIS types and headers, ATA codes |
| `LiteSATAPHYDatapath` | `litesata/phy/datapath.py`: width conversion, clock domain crossing |

Where to go next: [Link layer](#link-layer) for the frames on the cable, [Transport and command
layers](#transport-command) for the FIS sequences, [Frontends](#frontends) for the modules on the
user ports, [Integration](#integration) to add the core to a SoC.
