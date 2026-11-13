# Transport and command layers {#transport-command}

This chapter describes the two upper layers of the core. The transport layer turns FIS fields into
dwords and back; the command layer runs the FIS sequence of each ATA command and reports its
result on the user port. Read it to know what the core sends, when it reports a failure and what it
does not support. [SATA in a nutshell](#sata-primer) explains the FISes and the ATA commands.

## At a glance {#tc-glance}

| Item | Value |
|-------------------------|---------------------------------------------------------------------|
| Classes | `LiteSATATransport`, `LiteSATACommand` |
| Files | `litesata/core/transport.py`, `litesata/core/command.py` |
| FISes sent | Register H2D, Data |
| FISes received | Register D2H, DMA Activate, PIO Setup, Data; others dropped |
| ATA commands | READ DMA EXT (`0x25`), WRITE DMA EXT (`0x35`), IDENTIFY DEVICE (`0xec`) |
| Data FIS size | 2048 dwords (8 KiB) at most per FIS on writes |
| Failure | ERR or DF in the status, CRC error on a Data FIS, short read |
| Timeout | None |
| Commands in flight | One, kept by the frontends; the command layer does not enforce it |

## Transport layer {#transport-layer}

The transport layer knows the header layout of each FIS type. On transmission it serializes the
header fields of a request into dwords; on reception it collects the header dwords and presents
them as named fields. The header layouts are tables of `HeaderField` entries in
`litesata/common.py`, one per FIS type.

| FIS | Type | Direction | Header dwords | Handling |
|---------------|--------|-------------|---------------|----------------------------------------------|
| Register H2D | `0x27` | Sent | 5 | Command, LBA, count and constants |
| Data | `0x46` | Both | 1 | Type dword, then the payload |
| Register D2H | `0x34` | Received | 5 | Presented as one beat: status, errors, LBA, count |
| DMA Activate | `0x39` | Received | 1 | Presented as one beat |
| PIO Setup | `0x5f` | Received | 5 | Presented as one beat: status, errors, transfer count; E_Status not decoded |
| Other types | | Received | | Dropped dword by dword |

### Transmit {#transport-tx}

A request on the transport sink names a FIS type and carries the header fields as parameters. For a
Register H2D FIS, the transport layer sends the five header dwords, the last one with `last`. For a
Data FIS, it sends the type dword, then the payload dwords of the request with its `last`. A
request of another type is consumed and dropped.

The link layer adds the CRC and the framing. The transport layer does not limit the size of a Data
FIS: the command layer cuts long writes before they reach it.

### Receive {#transport-rx}

The receive side reads the type byte of the first dword of each frame. For a Register D2H, DMA
Activate or PIO Setup FIS, it stores the header dwords and presents the decoded fields as one
beat. For a Data FIS, it skips the type dword and passes the payload through, with `error` from
the link layer on the last dword.

Frames of other types, such as Set Device Bits or DMA Setup, are dropped. The receive side does not
check the CRC of the control FISes: a Register D2H FIS with a wrong CRC is answered with R_ERR by
the link layer and presented anyway, with `error` at 0.

The receive side does not track frame boundaries while it drops: it tests the first byte of each
dword, so a payload dword of a dropped FIS that starts with a known type byte would be decoded as
a new FIS. The supported commands do not make the drive send such FISes.

## Command layer {#command-layer}

The command layer has a TX half that sends the Register H2D FIS and the write data, and an RX half
that follows the answers of the drive. The figure below shows how the TX half fills the FISes from
a user port beat.

![From a user port beat to the FISes. The command flags select the ATA command code, the sector and
the count go to the LBA and count fields, the other fields are constants, and the data of a write
goes to Data FISes.](cmd_layouts.svg)

The Register H2D FIS always has its C bit set, which marks a new command, and the device field at
`0xe0`. The features, ICC and control fields are 0.

The command layer does not wait for the end of a command before it accepts the next request: the
TX half is free again once the Register H2D FIS (or the last write beat) is sent. One command at a
time is in flight only because every frontend waits for the response beat; user logic must do the
same.

### WRITE DMA EXT {#command-write}

A write runs as the figure below shows. The TX half sends the Register H2D FIS, then waits for a
DMA Activate FIS before each Data FIS. It cuts the write data into Data FISes of 2048 dwords, 16
sectors, so a write of more than 16 sectors takes several DMA Activate and Data FIS pairs.

![WRITE DMA EXT. The left column names the state of the command layer while each FIS goes by; the
TX half sends at most 2048 dwords per DMA Activate, and the Register D2H FIS ends the
command.](cmd_write.svg)

The RX half waits for the answers. It turns each DMA Activate FIS into a pulse for the TX half. On
the Register D2H FIS it records the status and presents the response beat: `write`, `end` and
`last` set, `failed` set when the status has ERR or DF.

If the drive sends a Register D2H FIS with ERR or DF while the TX half waits for DMA Activate, the
TX half gives up the command: it accepts one more beat of the user packet and returns to IDLE,
where the rest of the packet starts a new command ([Limits](#command-limits)).

### READ DMA EXT {#command-read}

A read runs as the figure below shows. The TX half sends the Register H2D FIS and is done. The RX
half presents each Data FIS as a packet of `read` beats ended by `last`, and counts the dwords
against the `count` of the request.

![READ DMA EXT. The drive returns the data in one or more Data FISes, then the Register D2H FIS that
closes the command.](cmd_read.svg)

The Register D2H FIS ends the command with the response beat. It is marked `failed` when the status
has ERR or DF, when a Data FIS had a CRC error, or when fewer than `count` times 128 dwords
arrived. Extra dwords beyond the count are not detected. A FIS that a read does not expect, a DMA
Activate FIS or a PIO Setup FIS, is dropped.

### IDENTIFY DEVICE {#command-identify}

IDENTIFY DEVICE is a PIO data-in command: the drive sends a PIO Setup FIS, then a Data FIS with the
512 bytes of the IDENTIFY data. Some drives split the block into several PIO Setup and Data FIS
pairs. The RX half accepts both forms and counts 128 dwords in total.

The RX half first waits for the PIO Setup FIS, dropping any other FIS until then. It presents the
data with `identify` set and sets `end` on the 128th dword; no response beat follows. A PIO Setup
FIS with ERR or DF, or a Register D2H FIS in place of the data, ends the command with a single
response beat with `failed` set.

### Receive states {#command-rx-states}

| State | Waits for or does |
|--------------------------------------------|---------------------------------------------|
| IDLE | A new request from the TX half |
| `WAIT_WRITE_ACTIVATE_OR_REG_D2H` | DMA Activate (pulse to TX) or Register D2H |
| `PRESENT_WRITE_RESPONSE` | Write response beat |
| `WAIT_READ_DATA_OR_REG_D2H` | Data FIS, PIO Setup (IDENTIFY) or Register D2H |
| `WAIT_PIO_SETUP_D2H` | First PIO Setup of an IDENTIFY, or Register D2H |
| `PRESENT_PIO_SETUP_D2H` | Checks the status of a PIO Setup FIS |
| `PRESENT_READ_DATA` | Passes a Data FIS to the user port |
| `PRESENT_READ_RESPONSE` | Read response beat, or failed IDENTIFY |
| FLUSH | Drops an unexpected FIS before the PIO Setup |

The RX half also keeps the last status and error fields of a Register D2H or PIO Setup FIS in
`d2h_status` and `d2h_errors`. They are plain signals for debugging, not registers.

## Failure rules {#command-failures}

The table below lists what sets `failed` on the user port. `failed` with `end` is the result of
the command; `failed` on the last data beat of a Data FIS only marks that this FIS had a CRC
error.

| Condition | Commands | Where |
|------------------------------------------|----------------|---------------------------------|
| ERR or DF set in the Register D2H status | All | Response beat |
| ERR or DF set in a PIO Setup status | IDENTIFY | Response beat |
| CRC error on a Data FIS | READ, IDENTIFY | Last beat of that Data FIS; READ: also the response beat; IDENTIFY: the beat with `end` when it is the last one |
| Fewer dwords than `count` times 128 | READ | Response beat |
| Register D2H instead of the data | IDENTIFY | Response beat |

The other status bits are not interpreted. The error register is kept in `d2h_errors` but does not
reach the user port: user logic only sees that the command failed.

## No timeout {#command-timeout}

The command layer waits for the drive as long as it takes. This is deliberate: a hard drive that
leaves a power-saving state can take a long time to answer, and a fixed timeout would fail good
commands. A request to a drive that never answers therefore blocks the core.

No frontend has a timeout either. The LiteX BIOS bounds each read and write in software, 100 ms
per try and up to 8 tries ([Integration](#integration)). A try after a timeout cannot help: the
DMA is not idle, so it ignores the new start, and the hardware stays blocked until the core is
reset.

## Supported commands {#command-supported}

| Command | Code | Protocol | Data |
|--------------------|--------|----------------|-------------------------------------------------|
| READ DMA EXT | `0x25` | DMA data-in | `count` sectors (0 means 65536), 48-bit LBA |
| WRITE DMA EXT | `0x35` | DMA data-out | `count` sectors (0 means 65536), 48-bit LBA |
| IDENTIFY DEVICE | `0xec` | PIO data-in | 512 bytes |

Other ATA commands, such as FLUSH CACHE EXT, DATA SET MANAGEMENT (TRIM) and the NCQ commands, are
not supported. A sector count of 0 means 65536 sectors, as in ATA: on a READ, the dword counter of
the command layer (`read_ndwords`) wraps to that length.

## Limits {#command-limits}

The command layer assumes a well-behaved drive and a stable link. These cases are not handled:

- **Link loss**: nothing resets the core when the PHY loses the link. A command in progress stays
  where it was; recovering it needs a reset of the core.
- **Back-to-back commands**: the command layer accepts a new command beat before the previous
  command has ended; a second command sent too early mixes the answers of both. Wait for `end`.
- **Write refused early**: when the drive fails a write before asking for all its data, the TX half
  takes one more beat of the user packet and returns to IDLE; the rest of the packet is then sent
  as a new WRITE DMA EXT. User logic must drop the rest of the packet. A drive that ends the write
  without an error and without asking for the data leaves the TX half waiting for DMA Activate.
- **Control FIS with a bad CRC**: it is taken as valid, as [Receive](#transport-rx) explains.
- **Header dword after a gap**: the receive side of the transport layer can capture the last header
  dword of a control FIS one cycle before it is valid, when the link delivers that dword after an
  idle cycle; the real dword is then dropped. The last header dword is reserved in Register D2H and
  holds the transfer count in PIO Setup, which the command layer does not use.

## In the code {#tc-code}

| Class | File and role |
|----------------------------|---------------------------------------------------------------|
| `LiteSATATransportTX` | `litesata/core/transport.py`: Register H2D and Data FIS encoding |
| `LiteSATATransportRX` | `litesata/core/transport.py`: FIS decoding, unknown types dropped |
| `LiteSATACommandTX` | `litesata/core/command.py`: Register H2D, DMA Activate wait, write split |
| `LiteSATACommandRX` | `litesata/core/command.py`: answers, responses, failure rules |
| FIS tables | `litesata/common.py`: `fis_types`, `fis_*_header`, `fis_max_dwords` |
| ATA tables | `litesata/common.py`: `regs` (command codes), `reg_d2h_status` (status bits) |

Where to go next: [Core architecture](#core-architecture) for the user port contract,
[Frontends](#frontends) for the modules that issue these commands, [Debugging](#debugging) for
failed commands on hardware.
