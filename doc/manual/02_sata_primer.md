# SATA in a nutshell {#sata-primer}

This chapter explains SATA to an FPGA engineer who has never read the specification. It starts
from the cable and the layers, then follows a link from power-up to a running command: the
out-of-band wake-up, the 8b/10b characters and primitives, the frames, the FISes and the ATA
commands. It ends with the IDENTIFY DEVICE data a host reads first.

Each section gives the idea, then the mechanism, and ends with the part of LiteSATA that
implements it. The references are the Serial ATA specification, for the physical, link and
transport layers, and the ATA Command Set (ATA/ACS), for the commands and the IDENTIFY data.

## Host and device {#primer-host-device}

SATA is a point-to-point link between one *host* and one *device*. The host starts every command;
the device, a hard disk or an SSD, executes it. In a PC the host is the SATA controller of the
chipset; with LiteSATA the host is the FPGA.

The data cable has seven contacts: two differential pairs and three grounds. One pair carries
data from the host to the device, the other from the device to the host. Both pairs are AC-coupled
by capacitors in the drive and on the host board, so only the transitions of the signal cross the
cable, not its DC level.

Each pair carries a serial bit stream at a fixed rate. There is no separate clock: the receiver
recovers the clock from the transitions of the data, with the clock and data recovery (CDR) of its
transceiver. An FPGA multi-gigabit transceiver does exactly this, so LiteSATA needs no external
chip, only the AC coupling capacitors where the board needs them.

## Layers {#primer-layers}

SATA is defined as a stack of four layers, the same on both sides of the link. Each layer talks to
its peer on the other side in its own unit, through the layers below it, as the figure below
shows.

![The layers of SATA on both sides of the link, and the unit each pair of layers exchanges. The
host is the FPGA running LiteSATA; the device is the drive.](primer_layers.svg)

- The *physical layer* serializes the bits, encodes them in 8b/10b, recovers the clock and brings
  the link up with out-of-band signals.
- The *link layer* sends and receives *frames*. It frames each FIS between delimiters, adds a CRC,
  scrambles the content and handles the flow control. Its unit is the dword, a data word or a
  *primitive*.
- The *transport layer* builds and parses the *FIS* (Frame Information Structure): a block of
  dwords whose first byte gives its type.
- The *command layer*, the application layer of the specification, runs the ATA commands as
  sequences of FISes.

In LiteSATA, the physical layer is the PHY ([PHY architecture and OOB
controller](#phy-architecture)); the three upper layers form the core, in `litesata/core/`
([Core architecture](#core-architecture)).

## Rates and clocks {#primer-rates}

SATA has three generations, all on the same cable and connectors. A drive of one generation works
with a host of another at the lower of their two rates (for a host that supports the lower rate;
LiteSATA runs at one rate, see [Out-of-band signalling](#primer-oob)). The table below gives the
rates; the 8b/10b code sends 10 bits for each byte, so the byte rate is a tenth of the line rate.

| Generation | Line rate | Bit time (UI) | Byte rate | 32-bit clock |
|------------|-----------|---------------|-----------|--------------|
| Gen1 | 1.5 Gbps | 666.7 ps | 150 MB/s | 37.5 MHz |
| Gen2 | 3 Gbps | 333.3 ps | 300 MB/s | 75 MHz |
| Gen3 | 6 Gbps | 166.7 ps | 600 MB/s | 150 MHz |

A *unit interval* (UI) is the time of one bit on the line. The specification gives the timings of
the out-of-band signals in Gen1 UI, whatever the generation of the link.

The byte rate is the ceiling of the link, before the framing and the protocol take their share
([Performance and resources](#performance)). A 32-bit datapath that carries one dword per clock
runs at the frequency of the last column; a 16-bit datapath runs at twice that frequency.

In LiteSATA, the generation is fixed when the design is built, by the `gen` parameter of the PHY.
The transceivers run from a 150 MHz reference clock, and the transceiver clocks follow the
generation and the data width; the core is 32 bits wide, in the system clock domain
([Core architecture](#core-architecture)).

## Out-of-band signalling {#primer-oob}

Before any data, the host and the device must find each other and agree on a rate. They do it with
*out-of-band* (OOB) signals: patterns of *bursts* of signal and *gaps* of electrical idle, which a
receiver can detect with a simple squelch detector, without a locked CDR.

A burst lasts 160 Gen1 UI, about 106.7 ns; it is made of ALIGN primitives sent at the Gen1 rate. A
gap is an electrical idle, where both wires of the pair sit at the same level. The length of the
gaps tells the three OOB signals apart, as the figure below shows.

![The OOB signals: six bursts of 160 Gen1 UI, separated by gaps of 480 UI for COMRESET and COMINIT
and of 160 UI for COMWAKE.](primer_oob_timing.svg)

| Signal | Sent by | Burst | Gap | Meaning |
|-----------|---------|---------------------|---------------------|-------------------------------|
| COMRESET | Host | 160 UI, 106.7 ns | 480 UI, 320 ns | Reset the device and start bring-up |
| COMINIT | Device | 160 UI, 106.7 ns | 480 UI, 320 ns | Device present, ready to start |
| COMWAKE | Both | 160 UI, 106.7 ns | 160 UI, 106.7 ns | Wake up, about to send data |

COMRESET and COMINIT are the same waveform: the name depends on who sends it. Each signal is six
bursts and their gaps. A receiver does not measure the bursts, only the gaps. It must recognise a
gap of 304 to 336 ns as COMRESET or COMINIT and a gap of 101.3 to 112 ns as COMWAKE; it may accept
any gap between 175 and 525 ns, and between 35 and 175 ns, respectively.

The bring-up follows the sequence of the figure below. The host sends COMRESET; the device answers
with COMINIT. The host may then calibrate its transmitter, and sends COMWAKE; the device answers
with COMWAKE. Both sides then leave the OOB world and start sending characters.

![Link bring-up. COMRESET, COMINIT and COMWAKE are OOB signals; after COMWAKE, the device sends
ALIGN at its fastest rate while the host sends D10.2, until the host locks and answers with ALIGN.
SYNC then marks an idle, ready link.](primer_oob.svg)

After COMWAKE, the device sends ALIGN primitives at its highest rate. Meanwhile the host sends the
D10.2 character, an alternating `0101` pattern that lets the device lock its CDR. When the host
receives ALIGN, it locks on the device's rate and sends ALIGN back. The device then sends SYNC: the
link is up and idle.

The rate is negotiated by the device. If the host does not answer its ALIGN within about 54.6 µs,
the device steps down to the next lower rate and tries again. The host waits 873.8 µs for ALIGN
(32768 Gen1 dwords); if none comes, it starts over with COMRESET.

In LiteSATA, the bring-up is the controller of the PHY, in `litesata/phy/ctrl.py`: it sends
COMRESET, waits for COMINIT, sends COMWAKE, waits for the device's COMWAKE, sends D10.2 until it
receives ALIGN, then answers with ALIGN. Its ALIGN timer is 873 µs.

The Xilinx transceivers generate and detect the OOB signals in hardware; the ECP5 PHY builds them
in the fabric ([Lattice ECP5 PHY](#ecp5-phy)). The host runs at one rate only, so a drive links
only if it supports that rate or a faster one.

## 8b/10b coding {#primer-8b10b}

The link encodes each byte as a 10-bit *character* with the 8b/10b code, as Gigabit Ethernet
(1000BASE-X) and PCI Express Gen1 and Gen2 do. The code bounds the number of identical bits in a
row, so the receiver always sees transitions to recover the clock, and it keeps as many ones as
zeros on the line, which the AC coupling needs.

Each character is either a data character, written Dx.y, or a control character, written Kx.y:
x is the value of the five low bits of the byte and y of the three high bits, so the byte `0x4a`
is D10.2. The encoder chooses between two encodings of each character to keep the balance; the
state that guides that choice is the *running disparity*.

The control character K28.5 holds a *comma*: a bit pattern that no sequence of data characters
can produce. A receiver searches the bit stream for it to find where characters, and dwords, begin.
SATA puts K28.5 at the start of ALIGN, so every ALIGN realigns the receiver.

Swapping the two wires of a pair inverts every bit. K28.5 survives the inversion, as the K28.5 of
the other disparity, but the data characters after it change: D10.2 turns into D21.5. A host can
therefore detect a swapped pair from the ALIGN it receives.

In LiteSATA, the transceiver encodes and decodes 8b/10b and aligns the received bytes on K28.5;
the datapath of the PHY realigns the dwords. The controller detects the polarity of the received
pair from the inverted ALIGN (`ALIGN_N` in `litesata/common.py`) and sets the RX polarity of the
transceiver. The transmit side cannot be observed: [Polarity](#phy-polarity) explains how the
controller sets it.

## Primitives {#primer-primitives}

The link layer exchanges dwords. A dword is either data, four data characters, or a *primitive*:
a dword of four characters whose first one is a control character, as the figure below shows.
Primitives carry the control of the link: idle, ready, start and end of a frame, flow control and
acknowledgment.

![A primitive on the wire: four characters sent byte 0 first. Byte 0 is the control character,
K28.5 for ALIGN and K28.3 for all the others; bytes 1 to 3 are data characters that name the
primitive. LiteSATA writes the dword with byte 0 in the low bits.](primer_primitive.svg)

The table below lists the primitives LiteSATA uses, with the dword value of `litesata/common.py`
(byte 0 in bits 7:0) and the characters on the wire, byte 0 first.

| Primitive | Value | Characters | Meaning |
|-----------|----------------|-------------------------|----------------------------------|
| ALIGN | `0x7b4a4abc` | K28.5 D10.2 D10.2 D27.3 | Alignment and clock compensation |
| SYNC | `0xb5b5957c` | K28.3 D21.4 D21.5 D21.5 | Idle, no frame in progress |
| X_RDY | `0x5757b57c` | K28.3 D21.5 D23.2 D23.2 | Transmitter has a frame to send |
| R_RDY | `0x4a4a957c` | K28.3 D21.4 D10.2 D10.2 | Receiver ready for a frame |
| SOF | `0x3737b57c` | K28.3 D21.5 D23.1 D23.1 | Start of frame |
| EOF | `0xd5d5b57c` | K28.3 D21.5 D21.6 D21.6 | End of frame |
| R_IP | `0x5555b57c` | K28.3 D21.5 D21.2 D21.2 | Reception in progress |
| WTRM | `0x5858b57c` | K28.3 D21.5 D24.2 D24.2 | Transmitter waits for the frame status |
| R_OK | `0x3535b57c` | K28.3 D21.5 D21.1 D21.1 | Frame received without error |
| R_ERR | `0x5656b57c` | K28.3 D21.5 D22.2 D22.2 | Frame received with an error |
| HOLD | `0xd5d5aa7c` | K28.3 D10.5 D21.6 D21.6 | Pause: no data to send, or no room |
| HOLDA | `0x9595aa7c` | K28.3 D10.5 D21.4 D21.4 | HOLD acknowledged |
| CONT | `0x9999aa7c` | K28.3 D10.5 D25.4 D25.4 | Repeat the previous primitive |

The two clocks of a link, one on each side, are never exactly equal. Every transmitter therefore
sends two ALIGN primitives at least every 256 dwords, inside frames as well as between them. A
receiver drops them, or adds to them, in its elastic buffer to absorb the difference, then
discards them: ALIGN carries no information above the physical layer.

A primitive such as SYNC or HOLD can be sent for a long time. To avoid a repetitive pattern on the
line, a transmitter may send the primitive twice, then CONT, then scrambled junk data until the
next primitive. The receiver acts as if the last primitive before CONT were still repeated.

In LiteSATA, the link layer inserts two ALIGN primitives every 256 dwords and removes the received
ones. It understands CONT on reception; it never sends CONT itself, which the specification allows
([Link layer](#link-layer)).

## Frames, CRC and scrambling {#primer-frames}

A *frame* carries one FIS. It starts with SOF, continues with the dwords of the FIS and a CRC dword,
and ends with EOF, as the figure below shows. Primitives can appear inside a frame: ALIGN pairs
for clock compensation, and HOLD or HOLDA for flow control. They are not part of the FIS and are
dropped by the receiver.

![A frame on the link: SOF, the dwords of the FIS, the CRC and EOF. An ALIGN pair and a HOLD
pause can sit between any two data dwords; the receiver drops them. The FIS and the CRC are
scrambled.](primer_frame.svg)

The CRC is a 32-bit cyclic redundancy check of all the dwords of the FIS. It uses the generator
polynomial of Ethernet, `0x04C11DB7`, starts from `0x52325032`, and is appended as the last dword
before EOF. A receiver that computes a different CRC rejects the frame with R_ERR.

The FIS and its CRC are *scrambled*: each dword is XORed with the output of a 16-bit linear
feedback shift register (LFSR) with the polynomial x^16 + x^15 + x^13 + x^4 + 1, seeded with
`0xFFFF` at each SOF. Scrambling spreads the energy of repetitive
data over the spectrum and reduces electromagnetic emissions. Primitives are never scrambled; the
receiver descrambles with the same LFSR.

In LiteSATA, the link layer computes the CRC on the FIS, then scrambles the FIS and the CRC
together; on reception it descrambles, then checks the CRC. Both blocks are in
`litesata/core/link.py` ([Link layer](#link-layer)).

## Link handshake {#primer-handshake}

Sending a frame is a handshake between the transmitter and the receiver, as the figure below
shows. While the link is idle, both sides send SYNC. The transmitter announces a frame with X_RDY;
the receiver answers R_RDY when it can take it. The transmitter then sends SOF, the FIS, the CRC
and EOF, while the receiver sends R_IP.

![Sending one frame. The transmitter, here the host, asks with X_RDY, the receiver grants with
R_RDY, then acknowledges with R_OK after checking the CRC. Both sides return to
SYNC.](primer_handshake.svg)

After EOF the transmitter sends WTRM and waits. The receiver checks the CRC and the frame, and
answers R_OK or R_ERR. Both sides then return to SYNC. Each primitive is repeated as long as its
state lasts; the figure shows each one once.

Either side can pause a frame. A receiver whose buffer fills sends HOLD; the transmitter answers
HOLDA and stops sending data until HOLD stops. A transmitter that runs out of data sends HOLD
itself, and the receiver answers HOLDA. This flow control works dword by dword, without losing data.

The host and the device can both want to send at the same time: both then send X_RDY. The
specification makes the host yield: it answers R_RDY to the device and sends its own frame later.

In LiteSATA, the transmit and receive state machines of the link layer follow these steps. The
receiver sends HOLD when its 256-dword buffer is half full, and the transmitter yields when it
sees the device's X_RDY ([Link layer](#link-layer)).

## FIS types {#primer-fis}

The transport layer gives each frame a meaning. A FIS is a block of dwords whose first byte is its
type; the type sets its length and the layout of its fields. The table below lists the FIS types
of the specification for a host and a drive, and the ones LiteSATA uses.

| FIS | Type | Direction | Length (dwords) | Used by LiteSATA |
|---------------------|--------|-----------|-----------------|------------------|
| Register H2D | `0x27` | H2D | 5 | Yes, sends commands |
| Register D2H | `0x34` | D2H | 5 | Yes, command status |
| DMA Activate | `0x39` | D2H | 1 | Yes, write data request |
| PIO Setup | `0x5f` | D2H | 5 | Yes, for IDENTIFY DEVICE |
| Data | `0x46` | Both | 1 + up to 2048 | Yes, sector data |
| Set Device Bits | `0xa1` | D2H | 2 | No, dropped |
| DMA Setup | `0x41` | Both | 7 | No, dropped |
| BIST Activate | `0x58` | Both | 3 | No, dropped |

The *Register H2D FIS* carries a command from the host to the device, the way an older parallel
ATA host wrote its *taskfile* registers. The figure below shows its fields. Its *Register D2H*
counterpart carries back the status and the error registers at the end of a command.

![The Register H2D FIS, one row per dword, bit 31 on the left. The C bit tells a command from a
control update; the 48-bit LBA and the 16-bit features are split over two dwords
each.](primer_fis.svg)

The *Data FIS* carries sector data: one header dword and up to 2048 dwords, 8 KiB, of payload. A
transfer larger than that takes several Data FISes. *DMA Activate* is the device's go-ahead for
the next Data FIS of a write. *PIO Setup* announces a programmed I/O data block and its length.

In LiteSATA, the transport layer of `litesata/core/transport.py` builds and parses the five FIS
types it uses; it drops any other FIS. The fields of each header are in `litesata/common.py`
([Transport and command layers](#transport-command)).

## ATA commands and status {#primer-ata}

A drive is driven by ATA commands. The host fills the taskfile in a Register H2D FIS: the command
code, the LBA of the first sector, the sector count, the device register and the features. The
device runs the command and reports the result in its status and error registers.

The table below gives the status bits a host checks. A command has succeeded when the final status
shows neither ERR nor DF; when ERR is set, the error register gives the reason.

| Bit | Name | Meaning when set |
|-----|------|----------------------------------------------------------|
| 7 | BSY | Device busy, the other bits are not valid |
| 6 | DRDY | Device ready to accept commands |
| 5 | DF | Device fault |
| 3 | DRQ | Device ready to transfer a block of data |
| 0 | ERR | The command ended with an error |

LiteSATA issues three commands: READ DMA EXT (`0x25`), WRITE DMA EXT (`0x35`) and IDENTIFY DEVICE
(`0xec`). The DMA commands move sectors in Data FISes with no PIO Setup FIS per block; IDENTIFY
DEVICE uses the PIO data-in protocol.

## READ DMA EXT and WRITE DMA EXT {#primer-dma}

A DMA read is the simplest exchange, as the figure below shows. The host sends the command in a
Register H2D FIS. The device sends the data in Data FISes of up to 8 KiB each, then a Register D2H
FIS with the final status.

![READ DMA EXT. One Register H2D FIS carries the command; the device sends the data in one or
more Data FISes, then the status in a Register D2H FIS.](primer_read_dma.svg)

A DMA write needs the device's permission for each Data FIS, as the figure below shows. After the
command, the device sends DMA Activate when it can take data. The host sends one Data FIS of up to
8 KiB, and waits for the next DMA Activate before the next one. The device ends with a Register D2H
FIS.

![WRITE DMA EXT. The device paces the transfer: each Data FIS from the host follows a DMA Activate
FIS from the device. The Register D2H FIS reports the final status.](primer_write_dma.svg)

Every FIS of these exchanges is a separate frame with its own link handshake. A CRC error on any
frame makes the receiver answer R_ERR; the command then ends with an error status, or with no
status at all.

In LiteSATA, the command layer of `litesata/core/command.py` runs both sequences. It splits a
write into Data FISes of 2048 dwords and waits for a DMA Activate before each one. A command fails
when the final status has ERR or DF set, when a received Data FIS has a CRC error, or when a read
returns fewer dwords than requested. LiteSATA does not act on the R_ERR answer to a frame it sent;
it relies on the status the drive reports afterwards ([Transport and command
layers](#transport-command)).

## IDENTIFY DEVICE {#primer-identify}

IDENTIFY DEVICE returns 512 bytes that describe the drive: its model, serial number, capacity and
supported features. It is the first command a host sends, and the simplest way to check that the
whole stack works.

The command uses the PIO data-in protocol, shown in the figure below. After the Register H2D FIS,
the device sends a PIO Setup FIS that gives the direction, the length of the data block and the
final status, then the data in one Data FIS. No Register D2H FIS follows: the status was in the
PIO Setup FIS.

![IDENTIFY DEVICE with the PIO data-in protocol. The PIO Setup FIS announces the 512-byte block
and carries the status; the data follows in a Data FIS.](primer_identify.svg)

The data is 256 words of 16 bits. The table below lists the words a host usually reads. Strings
store two characters per word, the first one in the high byte.

| Words | Content |
|----------|----------------------------------------------------------|
| 10 to 19 | Serial number, 20 characters |
| 23 to 26 | Firmware revision, 8 characters |
| 27 to 46 | Model number, 40 characters |
| 76 | SATA capabilities: bits 1, 2 and 3 for Gen1, Gen2 and Gen3 |
| 83 | Supported features: bit 10 for the 48-bit address feature set |
| 100 to 103 | Number of user-addressable sectors with 48-bit addresses |

In LiteSATA, the Identify frontend stores the 128 dwords in a FIFO for the software to read. The
command layer also accepts a drive that splits the block in several PIO Setup and Data FIS pairs.
It checks the Status field of the PIO Setup FIS, not its ending status (E_Status), which it does
not decode. The BIOS command `sata_init` prints the model and the capacity from these words
([Frontends](#frontends)).

## LBA and sector count {#primer-lba}

A drive is an array of sectors of 512 bytes, numbered from 0. The number of a sector is its
*logical block address* (LBA). The EXT commands use 48-bit LBAs, enough for 128 PiB of 512-byte
sectors, and a 16-bit sector count.

The capacity of a drive, in sectors, is in words 100 to 103 of the IDENTIFY data; multiplied by
512, it gives the capacity in bytes. In ATA, a count of 0 in an EXT command means 65536 sectors.

In LiteSATA, a command carries a 48-bit `sector` and a 16-bit `count` on the user port, and the
sector size is the constant `logical_sector_size` (512) of `litesata/common.py`. A count of 0 is
not supported: use counts of 1 to 65535 sectors.

Where to go next: [Core architecture](#core-architecture) maps these layers onto the modules of
the core; [PHY architecture and OOB controller](#phy-architecture) details the bring-up.
