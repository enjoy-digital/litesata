# Link layer {#link-layer}

This chapter describes the link layer of the core, `LiteSATALink`: how it frames a FIS, protects
it with a CRC and scrambling, runs the handshake with the drive and paces the frames with HOLD.
Read it to follow a frame in a simulation or a capture, or to understand an R_ERR. The primitives,
frames and handshake it implements are explained in [SATA in a nutshell](#sata-primer).

## At a glance {#link-glance}

| Item | Value |
|------------------------|-----------------------------------------------------------------------|
| Top-level class | `LiteSATALink(phy)` in `litesata/core/link.py` |
| Width and clock | 32-bit dwords in `sys`, one per cycle at most |
| TX pipeline | CRC inserter, scrambler, TX state machine, ALIGN inserter |
| RX pipeline | ALIGN remover, CONT remover, RX state machine, descrambler, CRC checker, buffer |
| CRC | Polynomial `0x04C11DB7`, initial value `0x52325032`, 32 bits in parallel |
| Scrambler | LFSR x^16 + x^15 + x^13 + x^4 + 1 seeded with `0xFFFF`, as SATA defines it (first dword `0xC2D2768D`); the 16-bit context of the parallel form resets to `0xf0f6` at the start of each frame |
| ALIGN | Two ALIGN primitives in every 256 transmitted dwords |
| CONT | Understood on reception, never sent |
| Receive buffer | 256 dwords; HOLD above 128 |
| Retries | None: R_ERR is not retried by the link layer |

## Pipelines {#link-pipelines}

The link layer has a transmit pipeline and a receive pipeline, as the figure below shows. The two
state machines in their middle run the handshake. The receive state machine also chooses the
primitives that the host sends while it receives a frame: R_RDY, R_IP, HOLD, HOLDA, R_OK and R_ERR.

![Link layer pipelines. The top row sends the frames of the transport layer, the bottom row
receives the frames of the drive; the RX state machine passes the primitives to send to the TX
side, and the buffer level asks for HOLD.](link_pipeline.svg)

On transmission, the transport layer hands over the dwords of a FIS with `last` on the final one.
The CRC inserter appends the CRC dword, the scrambler scrambles the FIS and the CRC, and the TX
state machine wraps them between SOF and EOF. The ALIGN inserter then adds two ALIGN primitives in
every 256 dwords, inside frames or between them; the receiver drops them without counting them as
frame content.

On reception, the ALIGN remover drops the ALIGN primitives and the CONT remover restores the
primitives that CONT stands for. The RX state machine extracts the data dwords of a frame. The
descrambler and the CRC checker then recover the FIS and flag a wrong CRC on its last dword. A
256-dword buffer decouples the transport layer from the line.

### Output priority {#link-output}

One dword leaves the TX state machine per cycle, chosen in this order:

1. The primitive requested by the RX state machine, while the host receives a frame.
2. The primitive of the TX state machine: SYNC, X_RDY, SOF, HOLDA, EOF or WTRM.
3. In COPY, the next scrambled data dword, or HOLD when the transport layer has none
   ready.

The two state machines never compete in practice. The TX side only leaves IDLE when the RX side
is idle and, while it waits in RDY, returns to IDLE as soon as the drive starts a frame of its own.
Once the frame has started (SOF), the drive is not expected to send X_RDY.

## CRC {#link-crc}

The CRC engine computes the next CRC from the previous one and a 32-bit data dword in one cycle
(`CRCEngine`). It builds the parallel equations of the LFSR at elaboration: it unrolls the
polynomial over 32 bits and keeps, for each output bit, the XOR of the input bits that appear an
odd number of times.

`LiteSATACRC` wraps the engine with the SATA parameters: polynomial `0x04C11DB7`, initial value
`0x52325032`. The inserter feeds the FIS through, then appends the CRC value as one more dword with
`last`; it restarts the CRC between packets.

The checker computes the CRC over the FIS and the received CRC together, which gives 0 for a good
frame. It holds one dword back so that it can drop the CRC dword, and sets `error` on the last FIS
dword when the result is not 0.

## Scrambler {#link-scrambler}

The scrambler XORs every data dword of a frame, the CRC included, with 32 bits from a 16-bit LFSR
(`Scrambler`). The next 32 output bits are fixed XOR combinations of the 16 context bits, written
as a table that the code takes from the specification; the upper half of each output becomes the
next context. The sequence is the one of the SATA scrambler, the serial LFSR x^16 + x^15 + x^13 +
x^4 + 1 seeded with `0xFFFF`, whose first output dword is `0xC2D2768D`. The context of this
parallel form resets to `0xf0f6`, the value that produces that sequence from its first dword; it
is an implementation detail, not the seed of the specification.

Both scramblers run once per data dword and restart between frames. The TX scrambler resets while
the TX state machine is in IDLE; the RX descrambler resets while the RX state machine is in IDLE.
Primitives are never scrambled. The BIST frontend reuses the same `Scrambler` class to produce its
pseudo-random data ([Frontends](#frontends)).

## ALIGN insertion and removal {#link-align}

`LiteSATAALIGNInserter` counts the dwords it sends with an 8-bit counter and sends ALIGN in the
first two slots of every 256, holding the stream meanwhile. It works below the TX state machine,
so the ALIGN pairs land between frames and inside them alike, as the specification allows.

`LiteSATAALIGNRemover` drops every received ALIGN before the rest of the pipeline sees it. The PHY
uses the same received ALIGN primitives to tell an active link from a silent one ([PHY architecture
and OOB controller](#phy-architecture)).

## CONT {#link-cont}

A drive may replace a long run of the same primitive with two copies, CONT and scrambled junk.
`LiteSATACONTRemover` undoes it on reception: after a CONT, it replaces every following data dword
with the last primitive seen before the CONT, until the next primitive arrives. The state machines
therefore see an ordinary run of repeated primitives.

The host never sends CONT: it repeats each primitive for as long as its state lasts. The CONT
inserter exists, with its own test (`test/test_link_cont.py`), but the link does not use it
(`LiteSATACONTInserter`).

## TX state machine {#link-tx}

The TX state machine sends the frames of the transport layer, as the figure below shows. Each box
gives the state and the primitive it sends.

![Link TX state machine. Follow the main path from IDLE to WTRM and back; RDY returns to IDLE when
the drive starts a frame first, and WTRM returns to IDLE on the drive's answer or on
SYNC.](link_tx_fsm.svg)

| State | Sends | Leaves when |
|--------|--------------|-----------------------------------------------------------------------|
| IDLE | SYNC | A FIS is ready and SYNC is received: to RDY |
| RDY | X_RDY | R_RDY is received: to SOF; the RX side leaves IDLE: back to IDLE |
| SOF | SOF | Sent: to COPY |
| COPY | Data or HOLD | The last dword is sent: to EOF; HOLD is received: to HOLDA |
| HOLDA | HOLDA | A primitive other than HOLD is received: to COPY |
| EOF | EOF | Sent: to WTRM |
| WTRM | WTRM | R_OK or R_ERR is received: to IDLE; SYNC: to IDLE on the next cycle without a received primitive |

The host yields when both sides want to send. In RDY, a device X_RDY moves the RX state machine out
of IDLE, which sends the TX state machine back to IDLE: the RX side answers R_RDY and receives the
device frame. The host frame stays pending and starts again once the link is idle.

The TX state machine also raises `error` when it receives SYNC outside IDLE and RDY, which means
that the drive gave up the frame. Only WTRM acts on it, by returning to IDLE, and only on a cycle
without a received primitive, which comes at the latest with the next ALIGN. HOLDA also checks it,
but any primitive other than HOLD, SYNC included, sends HOLDA back to COPY first.

## RX state machine {#link-rx}

The RX state machine receives the frames of the drive, as the figure below shows. Each box gives
the state and the primitive it asks the TX side to send.

![Link RX state machine. A frame goes from IDLE through COPY to WTRM; the CRC result chooses R_OK
or R_ERR, which lasts until the drive sends SYNC.](link_rx_fsm.svg)

| State | Sends | Leaves when |
|------------|----------------------|-------------------------------------------------------|
| IDLE | nothing (TX side) | X_RDY is received: to RDY |
| RDY | R_RDY | SOF is received: to WAIT_FIRST |
| WAIT_FIRST | R_IP | A data dword is received: to COPY |
| COPY | R_IP, HOLD or HOLDA | EOF is received: to WTRM |
| WTRM | R_IP | After one cycle: to R_OK or R_ERR |
| R_OK | R_OK | SYNC is received: to IDLE |
| R_ERR | R_ERR | SYNC is received: to IDLE |

In COPY, the state machine passes each data dword to the descrambler one cycle late, so that it can
mark the dword before EOF, the CRC, as the last one. It answers a HOLD of the drive with HOLDA, and
sends HOLD itself while the receive buffer is too full. The code also declares an EOF state that no
transition reaches. The WTRM state of the RX side lasts one cycle: the state machine answers R_OK
or R_ERR one cycle after EOF, without waiting for the WTRM of the drive.

## Flow control {#link-hold}

The receive buffer holds 256 dwords. When it holds more than 128, the RX state machine sends HOLD
instead of R_IP; the drive answers HOLDA and stops sending data until HOLD stops, as the figure
below shows. The other 128 dwords absorb the data already on its way while the drive reacts.

![Flow control on reception. The host sends HOLD while its buffer holds more than 128 dwords, the
drive pauses the frame with HOLDA, and the frame goes on once the host sends R_IP
again.](link_hold.svg)

On transmission, the TX state machine sends HOLD whenever the transport layer has no dword ready in
the middle of a FIS. When the drive sends HOLD, the TX state machine answers HOLDA and resumes the
frame on the next other primitive. The core feeds whole sectors from the user port, so transmit
pauses depend on how fast user logic provides the write data.

## Errors and limits {#link-limits}

The link layer detects two errors and recovers from neither by itself:

- **CRC error on reception**: the RX state machine answers R_ERR and the frame reaches the
  transport layer with `error` on its last dword. A Data FIS then makes the command fail
  ([Transport and command layers](#transport-command)).
- **R_ERR on transmission**: the TX state machine returns to IDLE as after R_OK, and does not resend
  the frame. Nothing is reported upwards; the drive reports the result of the command in its status.

The handling of a drive that aborts a frame with SYNC is partial. On transmission, only WTRM returns
to IDLE on SYNC; a frame in SOF, COPY, HOLDA or EOF runs to its end first. On reception, RDY,
WAIT_FIRST and COPY have no exit on SYNC, so a frame cut by the drive leaves the RX state machine
waiting for its EOF. The link then stays stuck: the X_RDY of the next drive frame is ignored and the
host cannot send. Only a reset of the core recovers it, and nothing resets the core on its own (see
[Core architecture](#core-pace)).

The power management primitives (PMREQ_P, PMREQ_S, PMACK, PMNAK) are not defined, and the host
never asks for a low-power state.

## In the code {#link-code}

| Class | Role |
|--------------------------------|--------------------------------------------------------------|
| `LiteSATALink` | Assembles both pipelines, the receive buffer and the HOLD threshold |
| `LiteSATALinkTX` | TX state machine, output multiplexer, CRC inserter and scrambler |
| `LiteSATALinkRX` | RX state machine, descrambler and CRC checker |
| `CRCEngine` | Parallel CRC equations for any width and polynomial |
| `LiteSATACRC` | SATA CRC: polynomial, initial value, check value |
| `LiteSATACRCInserter` | Appends the CRC dword after `last` |
| `LiteSATACRCChecker` | Drops the CRC dword, flags `error` on the last dword |
| `Scrambler` | 16-bit context, 32 output bits per dword |
| `LiteSATAScrambler` | Scrambles the data of a stream, with a reset |
| `LiteSATAALIGNInserter` | Two ALIGN primitives every 256 dwords |
| `LiteSATAALIGNRemover` | Drops received ALIGN primitives |
| `LiteSATACONTRemover` | Restores the primitive that CONT repeats |
| `LiteSATACONTInserter` | CONT with scrambled junk; not used by `LiteSATALink` |

All the classes are in `litesata/core/link.py`; the primitive values are in `litesata/common.py`.

Where to go next: [Transport and command layers](#transport-command) for what the frames carry,
[PHY architecture and OOB controller](#phy-architecture) for the link below, [Simulation and
verification](#simulation) for the link tests and the drive model.
