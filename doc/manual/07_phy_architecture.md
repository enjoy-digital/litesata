```{=latex}
\part{The PHYs}
```

```{=html}
<h1 class="part">Part III. The PHYs</h1>
```

# PHY architecture and OOB controller {#phy-architecture}

The PHY turns the 32-bit dwords of the link layer into a serial stream on an FPGA transceiver, and
brings the link up before any dword flows. This chapter describes what every PHY shares: the three
parts, the out-of-band (OOB) state machine, the byte realignment and the clock domain crossings.
Read it to integrate a PHY, to port one, or to understand why a link does not come up.

The family-specific parts are in [Xilinx transceiver PHYs](#xilinx-phys) and
[Lattice ECP5 PHY with OOB in fabric](#ecp5-phy). The protocol behind OOB, ALIGN and 8b/10b is in
[SATA in a nutshell](#sata-primer).

## At a glance {#phy-glance}

| Item | Value |
|--------------------------|----------------------------------------------------------------|
| Class | `LiteSATAPHY`, selected by the FPGA device name |
| Parts | Transceiver and CRG, controller, datapath, link monitor |
| Generations | Gen1, Gen2, Gen3 on Xilinx; Gen2 only on ECP5 |
| Transceiver width | 16 or 32 bits (`data_width`); 16 bits only on ECP5 |
| Core side | 32-bit dwords with a 4-bit K mask, in `sys` |
| Clock domains | `sata_tx`, `sata_rx` (transceiver), `sys` (controller, core) |
| Reference clock | 150 MHz on every family |
| OOB | Transceiver hardware on Xilinx, fabric logic on ECP5 |
| Registers | `enable`, `status` ([PHY registers](#reg-sata-phy)) |

## Three parts {#phy-parts}

A PHY has three parts with a clear boundary between them. The figure below shows them between the
link layer of the core and the drive.

![PHY blocks. The transceiver wrapper and its CRG are the only family-specific part. The controller
drives the TX path until the link is ready, then hands it over to the core; it also reads the RX
stream and the two flags of the datapath.](phy_blocks.svg)

**Transceiver and CRG.** A wrapper instantiates the transceiver primitive of the FPGA family, its
PLL, its reset sequence and its OOB signalling. Its CRG creates the `sata_tx` and `sata_rx` clock
domains from the transceiver output clocks. This is the vendor-specific part; one wrapper exists
per transceiver family.

**Controller.** A state machine runs the OOB sequence and the ALIGN exchange with the drive, then
watches the link (`LiteSATAPHYCtrl`). It is the same for every family. The ECP5 PHY subclasses it
to adjust three states ([ECP5 controller overrides](#ecp5-ctrl)).

**Datapath.** The datapath converts between the transceiver width and the 32-bit core width,
realigns the received bytes on the K character, and crosses the clock domains
(`LiteSATAPHYDatapath`). It also measures the time since the last ALIGN and flags misaligned K
characters for the controller.

`LiteSATAPHY` builds the three parts from the device name, connects them and adds the link monitor
and the registers. The selection rules are in [Device selection](#xil-selection).

## Parameters {#phy-params}

| Parameter | Default | Role |
|---------------------------|----------|-----------------------------------------------------|
| `device` | Required | FPGA device name; selects the transceiver wrapper |
| `pads` | Required | TX and RX pairs, and the refclk pair when `refclk` is `None` |
| `gen` | Required | `"gen1"`, `"gen2"` or `"gen3"`, fixed at build time |
| `clk_freq` | Required | `sys` clock frequency in Hz, for the timers |
| `refclk` | `None` | 150 MHz reference clock signal; `None` uses the pads |
| `data_width` | 16 | Transceiver data width: 16 or 32 |
| `qpll` | `None` | Artix-7 only: an external GTP quad PLL to share |
| `gt_type` | `"GTY"` | UltraScale+ only: `"GTY"` or `"GTH"` |
| `use_gtgrefclk` | `True` | UltraScale+ only: reference clock through the fabric input |
| `with_csr` | `True` | Add the `enable` and `status` registers |
| `dual`, `channel` | 0, 0 | ECP5 only: DCU and channel |
| `link_down_timeout` | 100 ms | Debounce time of the link monitor, in seconds |

The pads record carries `tx_p`, `tx_n`, `rx_p` and `rx_n`, plus `clk_p` and `clk_n` for a reference
clock on dedicated pins. The PHY exposes `sink` and `source` (32-bit dwords for the link layer),
`ready`, `enable`, `link_up`, `connect_irq` and `disconnect_irq`. Its CRG is `crg`, the wrapper
`phy`, the controller `ctrl` and the datapath `datapath`.

## The transceiver contract {#phy-contract}

The controller and the datapath only see a small set of signals. Any wrapper that provides them
works with the rest of the PHY; this is the contract to implement when porting LiteSATA to a new
transceiver. All control signals are in `sys`; `sink` is in `sata_tx` and `source` in `sata_rx`.

| Signal | Direction | Meaning |
|----------------------|-----------|----------------------------------------------------------|
| `data_width` | Attribute | 16 or 32 |
| `ready` | Out | Transceiver reset and PLL lock done |
| `tx_idle` | In | Hold the TX pair in electrical idle |
| `tx_polarity` | In | Invert the TX pair |
| `tx_cominit_stb` | In | Request a COMRESET burst sequence |
| `tx_cominit_ack` | Out | COMRESET sent |
| `tx_comwake_stb` | In | Request a COMWAKE burst sequence |
| `tx_comwake_ack` | Out | COMWAKE sent |
| `rx_cominit_stb` | Out | COMINIT detected from the drive |
| `rx_comwake_stb` | Out | COMWAKE detected from the drive |
| `rx_cdrhold` | In | Hold the clock and data recovery |
| `rx_polarity` | In | Invert the RX pair |
| `rx_idle` | Out | Electrical idle detected; 0 on Xilinx, RLOS on ECP5 |
| `rxdisperr` | Out | Disparity error per byte (debug) |
| `rxnotintable` | Out | Invalid 8b/10b code per byte (debug) |
| `sink`, `source` | Stream | `data` and `charisk` of `data_width` bits |

The host sends COMRESET through the COMINIT pins of the transceivers: COMRESET and COMINIT have
the same burst and gap timing, and the transceiver ports carry the device-side name.

The CRG provides `cd_sata_tx`, `cd_sata_rx`, `tx_reset` and `rx_reset`. The controller asserts both
resets in its RESET state, and the RX reset alone on a misalignment. The `sata_tx` and `sata_rx`
resets also follow the PLL lock of the transceiver (the TX and RX ready of the DCU on ECP5).

In READY, the controller takes its idle signal from the datapath, which detects missing ALIGN
primitives ([RX idle and misalignment](#phy-idle)). The `rx_idle` of the transceiver only gates the
ALIGN detection of AWAIT-ALIGN. The Xilinx wrappers leave it at 0, and the ECP5 controller replaces
that state.
The `rxdisperr` and `rxnotintable` flags are crossed into `sys` but are not used by the controller
or the datapath.

## The controller state machine {#phy-ctrl}

The controller brings the link from power-up to the point where the core can send frames. It
sends COMRESET, waits for the COMINIT of the drive, exchanges COMWAKE, then exchanges ALIGN
primitives at the line rate, as [Out-of-band signalling](#primer-oob) describes. The figure below
shows its states.

![PHY controller states. Follow the thick arrows from RESET to READY: the left column is the OOB
handshake, the right column the ALIGN exchange and the ready link. The dotted lines are the timer
resets and the RX idle return to RESET.](phy_ctrl_fsm.svg)

| State | TX sends | Leaves when |
|--------------------|--------------------|---------------------------------------------------|
| RESET | Electrical idle | At once; restarts the RX reset sequence, resets the SATA clock domains |
| AWAIT-CRG-RESET | Electrical idle | Transceiver `ready`; sets the polarities |
| COMINIT | COMRESET | COMRESET sent and no COMINIT detected |
| AWAIT-COMINIT | Electrical idle | COMINIT detected |
| AWAIT-NO-COMINIT | Electrical idle | COMINIT detection released |
| CALIBRATE | Electrical idle | At once (no calibration is done) |
| COMWAKE | COMWAKE | COMWAKE sent |
| AWAIT-COMWAKE | Electrical idle | COMWAKE detected |
| AWAIT-NO-COMWAKE | Electrical idle | COMWAKE detection released |
| AWAIT-ALIGN | D10.2 | ALIGN or inverted ALIGN received |
| SEND-ALIGN | ALIGN | Three primitives other than ALIGN received |
| READY | SYNC | RX idle, or misalignment |
| RESET_RX | D0.0 (`0x00000000`) | Transceiver `ready` again |

**OOB handshake.** The controller keeps the TX pair in electrical idle from RESET to
AWAIT-NO-COMWAKE, and holds the clock and data recovery. It raises `tx_cominit_stb` and waits for
the acknowledge of the wrapper, which comes once the burst sequence is out (its length is set by
the transceiver attributes on Xilinx and by the generator on ECP5). It then waits for the
COMINIT of the drive to start and to end before it sends COMWAKE the same way.

**ALIGN exchange.** After the COMWAKE of the drive, the controller leaves electrical idle and sends
D10.2 characters (`0x4a4a4a4a`, no K character). This alternating pattern lets the receiver of the
drive lock. The drive then sends ALIGN primitives. The first ALIGN received moves the controller to
SEND-ALIGN, where it answers with ALIGN and releases the clock and data recovery.

**Entering READY.** In SEND-ALIGN the controller counts K28.3 primitives, the first byte of every
primitive except ALIGN. Three in a row move it to READY; an ALIGN in between restarts the count. In
practice these primitives are the SYNC that the drive sends once it has seen the ALIGN of the host.

**Stable link.** In READY the controller sends SYNC and starts a 5 ms timer. Only when that timer
expires does it assert `ready`. The datapath then switches the TX stream from the controller to the
link layer of the core, and the RX stream from the controller to the core.

### Timers {#phy-timers}

Two timers bound the handshake; when either expires, the whole state machine resets to RESET and
tries again. A third one delays `ready` once the link is up.

| Timer | Duration | Runs in |
|-----------------|----------|-------------------------------------------------------------|
| Retry timer | 10 ms | AWAIT-COMINIT and AWAIT-NO-COMINIT, then again in AWAIT-COMWAKE |
| ALIGN timer | 873 µs | AWAIT-ALIGN and SEND-ALIGN |
| Stability timer | 5 ms | READY, before `ready` rises |

The timers count `sys` cycles computed from `clk_freq`, so a wrong `clk_freq` changes every
duration. Without a drive, the controller loops through COMRESET and the 10 ms retry timer forever;
this is the expected state of an empty port.

The generic AWAIT-NO-COMWAKE state has no timer: it waits until the COMWAKE detection of the
transceiver drops. On the Xilinx transceivers that detection holds the flag only briefly. The ECP5
controller adds a 0.4 µs bound to this state, because its fabric detector holds the flag longer.

### Polarity {#phy-polarity}

The controller can correct an inverted pair in both directions, so that a board or an adapter with
swapped P and N lines links all the same. The receive side is detected from the first ALIGN, as
the figure below shows.

![RX polarity detection. The first ALIGN of the drive decodes as `0x7b4a4abc` when the RX pair is
straight and as `0x7bb5b5bc` when it is inverted; the controller sets the RX polarity accordingly
before it answers.](phy_polarity.svg)

An inverted pair turns D10.2 into D21.5 and leaves K28.5 and D27.3 as they are
([8b/10b coding](#primer-8b10b)). The inverted ALIGN therefore decodes as `0x7bb5b5bc`, which the
code names `ALIGN_N`. The controller sets the RX polarity to 1 when it sees `ALIGN_N` and
to 0 when it sees ALIGN, and keeps it for the rest of the link.

The transmit side cannot be observed, so the controller inverts the TX polarity each time it passes
through AWAIT-CRG-RESET. OOB bursts do not depend on polarity, and D10.2 inverted is D21.5, another
alternating pattern: the first dwords that the drive must decode are the ALIGN primitives of
SEND-ALIGN.

The TX polarity is a register outside the state machine, so the timer reset of the state machine
does not clear it. Every attempt passes through AWAIT-CRG-RESET, whether it starts after power-up,
after a timer reset or after a link in READY is lost on RX idle, and the attempts alternate: the
first one after power-up runs with TX polarity 0, the next one with 1, then 0 again. With a
swapped TX pair, the attempts with TX polarity 1 are the ones that can reach READY. The controller
never detects the TX polarity: it has no way to observe it.

### RX idle and misalignment {#phy-idle}

In READY, two conditions send the controller back. Both come from the datapath.

**RX idle.** The datapath restarts a timer on every ALIGN it receives (`LiteSATAPHYAlignTimer`).
When no ALIGN arrives for 4096 `sys` cycles, it raises `rx_idle` and the controller goes to RESET
for a full OOB sequence. A drive sends ALIGN pairs every 256 dwords, so silence means the drive was
unplugged, powered off or lost the link. The transceiver electrical idle detectors are not used.

**Misalignment.** After realignment, a K character can only be on byte 0 of a dword. A received
dword with a K character on bytes 1 to 3 sets `misalign`. The controller then resets the RX side of
the transceiver only, waits for it in RESET_RX, and returns to READY without a new OOB sequence.
`ready` drops for the 5 ms of the stability timer. During RESET_RX the controller sends D0.0
dwords (`0x00000000`), not SYNC. On ECP5 the DCU is not reset; only the receive logic in the fabric
restarts, and RESET_RX ends on the next cycle.

## Datapath {#phy-datapath}

The datapath connects the 32-bit `sys` side of the controller and the core to the `sata_tx` and
`sata_rx` sides of the transceiver. A multiplexer on TX and a demultiplexer on RX select the
controller before `ready` and the core after it.

| Path | Steps | Clock domains |
|------|---------------------------------------------------------------------|---------------------|
| TX | Multiplexer, CDC FIFO, 32-bit to 16-bit converter when needed | `sys` to `sata_tx` |
| RX | Byte realignment, 16-bit to 32-bit converter when needed, CDC FIFO | `sata_rx` to `sys` |

Both clock domain crossings are 8-entry asynchronous FIFOs. Neither side applies back-pressure: the
transceiver takes a word every `sata_tx` cycle and delivers one every `sata_rx` cycle.

### Byte realignment {#phy-rx-align}

The transceiver aligns its words on the comma, but the datapath does not rely on the position it
picks. It keeps the previous word next to the new one and, when a K character arrives, records
its byte position. It then selects the bytes so that the K character lands on byte 0. The figure
below shows the 16-bit case.

![RX realignment at 16 bits. A K28.5 on byte 1 of a word shifts the output by one byte, so that it
lands on byte 0; two 16-bit words are then paired into a dword with the K character on byte 0, as
in the ALIGN primitive.](phy_rx_align.svg)

At 16 bits, a second step pairs two words into a dword. A K character in the upper half of a dword
resets the converter, which restarts the pairing one word later. At 32 bits the byte shift alone is
enough. Either way, the first byte of every primitive lands on byte 0, where the link layer
expects it.

### Clocks and the system clock {#phy-clocks}

The transceiver clocks run at the word rate of the line. The core always runs at 32 bits in `sys`,
so `sys` must carry one dword per transceiver dword.

| Generation | Line rate | Transceiver clocks, 16 bits | 32 bits | Dword rate |
|------------|-----------|-----------------------------|----------|------------|
| Gen1 | 1.5 Gbps | 75 MHz | 37.5 MHz | 37.5 MHz |
| Gen2 | 3 Gbps | 150 MHz | 75 MHz | 75 MHz |
| Gen3 | 6 Gbps | 300 MHz | 150 MHz | 150 MHz |

The `sys` clock must be at least the dword rate: 37.5, 75 and 150 MHz for Gen1, Gen2 and Gen3. At 16
bits the datapath states it as "`sys` faster than half the `sata_rx` clock". The rule that the LiteX
integration applies is in [Integration](#integration).

A 32-bit transceiver width halves the transceiver clocks for the same line rate. It is the way to
reach Gen3 on parts where 300 MHz in the fabric is hard to meet.

## Link monitor and events {#phy-link-monitor}

`ready` can drop briefly during a link: a misalignment costs 5 ms, an RX idle a full OOB sequence.
The link monitor filters these into a debounced link state (`LiteSATAPHYLinkMonitor`).

| Output | Behaviour |
|----------------------|---------------------------------------------------------------|
| `link_up` | Rises with `ready`; falls once `ready` stays low for the timeout |
| `connect_irq` | One-cycle pulse when `link_up` rises |
| `disconnect_irq` | One-cycle pulse when `link_up` falls |

With the default `link_down_timeout` of 100 ms, a short re-initialization is not reported, while an
unplugged drive is. The pulses suit a LiteX `EventSourcePulse`. The standalone generator exports
them as its `connect` and `disconnect` ports ([Integration](#integration)).

## Registers {#phy-registers}

With `with_csr=True`, the PHY adds two registers ([PHY registers](#reg-sata-phy)).

| Register | Field | Meaning |
|----------|--------------|------------------------------------------------------------|
| `enable` | Bit 0 | 1 at reset; 0 holds the transceiver in reset |
| `status` | `ready` | Transceiver and controller ready: the link is up |
| `status` | `tx_ready` | TX reset sequence done |
| `status` | `rx_ready` | RX reset sequence done |
| `status` | `ctrl_ready` | Controller in READY after the 5 ms stability time |
| `status` | `link_up` | Debounced link state of the link monitor |

Clearing then setting `enable` restarts the transceiver. The controller sees the ALIGN primitives
stop, returns to RESET and runs a full OOB sequence. The BIOS `sata_init` command and the
`test_init.py` script clear and set `enable`, then poll `status`
([Getting started](#getting-started)). With the five fields set, `status` reads `0x1f`.

## Limitations {#phy-limitations}

- One generation, fixed at build time. A drive that supports several generations tries its rates in
  turn during the ALIGN exchange; the host answers only at its own rate. There is no rate fallback.
- No power management: the PHY never enters or leaves the partial or slumber states.
- The transceiver electrical idle detectors are unused; RX idle is the absence of ALIGN.

## In the code {#phy-code}

| File | Content |
|-------------------------------------|-------------------------------------------------------|
| `litesata/phy/__init__.py` | `LiteSATAPHY`: selection, connections, link monitor, registers |
| `litesata/phy/ctrl.py` | `LiteSATAPHYCtrl`, `LiteSATAPHYLinkMonitor` |
| `litesata/phy/datapath.py` | `LiteSATAPHYDatapath` and its TX, RX and ALIGN timer parts |
| `litesata/common.py` | `phy_description`, `primitives` |
| `test/test_phy.py` | Device selection, RX realignment |
| `test/test_phy_ctrl.py` | Controller constructor, ECP5 early ALIGN latch, link monitor |

Where to go next: [Xilinx transceiver PHYs](#xilinx-phys) for the wrappers,
[Lattice ECP5 PHY with OOB in fabric](#ecp5-phy) for OOB in the fabric,
[Link layer](#link-layer) for what flows once `ready` is set,
[Bringing up the link](#dbg-oob) for a link that does not come up.
