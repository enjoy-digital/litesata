# Lattice ECP5 PHY with OOB in fabric {#ecp5-phy}

The ECP5-5G has transceivers fast enough for SATA Gen2 but no SATA support: no OOB engine, and an
electrical idle too slow for OOB timing. The ECP5 PHY builds OOB in the fabric, with gaps made of
data. This chapter describes that PHY, its OOB generator and checker,
the controller changes it needs and the lessons for other transceivers without SATA support.

The shared controller, datapath and registers are in
[PHY architecture and OOB controller](#phy-architecture); the OOB signals themselves in
[Out-of-band signalling](#primer-oob).

## At a glance {#ecp5-glance}

| Item | Value |
|-------------------|------------------------------------------------------------------------|
| Devices | ECP5-5G, device names starting with `LFE5UM5G-` |
| Transceiver | DCU channel through `SerDesECP5` of LiteICLink, raw 20-bit words |
| Generation | Gen2 only, 3 Gbps |
| Transceiver width | 16 bits only |
| Reference clock | 150 MHz, from a fabric clock signal |
| Clock domains | `sata_tx`, `sata_rx` at 150 MHz; `sys` 75 to about 95 MHz, 90 MHz on the bench |
| OOB | Generator and checker in the fabric |
| Bench target | `bench/ecpix5.py`, LambdaConcept ECPIX-5 |
| Status | Validated on hardware at Gen2 |

The hardware result recorded in the commit history is: "Validated with complete IDENTIFY and a
16 MiB pseudorandom write/verify BIST." It was obtained on the ECPIX-5, built with the open-source
toolchain.

## Why the ECP5 is different {#ecp5-why}

OOB carries its information in the length of its gaps: 106.7 ns for COMWAKE and 320 ns for
COMRESET and COMINIT. A Xilinx transceiver makes these gaps with its own electrical idle and
detects them in hardware. The DCU of the ECP5 offers neither.

**Transmit.** The DCU can put its output in electrical idle, but not quickly. In the public
bring-up work, the shortest electrical idle measured on its output was about 220 ns in an earlier
attempt and about 650 ns in the bring-up that produced this PHY. Both are longer than a COMWAKE
gap, and the second is longer than a COMRESET gap.

**Receive.** The DCU has a loss-of-signal detector (RLOS), which tells whether the input carries
a signal. It does not classify bursts and gaps: the fabric has to time them.

The PHY answers both with fabric logic. On transmit, it makes the gaps with a constant data word:
the line carries a voltage, but no transitions. The squelch detector of a drive responds to
transitions, so it reads such a gap as idle. On receive, it times the RLOS gaps in `sys` cycles.

## Blocks {#ecp5-blocks}

The figure below shows the parts of the ECP5 PHY around the shared controller.

![ECP5 PHY blocks. The OOB generator drives the raw TX path of the DCU; the SYNC filler sends the
words of the datapath; the OOB checker times the gaps of the loss-of-signal output. The controller
is the generic one with three states changed.](ecp5_blocks.svg)

| Block | Class | Domain | Role |
|---------------|-------------------------------------|--------|---------------------------------|
| PHY | `ECP5LiteSATAPHY` | All | DCU, OOB blocks, SYNC filler |
| CRG | `ECP5LiteSATAPHYCRG` | DCU | `sata_tx`, `sata_rx` clocks |
| OOB generator | `ECP5SATAOOBGenerator` | `tx` | Bursts and gaps as raw words |
| OOB checker | `ECP5SATAOOBChecker` | `sys` | COMINIT, COMWAKE from RLOS gaps |
| Controller | `ECP5LiteSATAPHYCtrl` | `sys` | Generic, three states changed |

The PHY drives the DCU through `SerDesECP5` with `with_oob=True`. In that mode LiteICLink exchanges
raw 20-bit words with the DCU and does the 8b/10b coding and the comma alignment in the fabric. It
brings the TX side up before the RX side, since the host must send COMRESET before it receives
anything. The PHY reports `ready` as soon as the TX side is up.

The PHY also applies a set of DCU receive settings that decouple the clock and data recovery from
the loss-of-signal detector. The code credits them to another open-source ECP5 SerDes design.

| Setting | Value |
|------------------------|---------|
| `CHX_REQ_EN` | `0b1` |
| `CHX_REQ_LVL_SET` | `0b01` |
| `CHX_RXTERM_CM` | `0b10` |
| `CHX_RX_LOS_HYST_EN` | `0b0` |
| `CHX_RX_LOS_LVL` | `0b010` |
| `CHX_PDEN_SEL` | `0b0` |
| `D_CDR_LOL_SET` | `0b10` |

## OOB generator {#ecp5-oob-tx}

The generator turns a COMRESET or COMWAKE request into six bursts and six gaps of raw words. It runs
in the transmit clock domain, at 150 MHz, one 20-bit word every 6.7 ns. The figure below shows the
waveform it produces.

![ECP5 OOB transmit. Each burst is a square wave of raw words; each gap is a constant word, data
without transitions. The output is in electrical idle only before and after the
sequence.](ecp5_oob_tx.svg)

| Item | Gen1 UI | Time | `tx` cycles |
|----------------|---------|----------|-------------|
| Burst | 160 | 106.7 ns | 16 |
| COMWAKE gap | 160 | 106.7 ns | 16 |
| COMRESET gap | 480 | 320 ns | 48 |

OOB timing is defined in Gen1 UI even on a Gen2 link, so the generator converts each length with
the Gen1 rate: cycles = UI times the TX clock divided by 1.5 GHz.

**Bursts.** A burst alternates the raw words `0x0f0f0` and `0xf0f0f`. Each word holds runs of four
identical bits, and the alternation keeps the runs regular across the word boundary. At 3 Gbps
this is a 375 MHz square wave, which the code calls the Gen1 OOB carrier.

**Gaps.** A gap is the raw word 0: a constant level without transitions. The electrical idle of
the DCU is only used outside the sequence. The generator releases it one cycle before the first
burst and returns to it right after the last gap.

**Handshake.** The request of the controller crosses into the transmit domain. When the last gap
ends, the generator pulses `finish`, which crosses back into `sys` and acknowledges the request.
The controller then sees the same `tx_cominit_ack` or `tx_comwake_ack` as with a Xilinx
transceiver.

## OOB checker {#ecp5-oob-rx}

The checker times the gaps of the loss-of-signal output and recognizes COMINIT and COMWAKE from
them. It runs in `sys`. The figure below shows the windows it uses.

![ECP5 OOB receive windows. A gap between 55 and 175 ns counts as a COMWAKE gap, a gap between
175 and 525 ns as a COMINIT gap; the nominal gaps sit inside each window. Gaps outside both windows
clear the count. A silence one cycle longer than the COMINIT window, 48 `sys` cycles (533 ns) at
90 MHz, clears the detections.](ecp5_oob_rx.svg)

1. The RLOS output crosses into `sys` and is filtered: a change must hold for three `sys` cycles
   before the checker sees it.
2. While RLOS reports no signal, a counter counts `sys` cycles. When the signal returns, the gap
   has ended and its length is compared with the two windows.
3. Each gap in a window increments the count of that window and clears the other. A gap outside
   both windows clears both.
4. The fourth matching gap in a row raises `cominit_det` or `comwake_det`.
5. A silence of the quiet interval clears both detections and both counts.

Four gaps are enough to recognize a six-burst sequence. The windows are wide, around the receive
tolerances of [Out-of-band signalling](#primer-oob), not the nominal gaps. The COMINIT bounds are
those of the specification; the COMWAKE minimum of 55 ns is the choice of the checker, above the
35 ns that the specification allows. The detection stays
raised until the line has been quiet for the quiet interval, which is how the controller sees the
end of a COMINIT.

| Item | Rule | At 90 MHz |
|-----------------|----------------------|------------------|
| COMWAKE window | 55 to 175 ns | 5 to 15 cycles |
| COMINIT window | 175 to 525 ns | 16 to 47 cycles |
| Quiet interval | One cycle more than the COMINIT window, 32 cycles at least | 48 cycles, 533 ns |

The quiet interval follows the `sys` frequency: `max(32, init_max + 1)`, where `init_max` is the
upper bound of the COMINIT window in cycles. It is longer than any gap the checker classifies, so it
never clears the counts in the middle of a COMINIT or COMWAKE gap. The 32-cycle floor applies
below about 59 MHz only, under the Gen2 minimum of 75 MHz ([Clocks and the system
clock](#phy-clocks)). A simulation test, `test/test_ecp5_oob_checker.py`, checks the detection of
COMINIT (320 ns gaps) and COMWAKE (106.7 ns gaps) at 75, 90, 100, 125 and 150 MHz. The bench
target runs `sys` at 90 MHz, the ECPIX-5 target of LiteX-Boards at 75 MHz.

## Controller overrides {#ecp5-ctrl}

The ECP5 controller is the generic one with three states replaced (`ECP5LiteSATAPHYCtrl`). The
changes deal with a detection flag that holds longer, with drives that send ALIGN early, and with a
receive aligner that settles after the ALIGN exchange has started.

| State | Generic behaviour | ECP5 behaviour |
|----------------------|------------------------------|---------------------------------------------|
| AWAIT-NO-COMWAKE | Wait for COMWAKE to drop | Same, or 0.4 µs at most |
| AWAIT-ALIGN | Wait for ALIGN, CDR held | Use an ALIGN latched since COMWAKE, CDR free |
| SEND-ALIGN | Three K28.3 primitives in a row | Three qualified dwords in a row |

**Bounded COMWAKE.** Once its COMWAKE ends, a drive soon sends ALIGN. If the line does not stay
quiet for the quiet interval in between, the COMWAKE detection of the checker stays raised. The
controller leaves AWAIT-NO-COMWAKE after 0.4 µs even if the flag is set.

**Early ALIGN.** The controller latches any ALIGN, straight or inverted, that arrives after it has
sent its COMWAKE, with its polarity. AWAIT-ALIGN moves on as soon as the latch is set, so a drive
that answers early is not missed. The clock and data recovery is not held in this state: it keeps
tracking the ALIGN stream.

**Qualified primitives.** In SEND-ALIGN, the controller compares the whole dword, not only its
first byte. A dword that matches a known primitive other than ALIGN counts at once. A repeated
ALIGN counts only after 20 µs in the state, the time for the aligner to settle. Three counted dwords
in a row move the controller to READY; any other dword restarts the count.

## TX filler and receive flags {#ecp5-sync}

When the TX datapath has no valid word for a cycle, the ECP5 PHY sends SYNC, as two 16-bit
halves, rather than a constant data word. The code states the reason:
"persistent D0.0 makes devices drop a healthy link".

On receive, the PHY reports the invalid-code flags of the fabric 8b/10b decoders as `rxnotintable`
and drives `rxdisperr` to 0. Neither flag is used by the controller or the datapath.

## Clocks and integration {#ecp5-clocks}

The ECP5 PHY takes its 150 MHz reference clock from a clock signal of the design: `refclk` must be a
`Signal` or a `ClockSignal`, and there is no dedicated-pin path. The `refclk_freq` parameter
(150 MHz) sets the PLL of the DCU, which runs the line at 3 Gbps. The transmit and receive clocks
run at the line rate divided by 20, 150 MHz.

| Parameter | ECPIX-5 bench | Role |
|--------------|---------------------|--------------------------------------------------|
| `refclk` | Fabric PLL, 150 MHz | Reference clock of the DCU PLL |
| `gen` | `"gen2"` | The only generation accepted |
| `data_width` | 16 | The only width accepted |
| `dual` | 1 | DCU of the channel |
| `channel` | 0 | Channel in the DCU |
| `clk_freq` | 90 MHz | `sys` clock, below about 95 MHz ([OOB checker](#ecp5-oob-rx)) |

The `enable` register resets the DCU initialization of LiteICLink instead of a transceiver reset
sequence. The `tx_ready` and `rx_ready` fields of `status` report the TX and RX sides of the DCU.

The ECP5 PHY is built from a LiteX SoC: the bench target `bench/ecpix5.py`, or the ECPIX-5 target
of LiteX-Boards with `--with-sata`. The standalone generator does not offer it
([Integration](#integration)).

```sh
python3 -m litex_boards.targets.lambdaconcept_ecpix5 --with-sata --build --load
```

The board then answers the BIOS `sata_init` and `sata_read` commands
([Getting started](#getting-started)).

## Hardware result {#ecp5-result}

The ECP5 PHY reads the full IDENTIFY data of a drive. The decoded fields below come from the
public article of the Enjoy-Digital blog on the ECP5 PHY, on the ECPIX-5 at Gen2.

```text
model       TOSHIBA DT01ACA100
firmware    MS2OA8U0
sectors     1,953,525,168
capacity    1,000,204,886,016 bytes
features    Gen1, Gen2, Gen3, LBA48
```

The drive supports Gen3 and links at Gen2, the rate of the host. The gap sweep that showed a drive
accepting gaps made of data is in [ECP5 bring-up notes](#dbg-ecp5).

## Porting to other transceivers {#ecp5-porting}

The ECP5 PHY shows what a transceiver without SATA support needs. The same steps apply to any
transceiver that offers raw data, 8b/10b and a loss-of-signal output.

- Make the gaps with a constant data word when electrical idle cannot switch within about 100 ns.
  Keep electrical idle for the time outside the OOB sequences.
- Make the bursts with a transition-rich pattern at the Gen1 OOB frequency, timed in Gen1 UI.
- Classify the received gaps from the loss-of-signal output with wide windows, and wait for several
  matching gaps in a row before declaring a detection.
- Expect the drive to send ALIGN right after its COMWAKE: latch it early, and bound any wait for a
  detection flag to drop.
- Never let the TX path idle on a constant word once the link is up: fill with SYNC.
- Implement the [transceiver contract](#phy-contract) and reuse the generic controller, overriding
  only the states that need it.

## Limitations {#ecp5-limitations}

- Gen2 and a 16-bit width only; `LiteSATAPHY` raises `NotImplementedError` for another generation
  and the PHY asserts on another width.
- The gaps made of data rely on the squelch detector of the drive. The public bring-up notes report
  that some drives accept them and others do not; a drive that rejects them never gets past OOB.
- Not available in the standalone generator.

## In the code {#ecp5-code}

| File | Content |
|-----------------------------------------|---------------------------------------------------|
| `litesata/phy/ecp5sataphy.py` | The PHY, CRG, OOB generator, OOB checker, controller |
| `litesata/phy/__init__.py` | Selection on `LFE5UM5G-`, Gen2 check |
| `bench/ecpix5.py` | ECPIX-5 bench target, BIST, UARTBone |
| `test/test_phy_ctrl.py` | Early ALIGN latch of the ECP5 controller |

Where to go next: [PHY architecture and OOB controller](#phy-architecture) for the generic
controller, [ECP5 bring-up notes](#dbg-ecp5) for the measurements, [Supported FPGAs and
boards](#boards) for the ECPIX-5 wiring.
