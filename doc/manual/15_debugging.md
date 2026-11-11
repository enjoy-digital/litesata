# Debugging {#debugging}

This chapter is the method to follow when a link does not come up or a command fails on hardware:
which registers to read first, how to decode the PHY status, what goes wrong in the OOB handshake
on each FPGA family, and how to capture the state machines with LiteScope. It assumes a bench
target or a SoC built with `add_sata` ([Example designs and targets](#examples)).

## Method {#dbg-method}

Work from the outside in, and change one thing at a time.

1. Check that the host reaches the right bitstream: `litex_cli --ident` prints the identifier of
   the SoC, such as "LiteSATA bench on KC705".
2. Read the PHY status before anything else ([Reading the PHY status](#dbg-status)). It tells
   whether the transceiver, the OOB handshake or the drive is the problem.
3. Run IDENTIFY DEVICE. A drive that answers with its model and capacity proves the link, the core
   and the command path in one step.
4. Only then run BIST transfers, and capture traces with LiteScope when a register shows the
   failing state but not its cause.

```sh
# From the build directory of the target (it holds csr.csv).
# Identifier of the SoC, then the PHY registers
litex_cli --ident
litex_cli --regs --filter sata_phy
# Restart the PHY and poll its status
python3 ../bench/test_init.py
# IDENTIFY only
python3 ../bench/test_bist.py -i
```

Keep a known-good drive as a reference and note which drive gave each result: drives differ in
their OOB timing, their tolerance to the signal and their behaviour after power-up. A link that
fails with one drive and works with the reference points to the drive, not to the gateware.

## Reading the PHY status {#dbg-status}

The `status` register of the PHY has five bits ([Register reference](#registers)). Each bit is the
result of one stage of the bring-up, in this order.

| Bit | Name | Set when |
|-------|---------------|------------------------------------------------------------|
| 1 | `tx_ready` | The transceiver TX is initialized: its PLL locked on the reference clock |
| 2 | `rx_ready` | The transceiver RX is initialized |
| 3 | `ctrl_ready` | The controller finished the OOB and ALIGN handshake and saw 5 ms of stable link |
| 0 | `ready` | Transceiver and controller ready: the core can send frames |
| 4 | `link_up` | `ready`, debounced: falls only after 100 ms without `ready` |

`test_init.py` prints the status word and, on failure, the three middle bits. Read the first bit at
0 in the table order: the stage before it is the one to look at.

| Status | Likely cause | Look at |
|--------------------------------------|-------------------------------------|--------------------------------------------|
| `tx_ready` 0 | No reference clock, or not 150 MHz | Clock source, pins, `use_gtgrefclk` |
| `tx_ready` 1, `rx_ready` 0 | RX initialization does not end | Reference clock, RX buffer settings |
| Both ready, `ctrl_ready` 0 | OOB or ALIGN handshake fails | Drive, power, cable, adapter, [Bringing up the link](#dbg-oob) |
| `ready` 1, `link_up` toggles | Link drops and comes back | [Link drops and misalignment](#dbg-drops) |
| `ready` 1, IDENTIFY fails | Command path or drive refuses | [Command failures](#dbg-commands) |

`enable` restarts the transceiver when written 0 then 1; the controller sees the ALIGN primitives
stop, returns to RESET and runs a new OOB sequence. The controller also retries by itself, so a
status read once after power-up may catch it in the middle of a retry: poll it for a few seconds, as
`test_init.py` does, before concluding.

## Bringing up the link {#dbg-oob}

The controller drives the OOB sequence of [PHY architecture and OOB controller](#phy-architecture).
When `ctrl_ready` stays at 0, the question is where its state machine stops.

| State reached | Meaning | Next check |
|-------------------------|-------------------------------------------|-------------------------------------------|
| `COMINIT`, `AWAIT-COMINIT` | The drive does not answer the COMRESET | Drive powered, cable seated, TX lanes |
| `AWAIT-COMWAKE` | COMINIT seen, no COMWAKE from the drive | RX lanes, signal level, OOB detection |
| `AWAIT-ALIGN` | COMWAKE done, no ALIGN received | Line rate, reference clock, RX pair order |
| `SEND-ALIGN` | ALIGN received, primitives not stable | Signal quality, comma alignment |
| `READY` then `RESET` | Link up, then lost | [Link drops and misalignment](#dbg-drops) |

Two timers bound the sequence. A state that waits for the drive restarts the whole sequence after
10 ms, and the ALIGN exchange after 873 µs. A controller that cycles through the states every few
milliseconds is retrying: the drive answers part of the sequence, and the next state shows which
part fails.

The RX polarity needs no setting: an ALIGN received inverted sets the RX polarity of the
transceiver. The TX polarity is not detected: it alternates on each attempt, 0 on the first one
after power-up, then 1, then 0 ([Polarity](#phy-polarity)).

The drive answers COMRESET only once it is powered and ready. A drive powered after the FPGA, or a
hot-plugged drive, is found by the retries of the controller without a restart from software.

## OOB on each FPGA family {#dbg-oob-family}

OOB depends on the transceiver: test both families when changing the controller.

**Xilinx GTP, GTX, GTH and GTY.** The transceiver makes and detects the OOB bursts in hardware: the
controller pulses its COMINIT and COMWAKE requests and reads its detection outputs. On the GTX, a
toggle flip-flop divides the reference clock by two and feeds the 75 MHz result to the OOB clock
input (`CLKRSVD`), so a wrong reference clock also breaks OOB detection. The GTP wrapper drives
`SIGVALIDCLK` the same way, but selects the PMA clock for OOB (`RXOOB_CLK_CFG`). The electrical idle
output of the transceiver is not used: the datapath detects a silent drive from the missing ALIGN
primitives instead.

**Lattice ECP5.** The DCU has no usable OOB support and its electrical idle cannot follow the OOB
timing. The PHY makes the bursts and the gaps with data patterns and classifies the gaps it
receives from the loss-of-signal output of the DCU ([Lattice ECP5 PHY with OOB in
fabric](#ecp5-phy)). Its limits matter when debugging:

- Gen2 and a 16-bit datapath only; the PHY raises an error for any other setting.
- The gaps that the PHY sends are data, seen as idle by the squelch of the drive. A drive with an
  unusual squelch may not recognize them.
- The receive gap windows of the checker are 55 ns to 175 ns for COMWAKE and 175 ns to 525 ns
  for COMINIT, counted in system clock cycles. A silence one cycle longer than the COMINIT window
  clears the counts and the detections, so a detection stays raised for about 525 ns after the
  last burst ([OOB checker](#ecp5-oob-rx)). The bench target runs at 90 MHz, the LiteX-Boards
  ECPIX-5 target at 75 MHz.
- The ECP5 controller latches an ALIGN that arrives once its own COMWAKE is sent, before the
  COMWAKE detection of the drive has ended, since some drives start their ALIGN early.

## Hardware setup {#dbg-hardware}

Most bring-up failures are in the setup, not in the gateware.

- **Reference clock.** The transceiver needs 150 MHz. On an FMC adapter it comes from the adapter;
  without one, a PLL of the FPGA makes it. On UltraScale+, a clock from the fabric needs
  `use_gtgrefclk=True` and a clock on the dedicated pins needs `use_gtgrefclk=False`, as
  `bench/zcu102.py` sets for `--refclk external`.
- **Adapters.** The bench targets use an FMC RAID adapter, an SFP-to-SATA adapter or a modified
  PCIe riser ([Supported FPGAs and boards](#boards)). Check which lanes of the connector the target
  uses, and that the adapter couples the lanes as SATA expects.
- **Cable and power.** Seat the cable on both ends, power the drive from a supply that is on before
  the link starts, and prefer a short cable for a first bring-up.
- **Line rate.** A drive accepts the lower generations, but some FPGAs cannot reach Gen3. Start at
  Gen1 or Gen2, then raise the generation once IDENTIFY works.

## Link drops and misalignment {#dbg-drops}

Once ready, the controller watches two conditions and restarts part of the link:

| Condition | Detection | Action |
|------------------------|------------------------------------------------|-------------------------------|
| Misalignment | A K character on a byte other than the first | Resets the RX (fabric only on ECP5) |
| Idle drive | No ALIGN for 4096 system clock cycles | Restarts the whole OOB sequence |

A drive sends two ALIGN primitives every 256 dwords, so the idle condition means that the drive
stopped sending, was unplugged or lost its power. Repeated misalignment points to the signal: an
adapter, a cable, or a line rate the setup does not support.

The `link_up` bit and the `connect` and `disconnect` events ignore the short restarts: `link_up`
falls only after 100 ms without `ready` (`link_down_timeout` of `LiteSATAPHY`). A design that needs
to see each restart reads `ready` instead.

## Command failures {#dbg-commands}

A command that fails ends with the `failed` flag of its response
([Transport and command layers](#transport-command)). The frontends report it in their registers:
`error` of Identify and of the DMAs, `aborted` of the BIST.

| Failure | Cause | What to check |
|----------------------------|----------------------------------------------|--------------------------------------|
| ERR or DF in the status | The drive refused or failed the command | LBA beyond the capacity, drive health |
| CRC error | A received Data FIS had a bad CRC | Signal quality, then the link layer |
| Short read | Fewer data dwords than the sectors asked | The drive ended the command early |
| No response | The drive never ends the command | Drive powered and linked, see below |

The command layer has no timeout: a drive that never answers keeps the command pending, and with
it the user port. The BIOS bounds each read and write with `SATA_OP_TIMEOUT_US` and retries it
`SATA_OP_RETRIES` times ([Build options](#build-options)), but a retry after a timeout cannot
start the DMA again and the core stays blocked until it is reset ([BIOS commands](#integ-bios)). A
design without the BIOS has to watch the `done` registers itself.

The status and error bytes of the last Register D2H FIS are available as the signals `d2h_status`
and `d2h_errors` of the command layer, not as registers. Add them to a LiteScope capture to see why
a drive refused a command.

## LiteScope captures {#dbg-litescope}

LiteScope records signals of the design into a buffer of the FPGA and dumps them to a VCD file. The
Xilinx bench targets add an analyzer with `--with-analyzer` (`kc705`: `--with-global-analyzer`), on
the state machines of every layer; `ecpix5` has none.

| Signal group | Signals |
|------------------------|------------------------------------------------------------------|
| Transceiver init | `tx_init` and `rx_init` state machines (Xilinx targets) |
| PHY controller | Its state machine and `ready` |
| OOB (`acorn` only) | COMINIT and COMWAKE requests, acknowledges and detections; TX and RX polarity; RX idle |
| PHY streams | `sink` and `source` of the PHY: the dwords and K flags of both directions |
| Command streams | `sink` and `source` of the command layer |
| Core state machines | Link TX and RX, transport TX and RX, command TX and RX |

The DMA analyzers of `kc705` record the state machine of one DMA, its user port and its Wishbone
bus instead. The analyzers are 512 samples deep, 2048 for the DMA ones, so a capture holds a short
window: trigger on the event, not before it.

```sh
# From the build directory: build with the analyzer, then capture with the bridge running
python3 ../bench/acorn.py --gen 2 --with-analyzer --build --load
litescope_cli --list
# Trigger on the rising edge of a signal, keep 64 samples before it
litescope_cli -r <signal> --offset 64
```

On `kc705`, add `--csv global_analyzer.csv` (or the CSV of a DMA analyzer,
`sector2mem_analyzer.csv` or `mem2sector_analyzer.csv`): `litescope_cli` reads `analyzer.csv` by
default.

`--list` prints the names of the signals in the capture. Trigger on the PHY controller `ready` to
see the end of the handshake, or on the first command of the command layer `sink` to follow a
command through the layers. Open `dump.vcd` with a waveform viewer.

## BIOS commands {#dbg-bios}

In a SoC built with `add_sata` and a CPU, the BIOS commands give the same steps from the console.
`sata_init` restarts the PHY up to 16 times, waits for its `ready` bit and runs IDENTIFY. It prints
the model and the capacity of the drive, or `Failed`; [Step 8 of Getting started](#gs-bios) shows
real captures of both cases.

The BIOS finds the SATA registers by their names: the commands exist only when the SoC calls
`add_sata` with the default name `sata` and the PHY is named `sata_phy`. A SoC that uses other
names builds without the SATA commands, which is a common reason for a missing `sata_init`.

`sata_read <sector>` then dumps one sector through the Sector2Mem DMA. Read a sector whose content
is known, such as sector 0 of a formatted drive with its partition table, to check the data path
and the byte order.

## ECP5 bring-up notes {#dbg-ecp5}

The ECP5 PHY was brought up on the ECPIX-5 with the method of this chapter, and the measurements of
that work show what to look for on a new transceiver without SATA support.

The drive of that bench sent a COMINIT beacon on its own, about every 10 ms, while it waited for a
host. A COMINIT received after a COMRESET is therefore not a proof that the drive heard the
COMRESET: it may be the beacon. The sweep below sent OOB sequences with gaps of growing length and
measured the beacon rate of the drive in three passes.

```text
gap(ns)  pass1 pass2 pass3
    53    600   600   600   (below the COMWAKE window)
   107     78    26     6   SUPPRESSED       <- COMWAKE nominal 106.7 ns
   160   1280  1280  1280   rate doubles
   213    600   600   600
   320      0     0     0   FULLY SUPPRESSED <- COMRESET nominal 320 ns
   427      0     0     0   FULLY SUPPRESSED
   533    600   600   600   (above the window)
```

Source: the LiteSATA ECP5 article of the Enjoy-Digital blog, ECPIX-5, Gen2. The beacon stops
when the gaps of the host land in the windows that the drive recognizes, around the nominal
COMWAKE and COMRESET gaps. A sweep like this one finds the gap windows of a drive without an
oscilloscope.

The same work gave the rules applied in the ECP5 PHY: send SYNC rather than a constant data word
when the core has nothing to send, since some drives drop a link that carries a constant word,
and qualify the ALIGN primitives on the full dword while the receive aligner settles.

## Known issues {#dbg-known}

The behaviours below are those of the code; check them first when a symptom matches.

- The command layer has no timeout ([Command failures](#dbg-commands)).
- A READ DMA EXT with a count of 0, which ATA defines as 65536 sectors, is not handled by the
  command layer.
- A DMA started with `nsectors` at 0 does not end ([Register reference](#registers)).
- Each BIST command of a run clears `errors` and `aborted`: after a run of several loops, they
  report the last command only.
- The link layer does not act on an R_ERR answer to a frame it sent: it neither retries the frame
  nor fails the command, and relies on the status the drive reports afterwards.
- The host never sends CONT, so the link carries repeated primitives as they are.
- The link events of the PHY are not wired to the interrupts of `add_sata`.

## Where to go next {#dbg-next}

[PHY architecture and OOB controller](#phy-architecture) for the state machine,
[Lattice ECP5 PHY with OOB in fabric](#ecp5-phy) for the ECP5 specifics,
[Simulation and verification](#simulation) to reproduce a failure without a board, and
[Register reference](#registers) for every register.
