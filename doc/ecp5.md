# Experimental ECP5-5G SATA support

This branch contains an experimental Gen2 SATA PHY for the Lattice ECP5-5G
DCU, validated on the LambdaConcept ECPIX-5 SATA connector (DCU1, channel 0).
It can establish a link with the tested drive, but IDENTIFY DEVICE is not yet
working end to end. It must not yet be presented as complete ECP5 support.

## Supported configuration

- ECP5-5G, Gen2 (3.0 Gbit/s) only.
- 16-bit DCU interface with fabric 8b/10b and fabric word alignment.
- 150 MHz DCU reference clock (`linerate/20`, using the x20 multiplier).
- ECPIX-5 system clock defaults to 90 MHz. It must remain above 75 MHz for the
  16-to-32-bit receive stride converter.
- ECPIX-5 placement is DCU1/channel 0. Other placements are selected with the
  existing `dual` and `channel` arguments but have not been hardware-qualified.

Gen1 is deliberately rejected instead of exposing the unsuccessful half-rate
experiments as a supported mode. The normal `LiteSATAPHY` API also keeps the
bring-up controls private. The ECPIX-5 bench explicitly adds the OOB debug
CSRs needed by the acceptance runner.

The ECP5 DCU additionally needs `D_SYNC_LOCAL_EN=1`. Without it the TX PLL can
report lock while the TX word clock and gearbox remain stopped. The matching
LiteICLink change is commit `0db96f7` on branch `ecp5-dcu-tx-fix`.

## Build and run

Build the instrumented ECPIX-5 image:

```sh
./bench/ecpix5.py --with-bist --with-analyzer --build
```

The Trellis build defaults to placement seed 3, the first seed that met all
three timing constraints with the command-path analyzer enabled. Override it
with `--seed` when screening a materially changed image.

The default Trellis build produces:

```text
build/lambdaconcept_ecpix5/gateware/lambdaconcept_ecpix5.bit
csr.csv
analyzer.csv
```

Start UARTBone with the ECPIX-5 UART interface that corresponds to your board:

```sh
litex_server --uart --uart-port=/dev/ttyUSB2 --uart-baudrate=1000000
```

Then run one bounded, strict attempt:

```sh
python3 bench/ecpix5_sata_test.py \
    --bitstream build/lambdaconcept_ecpix5/gateware/lambdaconcept_ecpix5.bit \
    --csr-csv csr.csv \
    --analyzer-csv analyzer.csv \
    --output-dir /tmp/litesata-ecp5-strict
```

The runner:

- programs an explicitly named image, or requires `--reuse-bitstream`;
- hashes the bitstream and CSV maps;
- records the Git revision, dirty status, untracked hashes, and a binary
  `source.patch` for tracked working-tree changes;
- parks the line before the test and again in a `finally` block;
- arms the analyzer before enabling the PHY;
- bounds link, signature, IDENTIFY, and analyzer waits; and
- incrementally writes `result.json`, plus any LiteScope CSV captures.

Use `--no-analyzer` only when a capture is intentionally unnecessary. Otherwise
the matching analyzer CSV is mandatory. A strict run exits nonzero on a link
timeout, unstable link, IDENTIFY timeout, or partial IDENTIFY response.

`--lenient-exit` is a diagnostic A/B switch. It permits device ALIGN primitives,
in addition to the K28.3 primitive required by the normal handshake, to finish
the host SEND-ALIGN state. `--lenient-dwell-us` can keep transmitting ALIGN for
a measured interval before that diagnostic exit is allowed:

```sh
python3 bench/ecpix5_sata_test.py \
    --bitstream build/lambdaconcept_ecpix5/gateware/lambdaconcept_ecpix5.bit \
    --csr-csv csr.csv \
    --analyzer-csv analyzer.csv \
    --output-dir /tmp/litesata-ecp5-lenient \
    --lenient-exit \
    --lenient-dwell-us 20
```

The dwell is expressed in 90 MHz ECPIX-5 system-clock cycles in hardware and
is limited to the 16-bit counter range. Do not enable either lenient option in
a claimed production configuration.

To issue the standard two-FIS ATA software reset before IDENTIFY, add:

```sh
    --soft-reset --post-reset-delay 30
```

The hardware asserts SRST for 6 us, clears it with a second Register H2D FIS
while the device can still be busy, and completes each internal command so the
crossbar grant is released. The runner separately captures the reset traffic,
watches for the post-reset signature, and bounds reset recovery.

## Current hardware result

On 2026-07-29, the reduced ECPIX-5 build routed with timing met:

| Domain | Achieved | Required |
| --- | ---: | ---: |
| SATA RX | 163.19 MHz | 150.01 MHz |
| SATA TX | 178.99 MHz | 150.01 MHz |
| System | 111.74 MHz | 90.00 MHz |

The later BIST/analyzer image containing the configurable ALIGN dwell also met
timing: 151.65 MHz SATA RX, 162.92 MHz SATA TX, and 108.10 MHz system.

The strict SEND-ALIGN exit does not reliably reach READY with the current test
drive. With the diagnostic lenient exit, the link reaches and holds READY. An
IDENTIFY attempt then has the following repeatable link-layer behavior:

- host TX traverses `X_RDY`, `SOF`, frame copy, `EOF`, and `WTRM`;
- the device returns `R_RDY`, `R_IP`, and `R_OK`;
- the host link TX reports no error and the PHY remains READY; and
- no device `SOF`/D2H FIS follows, so IDENTIFY times out.

`R_OK` is important: the device accepted the complete link-layer frame,
including its CRC. This rules out a dead board, dead TX path, or a basic frame
framing failure for that attempt. It does not prove that the device accepted or
executed the command FIS. The open problem is now above successful link-layer
delivery, or in the exact FIS observed by the device.

The transport/link-boundary capture has since closed the exact-FIS question.
The accepted IDENTIFY Register H2D FIS is:

```text
00ec8027 a0000000 00000000 08000000 00000000
```

It is exactly five dwords with type `0x27`, C=1, command `0xec`, device
`0xa0`, control `0x08`, and all other taskfile fields zero. The device returns
`R_OK` and the host reports no TX error, but no PIO Setup, Data, Register D2H,
or initial signature FIS follows.

An ATA software-reset discriminator was also exercised. The host emitted:

```text
00000027 00000000 00000000 0c000000 00000000
00000027 00000000 00000000 08000000 00000000
```

Both C=0 control FISes were accepted with `R_OK`, and the PHY remained READY.
No post-reset signature appeared. An initial one-second recovery experiment
left the drive no longer answering IDENTIFY `X_RDY` with `R_RDY`; reloading the
FPGA, issuing a new COMRESET, and parking the line for 30 seconds did not clear
that state.

After a true drive power cycle, a strict capture measured the device's clean
Gen2 ALIGN interval from host SEND-ALIGN entry to its rate step as 54.49 us
(613 samples at 8x subsampling of the 90 MHz system clock). The actual DCU TX
input remained the correct alternating halves of `7b4a4abc/k0001` throughout,
but the device never changed from ALIGN to SYNC. A diagnostic dwell sweep then
gave:

| Lenient dwell | Hardware result |
| ---: | --- |
| 5 us | eventually held READY; 16 early ready drops; IDENTIFY timed out |
| 15 us | one zero-drop held link; later runs were less stable |
| 20 us | repeatably reached a held link; best overall operating point |
| 40 us | 35 ready drops and no two-second hold |

On a fresh FPGA load at 15 us, IDENTIFY again received
`R_RDY/R_IP/R_OK` with zero TX errors and no D2H FIS. The planned single
software-reset discriminator was then repeated at 20 us with a full 30-second
post-reset watch. Both reset FISes received `R_OK`, the PHY stayed READY, no
signature appeared, and the subsequent IDENTIFY also received `R_OK` but no
response. SRST therefore does not recover the missing device protocol-ready
state.

Readback-verified ECP5 DCU transmit-equalization experiments subsequently
tested approximately 2 dB, 4 dB, and 5 dB of both pre- and post-cursor
emphasis while retaining the stock 6.0 mA steady-state current. None caused a
strict link to observe device SYNC. A read-only BIST checker probe then sent
one-sector READ DMA EXT at LBA 0 over a held 15 us diagnostic link:

```text
00258027 e0000000 00000000 08000001 00000000
```

The frame received `R_RDY/R_IP/R_OK` and no TX error, but the device returned
neither data nor status and the checker remained busy. Thus the failure is
not specific to IDENTIFY or its PIO data phase: ATA command execution is not
active on the lenient link. No generator was enabled and this probe did not
write the disk.

A later capture against an independently known-healthy Toshiba exposed a
false strict exit. The ECP5 decoder produced four corrupted
`0x7878787c/k0001` dwords; the generic low-byte test accepted them as
K28.3-family primitives even though a complete SYNC is
`0xb5b5957c/k0001`. ECP5 now enables full-primitive qualification: the strict
exit accepts three consecutive complete, valid non-ALIGN primitives (SYNC,
X_RDY, R_RDY, and the other defined link primitives), which follows the SATA
host initialization state machine and preserves an immediate signature-FIS
offer. Decoded data interrupts and resets that sequence. The diagnostic exit
accepts only complete ALIGN after its configured dwell. Existing PHY families
retain their established four-sample low-byte behavior.

With this correction, the timing-clean image produced no false READY event in
a 35-second strict attempt. The result is less optimistic but accurate: the
healthy disk sends ALIGN during Gen2 speed negotiation and never advances to
complete SYNC, so the ATA command layer must not be attached. The corrected
image met timing at 159.54 MHz SATA RX, 177.75 MHz SATA TX, and 108.34 MHz
system. All 50 regression tests pass.

A follow-up known-healthy-disk run separated strict and diagnostic behavior.
The complete-valid-primitive strict path produced no link in 60 seconds. A
20 us diagnostic exit based on complete ALIGN eventually held READY after
multiple drops, but no startup signature arrived. The exact IDENTIFY frame was
accepted at the link layer with `R_RDY/R_IP/R_OK` and zero TX errors, yet no
PIO Setup, data, or Register D2H response followed. Thus forcing READY after
ALIGN can create a functioning link-layer exchange without completing the
device's ATA protocol initialization; it is not an acceptable production
link-up criterion.

The final three-primitive build met timing at 161.73 MHz SATA RX, 162.07 MHz
SATA TX, and 102.43 MHz system. A final 30-second strict run on that exact
artifact produced no READY event, issued no command, and parked the line.

A subsequent Xilinx/ECP5 transition audit found no controller-state
divergence: both implementations hold COMWAKE until `TXCOMFINISH`, wait for
device COMWAKE, then move toward the D10.2 phase. Runtime diagnostics tested a
1us genuine-idle interval before `TXCOMFINISH`, a genuine-electrical-idle
final COMWAKE gap, and device COMWAKE qualification after three, four, or five
gaps. Cycle-accurate capture also confirmed that the ECP5 controller requests
D10.2 11ns after detecting device COMWAKE, drops its TX-idle request by 56ns,
and presents continuous encoded D10.2 to the DCU by 100ns. Every strict A/B
still ended at status `0x6`, so a digital state-transition error at this
boundary is no longer the leading explanation.

The remaining implementation difference is electrical: Xilinx transceivers
generate true OOB internally, whereas the ECP5 path uses data-driven inner
gaps because `FFC_EI_EN` cannot engage and release within a 106.7ns COMWAKE
gap. An external OOB-envelope/serialized-eye capture or a golden SATA
receiver is now the useful next discriminator.

Do not start the write BIST until IDENTIFY succeeds and a disposable, nonzero
sector range has been selected. The generator intentionally overwrites its
target range.

## Regression tests

Run the focused ECP5 and acceptance tests with:

```sh
python3 -m pytest -q \
    test/test_ecp5_oob.py \
    test/test_command_signature.py \
    test/test_soft_reset.py \
    test/test_ecpix5_sata_test.py
```

The ECP5 tests cover OOB timing/detection, strict SEND-ALIGN behavior, sticky
ALIGN detection, fabric word alignment, production defaults/API selection, and
the bounded runner. The command regressions cover consumption of a non-error
unsolicited register D2H signature, Linux-compatible IDENTIFY taskfile
defaults, and the bounded assert/hold/deassert soft-reset sequence.

## Suggested upstream split

The experimental branch has a long measurement history. It should be
reconstructed as reviewable commits rather than submitted wholesale:

1. LiteICLink: the independent ECP5 DCU `D_SYNC_LOCAL_EN` fix
   (`0db96f7`).
2. LiteSATA PHY mechanics: ECP5 SerDes wrapper, fabric 8b/10b/word aligner,
   clocking, OOB generator/checker, and their focused simulations.
3. LiteSATA integration: Gen2-only ECP5 selection and conservative production
   defaults, without bench CSRs or experimental link-layer relaxations.
4. ECPIX-5 bench and the bounded acceptance runner.
5. The independent command-layer signature fix and regression.

Historical journals, generated CSR/analyzer maps, bitstreams, waveforms, and
one-off campaign scripts are evidence, not upstream source changes. Keep them
out of the first review series.
