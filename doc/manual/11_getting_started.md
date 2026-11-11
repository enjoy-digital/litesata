# Getting started {#getting-started}

This chapter takes you from a fresh machine to a drive that answers IDENTIFY DEVICE and passes a
write and read test. It installs the tools, checks them without hardware, builds a bench target,
then drives the link from a host computer and from the LiteX BIOS. Each step gives the commands,
what you should see, and what to check when you do not.

## Prerequisites {#gs-install}

LiteSATA is a Python package built on LiteX and needs Python 3.9 or newer. Install LiteX by
following the installation page of its wiki (`litex_setup.py`): its standard configuration
installs Migen, LiteX, LiteX-Boards, LiteScope, LiteICLink and LiteSATA itself. To work on a
clone of the repository, install it in development mode:

```sh
git clone https://github.com/enjoy-digital/litesata
cd litesata
pip3 install -e .
```

The other tools depend on what you run. The table below lists them.

| Task | Tools |
|--------------------------------|----------------------------------------------------------|
| Unit tests | Python 3, pytest |
| Standalone core (`litesata_gen`) | PyYAML; no FPGA tool |
| AMD 7-Series, UltraScale(+) bitstreams | Vivado |
| Lattice ECP5 bitstreams | Yosys, nextpnr and Trellis |
| SoC with a CPU and the BIOS | A RISC-V GCC toolchain (`litex_setup.py --gcc=riscv`) |
| Loading over JTAG | The programmer of the platform: openFPGALoader, OpenOCD or Vivado |
| Host access to registers | `litex_server`, `litex_cli`, `litex_term`, installed with LiteX |

You also need a drive whose content you can lose. The BIST and the write commands of this chapter
overwrite sectors, starting at sector 0 by default.

## Step 1: check the installation {#gs-check}

The unit tests check the link, transport and command layers, the frontends and the PHY controller
against a Python model of a drive, with no board and no FPGA tool. They are what the continuous
integration runs; [Simulation and verification](#simulation) describes them.

```sh
python3 -m pytest -v
```

A bench target elaborates without Vivado when it is run without `--build`: it writes the Verilog
of the SoC, its constraints and the register map, then stops before the vendor tool. Run it in a
directory of your own, since the target writes `csr.csv` in the current directory.

```sh
mkdir kc705 && cd kc705
python3 ../bench/kc705.py
```

**What you should see.** All tests pass. The target prints the LiteX summary of the SoC and writes
`csr.csv`, whose register names start with `sata_phy`, `sata_bist`, `sata_sector2mem` and
`sata_mem2sector`.

**If not.** An import error names the missing package: LiteX-Boards for `litex_boards`, LiteScope
for `litescope`, LiteICLink for the ECP5 PHY. Rerun the LiteX installation, or
`pip3 install -e .` in the missing repository.

## Step 2: choose the hardware {#gs-hardware}

You need a board with a transceiver, a way to bring a lane out to a SATA connector, and a drive.
[Supported FPGAs and boards](#boards) lists the boards and the adapters. This chapter uses two of
them:

- **KC705** with the AB09-FMCRAID FMC adapter, the default path of `bench/kc705.py`, at Gen3. The
  target reaches its registers through the USB-UART of the board.
- **Acorn CLE-215+** with the PCIe riser mod of [From the transceiver to the
  drive](#boards-adapters), at Gen1 by default. The target reaches its registers through JTAG.

Connect the drive with a SATA data cable, and power it from a supply of its own: the data cable
carries no power.

## Step 3: build and load a bench target {#gs-build}

The bench targets are SoCs without a CPU: the PHY, the core, a crossbar, the BIST and its
Identify, plus a bridge for the host. The KC705 target also has the two DMAs. Build and load it
from the directory of step 1:

```sh
python3 ../bench/kc705.py --build --load
```

For the Acorn, select the generation and the board variant on the command line:

```sh
python3 ../bench/acorn.py --variant cle-215+ --gen 1 --build --load
```

`--load` writes the volatile configuration memory of the FPGA through JTAG; a power cycle brings
back the bitstream of the flash. Keep the directory: the host scripts of the next steps read its
`csr.csv`.

**What you should see.** Vivado builds the bitstream, then the programmer loads it. On the KC705,
LED 0 blinks with the system clock, LEDs 1 and 2 with the SATA TX and RX clocks, and LED 3 lights
when the PHY controller has finished the OOB and ALIGN handshake. The Acorn target drives the same
four LEDs.

**If not.** A timing failure at Gen3 points to the 300 MHz transceiver clocks of the 16-bit PHY;
[Generation, width and system clock](#integ-clocks) explains the trade-off. LED 0 blinking
while LEDs 1 and 2 do not means that the transceiver has no reference clock or no PLL lock:
check the adapter and the clock source of [Bench targets](#boards-bench).

## Step 4: connect the host {#gs-server}

`litex_server` opens the bridge of the target and serves its bus to the host scripts on TCP port
1234. Start it in a terminal of its own, with the bridge of your target:

```sh
# KC705 and the other UARTBone targets: the USB-UART of the board, 115200 baud.
litex_server --uart --uart-port /dev/ttyUSBx

# ECPIX-5: UARTBone at 1 Mbaud.
litex_server --uart --uart-port /dev/ttyUSBx --uart-baudrate 1000000

# Acorn: JTAGBone, with the OpenOCD configuration of your JTAG cable.
litex_server --jtag --jtag-config openocd_xc7_ft232.cfg
```

`/dev/ttyUSBx` stands for the serial port of the board. Then check, from the build directory,
that the host reaches the right bitstream:

```sh
litex_cli --ident
```

**What you should see.** The identifier of the target, such as `LiteSATA bench on KC705`.

**If not.** No answer points to the serial port, the baud rate or the JTAG cable. A different
identifier means another bitstream is loaded: load the board again.

## Step 5: bring the link up {#gs-init}

`test_init.py` resets the PHY through its `enable` register, then polls its `status` register
until the link is ready. The PHY controller retries the OOB sequence by itself during that time:
80 polls at 0.1 s by default, `--retries` and `--interval` change them.

```sh
python3 ../bench/test_init.py
```

**What you should see.** Dots while the link trains, then a line starting with `Success`, with the
number of polls and the status register in hexadecimal. Bit 0 of the status is `ready`: the
transceiver, the PHY controller and the ALIGN exchange all succeeded.

**If not.** The script prints `Failed` and decodes three bits of the status: `tx_ready` and
`rx_ready`, the reset sequences of the transceiver, and `ctrl_ready`, the OOB and ALIGN state
machine. With the two transceiver bits at 0, check the reference clock. With them at 1 and
`ctrl_ready` at 0, the drive did not complete the OOB sequence: check the cable, the drive power
and the pair order, then read [Debugging](#debugging).

## Step 6: identify the drive {#gs-identify}

`test_bist.py -i` runs IDENTIFY DEVICE through the Identify module of the BIST, reads the 256
words of the answer from its FIFO, and decodes them. It writes nothing to the drive.

```sh
python3 ../bench/test_bist.py -i
```

**What you should see.** The serial number, the firmware revision and the model of the drive, as
the drive reports them (the model is the 40 characters of words 27 to 46); the capacity in GiB, from
words 100 to 103; three lines `SATA Gen1` to `SATA Gen3`, 1 for each generation the drive supports
(word 76); and `48 bits LBA supported: 1`, from word 83.

**If not.** Empty or garbled strings with a link that came up in step 5 point to the command or
transport layer. Run the target with its LiteScope analyzer (`--with-global-analyzer` on the
KC705, `--with-analyzer` on the other targets) and see [Debugging](#debugging).

## Step 7: run the BIST {#gs-bist}

Without `-i`, `test_bist.py` writes a pattern to the drive with the BIST generator, reads it back
with the checker, and counts the dwords that differ. It moves through the drive in transfers of
1 MiB, up to 256 MiB by default, starting at sector 0.

```sh
# 1 MiB transfers, 64 MiB in total, pseudo-random data.
python3 ../bench/test_bist.py -s 1024 -l 64 -r
```

The main options are below; `-h` lists them all.

| Option | Meaning |
|--------|---------------------------------------------------------------|
| `-s` | Transfer size in KiB, one BIST run each (default 1024) |
| `-l` | Total length in MiB (default 256) |
| `-n` | Loops per transfer, for a more precise rate on small transfers |
| `-r` | Pseudo-random data instead of a counter |
| `-c` | Continuous: run until Ctrl-C or the end of the drive |
| `-a` | Random sector addresses |
| `-d` | Write the whole length first, then read it |

**What you should see.** The IDENTIFY summary of step 6, then one line per transfer with its first
sector, the write and read rates in MiB/s, `errors:0` and `retry:0`, and the MiB done so far. The
rates come from the cycle counter of the BIST, so the speed of the host bridge does not count.

**If not.** Non-zero `errors` means that data came back different from what was written. A
`retry` means that a BIST run ended with its `aborted` register set: a command of the run came
back with `failed`, and the script ran it again. [Debugging](#debugging) covers both. A rate far
below the line rate on writes is often the drive itself, as [Drives](#boards-drives) explains.

## Step 8: use the drive from the LiteX BIOS {#gs-bios}

A LiteX-Boards target with `--with-sata` builds a SoC with a CPU and the BIOS, and adds LiteSATA
with `add_sata`: the core, Identify and the two DMAs. The BIOS then has commands to initialize the
drive and to read and write sectors. Build the KC705 target, then open the serial console:

```sh
python3 -m litex_boards.targets.xilinx_kc705 --with-sata --build --load
litex_term /dev/ttyUSBx
```

This target uses the SFP-to-SATA adapter at Gen2; [LiteX-Boards targets with
SATA](#boards-litex-boards) lists the others. At the `litex>` prompt, `sata_init` resets the PHY,
waits for the link, runs IDENTIFY DEVICE and prints the model (its 40 characters) and the
capacity of the drive.

**What you should see.** The capture below comes from the LiteX repository, with and without a
drive connected:

```console
With SATA Disk:
litex> sata_init
Initialize SATA...
Model:    KINGSTON SA400S37240G
Capacity: 240GB
Successful.

Without SATA Disk:
litex> sata_init
Initialize SATA... Failed.
```

This one was made on an Acorn Baseboard Mini at Gen2, with Ethernet and SATA in the same
bitstream:

```console
litex> sata_init
Initialize SATA...
Model:    WDC WDS120G1G0A-00SS50
Capacity: 120GB
Successful.
```

The capacity is in GB of 10^9 bytes. Then `sata_read 0` reads sector 0 and dumps its 512 bytes:
on a partitioned drive, the partition table, ending with the bytes `55 aa`.

**If not.** `Failed.` means that the 16 attempts of `sata_init`, about 0.1 s each, found no
link or no IDENTIFY answer: go back to step 5 with the bench target of the board. A missing
`sata_init` command means that the SoC has no register bank named `sata_phy`, see [BIOS
commands](#integ-bios).

Where to go next: [Integration](#integration) to add LiteSATA to your own design, [Example designs
and targets](#examples) for the options of each target, [Debugging](#debugging) when a step
fails.
