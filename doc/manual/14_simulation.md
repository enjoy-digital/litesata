# Simulation and verification {#simulation}

This chapter describes how LiteSATA is checked before a bitstream is built: the Python model of a
drive, the reference models of the CRC and the scrambler, the unit tests and the continuous
integration. It is for contributors who change the gateware and for integrators who want to
reproduce a problem without a board.

## Verification strategy {#sim-strategy}

A SATA problem on a board is slow to diagnose: the link does not come up, or a transfer fails once
in a while with a drive that behaves its own way. Each part is therefore checked in simulation at
the cheapest level that exercises it, and only the transceivers and the OOB timing are left to the
board.

| Level | What runs | Device side | Simulator |
|-----------------------|----------------------------------------|------------------------------|---------------|
| Link blocks | CRC, scrambler, CONT inserter and remover | Python reference models | Migen |
| Link | Link layer, frames both ways | Drive model, transport loopback | Migen |
| Command | Core with READ, WRITE, IDENTIFY, faults | Drive model | Migen |
| Frontends | Identify, BIST, DMAs, striping, mirroring | Drive model, one per core | Migen |
| PHY logic | Controller, link monitor, RX realignment | Python stimulus | Migen |
| Elaboration | PHY selection, generator examples, bench | None | None |
| Hardware | Bench or LiteX-Boards target | The drive | |

Every test runs with Migen's Python simulator: no Verilator, no vendor simulation model. The
transceiver primitives have no model in the project, so the PHY tests check the logic around them
and the selection of the right primitive, not the serial link.

## The drive model {#sim-hdd-model}

Most tests need a drive behind the core but not the electrical details of a link. The drive model
(`HDD`, in `test/model/hdd.py`) stands for the PHY and the drive together: it exchanges dwords with
the link layer of the core, with no OOB, no 8b/10b and no clock domain crossing.

The model is built like the core, one layer per Python class, from a PHY layer that moves one dword
per clock cycle up to a command layer that answers the ATA commands. The figure below shows it in a
unit test.

![Unit test with the drive model: the test drives the command layer of the core, whose link layer
exchanges dwords with the PHY layer of the model; the model answers the commands from its sector
memory.](sim_hdd_model.svg)

| Layer | Class | Role |
|--------------------|------------------------|----------------------------------------------------|
| PHY | `PHYLayer` | Moves one dword per cycle; marks the primitives as K characters |
| Link | `LinkLayer` | Frame handshake both ways, CRC, scrambling, CONT, HOLD |
| Transport | `TransportLayer` | Encodes and decodes the FIS; optional loopback |
| Command | `CommandLayer` | Calls the drive for each Register H2D and Data FIS |

The link layer of the model sends CONT after a repeated primitive, as a drive may. It also sends
HOLD instead of R_IP at random while it receives a frame, with a probability set by
`link_random_level` (0 to 100). A test with a non-zero level exercises the flow control of the
core.

The drive answers WRITE DMA EXT with a DMA Activate FIS, takes the Data FISes into its memory and
ends with a Register D2H FIS. It answers READ DMA EXT with Data FISes of up to 2048 dwords, then a
Register D2H FIS, and IDENTIFY DEVICE with a PIO Setup FIS and a 128-dword block of a known
pattern.

| Method | Effect |
|--------------------------------------|------------------------------------------------|
| `malloc(sector, count)` | Allocates the sectors the test uses |
| `set_reg_d2h_status` | Status of the Register D2H FIS: ERR or DF fail the command |
| `set_pio_setup_status` | Status of the PIO Setup FIS of IDENTIFY |
| `set_busy` | Aborts the commands with a Register D2H FIS, no data |
| `set_data_error_injection` | Corrupts the CRC of the read Data FISes |

With `transport_loopback`, the transport layer of the model sends every received FIS back. The link
test uses it to check frames both ways without the command layer.

## Reference models {#sim-reference}

The CRC and the scrambler of the link layer are checked against software models
(`test/model/common.py`). These models mirror the sample code of the SATA specification, kept in
`test/model/crc.c` and `test/model/scrambler.c` for reference; the C files are not compiled.

| Test | Checks |
|----------------------------|----------------------------------------------------------------|
| `test_link_crc` | `LiteSATACRC` against the model on 1024 dwords |
| `test_link_scrambler` | `Scrambler` against the model on 1024 values |
| `test_link_cont` | `LiteSATACONTInserter` then `LiteSATACONTRemover` give back the input |

The CONT inserter is tested on its own: the link layer of the core does not use it, so the host
never sends CONT ([Link layer](#link-layer)).

## Tests by area {#sim-tests}

The tests are Python `unittest` classes in `test/`, collected by pytest.

| File | What it checks |
|--------------------------|--------------------------------------------------------------------|
| `test_link.py` | Frames through `LiteSATALink` and the model in transport loopback, with random HOLD |
| `test_command.py` | WRITE and READ through the core, random HOLD and random valid/ready; ERR and DF on writes and reads; IDENTIFY, its failures and a busy drive |
| `test_identify.py` | `LiteSATAIdentify`: the 128 dwords of the model, an aborted command |
| `test_bist.py` | BIST generator then checker on one to four sectors, no error |
| `test_dma.py` | Mem2Sector beats on 32 and 64-bit buses in both endiannesses; a Mem2Sector and Sector2Mem round trip through the model, one interrupt per transfer |
| `test_striping.py` | BIST through `LiteSATAStriping` over two cores and two models |
| `test_mirroring.py` | BIST on both ports of `LiteSATAMirroring` over two cores and two models |
| `test_phy.py` | Device selection per family, the legacy aliases, the comma settings of the GTY, RX realignment after a misplaced K character |
| `test_phy_ctrl.py` | The ALIGN latched during COMWAKE by the ECP5 controller and not by the generic one, the link up and down events |
| `test_gen.py` | Transceiver selected by each generator PHY name; the five examples generate with their pads |
| `test_bench.py` | `test_init.py` resets the PHY once then polls; `acorn.py` refuses a slow system clock |

The streaming frontends (`frontend/stream.py`) have no test.

## Running the tests {#sim-run}

```sh
# The whole suite, as the continuous integration runs it
python3 -m pytest -v
# The same on all the cores of the machine (pytest-xdist)
python3 -m pytest -v -n auto
# One file, or one test
python3 -m pytest -v test/test_command.py
python3 -m pytest -v test/test_command.py -k identify
```

The suite takes several minutes even with `-n auto`; the simulations of the BIST, mirroring,
striping and DMA tests through the drive model take most of it. A failing test prints the dwords it
compared (`shift`, `length`, `errors`), and setting `link_debug`, `transport_debug` or `hdd_debug`
of the model prints every dword and FIS it sees.

## Continuous integration {#sim-ci}

Every push and pull request runs `.github/workflows/ci.yml` on Ubuntu 22.04 with Python 3.9 and
3.12. The job installs LiteX and its cores with `litex_setup.py`, installs LiteSATA, checks that
the register appendix of this manual is up to date ([Maintaining this manual](#maintenance)), and
runs `python3 -m pytest -v`. The manual has its own workflow, `.github/workflows/doc.yml`.

`test/Makefile` targets an `examples/make.py` that does not exist in the repository: it is not
used.

## Adding a test {#sim-adding}

A new test follows the pattern of the existing ones:

1. Build a `DUT` module with an `HDD` model, a `LiteSATACore` on `hdd.phy`, a crossbar and the
   frontend under test on a user port.
2. Allocate the sectors with `hdd.malloc`, then drive the frontend from a generator.
3. Run the generator together with the passive generators of the model.

```python
generators = {
    "sys" : [generator(dut),
             dut.hdd.link.generator(),
             dut.hdd.phy.rx.generator(),
             dut.hdd.phy.tx.generator()]
}
run_simulation(dut, generators, {"sys": 10})
```

Test the failure paths too: set an error status, a busy drive or a data error with the methods of
the model, and check the `failed` and `end` flags of the response. A test on two drives builds two
models with `n=0` and `n=1`, so that their debug lines can be told apart.

## Testing without OOB {#sim-no-oob}

The drive model starts after the OOB handshake: its PHY layer exchanges dwords from the first
cycle. The OOB sequence, the polarity detection and the transceiver settings are checked on a board
only, with a drive at the other end.

A plain loopback of the transceiver, TX to RX, cannot replace the drive. After the OOB bursts, the
host sends D10.2 and waits for the ALIGN of the drive; in loopback it receives its own D10.2, never
an ALIGN, and the controller starts over when its ALIGN timer expires. The datapath after the
handshake is therefore tested in simulation with the drive model, and on hardware with a drive.
Drives differ in their OOB timing and their tolerance: keep a known-good drive as a reference and
record which drive a result was obtained with ([Debugging](#debugging)).

## Where to go next {#sim-next}

[Debugging](#debugging) for the hardware side, [Link layer](#link-layer) and
[Transport and command layers](#transport-command) for what the tests check, and
[Example designs and targets](#examples) for the bench targets.
