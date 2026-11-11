#
# This file is part of LiteSATA project.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause
#
# Register appendix of the manual: which designs tools/csr_doc.py elaborates and which register banks
# it documents (the engine is generic, part of the manual kit).

import copy
import importlib.util
import json
import os
import sys
import tempfile

# Configurations: script (path from the repository root), command line and YAML keys changed on a
# generator example. The UltraScale+ example of the control/DMA flavour has every register bank of
# LiteSATA: PHY, Identify, BIST (generator, checker and its own Identify), both DMAs and the IRQ
# event manager. The PHY bank is the same for every transceiver family, ECP5 included.
CONFIGS = {
    "usp_axi_lite_dma" : ("litesata/gen.py", ["examples/ultrascale_plus_axi_lite_dma.yml"], {}),
}

# Banks documented in the appendix (prefix match on the bank name), in order, with their title and
# the Python class that defines them. The longest matching prefix owns a bank.
BANKS = [
    ("sata_phy",            "PHY",            "LiteSATAPHY",           "litesata/phy/__init__.py"),
    ("sata_identify",       "Identify",       "LiteSATAIdentifyCSR",   "litesata/frontend/identify.py"),
    ("sata_bist_generator", "BIST generator", "LiteSATABISTUnitCSR",   "litesata/frontend/bist.py"),
    ("sata_bist_checker",   "BIST checker",   "LiteSATABISTUnitCSR",   "litesata/frontend/bist.py"),
    ("sata_bist_identify",  "BIST Identify",  "LiteSATAIdentifyCSR",   "litesata/frontend/bist.py"),
    ("sata_sector2mem",     "Sector2Mem DMA", "LiteSATASector2MemDMA", "litesata/frontend/dma.py"),
    ("sata_mem2sector",     "Mem2Sector DMA", "LiteSATAMem2SectorDMA", "litesata/frontend/dma.py"),
    ("sata_irq",            "Interrupts",     "the `add_sata` method of the LiteX SoC", "`soc.py` of LiteX"),
]

# Descriptions of the LiteSATA registers. The gateware declares its registers without descriptions
# (CSRStorage/CSRStatus/CSR without `description`), so they are given here and attached to the
# elaborated registers before the dump; a description declared in the gateware takes precedence.
# Keys: (group, register) or (group, register, field); a group covers the banks of _DESC_GROUPS.
DESCRIPTIONS = {
    ("sata_phy", "enable") : "PHY enable: 0 holds the transceiver initialization in reset, 1 lets it run and the OOB sequence start.",
    ("sata_phy", "status") : "PHY status.",
    ("sata_phy", "status", "ready")      : "1: transceiver and controller ready, the link carries the core traffic.",
    ("sata_phy", "status", "tx_ready")   : "1: transceiver TX initialized.",
    ("sata_phy", "status", "rx_ready")   : "1: transceiver RX initialized.",
    ("sata_phy", "status", "ctrl_ready") : "1: OOB and ALIGN handshake of the PHY controller complete.",
    ("sata_phy", "status", "link_up")    : "1: link up, debounced: falls after `link_down_timeout` (100 ms) without `ready`.",

    ("identify", "start")        : "Write 1: empty the data FIFO and send an IDENTIFY DEVICE command.",
    ("identify", "done")         : "1: idle, no command in progress.",
    ("identify", "data_width")   : "Width of the user port, in bits.",
    ("identify", "source_valid") : "1: a dword of the IDENTIFY data is in `source_data`.",
    ("identify", "source_ready") : "Write 1: pop the dword of `source_data`.",
    ("identify", "source_data")  : "Current dword of the IDENTIFY data (FIFO of 512 dwords, 128 used).",
    ("identify", "error")        : "1: the last command failed (drive error or CRC error).",

    ("bist", "start")   : "Write 1: run `loops` commands on `count` sectors from `sector`.",
    ("bist", "sector")  : "First sector (48-bit LBA).",
    ("bist", "count")   : "Number of sectors per command.",
    ("bist", "loops")   : "Number of commands of a run.",
    ("bist", "random")  : "Data pattern: 0 counter, 1 scrambler (pseudo-random).",
    ("bist", "done")    : "1: idle, run finished or never started.",
    ("bist", "aborted") : "1: the last command of the run failed (drive error or CRC error).",
    ("bist", "errors")  : "Dwords read back with a wrong value in the last command (checker only).",
    ("bist", "cycles")  : "System clock cycles of the whole run.",

    ("dma", "sector")   : "First sector (48-bit LBA).",
    ("dma", "nsectors") : "Number of sectors, one ATA command per sector; must not be 0.",
    ("dma", "base")     : "Byte address of the memory buffer.",
    ("dma", "start")    : "Write: start the transfer.",
    ("dma", "done")     : "1: idle, transfer finished or never started.",
    ("dma", "error")    : "1: a command of the transfer failed; the transfer stopped there.",
}

# Banks sharing a group of descriptions (the BIST and Identify units, the two DMAs).
_DESC_GROUPS = {
    "sata_identify"       : "identify",
    "sata_bist_identify"  : "identify",
    "sata_bist_generator" : "bist",
    "sata_bist_checker"   : "bist",
    "sata_sector2mem"     : "dma",
    "sata_mem2sector"     : "dma",
}

# Notes printed after the title of a bank family: what the register descriptions do not say.
NOTES = {
    "sata_phy": [
        "Present when the PHY is built with `with_csr=True`, the default. The bank is the same",
        "for every transceiver family, ECP5 included. The BIOS `sata_init` and",
        "`bench/test_init.py` toggle `enable` to restart the link, then poll `status` until",
        "`ready` is set ([PHY architecture and OOB controller](#phy-architecture)).",
    ],
    "sata_identify": [
        "Present with `with_identify` of `add_sata`, default on. Software writes `start`, waits",
        "for `done`, then reads the 128 dwords (256 words) of the IDENTIFY data: it reads",
        "`source_data` while `source_valid` is set and writes `source_ready` after each dword",
        "([Frontends](#frontends)).",
    ],
    "sata_bist_generator": [
        "Present with `with_bist` of `add_sata`, or with `LiteSATABIST(with_csr=True)` in the",
        "bench targets. The generator writes `count` sectors from `sector`, `loops` times. The",
        "throughput of a run is `loops*count*512/(cycles/clk_freq)` bytes per second, the",
        "formula of `bench/test_bist.py`. The `errors` register of the generator stays at 0.",
        "",
        "The generator, the checker and the Identify unit of the BIST share the `sata_bist`",
        "bank: their full names start with `sata_bist_generator_`, `sata_bist_checker_` and",
        "`sata_bist_identify_`, and their offsets are from the base of `sata_bist`.",
    ],
    "sata_bist_checker": [
        "Same layout as the generator. The checker reads the sectors back and counts the dwords",
        "that differ from the pattern the generator writes with the same `sector` and `random`.",
        "Each command of a run clears `errors` and `aborted`, so they report the last command.",
    ],
    "sata_bist_identify": [
        "The BIST has its own Identify unit, with the layout of the Identify bank, so that a",
        "bench target needs no other frontend to read the drive information.",
    ],
    "sata_sector2mem": [
        "Present when the `mode` of `add_sata` contains `read`. The DMA reads `nsectors`",
        "sectors from `sector` and writes them to memory from `base`, 512 bytes per sector,",
        "with one READ DMA EXT command per sector. The end test compares the current sector",
        "with `sector + nsectors - 1`, so a transfer with `nsectors` at 0 does not end.",
    ],
    "sata_mem2sector": [
        "Present when the `mode` of `add_sata` contains `write`. It mirrors Sector2Mem: one",
        "WRITE DMA EXT command per sector, with the same rule on `nsectors`.",
    ],
    "sata_irq": [
        "Present with `with_irq` of `add_sata`, default on: a LiteX event manager with one pulse",
        "event per DMA, raised when the DMA terminates, after its last sector or on an error.",
        "Writing 1 to a bit of `pending` clears it. The link events of the PHY (`connect_irq`,",
        "`disconnect_irq`) are not wired to it.",
    ],
}

# Opening paragraph of the appendix and the sentence that ends the access paragraph.
INTRO = ["This appendix lists the registers of LiteSATA: PHY, Identify, BIST, DMAs and interrupts.",
         "It is generated from the register definitions of the gateware, elaborated on the",
         "control/DMA example `examples/ultrascale_plus_axi_lite_dma.yml` of the generator,",
         "without running an FPGA toolchain. Offsets are in",
         "bytes from the base of the bank; the base addresses depend on the SoC and are in the",
         "`csr.csv` of a build. A LiteX SoC names the banks after the `name` given to `add_sata`,",
         "`sata` by default, as in this example."]
USAGE = ["The BIOS, `liblitesata` and the host scripts of", "`bench/` use these names through the generated `csr.h` and `csr.csv`, so they rarely need the offsets."]
MAIN_BANK = "Registers of the top-level `main` bank of the SoC."

# The BIST bank holds three units: it is split into banks named after their prefix, with their
# offsets kept from the base of the `sata_bist` bank.
SPLITS = {"sata_bist": ["generator", "checker", "identify"]}


def _split_banks(soc, csr_json):
    for bank, subs in SPLITS.items():
        region = soc.csr.regions.get(bank)
        if region is None:
            continue
        keep = list(region.obj)
        for sub in subs:
            regs = []
            for csr in [c for c in keep if c.name.startswith(sub + "_")]:
                keep.remove(csr)
                c = copy.copy(csr)
                c.name = csr.name[len(sub) + 1:]
                regs.append(c)
            if regs:
                name = f"{bank}_{sub}"
                soc.csr.regions[name] = copy.copy(region)
                soc.csr.regions[name].obj = regs
                csr_json["csr_bases"][name] = csr_json["csr_bases"][bank]
        region.obj = keep
        if not keep:
            del soc.csr.regions[bank]


def _describe(soc):
    """Attach the descriptions of DESCRIPTIONS to the registers that have none in the gateware."""
    for bank, region in soc.csr.regions.items():
        group = _DESC_GROUPS.get(bank, bank)
        for csr in region.obj:
            desc = DESCRIPTIONS.get((group, csr.name))
            if desc and not (getattr(csr, "description", None) or "").strip():
                csr.description = desc
            fields = getattr(csr, "fields", None)
            for f in (fields.fields if fields else []):
                desc = DESCRIPTIONS.get((group, csr.name, f.name))
                if desc and not (f.description or "").strip():
                    f.description = desc


def _load(repo_root, script):
    """Import a script of the repository under a unique module name."""
    name = "manual_" + script.replace("/", "_").replace(".py", "")
    spec = importlib.util.spec_from_file_location(name, os.path.join(repo_root, script))
    mod  = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def elaborate(name, repo_root):
    """Run the main() of the generator with its Builder replaced by one that elaborates without the
    toolchain; return (soc, csr_json, module, argv)."""
    import yaml
    script, argv, overrides = CONFIGS[name]
    sys.path.insert(0, repo_root)
    from litex.soc.integration.builder import Builder
    captured = {}
    old_cwd  = os.getcwd()

    with tempfile.TemporaryDirectory() as tmp:
        class CaptureBuilder(Builder):
            def __init__(self, soc, *args, **kwargs):
                kwargs.update(output_dir=tmp, csr_csv=os.path.join(tmp, "csr.csv"),
                    csr_json=os.path.join(tmp, "csr.json"), compile_software=False,
                    compile_gateware=False)
                Builder.__init__(self, soc, **kwargs)
                captured["soc"] = soc

            def build(self, *args, **kwargs):
                kwargs["run"] = False
                return Builder.build(self, *args, **kwargs)

        run_argv = list(argv)
        if overrides:
            # Variant of a generator example: the example with some keys changed, in a temporary file.
            config = yaml.load(open(os.path.join(repo_root, argv[0])).read(), Loader=yaml.Loader)
            config.update(overrides)
            path = os.path.join(tmp, os.path.basename(argv[0]))
            with open(path, "w") as f:
                yaml.dump(config, f)
            run_argv = [path] + argv[1:]

        os.chdir(repo_root)
        mod = _load(repo_root, script)
        mod.Builder = CaptureBuilder
        old_argv = sys.argv
        sys.argv = [script] + run_argv + ["--output-dir", tmp]
        try:
            mod.main()
        finally:
            sys.argv = old_argv
            os.chdir(old_cwd)
        captured["json"] = json.load(open(os.path.join(tmp, "csr.json")))

    # Memories mapped on the CSR bus have no register definitions: keep the register banks only.
    soc, csr_json = captured["soc"], captured["json"]
    for bank, region in list(soc.csr.regions.items()):
        if not isinstance(region.obj, list):
            del soc.csr.regions[bank]
    _split_banks(soc, csr_json)
    _describe(soc)
    module = "litesata_gen" if script == "litesata/gen.py" else os.path.splitext(script)[0]
    # Generator examples are shown by their file name (all in examples/), to fit the table.
    shown  = [os.path.basename(a) for a in argv] + [f"{k}={v}" for k, v in overrides.items()]
    return soc, csr_json, module, shown
