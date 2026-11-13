#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os
import re
import sys
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml

from migen import Instance, Signal

from litesata.gen import PHYS
from litesata.phy import LiteSATAPHY
from litesata.phy.a7sataphy import A7LiteSATAPHY
from litesata.phy.gth3sataphy import GTH3LiteSATAPHY
from litesata.phy.gth4sataphy import GTH4LiteSATAPHY
from litesata.phy.gty4sataphy import GTY4LiteSATAPHY
from litesata.phy.k7sataphy import K7LiteSATAPHY


class SATAPads:
    def __init__(self):
        self.rx_p = Signal()
        self.rx_n = Signal()
        self.tx_p = Signal()
        self.tx_n = Signal()


class TestGen(unittest.TestCase):
    def test_phys_select_expected_transceivers(self):
        expected = {
            "A7SATAPHY"   : (A7LiteSATAPHY,   "GTPE2_CHANNEL"),
            "K7SATAPHY"   : (K7LiteSATAPHY,   "GTXE2_CHANNEL"),
            "GTH3SATAPHY" : (GTH3LiteSATAPHY, "GTHE3_CHANNEL"),
            "GTH4SATAPHY" : (GTH4LiteSATAPHY, "GTHE4_CHANNEL"),
            "GTY4SATAPHY" : (GTY4LiteSATAPHY, "GTYE4_CHANNEL"),
        }
        self.assertEqual(set(PHYS.keys()), set(expected.keys()))
        for name, (device, gt_type, toolchain) in PHYS.items():
            phy_cls, primitive = expected[name]
            for data_width in [16, 32]:
                with self.subTest(phy=name, data_width=data_width):
                    self.assertEqual(toolchain, "vivado")
                    dut = LiteSATAPHY(
                        device     = device,
                        pads       = SATAPads(),
                        gen        = "gen3",
                        clk_freq   = 100e6,
                        refclk     = Signal(),
                        data_width = data_width,
                        gt_type    = gt_type if gt_type is not None else "GTY",
                        with_csr   = False,
                    )
                    instances = [special.of for special in dut.phy.get_fragment().specials
                        if isinstance(special, Instance)]
                    self.assertIsInstance(dut.phy, phy_cls)
                    self.assertIn(primitive, instances)


# Examples -----------------------------------------------------------------------------------------

_repo_root    = Path(__file__).resolve().parents[1]
_examples_dir = _repo_root / "examples"

def generate_example(name):
    """Run the generator on examples/<name>.yml in a temporary directory, return the Verilog and
    the list of generated build files."""
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join([str(_repo_root)] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
    with tempfile.TemporaryDirectory() as tmp:
        build_dir = Path(tmp) / "build"
        subprocess.run(
            [sys.executable, "-m", "litesata.gen", str(_examples_dir / f"{name}.yml"), "--output-dir", str(build_dir)],
            cwd    = tmp,
            env    = env,
            check  = True,
            stdout = subprocess.DEVNULL,
            stderr = subprocess.DEVNULL,
        )
        verilog = (build_dir / "gateware" / "litesata_core.v").read_text()
        files   = sorted(p.name for p in build_dir.rglob("*") if p.is_file())
        return verilog, files


class TestExamples(unittest.TestCase):
    # name -> (transceiver primitive, native port, control bus pad, dma pad, irq)
    examples = {
        "artix7"                       : ("GTPE2_CHANNEL", True,  None,              None,         False),
        "kintex7"                      : ("GTXE2_CHANNEL", True,  None,              None,         False),
        "ultrascale_plus"              : ("GTHE4_CHANNEL", True,  None,              None,         False),
        "artix7_wishbone_dma"          : ("GTPE2_CHANNEL", True,  "wb_ctrl_adr",     "wb_dma_adr", True),
        "ultrascale_plus_axi_lite_dma" : ("GTHE4_CHANNEL", False, "axil_ctrl_awaddr", "wb_dma_adr", True),
    }

    def test_all_examples_are_covered(self):
        names = sorted(p.stem for p in _examples_dir.glob("*.yml"))
        self.assertEqual(names, sorted(self.examples.keys()))

    def test_examples_clk_freq(self):
        # The core datapath is 32-bit in sys: sys_clk >= linerate/40 (37.5/75/150MHz for Gen1/2/3).
        min_clk_freq = {"gen1": 37.5e6, "gen2": 75e6, "gen3": 150e6}
        for name in self.examples.keys():
            with self.subTest(example=name):
                with open(_examples_dir / f"{name}.yml", "r", encoding="utf-8") as f:
                    core_config = yaml.load(f, Loader=yaml.SafeLoader)
                self.assertGreaterEqual(float(core_config["clk_freq"]), min_clk_freq[core_config["gen"]])

    def test_examples(self):
        for name, (primitive, native, ctrl, dma, irq) in self.examples.items():
            with self.subTest(example=name):
                verilog, files = generate_example(name)
                ports = re.findall(r"^\s*(?:input|output)\s+wire\s*(?:\[[^\]]*\])?\s*(\w+),?$", verilog, re.M)
                self.assertIn(primitive, verilog)
                for pad in ["enable", "ready", "link_up", "connect", "disconnect"]:
                    self.assertIn(pad, ports)
                # PHY enable: from the pad in the native flavour, from the register (only) with a
                # control bus.
                self.assertEqual(re.search(r"=\s*enable;", verilog) is not None, ctrl is None)
                self.assertEqual(re.search(r"=\s*\w*phy_enable_storage;", verilog) is not None, ctrl is not None)
                self.assertIn("litesata_core.v", files)
                self.assertIn("csr.csv", files)
                # Native port: the sink data pad must drive the crossbar (it was left unconnected).
                if native:
                    self.assertIn("user_port_native_0_sink_data", ports)
                    self.assertRegex(verilog, r"=\s*user_port_native_0_sink_data;")
                else:
                    self.assertNotIn("user_port_native_0_sink_data", ports)
                # Control/DMA flavour pads.
                for pad in (ctrl, dma):
                    if pad is not None:
                        self.assertIn(pad, ports)
                self.assertEqual("irq" in ports, irq)
                # UltraScale+: the dedicated-pin refclk (IBUFDS_GTE4) must drive GTREFCLK0, not GTGREFCLK.
                if primitive in ["GTHE4_CHANNEL", "GTYE4_CHANNEL"]:
                    cpllrefclksel = re.search(r"\.CPLLREFCLKSEL\s*\(([^)]*)\)", verilog).group(1)
                    gtgrefclk     = re.search(r"\.GTGREFCLK\s*\(([^)]*)\)",     verilog).group(1)
                    self.assertEqual(cpllrefclksel, "1'd1")
                    self.assertEqual(gtgrefclk,     "1'd0")

    def test_gt_type_mismatch_is_rejected(self):
        # gt_type is fixed by the PHY name: a contradicting value was silently overwritten.
        env = os.environ.copy()
        env["PYTHONPATH"] = os.pathsep.join([str(_repo_root)] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else []))
        with tempfile.TemporaryDirectory() as tmp:
            config = Path(tmp) / "core.yml"
            config.write_text((_examples_dir / "ultrascale_plus.yml").read_text().replace(
                '"gen": "gen3",', '"gen": "gen3", "gt_type": "GTY",'))
            result = subprocess.run(
                [sys.executable, "-m", "litesata.gen", str(config), "--output-dir", str(Path(tmp) / "build")],
                cwd    = tmp,
                env    = env,
                stdout = subprocess.DEVNULL,
                stderr = subprocess.PIPE,
                text   = True,
            )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("gt_type GTY does not match GTH4SATAPHY", result.stderr)


if __name__ == "__main__":
    unittest.main()
