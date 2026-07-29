#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import json
from types import SimpleNamespace

from bench import ecpix5_sata_test


class FakeRegister:
    def __init__(self, value=0, reads=None):
        self.value = value
        self.reads = list(reads or [])
        self.writes = []

    def read(self):
        if self.reads:
            self.value = self.reads.pop(0)
        return self.value

    def write(self, value):
        self.value = value
        self.writes.append(value)


class FakeRegs:
    def __init__(self):
        for name in [
            "sata_phy_enable",
            "sata_phy_status",
            "sata_phy_phy_oob_control",
            "sata_phy_phy_oob_txctl",
            "sata_phy_phy_oob_pattern",
            "sata_phy_phy_oob_gap_pattern",
            "sata_phy_phy_oob_burst_len",
            "sata_phy_phy_oob_quiet",
            "sata_phy_phy_oob_ei_shape",
            "sata_phy_phy_oob_align",
            "sata_phy_phy_oob_lenient_dwell",
            "sata_phy_phy_oob_post_idle",
            "sata_phy_phy_oob_match",
            "sata_phy_phy_oob_rec",
            "sata_bist_identify_start",
            "sata_bist_identify_done",
            "sata_bist_identify_source_valid",
            "sata_bist_identify_source_data",
            "sata_bist_identify_source_ready",
            "sata_bist_soft_reset_start",
            "sata_bist_soft_reset_done",
        ]:
            setattr(self, name, FakeRegister())


def test_canonical_configuration_and_park(monkeypatch):
    regs = FakeRegs()
    monkeypatch.setattr(ecpix5_sata_test.time, "sleep", lambda _: None)

    ecpix5_sata_test.configure_attempt(regs)
    assert regs.sata_phy_enable.writes == [0]
    assert regs.sata_phy_phy_oob_control.value == ecpix5_sata_test.OOB_CONTROL
    assert regs.sata_phy_phy_oob_txctl.value == ecpix5_sata_test.OOB_TXCTL
    assert regs.sata_phy_phy_oob_pattern.value == 0xF0F0
    assert regs.sata_phy_phy_oob_gap_pattern.value == 0
    assert regs.sata_phy_phy_oob_burst_len.value == 16
    assert regs.sata_phy_phy_oob_quiet.value == 32
    assert regs.sata_phy_phy_oob_ei_shape.value == 16 << 13
    assert regs.sata_phy_phy_oob_align.value == (4096 << 16) | 64
    assert regs.sata_phy_phy_oob_lenient_dwell.value == 0
    assert regs.sata_phy_phy_oob_post_idle.value == 0
    assert regs.sata_phy_phy_oob_match.value == 4

    ecpix5_sata_test.park(regs)
    assert regs.sata_phy_enable.writes[-2:] == [0, 1]
    assert regs.sata_phy_phy_oob_control.value == (
        ecpix5_sata_test.OOB_CONTROL | ecpix5_sata_test.CTRL_DISABLE
    )

    ecpix5_sata_test.configure_attempt(
        regs,
        lenient_exit=True,
        lenient_dwell_cycles=3600,
        post_idle_cycles=150,
        final_ei=True,
        match_gaps=5,
    )
    assert regs.sata_phy_phy_oob_txctl.value == (
        ecpix5_sata_test.OOB_TXCTL
        | ecpix5_sata_test.LENIENT_EXIT
        | ecpix5_sata_test.FINAL_EI
    )
    assert regs.sata_phy_phy_oob_lenient_dwell.value == 3600
    assert regs.sata_phy_phy_oob_post_idle.value == 150
    assert regs.sata_phy_phy_oob_match.value == 5


def test_phy_snapshot_can_reuse_sampled_status():
    regs = FakeRegs()
    regs.sata_phy_status.reads = [0xF]

    snapshot = ecpix5_sata_test.phy_snapshot(regs, status=0xA)

    assert snapshot["sata_phy_status"] == 0xA
    assert regs.sata_phy_status.reads == [0xF]


def test_decode_identify():
    words = [0] * 256

    def put_string(start, text, length):
        encoded = text.ljust(length * 2).encode("ascii")
        for index in range(length):
            words[start + index] = int.from_bytes(encoded[2 * index:2 * index + 2], "big")

    put_string(10, "SERIAL", 10)
    put_string(23, "FW1", 4)
    put_string(27, "MODEL", 19)
    sectors = 0x123456789ABC
    for index in range(4):
        words[100 + index] = (sectors >> (16 * index)) & 0xFFFF
    words[76] = (1 << 1) | (1 << 2)
    words[83] = 1 << 10

    identify = ecpix5_sata_test.decode_identify(words)
    assert identify["serial"] == "SERIAL"
    assert identify["firmware"] == "FW1"
    assert identify["model"] == "MODEL"
    assert identify["sectors"] == sectors
    assert identify["capacity_bytes"] == sectors * 512
    assert identify["capabilities"] == {
        "gen1": True,
        "gen2": True,
        "gen3": False,
        "lba48": True,
    }


def test_wait_until_is_bounded():
    ticks = iter([0.0, 0.0, 0.25, 0.5, 0.75, 1.0])
    calls = []

    value = ecpix5_sata_test.wait_until(
        lambda: calls.append(1) and False,
        timeout=0.75,
        interval=0,
        clock=lambda: next(ticks),
        sleep=lambda _: None,
    )

    assert value is None
    assert len(calls) == 3


def test_soft_reset_is_bounded_and_uses_dedicated_csr(monkeypatch):
    regs = FakeRegs()
    regs.sata_bist_soft_reset_done.value = 1
    monkeypatch.setattr(ecpix5_sata_test.time, "sleep", lambda _: None)

    state = ecpix5_sata_test.run_soft_reset(regs, timeout=0.1)

    assert state == "complete"
    assert regs.sata_bist_soft_reset_start.writes == [1]


def test_soft_reset_reports_unsupported_map():
    state = ecpix5_sata_test.run_soft_reset(SimpleNamespace(), timeout=0.1)

    assert state == "unsupported"


def test_tx_rterm_is_read_modify_write_verified(monkeypatch):
    regs = SimpleNamespace(
        sata_phy_phy_serdes_sci_reconfig_pause=FakeRegister(),
        sata_phy_phy_serdes_sci_reconfig_sel=FakeRegister(),
    )
    values = {0x11: 0x53}
    writes = []

    monkeypatch.setattr(ecpix5_sata_test.time, "sleep", lambda _: None)
    monkeypatch.setattr(
        ecpix5_sata_test,
        "sci_read",
        lambda _regs, address: values[address],
    )

    def fake_write(_regs, address, value):
        writes.append((address, value))
        values[address] = value

    monkeypatch.setattr(ecpix5_sata_test, "sci_write", fake_write)

    before, after = ecpix5_sata_test.apply_tx_rterm(regs, 60)

    assert (before, after) == (0x53, 0x4b)
    assert writes == [(0x11, 0x4b)]
    assert regs.sata_phy_phy_serdes_sci_reconfig_pause.writes == [1, 0]
    assert regs.sata_phy_phy_serdes_sci_reconfig_sel.writes == [0]


def test_result_is_incremental(tmp_path):
    result = ecpix5_sata_test.Result(tmp_path)
    result.event("stage", value=3)

    data = json.loads((tmp_path / "result.json").read_text())
    assert data["outcome"] == "running"
    assert data["events"][0]["name"] == "stage"
    assert data["events"][0]["value"] == 3


def test_git_state_archives_tracked_patch_and_untracked_hash(monkeypatch, tmp_path):
    untracked = tmp_path / "new.py"
    untracked.write_text("new source\n")
    monkeypatch.setattr(ecpix5_sata_test, "REPO", tmp_path)
    monkeypatch.setattr(ecpix5_sata_test, "git_revision", lambda: "abc123")

    def fake_run(command, **kwargs):
        if "status" in command:
            return SimpleNamespace(stdout=" M tracked.py\n?? new.py\n")
        if "diff" in command:
            return SimpleNamespace(stdout=b"tracked patch\n")
        raise AssertionError(command)

    monkeypatch.setattr(ecpix5_sata_test.subprocess, "run", fake_run)

    state = ecpix5_sata_test.capture_git_state(tmp_path / "results")

    assert state["revision"] == "abc123"
    assert state["dirty"]
    assert state["status"] == [" M tracked.py", "?? new.py"]
    assert (tmp_path / "results" / "source.patch").read_bytes() == b"tracked patch\n"
    assert state["tracked_patch_sha256"] == ecpix5_sata_test.file_sha256(
        tmp_path / "results" / "source.patch"
    )
    assert state["untracked_file_sha256"]["new.py"] == ecpix5_sata_test.file_sha256(
        untracked
    )


def test_analyzer_link_fsms_are_discovered_by_states(tmp_path):
    analyzer = tmp_path / "analyzer.csv"
    analyzer.write_text(
        "\n".join([
            "signal,2,fsm7_state,3",
            "enum,2,fsm7_state,0,IDLE",
            "enum,2,fsm7_state,1,RDY",
            "enum,2,fsm7_state,2,SOF",
            "enum,2,fsm7_state,3,COPY",
            "enum,2,fsm7_state,4,HOLDA",
            "enum,2,fsm7_state,5,EOF",
            "enum,2,fsm7_state,6,WTRM",
            "signal,2,fsm3_state,3",
            "enum,2,fsm3_state,0,IDLE",
            "enum,2,fsm3_state,1,RDY",
            "enum,2,fsm3_state,2,WAIT_FIRST",
            "enum,2,fsm3_state,3,COPY",
            "enum,2,fsm3_state,4,EOF",
            "enum,2,fsm3_state,5,WTRM",
            "enum,2,fsm3_state,6,R_OK",
            "enum,2,fsm3_state,7,R_ERR",
        ]) + "\n"
    )

    assert ecpix5_sata_test.analyzer_link_fsms(analyzer) == (
        "fsm7_state",
        "fsm3_state",
    )


def test_analyzer_ctrl_fsm_is_discovered_by_states(tmp_path):
    analyzer = tmp_path / "analyzer.csv"
    analyzer.write_text(
        "\n".join([
            "signal,0,fsm5_state,3",
            "enum,0,fsm5_state,0,RESET-ALL",
            "enum,0,fsm5_state,1,WAIT-TX-PLL-LOCK",
            "signal,0,fsm0_state,4",
            "enum,0,fsm0_state,0,SEND-ALIGN",
            "enum,0,fsm0_state,3,COMINIT",
            "enum,0,fsm0_state,7,COMWAKE",
            "enum,0,fsm0_state,10,AWAIT-ALIGN",
            "enum,0,fsm0_state,11,READY",
        ]) + "\n"
    )

    assert ecpix5_sata_test.analyzer_ctrl_fsm(analyzer) == "fsm0_state"


def test_tx_analyzer_com_fsm_state_and_signal_are_discovered(tmp_path):
    analyzer = tmp_path / "analyzer.csv"
    analyzer.write_text(
        "\n".join([
            "signal,0,fsm3_state,3",
            "enum,0,fsm3_state,0,IDLE",
            "enum,0,fsm3_state,1,PRE",
            "enum,0,fsm3_state,2,BURST",
            "enum,0,fsm3_state,3,GAP",
            "enum,0,fsm3_state,4,POST",
            "enum,0,fsm3_state,5,FINISH",
            "signal,0,sata_phy_phy_comgenerator_is_wake,1",
            "signal,0,sata_phy_phy_oob_d102_active,1",
        ]) + "\n"
    )

    assert ecpix5_sata_test.analyzer_com_fsm(analyzer) == "fsm3_state"
    assert (
        ecpix5_sata_test.analyzer_state_value(
            analyzer, "fsm3_state", "FINISH"
        )
        == "0b101"
    )
    assert (
        ecpix5_sata_test.analyzer_signal(analyzer, "is_wake")
        == "sata_phy_phy_comgenerator_is_wake"
    )
    assert (
        ecpix5_sata_test.analyzer_signal(analyzer, "oob_d102_active")
        == "sata_phy_phy_oob_d102_active"
    )


def test_link_capture_summary(tmp_path):
    capture = tmp_path / "capture.csv"
    capture.write_text(
        "\n".join([
            "tx_fsm,rx_fsm,datapath_sink_sink_payload_data,datapath_sink_sink_payload_charisk,"
            "linktx_from_rx_payload_primitive,linktx_from_rx_payload_primitive_valid,"
            "litesatalinktx_error,link_tx_payload_valid,link_tx_payload_ready,"
            "link_tx_payload_last,link_tx_payload_data",
            "3,3,32,4,32,1,1,1,1,1,32",
            "001,000,01010111010101111011010101111100,0001,"
            "01001010010010101001010101111100,1,0,1,1,0,"
            "00000000111011001000000000100111",
            "001,000,01010111010101111011010101111100,0001,"
            "00000000000000000000000000000000,0,0,1,1,0,"
            "11100000000000000000000000000000",
            "010,000,00110111001101111011010101111100,0001,"
            "00000000000000000000000000000000,0,0,1,1,0,"
            "00000000000000000000000000000000",
            "011,000,00000000000000000000000000000000,0000,"
            "00000000000000000000000000000000,0,0,1,1,0,"
            "00000000000000000000000000000000",
            "110,000,01011000010110001011010101111100,0001,"
            "00110101001101011011010101111100,1,0,1,1,1,"
            "00000000000000000000000000000000",
        ]) + "\n"
    )

    assert ecpix5_sata_test.summarize_link_capture(
        capture, "tx_fsm", "rx_fsm"
    ) == {
        "tx_states": {"RDY": 2, "SOF": 1, "COPY": 1, "WTRM": 1},
        "rx_states": {"IDLE": 5},
        "tx_wire_primitives": {"X_RDY": 2, "SOF": 1, "WTRM": 1},
        "rx_wire_primitives": {"R_RDY": 1, "R_OK": 1},
        "tx_error_samples": 0,
        "tx_link_packets": [[
            "0x00ec8027",
            "0xe0000000",
            "0x00000000",
            "0x00000000",
            "0x00000000",
        ]],
        "tx_h2d_register_fis": [{
            "valid_length": True,
            "type": 0x27,
            "pm_port": 0,
            "command_control": 1,
            "command": 0xEC,
            "features": 0,
            "lba": 0,
            "device": 0xE0,
            "count": 0,
            "icc": 0,
            "control": 0,
            "reserved": 0,
            "dwords": [
                "0x00ec8027",
                "0xe0000000",
                "0x00000000",
                "0x00000000",
                "0x00000000",
            ],
        }],
    }


def test_link_capture_qualifies_duplicated_scope_clock_rows(tmp_path):
    capture = tmp_path / "capture.csv"
    header = [
        "tx_fsm",
        "rx_fsm",
        "link_tx_payload_valid",
        "link_tx_payload_ready",
        "link_tx_payload_last",
        "link_tx_payload_data",
        "scope_clk",
    ]
    words = [0x00EC8027, 0xE0000000, 0, 0, 0]
    rows = [",".join(header), "3,3,1,1,1,32,1"]
    for index, word in enumerate(words):
        last = int(index == len(words) - 1)
        for scope_clk in [1, 0]:
            rows.append(
                f"011,000,1,1,{last},{word:032b},{scope_clk}"
            )
    capture.write_text("\n".join(rows) + "\n")

    summary = ecpix5_sata_test.summarize_link_capture(
        capture, "tx_fsm", "rx_fsm"
    )

    assert summary["tx_states"] == {"COPY": 5}
    assert summary["tx_link_packets"] == [[
        "0x00ec8027",
        "0xe0000000",
        "0x00000000",
        "0x00000000",
        "0x00000000",
    ]]
    assert summary["tx_h2d_register_fis"][0]["valid_length"]
    assert summary["tx_h2d_register_fis"][0]["command"] == 0xEC


def test_cli_requires_explicit_analyzer_map():
    try:
        ecpix5_sata_test.parse_args([
            "--reuse-bitstream",
            "--csr-csv",
            "csr.csv",
        ])
    except SystemExit as error:
        assert error.code == 2
    else:
        raise AssertionError("missing analyzer map was accepted")

    args = ecpix5_sata_test.parse_args([
        "--reuse-bitstream",
        "--csr-csv",
        "csr.csv",
        "--no-analyzer",
    ])
    assert args.reuse_bitstream
    assert args.no_analyzer
    assert args.poll_interval == 0.05
    assert args.oob_subsampler == 32
    assert args.post_oob_idle_us == 0
    assert not args.final_oob_ei
    assert args.oob_match_gaps == 4
    assert args.tx_rterm_ohms is None

    args = ecpix5_sata_test.parse_args([
        "--reuse-bitstream",
        "--csr-csv",
        "csr.csv",
        "--no-analyzer",
        "--tx-rterm-ohms",
        "60",
    ])
    assert args.tx_rterm_ohms == 60

    try:
        ecpix5_sata_test.parse_args([
            "--reuse-bitstream",
            "--csr-csv",
            "csr.csv",
            "--no-analyzer",
            "--oob-capture",
        ])
    except SystemExit as error:
        assert error.code == 2
    else:
        raise AssertionError("--oob-capture was accepted without an analyzer")
