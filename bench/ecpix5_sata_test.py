#!/usr/bin/env python3

#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""Bounded ECPIX-5 SATA Gen2 link/IDENTIFY acceptance runner.

This utility deliberately performs one spec-negotiated attempt.  It records the
exact bitstream/configuration used, arms the link analyzer before PHY enable,
bounds every wait, writes a machine-readable result, and parks the SATA line in
a ``finally`` block.

Examples
--------

Load an archived bitstream and run the complete test::

    python3 bench/ecpix5_sata_test.py \
        --bitstream bench/captures/bitstreams/P-gen2-linkup-drive/lambdaconcept_ecpix5.bit \
        --csr-csv bench/captures/bitstreams/P-gen2-linkup-drive/csr.csv \
        --analyzer-csv bench/captures/bitstreams/P-gen2-linkup-drive/analyzer.csv

Reuse a bitstream that is already loaded::

    python3 bench/ecpix5_sata_test.py --reuse-bitstream \
        --csr-csv csr.csv --analyzer-csv analyzer.csv
"""

import argparse
import csv
import datetime
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import time

from collections import Counter


REPO = pathlib.Path(__file__).resolve().parents[1]

# Canonical Gen2 configuration proven on ECPIX-5 (BRINGUP campaigns 40+).
EI_MODE     = 1 << 1
LDR_TIMEOUT = 4 << 8
BURST_MODE  = 1 << 18
ZERO_BUS    = 1 << 19
CTRL_DISABLE = 1 << 26

PAT_ALT       = 1 << 5
DEEMPH_GAP    = 1 << 6
EARLY_D102    = 1 << 9
LENIENT_EXIT  = 1 << 13

OOB_CONTROL = EI_MODE | LDR_TIMEOUT | BURST_MODE | ZERO_BUS
OOB_TXCTL   = PAT_ALT | DEEMPH_GAP | EARLY_D102

LINK_TX_STATES = ["IDLE", "RDY", "SOF", "COPY", "HOLDA", "EOF", "WTRM"]
LINK_RX_STATES = ["IDLE", "RDY", "WAIT_FIRST", "COPY", "EOF", "WTRM", "R_OK", "R_ERR"]
PRIMITIVE_NAMES = {
    0x7B4A4ABC: "ALIGN",
    0xB5B5957C: "SYNC",
    0x4A4A957C: "R_RDY",
    0x3535B57C: "R_OK",
    0x5656B57C: "R_ERR",
    0x5555B57C: "R_IP",
    0x5757B57C: "X_RDY",
    0x5858B57C: "WTRM",
    0x3737B57C: "SOF",
    0xD5D5B57C: "EOF",
}
MAX_READY_DROP_EVENTS = 10


class Result:
    """Incrementally written acceptance result.

    Writing after each major stage preserves the diagnosis if the process is
    interrupted or hardware access disappears.
    """

    def __init__(self, output_dir):
        self.output_dir = pathlib.Path(output_dir).resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.path = self.output_dir / "result.json"
        self.started = time.monotonic()
        self.data = {
            "schema": 1,
            "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "outcome": "running",
            "events": [],
        }
        self.flush()

    def event(self, name, **fields):
        event = {"time_s": round(time.monotonic() - self.started, 6), "name": name}
        event.update(fields)
        self.data["events"].append(event)
        self.flush()

    def flush(self):
        temporary = self.path.with_suffix(".json.tmp")
        with temporary.open("w", encoding="utf-8") as output:
            json.dump(self.data, output, indent=2, sort_keys=True)
            output.write("\n")
        os.replace(temporary, self.path)


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_revision():
    completed = subprocess.run(
        ["git", "-C", str(REPO), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.strip()


def capture_git_state(output_dir):
    """Record enough working-tree state to qualify a locally built image."""
    status = subprocess.run(
        ["git", "-C", str(REPO), "status", "--porcelain=v1"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    diff = subprocess.run(
        ["git", "-C", str(REPO), "diff", "--binary", "HEAD", "--"],
        check=True,
        capture_output=True,
    ).stdout

    patch = None
    if diff:
        patch = pathlib.Path(output_dir) / "source.patch"
        patch.parent.mkdir(parents=True, exist_ok=True)
        patch.write_bytes(diff)

    untracked = {}
    for entry in status:
        if not entry.startswith("?? "):
            continue
        relative = entry[3:]
        path = REPO / relative
        if path.is_file():
            untracked[relative] = file_sha256(path)

    return {
        "revision": git_revision(),
        "dirty": bool(status),
        "status": status,
        "tracked_patch": None if patch is None else str(patch.resolve()),
        "tracked_patch_sha256": None if patch is None else file_sha256(patch),
        "untracked_file_sha256": untracked,
    }


def wait_until(predicate, timeout, interval=0.01, clock=time.monotonic, sleep=time.sleep):
    deadline = clock() + timeout
    while clock() < deadline:
        value = predicate()
        if value:
            return value
        sleep(interval)
    return None


def _read_optional(regs, name):
    register = getattr(regs, name, None)
    return None if register is None else register.read()


def phy_snapshot(regs, status=None):
    names = [
        "sata_phy_phy_oob_rx_gap",
        "sata_phy_phy_oob_rx_burst",
        "sata_phy_phy_oob_rx_count",
        "sata_phy_phy_oob_beacon",
        "sata_phy_phy_oob_bp",
        "sata_phy_phy_oob_rxdet_status",
    ]
    if status is None:
        status = _read_optional(regs, "sata_phy_status")
    snapshot = {} if status is None else {"sata_phy_status": status}
    snapshot.update(
        {name: value for name in names if (value := _read_optional(regs, name)) is not None}
    )
    return snapshot


def park(regs):
    regs.sata_phy_enable.write(0)
    time.sleep(1e-3)
    regs.sata_phy_phy_oob_control.write(OOB_CONTROL | CTRL_DISABLE)
    regs.sata_phy_enable.write(1)


def configure_attempt(regs, lenient_exit=False):
    regs.sata_phy_enable.write(0)
    time.sleep(1e-3)
    regs.sata_phy_phy_oob_txctl.write(
        OOB_TXCTL | (LENIENT_EXIT if lenient_exit else 0)
    )
    regs.sata_phy_phy_oob_pattern.write(0xF0F0)
    regs.sata_phy_phy_oob_gap_pattern.write(0x0000)
    regs.sata_phy_phy_oob_burst_len.write(16)
    regs.sata_phy_phy_oob_quiet.write(32)
    regs.sata_phy_phy_oob_ei_shape.write(16 << 13)
    regs.sata_phy_phy_oob_align.write((4096 << 16) | 64)
    regs.sata_phy_phy_oob_control.write(OOB_CONTROL)


def reset_oob_recorders(regs):
    recorder = getattr(regs, "sata_phy_phy_oob_rec", None)
    if recorder is None:
        return
    recorder.write(0)
    time.sleep(1e-3)
    recorder.write(1)


def drain_identify(regs, limit=4096):
    words = []
    transactions = 0
    while regs.sata_bist_identify_source_valid.read() and transactions < limit:
        dword = regs.sata_bist_identify_source_data.read()
        words.extend([dword & 0xFFFF, (dword >> 16) & 0xFFFF])
        regs.sata_bist_identify_source_ready.write(1)
        transactions += 1
    return words


def decode_identify(words):
    if len(words) != 256:
        raise ValueError(f"IDENTIFY returned {len(words)} words, expected 256")

    def ata_string(start, stop):
        return "".join(
            word.to_bytes(2, byteorder="big").decode("ascii", errors="replace")
            for word in words[start:stop]
        ).strip()

    sectors = (
        words[100]
        | (words[101] << 16)
        | (words[102] << 32)
        | (words[103] << 48)
    )
    return {
        "serial": ata_string(10, 20),
        "firmware": ata_string(23, 27),
        "model": ata_string(27, 46),
        "sectors": sectors,
        "capacity_bytes": sectors * 512,
        "capabilities": {
            "gen1": bool((words[76] >> 1) & 1),
            "gen2": bool((words[76] >> 2) & 1),
            "gen3": bool((words[76] >> 3) & 1),
            "lba48": bool((words[83] >> 10) & 1),
        },
    }


def run_identify(regs, timeout):
    drain_identify(regs)
    regs.sata_bist_identify_start.write(1)
    done = wait_until(
        lambda: regs.sata_bist_identify_done.read(),
        timeout=timeout,
        interval=1e-3,
    )
    if not done:
        return None, "timeout"
    words = drain_identify(regs)
    if len(words) != 256:
        return words, "partial"
    return words, "complete"


def make_analyzer(regs, analyzer_csv):
    from litescope import LiteScopeAnalyzerDriver

    return LiteScopeAnalyzerDriver(
        regs,
        "analyzer",
        config_csv=str(analyzer_csv),
        debug=False,
    )


def arm_analyzer(analyzer, group, condition):
    analyzer.configure_group(group)
    analyzer.configure_subsampler(1)
    analyzer.add_trigger(cond=condition)
    analyzer.run(offset=128, length=1024)


def save_analyzer(analyzer, path):
    analyzer.upload()
    analyzer.save(str(path))


def analyzer_link_fsms(path, group=2):
    enums = {}
    with pathlib.Path(path).open(newline="", encoding="utf-8") as source:
        for row in csv.reader(source):
            if len(row) < 5 or row[0] != "enum" or int(row[1]) != group:
                continue
            enums.setdefault(row[2], set()).add(row[4])
    tx = [
        name for name, states in enums.items()
        if {"IDLE", "RDY", "SOF", "HOLDA", "EOF", "WTRM"} <= states
    ]
    rx = [
        name for name, states in enums.items()
        if {"IDLE", "RDY", "WAIT_FIRST", "R_OK", "R_ERR"} <= states
    ]
    if len(tx) != 1 or len(rx) != 1:
        raise ValueError(
            f"could not uniquely locate link TX/RX FSMs in analyzer group {group}: "
            f"tx={tx}, rx={rx}"
        )
    return tx[0], rx[0]


def summarize_link_capture(path, tx_fsm, rx_fsm):
    with pathlib.Path(path).open(newline="", encoding="utf-8") as source:
        rows = [row for row in csv.reader(source) if row]
    if len(rows) < 3:
        return {}
    header = [field.strip() for field in rows[0]]

    def state_occupancy(signal, names):
        if signal not in header:
            return {}
        index = header.index(signal)
        occupancy = Counter()
        for fields in rows[2:]:
            value = fields[index].strip()
            if not value:
                continue
            state = int(value, 2)
            occupancy[names[state] if state < len(names) else str(state)] += 1
        return dict(occupancy)

    summary = {
        "tx_states": state_occupancy(tx_fsm, LINK_TX_STATES),
        "rx_states": state_occupancy(rx_fsm, LINK_RX_STATES),
    }

    def unique_suffix(suffix):
        matches = [name for name in header if name.endswith(suffix)]
        return matches[0] if len(matches) == 1 else None

    def primitive_occupancy(data_name, valid_name):
        if data_name is None or valid_name is None:
            return {}
        data_index = header.index(data_name)
        valid_index = header.index(valid_name)
        occupancy = Counter()
        for fields in rows[2:]:
            data = fields[data_index].strip()
            valid = fields[valid_index].strip()
            if not data or not valid or int(valid, 2) != 1:
                continue
            value = int(data, 2)
            occupancy[PRIMITIVE_NAMES.get(value, f"0x{value:08x}")] += 1
        return dict(occupancy)

    data_name = unique_suffix("datapath_sink_sink_payload_data")
    charisk_name = unique_suffix("datapath_sink_sink_payload_charisk")
    if data_name is not None and charisk_name is not None:
        summary["tx_wire_primitives"] = primitive_occupancy(data_name, charisk_name)
    rx_primitive = unique_suffix("from_rx_payload_primitive")
    rx_primitive_valid = unique_suffix("from_rx_payload_primitive_valid")
    if rx_primitive is not None and rx_primitive_valid is not None:
        summary["rx_wire_primitives"] = primitive_occupancy(
            rx_primitive, rx_primitive_valid
        )
    tx_error = unique_suffix("litesatalinktx_error")
    if tx_error is not None:
        error_index = header.index(tx_error)
        summary["tx_error_samples"] = sum(
            int(fields[error_index].strip(), 2)
            for fields in rows[2:]
            if fields[error_index].strip()
        )
    return summary


def program_bitstream(path, timeout=60):
    completed = subprocess.run(
        ["openFPGALoader", "-c", "ft2232", str(path)],
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    if completed.returncode:
        raise RuntimeError(completed.stderr.strip() or completed.stdout.strip())


def run(args):
    output_dir = args.output_dir
    if output_dir is None:
        stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        output_dir = pathlib.Path("/tmp") / f"litesata-ecp5-{stamp}"
    result = Result(output_dir)

    csr_csv = pathlib.Path(args.csr_csv).resolve()
    analyzer_csv = None if args.no_analyzer else pathlib.Path(args.analyzer_csv).resolve()
    bitstream = None if args.reuse_bitstream else pathlib.Path(args.bitstream).resolve()
    link_tx_fsm = link_rx_fsm = None
    if analyzer_csv is not None:
        link_tx_fsm, link_rx_fsm = analyzer_link_fsms(analyzer_csv)

    result.data["metadata"] = {
        "source": capture_git_state(result.output_dir),
        "bitstream": None if bitstream is None else str(bitstream),
        "bitstream_sha256": None if bitstream is None else file_sha256(bitstream),
        "csr_csv": str(csr_csv),
        "csr_csv_sha256": file_sha256(csr_csv),
        "analyzer_csv": None if analyzer_csv is None else str(analyzer_csv),
        "analyzer_csv_sha256": None if analyzer_csv is None else file_sha256(analyzer_csv),
        "analyzer_link_tx_fsm": link_tx_fsm,
        "analyzer_link_rx_fsm": link_rx_fsm,
    }
    result.data["configuration"] = {
        "generation": "gen2",
        # This is the requested runtime policy.  The bitstream must have been
        # built from a revision that implements the corresponding controls;
        # an archived image cannot be inferred to do so from CSR writes alone.
        "lenient_send_align_exit_requested": args.lenient_exit,
        "bitstream_policy_verified_by_runner": False,
        "oob_control": OOB_CONTROL,
        "oob_txctl": OOB_TXCTL | (LENIENT_EXIT if args.lenient_exit else 0),
        "pattern": 0xF0F0,
        "gap_pattern": 0,
        "burst_cycles": 16,
        "quiet_cycles": 32,
        "wake_gap_cycles": 16,
        "align_holdoff": 64,
        "align_nocomma": 4096,
    }
    result.flush()

    if bitstream is not None:
        result.event("program_start")
        program_bitstream(bitstream, timeout=args.program_timeout)
        result.event("program_complete")
        time.sleep(args.program_settle)

    from litex import RemoteClient

    bus = RemoteClient(port=args.port, csr_csv=str(csr_csv))
    bus.open()
    regs = bus.regs
    try:
        park(regs)
        reset_oob_recorders(regs)
        result.event("parked", snapshot=phy_snapshot(regs))
        time.sleep(args.park_seconds)
        result.data["parked_baseline"] = phy_snapshot(regs)
        result.flush()

        signature_analyzer = None
        if analyzer_csv is not None:
            signature_analyzer = make_analyzer(regs, analyzer_csv)
            arm_analyzer(signature_analyzer, 2, {link_rx_fsm: "0b001"})
            result.event("signature_watch_armed")

        configure_attempt(regs, lenient_exit=args.lenient_exit)
        reset_oob_recorders(regs)
        regs.sata_phy_enable.write(1)
        result.event("phy_enabled")

        link_first_s = None
        stable_since = None
        ready_drops = 0
        recorded_ready_drops = 0
        link_deadline = time.monotonic() + args.link_timeout
        while time.monotonic() < link_deadline:
            now = time.monotonic()
            status = regs.sata_phy_status.read()
            if status & 1:
                if link_first_s is None:
                    link_first_s = round(now - result.started, 6)
                    result.event("link_first_ready", absolute_time_s=link_first_s)
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= args.link_hold:
                    break
            elif stable_since is not None:
                ready_drops += 1
                if recorded_ready_drops < MAX_READY_DROP_EVENTS:
                    result.event(
                        "link_ready_drop",
                        ready_duration_s=round(now - stable_since, 6),
                        snapshot=phy_snapshot(regs, status=status),
                    )
                    recorded_ready_drops += 1
                stable_since = None
            time.sleep(args.poll_interval)
        if stable_since is None or time.monotonic() - stable_since < args.link_hold:
            result.data["outcome"] = (
                "link_timeout" if link_first_s is None else "link_unstable"
            )
            result.data["link_ready_drops"] = ready_drops
            result.data["link_ready_drop_events_recorded"] = recorded_ready_drops
            result.data["final_snapshot"] = phy_snapshot(regs)
            result.event(result.data["outcome"])
            return 2

        result.data["link_ready_drops"] = ready_drops
        result.data["link_ready_drop_events_recorded"] = recorded_ready_drops
        result.event(
            "link_held",
            continuous_ready_s=round(time.monotonic() - stable_since, 6),
            snapshot=phy_snapshot(regs),
        )

        signature_seen = False
        if signature_analyzer is not None:
            signature_seen = bool(wait_until(signature_analyzer.done, args.signature_timeout, 0.01))
            if signature_seen:
                signature_path = result.output_dir / "signature.csv"
                save_analyzer(signature_analyzer, signature_path)
                result.data["signature_capture"] = {
                    "path": str(signature_path),
                    **summarize_link_capture(signature_path, link_tx_fsm, link_rx_fsm),
                }
            result.data["signature_seen"] = signature_seen
            result.event("signature_watch_complete", seen=signature_seen)

        identify_analyzer = None
        if analyzer_csv is not None:
            identify_analyzer = make_analyzer(regs, analyzer_csv)
            arm_analyzer(identify_analyzer, 2, {link_tx_fsm: "0b001"})
            result.event("identify_watch_armed")

        result.event("identify_start")
        words, identify_state = run_identify(regs, args.identify_timeout)
        result.data["identify_state"] = identify_state
        result.data["identify_words"] = 0 if words is None else len(words)

        if identify_analyzer is not None:
            capture_seen = bool(wait_until(identify_analyzer.done, args.analyzer_timeout, 0.01))
            result.data["identify_capture_seen"] = capture_seen
            if capture_seen:
                identify_path = result.output_dir / "identify-link.csv"
                save_analyzer(identify_analyzer, identify_path)
                result.data["identify_capture"] = {
                    "path": str(identify_path),
                    **summarize_link_capture(identify_path, link_tx_fsm, link_rx_fsm),
                }

        result.data["final_snapshot"] = phy_snapshot(regs)
        if identify_state == "timeout":
            result.data["outcome"] = "identify_timeout"
            result.event("identify_timeout")
            return 3
        if identify_state == "partial":
            result.data["outcome"] = "identify_partial"
            result.event("identify_partial", words=len(words))
            return 4

        result.data["identify"] = decode_identify(words)
        result.data["outcome"] = "success"
        result.event("identify_complete", words=len(words))
        return 0
    except Exception as error:
        result.data["outcome"] = "error"
        result.data["error"] = f"{type(error).__name__}: {error}"
        result.event("error")
        raise
    finally:
        try:
            park(regs)
            result.event("line_parked_finally")
        finally:
            bus.close()
            result.data["finished_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
            result.flush()
            print(f"Result: {result.path}")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Bounded ECPIX-5 Gen2 SATA link and IDENTIFY acceptance test."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--bitstream", help="Bitstream to load before testing.")
    source.add_argument(
        "--reuse-bitstream",
        action="store_true",
        help="Explicitly reuse the bitstream already loaded on the FPGA.",
    )
    parser.add_argument("--csr-csv", required=True, help="CSR map matching the bitstream.")
    parser.add_argument("--analyzer-csv", help="Analyzer map matching the bitstream.")
    parser.add_argument("--no-analyzer", action="store_true", help="Run without LiteScope captures.")
    parser.add_argument("--output-dir", help="Artifact directory (default: timestamped directory under /tmp).")
    parser.add_argument("--port", default=1234, type=int)
    parser.add_argument("--program-timeout", default=60.0, type=float)
    parser.add_argument("--program-settle", default=1.5, type=float)
    parser.add_argument("--park-seconds", default=20.0, type=float)
    parser.add_argument("--link-timeout", default=60.0, type=float)
    parser.add_argument("--link-hold", default=10.0, type=float)
    parser.add_argument("--poll-interval", default=0.05, type=float)
    parser.add_argument(
        "--lenient-exit",
        action="store_true",
        help="Diagnostic only: permit ALIGN as well as SYNC to exit SEND-ALIGN.",
    )
    parser.add_argument("--signature-timeout", default=1.0, type=float)
    parser.add_argument("--identify-timeout", default=5.0, type=float)
    parser.add_argument("--analyzer-timeout", default=1.0, type=float)
    args = parser.parse_args(argv)
    if not args.no_analyzer and not args.analyzer_csv:
        parser.error("--analyzer-csv is required unless --no-analyzer is used")
    if args.poll_interval <= 0:
        parser.error("--poll-interval must be greater than zero")
    return args


def main():
    try:
        return run(parse_args())
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
