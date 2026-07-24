#
# This file is part of LiteSATA.
#
# Copyright (c) 2026 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# Minimal Siglent SDS1104X-E SCPI helper (raw socket, port 5025) used for the ECP5 SATA OOB
# bring-up (see BRINGUP.md): waveform capture + parsing for burst/gap timing analysis.

import time
import re

class SDS1104XE:
    def __init__(self, host, port=5025, timeout=5.0):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.settimeout(timeout)

    def cmd(self, c):
        self.s.sendall((c + "\n").encode())

    def query(self, c):
        self.cmd(c)
        data = b""
        while not data.endswith(b"\n"):
            data += self.s.recv(4096)
        return data.decode(errors="replace").strip()

    def query_bin(self, c):
        self.cmd(c)
        data = b""
        # Read until we have the #9 length header.
        while b"#9" not in data:
            data += self.s.recv(4096)
        i = data.index(b"#9")
        while len(data) < i + 11:
            data += self.s.recv(4096)
        n = int(data[i+2:i+11])
        total = i + 11 + n + 2  # trailing \n\n
        while len(data) < total - 2:
            data += self.s.recv(65536)
        return data[i+11:i+11+n]

    def close(self):
        self.s.close()

def parse_num(s):
    # "C1:VDIV 2.00E-01V" or "2.00E-01V" or "1.00GSa/s"
    m = re.search(r"([-+0-9.E]+)\s*(G|M|k|u|n|p)?", s.split()[-1], re.I)
    v = float(m.group(1))
    mult = {"G": 1e9, "M": 1e6, "k": 1e3, "u": 1e-6, "n": 1e-9, "p": 1e-12, None: 1.0}
    suffix = m.group(2)
    if suffix is not None and "Sa" in s:
        v *= mult[suffix]
    return v

def capture(scope, npoints=70000):
    scope.cmd("WFSU SP,1,NP,%d,FP,0" % npoints)
    vdiv = parse_num(scope.query("C1:VDIV?"))
    ofst = parse_num(scope.query("C1:OFST?"))
    sara_raw = scope.query("SARA?")
    sara = parse_num(sara_raw)
    raw = scope.query_bin("C1:WF? DAT2")
    codes = [(b - 256 if b > 127 else b) for b in raw]
    volts = [c * (vdiv / 25.0) - ofst for c in codes]
    return volts, sara

def envelope_runs(volts, sara, win_ns=25, thresh_frac=0.4):
    import statistics
    n = len(volts)
    base = statistics.median(volts)
    dev = [abs(v - base) for v in volts]
    win = max(1, int(win_ns * 1e-9 * sara))
    env = []
    # Rolling max.
    from collections import deque
    dq = deque()
    for i, d in enumerate(dev):
        while dq and dev[dq[-1]] <= d:
            dq.pop()
        dq.append(i)
        if dq[0] <= i - win:
            dq.popleft()
        env.append(dev[dq[0]])
    peak = sorted(env)[int(0.98 * n)]
    th = peak * thresh_frac
    runs = []
    cur = None
    for i, e in enumerate(env):
        v = 1 if e > th else 0
        if cur and cur[0] == v:
            cur[1] += 1
        else:
            cur = [v, 1]
            runs.append(cur)
    ns = 1e9 / sara
    return [(v, l * ns) for v, l in runs], peak, base
