#!/usr/bin/env python3
# Analyze DCUA testbench VCD: differential activity on hdoutp/hdoutn vs test phase.
# Answers: S1 EI engage/release latency; S2 which gap requests emit idle (swallow test);
# S3 whether LDR bursts pass through held EI; S4 masked-EI COMWAKE shape.
import sys

def parse_vcd(fname, want=("hdoutp", "hdoutn", "ei_en", "ldr_en", "phase")):
    ids, values, t, timeline = {}, {}, 0, []
    for line in open(fname):
        line = line.strip()
        if line.startswith("$var"):
            parts = line.split()
            sym, name = parts[3], parts[4]
            if name in want:
                ids[sym] = name
        elif line.startswith("#"):
            t = int(line[1:])
        elif line and line[0] in "01xz" and line[1:] in ids:
            values[ids[line[1:]]] = line[0]
            timeline.append((t, dict(values)))
        elif line.startswith("b") and " " in line:
            v, sym = line[1:].split()
            if sym in ids:
                values[ids[sym]] = v
                timeline.append((t, dict(values)))
    return timeline

def main(fname):
    tl = parse_vcd(fname)
    print(f"== {fname}: {len(tl)} events ==")
    # Differential state: driven-active (p!=n valid), idle (p==n or z/x).
    runs, prev_state, prev_t, phase = [], None, 0, "0"
    for t, v in tl:
        phase = v.get("phase", phase)
        p, n = v.get("hdoutp", "x"), v.get("hdoutn", "x")
        if p in "01" and n in "01" and p != n:
            state = "ACTIVE"
        elif p in "xz" or n in "xz" or p == n:
            state = "IDLE"
        else:
            state = "?"
        if state != prev_state:
            if prev_state is not None:
                runs.append((prev_t, t, prev_state, phase))
            prev_state, prev_t = state, t
    for a, b, st, ph in runs[-200:]:
        if b - a > 5000:  # >5ns runs only
            print(f"  phase{ph} t={a/1e6:9.3f}us {st:6} {(b-a)/1000:8.1f}ns")

if __name__ == "__main__":
    main(sys.argv[1])
