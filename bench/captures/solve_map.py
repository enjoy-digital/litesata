#!/usr/bin/env python3
"""Solve the hybrid-mode TX bit mapping from the loopback observation.

We transmit ALIGN (K28.5 D10.2 D10.2 D27.3) through the fabric Encoder into the
raw TX bus, and our own DCU RX decoder reports 0xFC35B5EE / charisk=0b0001
instead of 0x7B4A4ABC / charisk=0b0001.

Brute-force: for every plausible serialization of the encoder output, generate
the serial bit stream, decode it at every bit phase with a software 8b10b
decoder, and see which candidate reproduces the observed dword.
"""
from migen import *
from litex.soc.cores.code_8b10b import Encoder

ALIGN = [(0xBC, 1), (0x4A, 0), (0x4A, 0), (0x7B, 0)]  # K28.5 D10.2 D10.2 D27.3

# --- 1. Get the real encoder output for a continuous ALIGN stream -------------
def encode_stream(pairs, n):
    """Return a list of (code10, ...) as emitted by litex Encoder(lsb_first=True)."""
    enc = Encoder(1, True)
    out = []

    def gen():
        for i in range(n + 4):
            d, k = pairs[i % len(pairs)]
            yield enc.d[0].eq(d)
            yield enc.k[0].eq(k)
            yield
            out.append((yield enc.output[0]))

    run_simulation(enc, gen())
    # Encoder has 2 cycles of latency; drop the primed samples.
    return out[2:2 + n]

codes = encode_stream(ALIGN, 40)
# Check we reached a steady repeating state.
print("encoder codes (bit0 = first on wire, 'a'):")
for i, c in enumerate(codes[:8]):
    print(f"  {i}: {c:010b}  (d={ALIGN[i%4][0]:02x} k={ALIGN[i%4][1]})")

# --- 2. Build a software 8b10b decode table ----------------------------------
# code10 (bit0='a', i.e. first on wire) -> (byte, k)
def build_decode_table():
    table = {}
    syms = [(d, 0) for d in range(256)] + [
        (0x1C, 1), (0x3C, 1), (0x5C, 1), (0x7C, 1),
        (0x9C, 1), (0xBC, 1), (0xDC, 1), (0xFC, 1),
        (0xF7, 1), (0xFB, 1), (0xFD, 1), (0xFE, 1),
    ]
    # Emit each symbol from both running-disparity states by prefixing a
    # disparity-setting symbol, then read the code out.
    for d, k in syms:
        for prefix in ((0x00, 0), (0xBC, 1)):
            seq = [prefix, (d, k)] * 8
            got = encode_stream(seq, 16)
            for j in range(2, 14, 2):
                table[got[j + 1]] = (d, k)
    return table

DEC = build_decode_table()
print(f"decode table: {len(DEC)} valid 10-bit codes")

def decode(code):
    return DEC.get(code, (0xEE, 1))  # DCU marks invalid symbols as 0xEE/K

# --- 3. Candidate serializations ---------------------------------------------
def rev10(c):
    return int(f"{c:010b}"[::-1], 2)

def bits_of(code):          # bit0 first on the wire
    return [(code >> i) & 1 for i in range(10)]

CANDIDATES = {
    "identity (s0 then s1, bit0 first)": lambda a, b: bits_of(a) + bits_of(b),
    "reverse each symbol":               lambda a, b: bits_of(rev10(a)) + bits_of(rev10(b)),
    "swap symbols":                      lambda a, b: bits_of(b) + bits_of(a),
    "swap + reverse each":               lambda a, b: bits_of(rev10(b)) + bits_of(rev10(a)),
}

OBSERVED = (0xFC35B5EE, 0b0001)

def stream_for(fn, codes, nwords=64):
    bits = []
    for i in range(0, nwords * 2, 2):
        bits += fn(codes[i % len(codes)], codes[(i + 1) % len(codes)])
    return bits

# codes repeat with period 4 symbols? disparity may make it 8; use a long run.
period = len(codes)
for name, fn in CANDIDATES.items():
    bits = stream_for(fn, codes, nwords=60)
    for phase in range(20):
        syms = []
        i = phase
        while i + 10 <= len(bits):
            code = sum(bits[i + j] << j for j in range(10))
            syms.append(decode(code))
            i += 10
        if len(syms) < 8:
            continue
        # assemble dwords (4 symbols, first symbol = byte0) at both parities
        for off in range(4):
            for s in range(0, 8, 4):
                idx = off + s + 8
                if idx + 4 > len(syms):
                    continue
                q = syms[idx:idx + 4]
                dw = sum(q[b][0] << (8 * b) for b in range(4))
                ck = sum(q[b][1] << b for b in range(4))
                if (dw, ck) == OBSERVED:
                    print(f"\n*** MATCH: {name}  phase={phase} dword-offset={off}")
                if (dw, ck) == (0x7B4A4ABC, 0b0001):
                    print(f"\n+++ CORRECT ALIGN: {name}  phase={phase} dword-offset={off}")

# Also report what each candidate decodes to at its best (comma-locked) phase.
print("\nper-candidate decode at every phase (first 4 symbols after settling):")
for name, fn in CANDIDATES.items():
    bits = stream_for(fn, codes, nwords=60)
    print(f"\n{name}:")
    for phase in range(10):
        syms = []
        i = phase
        while i + 10 <= len(bits):
            code = sum(bits[i + j] << j for j in range(10))
            syms.append(decode(code))
            i += 10
        q = syms[8:12]
        dw = sum(q[b][0] << (8 * b) for b in range(4))
        ck = sum(q[b][1] << b for b in range(4))
        print(f"  phase {phase:2d}: {dw:08X} / k{ck:04b}")
