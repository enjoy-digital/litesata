#!/usr/bin/env python3
"""Loopback word-alignment oracle.

With the SATA loopback cable fitted, force continuous ALIGN transmission and
look at what our own DCU RX decoder makes of it. A correct word boundary gives
0x7B4A4ABC/k0001; every other boundary has a known signature (solve_map.py).
"""
import os, sys, time, argparse, collections
from litex import RemoteClient
from litescope import LiteScopeAnalyzerDriver

# Decoder ring: decoded dword -> how many bits off the word boundary we are.
PHASE = {
    0x7B4A4ABC: 0, 0xEEB5B5EE: 1, 0xEE0A4AEE: 2, 0xFC35B5EE: 3, 0x6E8A4A57: 4,
    0xEE15B5AB: 5, 0xEE3F4A45: 6, 0xEE65B5A2: 7, 0xEEEE4A49: 8, 0xEEF0B5A4: 9,
}
NAMED = {0xB5B5957C: "SYNC", 0x7B4A4ABC: "ALIGN", 0x0: "zeros", 0xEEEEEEEE: "all-invalid"}

CTRL_EI_MODE   = 1 << 1
CTRL_CDRHOLD   = 1 << 2
CTRL_BURSTMODE = 1 << 18
CTRL_ZEROBUS   = 1 << 19
CTRL_CTRLDIS   = 1 << 26
CTRL_PATFORCE  = 1 << 27
CTRL_ECHOMASK  = 1 << 28
CTRL_ALIGNFRC  = 1 << 29
CTRL_LDRTMO    = 4 << 8

TXCTL_PATALT   = 1 << 5
TXCTL_DEEMPH   = 1 << 6

SCRATCH = os.path.dirname(os.path.abspath(__file__))
CSV     = os.path.join(SCRATCH, "lb.csv")

def configure(bus, align_cont, zero_bus, echo_mask, align_force, ctrl_dis=0):
    bus.regs.sata_phy_enable.write(0)
    time.sleep(1e-3)
    bus.regs.sata_phy_phy_oob_txctl.write(TXCTL_PATALT | TXCTL_DEEMPH)
    bus.regs.sata_phy_phy_oob_pattern.write(0xF0F0)
    bus.regs.sata_phy_phy_oob_gap_pattern.write(0x0000)
    bus.regs.sata_phy_phy_oob_burst_len.write(16)
    bus.regs.sata_phy_phy_oob_quiet.write(50)
    bus.regs.sata_phy_phy_oob_align.write((align_cont << 32) | (64 << 16) | 64)
    ctrl = CTRL_EI_MODE | CTRL_CDRHOLD | CTRL_BURSTMODE | CTRL_LDRTMO
    if zero_bus:    ctrl |= CTRL_ZEROBUS
    if echo_mask:   ctrl |= CTRL_ECHOMASK
    if align_force: ctrl |= CTRL_ALIGNFRC
    if ctrl_dis:    ctrl |= CTRL_CTRLDIS
    bus.regs.sata_phy_phy_oob_control.write(ctrl)
    bus.regs.sata_phy_enable.write(1)
    return ctrl

def capture(bus, group=0, length=512):
    a = LiteScopeAnalyzerDriver(bus.regs, "analyzer", debug=False)
    a.configure_group(group)
    a.configure_subsampler(1)
    a.add_trigger()
    a.run(offset=16, length=length)
    t0 = time.time()
    while not a.done():
        if time.time() - t0 > 10:
            return None
        time.sleep(0.01)
    a.upload()
    a.save(CSV)
    return CSV

def load(path):
    rows = [r for r in open(path).read().splitlines() if r.strip()]
    hdr  = [h.strip() for h in rows[0].split(",")]
    out  = []
    for r in rows[2:]:                       # row 1 = widths
        f = [x.strip() for x in r.split(",")]
        rec = {}
        for i, name in enumerate(hdr):
            if i < len(f) and f[i]:
                try:    rec[name] = int(f[i], 2)
                except ValueError: pass
        out.append(rec)
    return out

def hist_of(recs, data_key, ck_key, valid_key=None):
    h = collections.Counter()
    for r in recs:
        if valid_key and not r.get(valid_key, 0):
            continue
        if data_key in r:
            h[(r[data_key], r.get(ck_key, 0))] += 1
    return h

def report(h, label):
    total = sum(h.values())
    print(f"  [{label}] {total} samples")
    if not total:
        return
    for (d, c), n in h.most_common(6):
        ph  = PHASE.get(d)
        tag = f"  <== {NAMED[d]}" if d in NAMED else ""
        if ph == 0:   tag = "  <== CORRECT ALIGN (boundary locked)"
        elif ph:      tag = f"  <== ALIGN misaligned by {ph} bits"
        print(f"    {d:08X}/k{c:04b}  {n:5d} ({100*n/total:5.1f}%){tag}")

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cont",     default=1, type=int)
    p.add_argument("--zero-bus", default=0, type=int)
    p.add_argument("--echo-mask",default=0, type=int)
    p.add_argument("--align-force", default=1, type=int)
    p.add_argument("--ctrl-dis", default=0, type=int)
    p.add_argument("--settle",   default=2.0, type=float)
    p.add_argument("--passes",   default=3, type=int)
    args = p.parse_args()

    bus = RemoteClient(port=1234); bus.open()
    ctrl = configure(bus, args.cont, args.zero_bus, args.echo_mask, args.align_force, args.ctrl_dis)
    print(f"=== loopback: cont={args.cont} zero_bus={args.zero_bus} "
          f"echo_mask={args.echo_mask} align_force={args.align_force} ctrl_dis={args.ctrl_dis} ctrl=0x{ctrl:08x} ===")
    time.sleep(args.settle)
    st = bus.regs.sata_phy_status.read()
    print(f"phy_status = 0x{st:08x}  (ready={st&1} tx={(st>>1)&1} rx={(st>>2)&1} ctrl={(st>>3)&1})")
    for i in range(args.passes):
        path = capture(bus)
        if not path:
            print(f"-- pass {i}: analyzer timeout"); continue
        recs = load(path)
        print(f"-- pass {i}")
        report(hist_of(recs, "phy_serdesecp5_rx_word_data", "phy_serdesecp5_rx_word_ctrl"),
               "raw DCU word")
        report(hist_of(recs, "datapath_rx_source_source_payload_data",
                       "datapath_rx_source_source_payload_charisk",
                       "datapath_rx_source_source_valid"), "rx dword")
        idle = [r.get("phy_rx_idle0", 0) for r in recs]
        print(f"    rx_idle={sum(idle)}/{len(idle)}  fsm2_state hist="
              f"{collections.Counter(r.get('fsm2_state') for r in recs).most_common(4)}")
        time.sleep(0.3)
    bus.close()

if __name__ == "__main__":
    main()
