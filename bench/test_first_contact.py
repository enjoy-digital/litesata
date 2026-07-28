#!/usr/bin/env python3
"""First-contact protocol: run IMMEDIATELY after physically power-cycling the drive.

The drive's negotiation logic has been observed to latch into a reject-everything state within
a power session (bench/BRINGUP.md campaigns 48-52). This script owns the very first negotiation
after a true power-on - the one attempt per session where acceptance has never been tested clean:

  1. Loads the requested bitstream (default T-g1host = Gen1-rate host, campaign 52).
  2. Parks the line until you press Enter (plug the drive while parked).
  3. Arms the link-RX signature watch (fsm2==RDY, the drive's first X_RDY) BEFORE enabling.
  4. Runs ONE spec-exit attempt (we ALIGN until the DRIVE sends SYNC - true mutual acceptance),
     falling back to one lenient attempt if requested.
  5. On a held link: fires IDENTIFY once and decodes MODEL/SERIAL/FW/CAPACITY.

Usage: litex_server running; then
  python3 bench/test_first_contact.py [--bitstream T-g1host|S-txboost] [--lenient-fallback]
"""
import argparse, subprocess, sys, time, os, collections

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BITS = os.path.join(REPO, "bench", "captures", "bitstreams")

EI=1<<1; LDR=4<<8; BM=1<<18; ZB=1<<19; DIS=1<<26
PATALT=1<<5; DEEMPH=1<<6; EARLY=1<<9; RELAX=1<<10; LEN=1<<13
CTRL=LDR|BM|ZB|EI
LRX=["IDLE","RDY","WAIT_FIRST","COPY","EOF","WTRM","R_OK","R_ERR"]

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bitstream", default="T-g1host", choices=["T-g1host","S-txboost"],
                   help="T-g1host = Gen1-rate host (both rate fuses); S-txboost = Gen2 host.")
    p.add_argument("--lenient-fallback", action="store_true",
                   help="After a failed spec attempt, run one lenient attempt + identify.")
    args = p.parse_args()
    gen1 = args.bitstream == "T-g1host"
    burst, wake = (8, 8) if gen1 else (16, 16)

    bit = os.path.join(BITS, args.bitstream, "lambdaconcept_ecpix5.bit")
    print(f"loading {bit}...")
    r = subprocess.run(f"openFPGALoader -c ft2232 {bit}", shell=True, capture_output=True)
    assert r.returncode == 0, r.stderr.decode()
    time.sleep(1.5)

    from litex import RemoteClient
    from litescope import LiteScopeAnalyzerDriver
    b = RemoteClient(port=1234, csr_csv=os.path.join(REPO, "csr.csv")); b.open()

    def park():
        b.regs.sata_phy_enable.write(0); time.sleep(1e-3)
        b.regs.sata_phy_phy_oob_quiet.write(32)
        b.regs.sata_phy_phy_oob_control.write(CTRL|DIS)
        b.regs.sata_phy_enable.write(1)

    def attempt(txctl, window):
        b.regs.sata_phy_enable.write(0); time.sleep(1e-3)
        b.regs.sata_phy_phy_oob_txctl.write(txctl)
        b.regs.sata_phy_phy_oob_pattern.write(0xF0F0)
        b.regs.sata_phy_phy_oob_gap_pattern.write(0x0000)
        b.regs.sata_phy_phy_oob_burst_len.write(burst)
        b.regs.sata_phy_phy_oob_quiet.write(32)
        b.regs.sata_phy_phy_oob_ei_shape.write(wake<<13)
        b.regs.sata_phy_phy_oob_align.write((4096<<16)|64)
        b.regs.sata_phy_phy_oob_control.write(CTRL)
        a = LiteScopeAnalyzerDriver(b.regs, "analyzer", debug=False)
        a.configure_group(2); a.configure_subsampler(1)
        a.add_trigger(cond={"fsm2_state": "0b001"})   # drive's first X_RDY (signature FIS)
        a.run(offset=128, length=1024)
        b.regs.sata_phy_enable.write(1)
        t0=time.time(); up=False; first=None
        while time.time()-t0 < window:
            if b.regs.sata_phy_status.read()&1:
                if first is None: first=time.time()-t0
                time.sleep(1.5)
                if b.regs.sata_phy_status.read()&1: up=True; break
            time.sleep(0.05)
        st=b.regs.sata_phy_status.read()
        print(f"  link={'HELD' if up else 'NO'} first={first and f'{first:.2f}s'} status=0x{st:x}")
        t=time.time(); sig=False
        while time.time()-t < 5:
            if a.done(): sig=True; break
            time.sleep(0.1)
        print(f"  drive signature X_RDY: {sig}")
        if sig:
            a.upload(); a.save("/tmp/first_contact_sig.csv")
            rows=[r for r in open("/tmp/first_contact_sig.csv").read().splitlines() if r.strip()]
            hdr=[h.strip() for h in rows[0].split(",")]; i2=hdr.index("fsm2_state")
            occ=collections.Counter()
            for row in rows[2:]:
                f=[x.strip() for x in row.split(",")]
                if f[i2]: occ[LRX[int(f[i2],2)]]+=1
            print(f"  link-RX around it: {occ.most_common()}  (capture: /tmp/first_contact_sig.csv)")
        return up

    def identify():
        while b.regs.sata_bist_identify_source_valid.read():
            b.regs.sata_bist_identify_source_data.read()
            b.regs.sata_bist_identify_source_ready.write(1)
        b.regs.sata_bist_identify_start.write(1)
        t=time.time()
        while time.time()-t < 5:
            if b.regs.sata_bist_identify_done.read(): break
        else:
            print("  IDENTIFY timeout"); return False
        words=[]
        while b.regs.sata_bist_identify_source_valid.read():
            dw=b.regs.sata_bist_identify_source_data.read()
            words += [dw & 0xffff, (dw>>16)&0xffff]
            b.regs.sata_bist_identify_source_ready.write(1)
        s=lambda x,y: "".join(w.to_bytes(2,"big").decode("ascii","replace") for w in words[x:y])
        cap = words[100]|(words[101]<<16)|(words[102]<<32)|(words[103]<<48)
        print(f"  IDENTIFY COMPLETE ({len(words)} words)")
        print(f"  MODEL={s(27,46).strip()} SERIAL={s(10,20).strip()} FW={s(23,27).strip()} "
              f"CAPACITY={cap*512/1e9:.1f}GB")
        return True

    park()
    input("line parked - plug/power the drive now, then press Enter for the first attempt... ")
    print(f"[first contact, {args.bitstream}, SPEC exit]")
    if attempt(PATALT|DEEMPH|EARLY|RELAX, 60):
        if identify(): return
    elif args.lenient_fallback:
        print("[fallback, lenient exit] (reload recommended between attempts - identify FSM)")
        if attempt(PATALT|DEEMPH|EARLY|RELAX|LEN, 25):
            identify()
    park()
    b.close()

if __name__ == "__main__":
    main()
