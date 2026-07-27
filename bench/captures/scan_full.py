from migen import *
from litex.soc.cores.code_8b10b import Encoder
ALIGN=[(0xBC,1),(0x4A,0),(0x4A,0),(0x7B,0)]
def enc_stream(pairs,n):
    e=Encoder(1,True); out=[]
    def g():
        for i in range(n+4):
            d,k=pairs[i%len(pairs)]
            yield e.d[0].eq(d); yield e.k[0].eq(k); yield
            out.append((yield e.output[0]))
    run_simulation(e,g()); return out[2:2+n]
def table():
    t={}
    syms=[(d,0) for d in range(256)]+[(x,1) for x in (0x1C,0x3C,0x5C,0x7C,0x9C,0xBC,0xDC,0xFC,0xF7,0xFB,0xFD,0xFE)]
    for d,k in syms:
        for pre in ((0x00,0),(0xBC,1)):
            got=enc_stream([pre,(d,k)]*8,16)
            for j in range(2,14,2): t[got[j+1]]=(d,k)
    return t
DEC=table()
codes=enc_stream(ALIGN,40)
bits=[]
for c in codes: bits+=[(c>>i)&1 for i in range(10)]
N=len(bits)
TARGETS={0xFC35B5EE:"measured with LSM enabled",0xCF4A4AEE:"measured with LSM disabled",
         0x7B4A4ABC:"CORRECT ALIGN",0x65B5A2EE:"seen in the noisy capture"}
print(f"scanning {N} bit phases x 4 dword offsets ...")
found={}
for ph in range(N):
    syms=[DEC.get(sum(bits[(ph+j*10+b)%N]<<b for b in range(10)),(0xEE,1)) for j in range(8)]
    for off in range(4):
        q=syms[off:off+4]
        if len(q)<4: continue
        dw=sum(q[b][0]<<(8*b) for b in range(4)); ck=sum(q[b][1]<<b for b in range(4))
        if dw in TARGETS and dw not in found:
            found[dw]=(ph,off,ck)
for t,desc in TARGETS.items():
    if t in found:
        ph,off,ck=found[t]
        print(f"  {t:08X} ({desc}): bit phase {ph} (={ph%10} mod 10, symbol-pair {(ph//10)%2}), dword off {off}, k{ck:04b}")
    else:
        print(f"  {t:08X} ({desc}): NOT FOUND anywhere in a clean ALIGN stream")
