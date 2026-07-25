# ECP5 SATA Bring-up Journal (ECPIX-5, issue #27)

Overnight autonomous bring-up session, 2026-07-24. Branch: `ecp5-v2`.
Hardware: ECPIX-5 85F (LFE5UM5G-85F), SSD on SATA connector (DCU1/CH0), FT2232 JTAG+UART.

## Legend
- Bitstream A = `bench/ecpix5.py --gen 1 --with-analyzer` (PHY only, OOB debug, uartbone 1Mbaud)
- Bitstream P = liteiclink `bench/serdes/ecpix5.py --linerate 1.5e9` (PRBS sanity, uartbone 115200)
- Bitstream B = A + `--with-bist` (Core/Crossbar/BIST)

## Journal

- 00:45 Sim suite green: 13 new ECP5 OOB tests + full regression (28 tests OK).
- 01:02 Build A: 18s wall, timing PASS (sys 100MHz: max 133MHz; sata_tx/rx 75MHz: max 231/253MHz).
  nextpnr accepts all DCU LDR/EI ports. Build P: 12s wall, PASS. Builds are cheap (~20s) — rebuild
  economy is a non-issue on these SoCMini designs.
- 01:05 [M0] Bitstream P loaded, uartbone OK (ident responds).
- 01:06 [M0] PRBS7 internal loopback 10s: BER 0 — but VACUOUS: serdes0_tx_ready=0/rx_ready=0,
  word-clock counters frozen. Init never completes on upstream liteiclink bench.
- 01:10 [M0] Fixed litescope_cli enum-row parsing bug (litescope repo, local commit needed).
  Litescope capture of init FSM: state=RESET-PCS-WAIT-RX-CDR-LOCK, tx_lol=0, rx_lol=1 (100% of
  1016 samples).
  CONCLUSION: TX PLL LOCKS at 1.5Gbps from the fabric-PLL 150MHz refclk (the critical half of
  M0). RX CDR cannot lock: SATA line is electrically idle (SSD waits for OOB) — expected, and
  exactly why the vendored SerdesInit was split into tx_ready/rx_ready with no rx_los exit.
  Risk #6 (rx_lol high until real data ⇒ phy.ready blocked) is CONFIRMED PLAUSIBLE: on bitstream
  A, watch whether ctrl's rx_cdrhold (SCI) clears rx_lol; else fall back to ready=tx_ready.
- 01:25 [M1] Bitstream A up: tx_ready=1 rx_ready=1 (risk #6 resolved: ctrl's SCI cdr_hold clears
  rx_lol on idle line). RX recorders: SSD sends spontaneous COMINITs (bursts 100-110ns, gaps
  320-330ns — textbook; RLOS resolves 100ns events cleanly, historic worry dead). But ctrl stuck
  in COMINIT: tx_cominit_ack never pulses.
- 01:35 [M1][ROOT-CAUSE-HUNT] TX word clock measured DEAD (0MHz) in every config; RX = 150MHz.
  Bisect: OOB hookups innocent (baseline identical to acorn-link DCU also dead). Full param diff
  vs Diamond-generated Lattice SERDES Eye Demo netlist (~/dev/google/ecp5_serdes/reference/pcs.v)
  → D_SYNC_LOCAL_EN="0b1" in every Diamond netlist, never set by liteiclink (defaults 0).
- 01:55 [M1][FIX] p_D_SYNC_LOCAL_EN="0b1" → TX word clock ALIVE: tx=149.7MHz rx=149.7MHz @gen2.
  Silent liteiclink bug since 2019 (TX PCLK dead on trellis builds at least on ECPIX-5/DCU1).
  Also learned: nextpnr ignores D_TX_MAX_RATE/CDR_MAX_RATE (Diamond-only); prjtrellis maps
  VCO_CK_DIV/DCO_CK_DIV/RATE_MODE/CMU params directly.
- 02:10 [M1a/M1b ACHIEVED @gen2] With TX alive + LDR bursts: device answers our COMRESET at the
  ctrl retry rate (282 bursts/s vs 12/s spontaneous floor). ctrl walks COMINIT -> AWAIT-COMINIT ->
  CALIBRATE -> COMWAKE; our COMWAKE transmits with spec-exact total duration (ack 1.33us after
  stb, capture bench/captures/comwake.csv). RLOS-based COMChecker classifies device COMINIT
  cleanly (gaps 310-330ns).
- 02:30-03:30 [M1c BLOCKED - root cause characterized] Device never answers our COMWAKE. Device-
  in-the-loop gap sweep (tx_test free-running sequences, device COMRESET acceptance window
  [175,525]ns as the measuring instrument):
    gap request >= 226ns -> device responds (massively); <= 200ns -> NOTHING (sharp cliff).
  With ei_lead shaping: lead=4 cycles (27ns) converts COMRESET gaps into device-invisible ones.
  MODEL (all observations consistent): FFC_EI_EN engagement completes ~220ns after request
  assert; requests dropped before completion are SWALLOWED ENTIRELY (no idle emitted). Matches
  Florent's 2022 measurement (220ns min pulse) exactly.
  => Shortest emittable TX idle gap ~226ns > COMWAKE max gap 175ns:
  => HOST COMWAKE IS NOT PRODUCIBLE through the ECP5 DCU EI path. Also tested and dead:
    - EI held + LDR bursts: EI mutes LDR entirely (12/s floor).
    - LDR_CORE2TX_SEL=0b1: identical to 0b0.
    - LDR-constant-level gaps (AC-decay bet): not seen as idle even at 320ns (COMRESET dies).
    - FFC_PCIE_CT (with and without CHX_PCIE_MODE): kills TX entirely in 10BSER config.
- 03:45 [gen1 status] After SYNC_LOCAL_EN fix: gen1 (VCO 3G, d=2) TX word clock alive (74.9MHz),
  RX side dead (rx_lol stuck, RLOS blind). Diamond reference shows all 10 CHx_DCO* CDR values
  differ for divided-rate configs; transplanting the Lattice 1.25G set (312.5MHz-refclk-tuned)
  killed TX PLL lock too -> reverted. gen1 RX needs proper VCO-3G/d=2 DCO tuning (Diamond/
  Clarity generation or fuzzing) - follow-up work. gen2 is the healthy config.
- 04:00 Consolidation: canonical gen1/gen2 bitstreams rebuilt + archived under
  bench/captures/bitstreams/, captures archived, sims green (13 tests), board left on gen2
  (COMRESET/COMINIT handshake live, LEDs: sys/tx/rx heartbeats + ready off).

## Paths forward for COMWAKE (issue #27)

1. Try other SATA drives: many real PHYs accept COMWAKE gaps beyond the 175ns spec detector
   bound; gaps of 226-240ns ARE producible. The sweep harness automates the test (2min/drive):
   tx_test + wake_gap=34..36, watch for device COMWAKE-class response.
2. Scope the TX pair to confirm the swallow behavior and search SCI undocumented registers for
   an EI timing control (SCI reg map is largely undocumented; a targeted scan is possible).
3. Ask Lattice / check TN1261 errata for fast-EI or SATA OOB guidance on DCU.
4. Board-level assist (next revision / rework): differential analog switch or attenuator on the
   TX pair driven by a fabric GPIO would make OOB trivial and keep everything else as-is.
5. Upstream the independent fixes now (liteiclink): D_SYNC_LOCAL_EN (un-breaks ECP5 DCU TX on
   nextpnr/trellis for everyone), refclk-selection notes, litescope enum-row CLI fix.
- 04:30 [liteiclink] D_SYNC_LOCAL_EN fix applied upstream on branch `ecp5-dcu-tx-fix` (0db96f7).
  Note: the upstream bench cannot self-observe the fix with a silent link partner - cd_tx reset
  and sci_reconfig reset are both gated on full init.ready (needs RX lock), the exact structural
  issue the litesata vendored copy fixes with split tx_ready/rx_ready. For SGMII/PCIe use-cases
  (partner transmits immediately) the one-line fix is sufficient and effective. Recommend also
  upstreaming the split-ready SerdesInit later (API/semantics change - Florent's call).
- 04:35 [ktemkin blog re-check] Fetched https://ktemk.in/post/serdes-lfps/ - confirms the
  technique is the DCU LDR path (FFC_LDR_CORE2TX_EN/LDR_CORE2TX, LDR_RX2CORE), NOT fabric GPIO
  (DCU pads cannot be PIO). Blog says nothing about idle between bursts (USB3 LFPS gaps are us-
  scale, 20-80x longer than COMWAKE's 106.7ns, so the ~220ns EI engage floor never matters for
  LFPS). Our PHY already uses this exact technique - it is what makes COMRESET work.
- 05:00 [Drive 2 test] Second SSD connected, gen2 bitstream:
  * Answers our COMRESET at ~570 bursts/s (every retry, textbook COMINIT 100-120ns/310-330ns).
  * ZERO spontaneous COMINITs (TX off -> 0/s) - every response is caused by our TX (clean ruler
    for TX-aliveness, unlike drive 1's 12/s background).
  * Responds COMINIT (~330-680/s) to nearly any TX activity, including gap-swallowed continuous
    burst streams - unlike drive 1 which ignored those. So drive 2's response *rate* cannot
    measure our emitted gap lengths.
  * NEVER replies COMWAKE: full-sequence sweeps (wake_gap 93-293ns, trail 0/4, ei_mode legacy +
    shaped) all COMINIT-class only, ctrl_ready never set.
  * SCI experiment (runtime, no rebuild): set pcie_ei_en (CH reg 0x02 bit 6, read 0x0c ->
    wrote 0x4c) - no behavioral change on full sequence or tx_test. CH regs 0x00-0x3F dumped in
    journal history for reference.
  CONCLUSION: two drives with different personalities both answer COMRESET and both refuse
  COMWAKE -> the common factor is our transmitted COMWAKE, consistent with the EI ~220ns
  swallow pathology (and with the 2022 scope measurement). Definitive next step needs eyes on
  the TX pair: scope in tx_test mode (continuous COMWAKE pattern) shows in one glance whether
  107ns gaps exist on the wire.
- [scope session] Direct probing of the TX pair failed repeatedly (probe/access issues), but the
  crosstalk of our own TX onto the RX pair + the FPGA-side recorders turned out to be a working
  self-measurement channel: control pattern (320ns gap requests) shows our gaps on the wire
  (floor 310ns); COMWAKE patterns (107ns and 160ns requests) show NO emitted gaps at all (RX
  sees only the drive's COMINITs). Retroactively, the night's high-rate recorder counts (47k+)
  were this same self-echo. The ~220ns EI swallow is thereby confirmed by three independent
  measurements: drive-1 acceptance cliff (226 vs 200ns), self-echo cliff, explicit no-gap
  COMWAKE captures. Scope no longer needed.
- [force_wake experiment] Out-of-spec shortcut: skip COMWAKE (auto-ack + synthesized device
  response, CSR oob_control.force_wake). Mechanism verified with litescope (ctrl reaches
  AWAIT-ALIGN 1.8us after device COMINIT, transmits ALIGNs) - drive 2 refuses: keeps re-sending
  COMINIT (498/s), strictly waiting for a real COMWAKE. Negative, as spec predicts.
  FINAL: all software avenues exhausted; COMWAKE emission is the sole blocker for link-up.
- [scope campaign 2 - wire-guided COMWAKE shaping] With the probe finally landed, wire-measured
  and iterated: (1) EI "swallow" refined: emitted gap = max(request+~20ns, ~200ns) for isolated
  requests, BUT ei_trail pre-release BREAKS the floor: single-shot COMWAKE gaps shaped down to
  95-147ns (wire-verified, trial with gaps 134-147ns all in spec window captured). (2) ei_trail
  trades gap time for serializer-garbage burst extension -> added burst_mode CSR (serializer
  bursts like the 2022 design, full-rate content) - COMRESET still answered. (3) Added PRE state
  (pre-release EI before burst 1 to fix the runt first burst). Swept the full (wake_gap, trail)
  x burst_mode x rx_detector plane with wire-verified near-nominal COMWAKEs (gaps 95-128ns,
  bursts 85-119ns): drive answers COMINIT to everything COMRESET-like and NEVER replies COMWAKE.
  litescope proof: RX flat on both detectors for 9.2us after each COMWAKE.
  Open hypotheses requiring better instruments:
    a. Golden-reference diff: capture a real host's COMWAKE to this drive (PC SATA port + scope
       on the cable) and compare against ours - the definitive experiment.
    b. Residual differential activity during our EI gaps (invisible single-ended, would explain
       COMINIT-tolerant/COMWAKE-strict asymmetry): needs differential or two-channel A-B probing.
    c. Board-level TX switch assist (unchanged fallback).
- [campaign 3 - Xilinx comparison + Lattice documentation dig] Per Florent's suggestion:
  * Xilinx GTP OOB attributes (a7sataphy): detector windows in 75MHz oobclk units - notably
    SATA_MAX_WAKE=7 (~93ns-class upper bound), suggesting real detectors sit well below the
    175ns theoretical limit -> motivated sub-nominal gap sweep (emitted 60-100ns): no link.
  * Lattice FPGA-TN-02206 (SerDes/PCS User Guide) key finds (PDF text in scratchpad tn02206.txt):
    - 8.25: tx_idle_chx_c (FFC_EI_EN) is a PIPELINED WORD-SYNCHRONOUS control, idle achieved
      <20 UI after the designated word, and requires "all zeros" clocked on the parallel bus
      during EI. THIS DESCRIBES PCS-MANAGED MODES - our 10BSER/UC_MODE bypass config likely
      routes EI through a slow async path instead => PRIME ROOT-CAUSE SUSPECT: switch the PHY
      to G8B10B PCS mode (as LUNA does) and re-measure EI crispness. Next-session experiment.
    - Table 8.2: RLOS assert AND deassert response: 8-10ns typ/max (RX detection never the issue).
    - 8.30: official OOB/LDR path documentation (TXD_LDR/TX_LDR_EN, RXD_LDR/RXD_LDR_EN).
    - 8.27: PCIe receiver detect procedure (EI >=120ns then pci_det_en).
  * zero_bus experiment (TN 8.25 zeros discipline, tx_produce_pattern mux): made emission WORSE
    in current bypass mode (gaps nearly vanished on wire) - consistent with the mode hypothesis.
  * All sweeps negative for link. Board restored to best-known state (COMRESET/COMINIT handshake
    live at 480 responses/s).
  NEXT-SESSION SHORTLIST: (1) G8B10B PCS-mode PHY variant + wire EI re-measure [prime suspect],
  (2) golden-reference capture of real host COMWAKE to this drive (PC SATA + scope),
  (3) differential-probe gap measurement, (4) analog-switch interposer.
- [campaign 4 - G8B10B PCS mode] Implemented pcs_mode="g8b10b" in the vendored serdes (DCU-
  internal 8b10b, LUNA bus mappings: data[0:8]/K[8]/data[12:20]/K[20], invalid symbols decode
  0xEE+K, LSM_DISABLE=1 + edge-sensitive FFC_ENABLE_CGALIGN re-arm pulses, optional PCIE_MODE),
  plumbed through PHY/LiteSATAPHY/bench (--pcs-mode g8b10b). Results:
  * Mode fully healthy: word clocks 149.8MHz, drive answers COMRESET at full rate.
  * EI PATH IS MODE-DEPENDENT (TN-02206 hypothesis partially validated): at spec-timed COMWAKE
    requests, bypass mode emits NO gaps at all; g8b10b emits real gaps. Wire calibration found
    a textbook emission point: LDR bursts, wake_gap=20 -> burst med 114ns / gap med 126ns with
    28/29 gaps in the COMWAKE window.
  * DECISIVE NEGATIVE: 60s soak at that textbook emission (~5000+ valid COMWAKE sequences) ->
    drive never replies. G8B10B serializer-burst sweep also all-negative.
  REVISED CONCLUSION: burst/gap TIMING is achievable and is NOT the blocker. The drives reject
  a wire-verified spec-timed COMWAKE -> the remaining difference is invisible to single-ended
  band-limited probing: prime suspects are the differential/amplitude quality of LDR bursts at
  the device squelch (COMINIT's forgiving detector accepts them; COMWAKE's stricter one may
  not) or a device-side content qualification. Next instruments unchanged: golden-reference
  capture (real host + scope), differential probing. Note: a board-level TX switch interposer
  gates the full-quality serializer signal and is therefore MORE likely to work than before.
- [campaign 5 - drive hygiene + host-likeness] Per Florent's insights: (a) clean-shot protocol
  adopted (TX silent during drive plug, one spec-paced attempt then quiet - no more test-mode
  storms; drives CAN wedge on incoherent OOB floods, keep this discipline); (b) new knobs:
  oob_control.repeat (2^N back-to-back sequences = sustained host-like COMRESET assertion) and
  oob_control.d102 (D10.2 serializer burst content, exactly what Xilinx GTP emits during OOB).
  Fresh re-plugged drive, clean attempts: golden timing / 8-sequence sustained reset / D10.2
  content / combinations -> drive answers EVERY COMRESET (COMINIT-class only), never COMWAKE.
  EXONERATED SO FAR: gap/burst timing (wire-verified textbook), burst content (LDR square
  9-75MHz, serializer D0.0, serializer D10.2), sequence sustain (1/2/4/8), first-burst quality,
  both PCS modes, both EI modes, zeros-on-bus, PCIE_CT/pcie_ei_en, RX detectors, two drives,
  fresh-drive clean pacing.
  REMAINING SUSPECT (needs instruments beyond this bench): analog/differential behavior of our
  EI gaps and LDR bursts at the device's COMWAKE-path squelch (COMINIT path demonstrably
  tolerant). Endgame: golden-reference diff (real host + scope), differential probing, Lattice
  support case (data package ready), TX-switch interposer.
  TODO(gateware): ctrl retry limit + backoff CSR (polite-host behavior, avoids wedging drives).
- [drive 3 quick test] Third SSD, clean-shot protocol, near-optimal settings (G8B10B textbook
  wg=20 / SER+D10.2+8seq / vanilla wg=16): answers EVERY COMRESET (12k bursts per 20s attempt,
  textbook COMINIT 310-330ns gaps), never replies COMWAKE. Three drives, identical signature -
  the analog-quality suspicion stands. Line left quiet.
- [campaign 6 - out-of-the-box: Diamond behavioral simulation] Discovered /opt/diamond/3.12
  installed with bundled ModelSim + compiled DCUA BEHAVIORAL model (ovi_ecp5u lib). Built
  bench/dcusim/: auto-generated DCUA testbenches from our exact bypass/g8b10b parameter sets
  with 4 OOB scenarios (S1 EI step latency, S2 gap-request swallow sweep, S3 LDR-through-
  held-EI mute test, S4 masked-EI COMWAKE) + VCD analyzer. Compiles clean; simulation BLOCKED
  on license: /opt/diamond/license.dat expired 04-nov-2025 and lacks 'latticemsim', and its
  HOSTID matches no current NIC. ONE free-license renewal (latticesemi.com, MAC
  04:d9:f5:d4:31:4c) unlocks BOTH the DCUA behavioral sim (bench/dcusim/run_and_analyze.sh)
  AND Clarity/IPexpress generation of Lattice's official SATA-preset DCU netlist - the two
  most decisive remaining instruments. Also searched: no other public ECP5/Gowin SATA OOB
  implementation found (Antmicro's open-tools SATA = Xilinx GTP hard OOB; ECP3 SATA IP not
  located; whitequark IRC archive bot-blocked).
- [autonomous session, campaigns 7-9] All hands-free avenues executed:
  * SCI hidden-bit exploration: all mode-differing regs (0x00/0x03/0x04/0x38) and every unknown
    bit of reg 0x02/0x04 flipped with the drive as oracle -> completely flat (those SCI-visible
    regs behave as status shadows, not live controls). CLOSED, negative.
  * Burst content RATE hypothesis (Gen1-rate qualifier): new oob_pattern CSR drives raw
    serializer symbols during OOB (produce_pattern path, bypass mode): tested 750MHz fundamental
    (0x33333 = Gen1-D10.2-equivalent at gen2), 1.5GHz (0x4A4A), 375MHz (0x0F0F) -> identical
    behavior, no COMWAKE reply. CLOSED, negative (also explains nothing Xilinx-specific).
  * COMWAKE launch-timing dimension: new oob_quiet (COMChecker quiet threshold) and
    oob_wake_delay CSRs; swept launch from +600ns (Xilinx-like early) to +5ms (calibration
    window) after device COMINIT, on golden and spec shapings -> all negative. CLOSED.
  * Quantitative model elimination: no single TX-side impairment model (start/end lag, mid-gap
    transient, gap chopping) is consistent with the complete dataset - each predicts acceptance
    somewhere in the swept ranges or breaks COMINIT. The remaining explanation space is
    structurally invisible from this side of the link (differential/analog signature or a
    shared device-side qualification) - exactly what the golden-reference capture and the
    licensed DCUA behavioral sim (bench/dcusim, ready to run) will decide.
  * Polite-host feature: LiteSATAPHYCtrl gains opt-in oob_retries/oob_backoff (BACKOFF state,
    attempt counter outside the FSM ResetInserter domain), sim-tested (test_ctrl_backoff);
    default None = behavior identical for all existing PHYs.
  Board state: bypass gen2 bitstream, line QUIET. 14 ECP5 tests green.

## 2026-07-24 late session - campaign 10: ATTRIBUTION COLLAPSE - the drive never heard us at all
- [gap-visibility probe, v1+v2] New COMGenerator probe mode (oob_control.probe, bit 23): emit a
  COMINIT-shaped sequence with gap #3 = wake_gap, using the drive's COMINIT "response" as an
  oracle for gap visibility. First sweep: response ~300/0.5s for EVERY X from 26ns to 1200ns,
  including gaps far above the COMINIT window. Suspected boundary chaining (FINISH->IDLE is
  2 cycles, boundary gap ~333ns is itself in-window) -> added probe inter-sequence QUIET state
  (oob_seq_quiet CSR, tx cycles). Isolated probe: STILL response=300 at every X, including
  1200ns negative controls... and then the killer control:
- [BASELINE BLOWN] response=300 with tx_test OFF entirely. And ctrl parked (new
  oob_control.ctrl_dis bit 26: masks ctrl's OOB TX requests + forces EI => first truly silent
  TX line of the whole campaign): STILL 576-600 bursts/s, textbook COMINIT signature
  (bursts 100-110ns, gaps 310-330ns).
  **The drive free-runs an autonomous COMINIT beacon: period 10.003ms, rock-stable.**
- [beacon invariance] Beacon rate identical for: silent line, COMRESET storms at 2.3k/9k/55k
  sequences/s, continuous LDR carrier. Rate attribution is therefore worthless.
- [beacon-phase instrument] New CSRs _oob_lat (sys cycles from TX sequence end txcomfinish to
  next RX burst start) + _oob_beacon (interval between RX beacon starts, gap>100us qualifier).
  Phase test: COMRESET storm at P~441us (incommensurate with 10.003ms) -> latency distribution
  over [0,P): UNIFORM (chi2 ~ 1-2 over 10 bins, n=375). **The drive's beacon phase is completely
  uncorrelated with our COMRESET sequences: the drive has NEVER decoded a single sequence we
  sent - LDR or serializer, bypass or g8b10b, any trail/content/amplitude.**
  All prior "COMRESET answered" results were this beacon + attribution error (a 10ms-retry ctrl
  always "sees a response" within its window). COMWAKE was never the specific blocker.
  False-positive taxonomy (documented so nobody re-chases them):
  * trail=4 chi2 ~ 80-160 comb: appears ONLY at trail=4 (107+27ns bursts), vanishes at
    trail=6..15 (147-208ns bursts) -> non-monotone needle = internal artifact, not audibility
    (a real squelch-attack-time effect would be monotone in burst length).
  * single-shot "excess" of sub-ms latencies: exactly reproduced by the beacon landing inside
    the multi-ms tx_test-on host window (UART turnaround) - measurement-window artifact.
- [TX config exonerated vs Diamond] Full TX-analog param diff vs the Diamond reference netlist
  (method that found D_SYNC_LOCAL_EN): TDRV slices identical; RTERM_TX/RTERM_RX absent from our
  instance but nextpnr defaults RTERM_TX to 19 (50 ohm) in the emitted .config - verified in
  fuses; TXAMPLITUDE is documentation-only (slices are the real control). TX boost added
  (serdes tx_boost=True / bench --tx-boost: all 6 TDRV slices at max current, fuses verified):
  NO effect on beacon phase or handshake. LDR amplitude unaffected by TDRV slices (separate
  fixed-drive aux buffer, ~285mV/leg when the probe made good contact).
- [wire vs drive contradiction] Scope at C113 (TX pair AC cap): LDR carrier measured up to
  ~285mV/leg (~570mVppd, spec-level) in good-contact sessions; serializer ALIGN fuzz present.
  Probe contact is hand-placed and drifted badly across sessions (30-300mV for identical
  configs) - absolute amplitude claims unreliable, but signal presence + timing at C113 are
  solid (earlier wire-verified 95-147ns gaps). Meanwhile the drive hears nothing.
- [receiver-detect attempt] Wired DCU FFC_PCIE_DET_EN/CT + FFS_PCIE_DONE/CON (_oob_rxdet CSRs)
  to sense far-end RX termination through the caps: DONE never asserts in bypass config on
  trellis - inconclusive, likely needs PCS mode/clocks we don't run.
- [WHERE THIS LEAVES US] Our RX path is perfect (3 drives, textbook beacon signatures). Our TX
  is spec-configured, present and correctly timed at C113, yet zero reception at the drive.
  Fault domain: physical TX path beyond C113 (cap solder/crack, connector pin/footprint, cable
  pair) OR drive-side gap/burst visibility through the AC caps (residual-differential during
  EI, tau ~ 300ns vs 320ns gaps - old hypothesis b, still uneliminated remotely).
  Hands-on next steps (morning):
  1. SATA LOOPBACK: cable/adapter from our TX back to our RX - our own proven-good RX becomes
     the far-end detector for the whole TX+caps+connector+cable path. One experiment, splits
     the fault space in half. (LambdaConcept factory-tested this port, method unknown.)
  2. Two-channel/differential probing of BOTH TX legs at BOTH sides of C112/C113 during OOB.
  3. Cable swap + continuity meter on the TX pair.
  4. Golden-reference capture of a real host's COMRESET (unchanged).
  New knobs this session: oob_control.probe(23)/kick(24-25)/ctrl_dis(26)/pat_force(27),
  _oob_seq_quiet, _oob_lat, _oob_beacon, _oob_rxdet, serdes tx_boost (+ bench --tx-boost),
  wake_gap widened to 8 bits. 14 ECP5 tests green throughout.
  Board state: A-gen2-boost bypass bitstream loaded, ctrl parked, line at EI (silent).

## 2026-07-25 remote-only addendum - internal loopback + crosstalk hunt (no hands available)
- [SCI raw access] sci_reconfig CSRs (pause/sel/adr/we/re/dat) work on the live bitstream; full
  CH reg dump recorded (see session log). Protocol: pause=1 freezes the reconfig FSM (which
  otherwise continuously rewrites CH 0x15/0x18 from the wrapper control signals - raw writes
  only stick while paused; cdr_hold/polarity changes need pause=0 to propagate).
- [SCI serial loopback: NONFUNCTIONAL] Swept CH reg 0x15 lb_ctl nibble through all 15 non-zero
  values (writes verified by readback) with three independent indicators: (1) drive beacon
  presence on RX (never vanished -> RX input mux never switched), (2) RX decode content via new
  rx-domain analyzer build (all-zero always), (3) RX recovered word clock with CDR released and
  continuous D10.2 on TX (never snapped to 150MHz -> CDR never saw looped data). liteiclink's
  "FIXME: lb_ctl 0b0001 does not seem enough" is CONFIRMED: no SCI-only serial loopback in this
  10BSER/trellis config. SB_BYPASS/RX_SB_BYPASS fuses are 0 (SB active, same as Diamond ref).
  => Internal loopback CANNOT substitute for a physical loopback cable.
- [NEW SILICON FINDING - gen2 RX word clock fragility] With CDR released (cdrhold_dis) on an
  idle line, FF_RX_PCLK collapses/wanders (~1-40MHz measured vs 150MHz nominal; clock-counter
  latch CDC itself degrades in this state, negative deltas). Never noticed before because ALL
  OOB instrumentation (RLOS, COMChecker, recorders) is clockless/sys-domain. Re-verify CDR
  acquisition + word clock once a real link partner sends ALIGNs; potential post-link-up
  landmine. (Historical note: M0 "PRBS BER 0" was already flagged VACUOUS on night one; and the
  01:25 M1 entry literally says "SSD sends spontaneous COMINITs" - the beacon was observed and
  its attribution hazard missed. Hindsight is 20/20.)
- [RX_LOS_LVL knob + crosstalk hunt] p_CHX_RX_LOS_LVL plumbed as serdes/bench arg
  (--rx-los-lvl, default 4 = wizard value). Calibration: LVL=1 is below the RX noise floor
  (RLOS saturates >65k bursts/s on a quiet line); LVL=2 is clean (600/s beacon only).
  Crosstalk hunt at LVL=2: TX storms (LDR + serializer/D102/boost) produce ZERO burst-rate
  increase and zero crosstalk-class (<5us) latencies on the RX pair -> no detectable NEXT from
  our TX into the RX pair. INCONCLUSIVE by design (expected cable NEXT ~10mVppd is likely below
  the LVL=2 threshold); only a positive would have been informative.
- [bench fix] rx-domain analyzer group used an unhashable Cat() (never buildable since written)
  - split into individual decoder invalid signals; A-gen2-boost-rxan archived.
- Bitstream archives added: A-gen2-boost-rxan (rx-domain analyzer), A-gen2-los1, A-gen2-los2.
- CONCLUSION unchanged and sharpened: every remote avenue is now exhausted. The fault domain is
  strictly physical/external (TX path beyond C113, or drive-side gap visibility through the AC
  caps). Next session needs hands: (1) SATA loopback cable TX->RX (single decisive experiment),
  (2) both-legs/both-sides probing of C112/C113, (3) cable swap + continuity.
  Board state: A-gen2-boost loaded, ctrl parked, TX at EI, beacon 600 bursts/s confirmed.

## Campaign 12 (2026-07-25, loopback cable): SELF LINK-UP + EI amplitude-starvation discovery

Setup: user-installed SATA loopback cable TX->RX (straight, no P/N swap: rx_polarity=0 all night).
Schematic fact (SCH_ECPIX-5_R02.PDF): RX path has series caps C122/C127 (100nF) mirroring TX
C112/C113 => in loopback the connector-to-connector segment is a DC-FLOATING island (4 caps, no
DC termination). With a drive attached the drive terminates DC and the island vanishes.

- [beacon origin re-confirmed] Silent line with loopback (no drive): RX = 0 events. The 10.003ms
  beacon was the drive's, and our RX front-end is quiet.
- [EI STARVES THE TX DRIVER - likely ROOT CAUSE of drive deafness] AC-probe at C113 (FPGA side,
  DC solid at 1.2V rail in all states => not island drift):
  * EI-gapped OOB storms (any PCS mode, any gap 106-1333ns, LDR or serializer content):
    amplitude collapses to ~4-12mV from a drained state; from a freshly-driven state it starts
    ~25% (90-350mV) and decays over seconds.
  * Continuous driven carrier: recovers/holds 350-416mV (charge time seconds, full charge tens
    of seconds).
  * G8B10B EI identical (word-sync is digital only). TX/RX word clocks alive in all states
    (clock death ruled out).
  * Historical reconciliation: all good-amplitude scope sightings (285mV/leg) were continuous
    carriers; the 30-300mV "probe contact" spread across identical configs was partly REAL
    (charge-state dependence). EI-gapped handshake bursts (always launched from long EI parks)
    likely left the pins far below any drive's squelch => 3 deaf drives.
- [driven-gap OOB works through the physical path] gap_mode=1 (LDR keeps driving a constant
  level during gaps) keeps full amplitude; through 4 caps + cable, our rx_sel=1 transition
  detector (ldr_idle) decodes it: COMRESET gaps [170-570ns] @lt=8, in-window; COMWAKE needs
  lt=4 (the ldr_timeout subtracts from the measured gap: gap_meas ~ gap - lt*10ns).
  Amplitude detector (RLOS) cannot see driven gaps (expected).
- [3 RTL bugs found & fixed via the loopback]
  1. Handshake deadlock: tx_cominit/comwake requests crossed as one edge-derived pulse; if it
     lands while the generator is busy (startup race) it is swallowed and ctrl waits forever
     (stb held high, no new edge). Fix: pending requests re-pulse every 2^13 sys cycles.
  2. COMChecker chatter sensitivity: 1-2 cycle idle glitches reset the consecutive-gap count.
     Fix: 3-cycle persistence filter on the rx_idle observation.
  3. ctrl.rx_reset unwired on ECP5 + RX 16->32 gearbox phase: misalign flapping READY->RESET_RX
     every 160ns (beat pattern) can hold ctrl.ready off; converter self-reset usually lands the
     right phase (10/10 in final build) - deterministic rephase is a TODO.
- [loopback self-handshake support, echo_mask CSR bit 28 + ctrl loopback path]
  ctrl COMINIT exit needs ack & ~rx_cominit which an instant echo makes unsatisfiable =>
  echo_mask hides detections while our stb is high (echo outlives stb by the quiet window).
  AWAIT-ALIGN transmits ALIGN (a real device link sends D10.2), SEND-ALIGN also counts our own
  ALIGN echo (0xBC low byte), READY ignores RLOS rx_idle chatter in loopback.
- [RESULT: SELF LINK-UP] gen2 G8B10B boost bitstream (B-gen2-g8b10b-fix), config: rx_sel=1,
  ldr_timeout=4, gap_mode=1, ei_mode=1, cdrhold_dis=1, echo_mask=1: full OOB handshake +
  ALIGN/SYNC 8b10b data at 3.0Gbps + 5ms stability => ctrl ready. 10/10 enable cycles, ~0.1s
  each, holds indefinitely. The COMPLETE PHY chain (DCU config, OOB gen/det, EI, CDR, comma
  align, 8b10b, gearbox, CDC, ctrl FSM) is hardware-proven through the physical connector path.
- [MORNING PLAN with drive] (1) gap_mode=1 handshake against the drive (full amplitude + gaps
  visible to transition-style squelch - never tested against a drive post-attribution-collapse);
  (2) precharge carrier (>=35s driven) then immediate EI-gapped clean-shot handshake (bursts at
  ~full amplitude for the first seconds); (3) scope C113 during both to confirm burst amplitude
  at the drive-relevant states.

## Campaign 13 (2026-07-25, drive replugged): root cause CLOSED - EI mute is unfixable in silicon

Drive replugged after loopback campaign; beacon textbook (600 bursts/s, 100-110ns/310-330ns).
Handshake attempts (all no-link, RX = pure beacon, zero COMWAKE-class gaps):
  A) gap_mode=1 driven-gap handshake  B) 40s precharge + driven-gap  C/D) precharge + legacy-EI
  gaps (scoped)  E) force_wake leniency shot.

DEFINITIVE with-drive measurements (drive termination = no loopback island effects):
- LDR carrier: 236-256mV at t=1s, flat at 30s => the seconds-scale "charge" dynamics of the
  loopback night were the DC-floating island, NOT the driver. With a real load there is no
  slow charging.
- EI-gapped OOB bursts NEVER cross 25mV at C113 (scope NORM trigger): trail 4-15 (bursts
  133-207ns), gaps 320ns-1.7us, legacy or shaped, launched from a charged carrier or not.
  Meanwhile the same LDR content as continuous carrier = 236mV, driven-gap storm = 158mV+.
  => FFC_EI_EN mute LATCHES: un-mute >> 1.7us (likely needs sustained drive). No compliant
  OOB burst can follow a real EI gap on this silicon config. This retro-explains the entire
  3-drive deafness (every handshake burst was sub-30mV).
- PCIe path (G8B10B + p_CHX_PCIE_MODE=1, FFC_PCIE_CT as idle, oob_config pcie_ct): TX
  completely dead including the driven-gap control => PCIe EI unusable too (matches the old
  bypass-mode note). ALL silicon EI mechanisms are now closed.

THE DILEMMA (final): full amplitude XOR real gaps.
- Driven-constant gaps (gap_mode=1): full amplitude, but through our 100nF caps into the
  drive's 50R the differential decays with tau=5us => a 320ns "gap" retains ~94% swing =>
  invisible to an amplitude squelch. (Our transition-based rx_sel=1 sees them; this drive's
  squelch evidently does not.)
- Real EI gaps: bursts muted to <25mV.

HARDWARE fixes identified (need soldering, pick one):
1. **Swap C112/C113 from 100nF to ~1nF** (tau=50ns): driven-constant gaps then decay to ~2%
   within 320ns => drive-visible OOB with FULL amplitude and NO silicon EI involved.
   gap_mode=1 handshake + 1nF caps is the complete recipe. (4.7nF marginal, 10nF too slow.
   1.5/3.0Gbps data through 1nF: Xc~0.1R, baseline wander 50ns >> 3.3ns max run - fine.)
2. Fabric-IO differential pair resistively bridged onto the TX pair for OOB (LUNA-inverse;
   more invasive).
Note: drive long-carrier events (655us continuous RX activity, only during our TX phases,
never on silent line) remain unexplained - possibly TX->RX crosstalk at RLOS threshold,
possibly a real drive reaction; content capture needed if pursued.

Board state: B-gen2-g8b10b-fix reloaded, ctrl parked, line silent, drive beaconing happily.

## Campaign 14 (2026-07-25, "think harder" pass): cap values verified, serializer-EI bug, long events resolved

- [SATA cap standard - user question CONFIRMED] Visually verified in SCH_ECPIX-5_R02.PDF p5:
  C112/C113 (TX) and C122/C127 (RX) are 100nF - the PCIe convention, NOT the SATA standard
  (10nF nominal, 12nF max per SATA-IO). All ECPIX-5 serdes lanes use 100nF (GTP4 even 0R).
  Real hosts tolerate 100nF because their PHYs ACTIVELY drive differential-zero at EI entry;
  gap visibility never relies on cap decay. IMPORTANT: even the standard 10nF would NOT fix
  our driven-gap scheme: tau_diff = 5nF x 100R = 500ns -> a 320ns gap retains ~53% swing
  (~265mV) - still above squelch. Passive decay needs ~1nF (tau=50ns: COMRESET gap ->0.2%,
  COMWAKE 106ns gap ->12%). The 1nF recommendation stands and is REQUIRED, not optional.
- [burst_mode config bug found] In legacy EI mode, burst_mode leaves tx_oob_en=0 so
  ei_legacy=(tx_idle|tx_oob_idle)&~tx_oob_en is asserted THROUGH THE BURSTS - all previous
  "serializer bursts + legacy EI" tests had EI permanently on (invalid). Serializer bursts
  require ei_mode=1 (shaped). Retested properly vs drive (g8b10b, shaped, D10.2 content):
  still no response - but amplitude unverifiable (scope SCPI crashed; needs power cycle).
- [long-carrier events RESOLVED - drive analog reaction, not data] Full anatomy: rx_idle
  (RLOS) low for >=655us, but RAW pre-decoder rx_word bus = constant 0x0000/k=0/no error
  markers for the whole event (full-rate litescope capture) => no bit transitions; and no
  rotation of ALIGN or D10.2 streams can decode as D0.0 (exhaustive 8b10b rotation analysis)
  => the events are NOT data. They are the drive's analog squelch-exit/termination reaction
  (DC step settling through the 100nF caps, tau-consistent) to detecting our carrier:
  0 events on silent line, many with carrier on. VALUE: proves our full-amplitude carrier IS
  detected by the drive's receiver - the deafness is specifically our muted OOB bursts.
- [ALIGN-answer experiments] New align_force CSR (bit 29): continuous ALIGN primitive
  transmission (100% duty). Drive elicited analog events in ~50% of 5s windows but beacon
  never stopped over minutes: the drive requires real OOB before speed-negotiation; it does
  not leniently lock on sustained ALIGNs. force_wake + echo_mask ALIGN-answering: no link.
- [gen2 RX word clock collapse RECONFIRMED] With CDR released on the mostly-idle line, the
  rx word clock dies and rx-domain analyzer signals FREEZE (the "100% valid 570us" capture
  was this artifact). CDR-held captures decode with periodic ppm slips (93% valid) - use
  CDR-held for content work pre-linkup.

CONCLUSION UNCHANGED AND SHARPENED: hardware fix required. Swap C112/C113 100nF -> 1nF
(NOT 10nF), then gap_mode=1 driven-gap handshake. All-silicon paths exhausted.

## Campaign 15 (2026-07-25, Codex-review follow-ups): true PCIe-EI path implemented, verification blocked

External review (Codex) invalidated two claims: (1) the "EI mute latches >1.7us" inference was
wrong - ei_trail gates EI release to the FINAL trail cycles of each gap (4-bit field, max
100ns), so the 1.7us gap sweep never tested more than ~100-200ns of release lead; the data
only shows unmute > ~200ns. (2) FFC_PCIE_CT is the receiver-DETECT strobe, not PCIe EI -
abusing it proves nothing about the PCIe idle path.

The REAL PCIe electrical idle (TN1261 pp.~50/163): per-byte EI enables ride the TX data bus
(FF_TX_D bits 11/23 = pci_ei_en[0]/[1], verified in TN1261 bus table), pipelined with data,
EI reached <20UI after assertion, min idle 50UI; documented word-synchronous. Additionally
prjtrellis fuzzer exposes a CHx_PCIE_EI_EN feature-enable defparam (present in the Diamond
reference netlist) that gates the whole mechanism - never set by any of our configs.

IMPLEMENTED (pcs_mode="pcie"): PROTOCOL="PCIE" + PCIE_MODE=1 + PCIE_EI_EN=1, G8B10B-style
bus map + tx_bus[11]/[23] driven word-synchronously from the existing ei_en expression;
FFC_EI_EN left unwired (--oob-config ldr_tx,ldr_rx). Bitstream C-gen2-pcie. Serdes inits,
beacon decodes (576-600/s). Also widened ei_trail 4->8 bits (ei_shape layout NOW: lead[0:5]
trail[5:13] wake_gap[13:21]) so PRE can pre-release EI up to 1.7us for a proper unmute-
latency measurement.

RESULTS SO FAR: no drive response to PCIe-EI handshakes (serializer D10.2 bursts or LDR
bursts). BUT emission is unverified: scope SCPI crashed (needs power cycle) and the drive
occupies the connector (no loopback). The drive-reaction energy probe was insensitive (even
the driven-gap positive control elicited 0 windows this session - the reaction cycle is not
a reliable meter).

NEXT (needs hands, either one):
- Scope power cycle -> triggered burst-amplitude test of the PCIe-EI storm at C113 (LDR
  bursts are in-band; also unmute-latency sweep with the widened trail via PRE).
- OR swap drive->loopback cable -> rx_sel=0 RLOS decode of our own PCIe-EI storm (the RLOS
  decoded the drive beacon through the same path, so full-amplitude bursts + real EI gaps
  would decode beacon-like: ~107ns bursts / 320ns gaps at wake_gap=48).
Also queued from review: empirical tau measurement (driven carrier -> 20-50us constant gap,
fit decay at connector side); cap choice 470-680pF (NOT 1nF, NOT 10nF) if soldering; RF
series switch as production-grade alternative; TXPWDNB/PCIE_DET_EN characterization.

### Campaign 15 addendum: fuse-level verification of the PCIe-EI config
- nextpnr has NO CHX_PROTOCOL support (silently ignored, same class as D_TX_MAX_RATE).
- BUT the trellis DB has no DCU.CHx_PROTOCOL entry either, despite the fuzzer fuzzing it =>
  the enum touches NO fuses: Diamond's PROTOCOL attribute is a wizard-level macro; the real
  configuration is entirely the individual words (UC_MODE, ENC/DEC_BYPASS, PCIE_MODE,
  PCIE_EI_EN, CTC...). p_CHX_PROTOCOL can be dropped from the wrapper.
- Emitted .config verified: DCU.CH0_PCIE_MODE=1, DCU.CH0_PCIE_EI_EN=1, and a full diff of
  DB config words vs our emitted words shows NOTHING missing. The PCIe-EI feature is as
  enabled as the open toolchain can express.
- Drive still silent to PCIe-EI OOB (LDR and serializer bursts). Emission verification
  blocked on: scope power-cycle (SCPI crashed) OR drive->loopback swap (RLOS self-decode).

### Campaign 15 addendum 2: scope recovered via VXI-11; PCIe-EI vs LDR exclusivity
- Scope SCPI (raw 5025) dead but VXI-11 (port 111) alive: device_clear() + full control
  restored WITHOUT power cycle. bench/sds1104.py now has SDS1104XEVXI transport.
- PCIE_MODE=1: TX completely dead (LDR carrier 4mV) - BUT the PCIe receiver-detect hard
  sequence WORKS in this mode: FFC_PCIE_DET_EN + CT pulse -> FFS_PCIE_DONE=1, PCIE_CON=1
  (drive termination sensed THROUGH the 100nF caps; never worked in bypass). New capability.
- PCIE_EI_EN=1 alone (PCIE_MODE=0): LDR carrier STILL dead. Also dead with FFC_EI_EN wired
  (unconnected-port confound eliminated). Attribution: the PCIe-EI flag feature disables the
  LDR output path (mutually exclusive muxing). G8B10B ref build (EI_EN=0): 152-256mV alive.
- Consequence: PCIe-EI (bits 11/23) can only be verified with SERIALIZER burst content,
  which is invisible to the 100MHz scope. FINAL VERIFICATION = loopback swap: RLOS decode of
  our own serializer-burst + EI-bit-gap storm (expect beacon-like ~107ns bursts / 320ns gaps
  at wake_gap=48 if the feature works).

## Campaign 16 (2026-07-25/26 night): EI characterized, first spec-shaped OOB waveform, 3 retractions

**Codex's correction was right: EI does NOT latch.** Measured un-mute latency by sweeping the
EI release lead (widened ei_trail, PRE pre-release): bursts are absent at 213ns lead, present
at 427ns (162mV) and full at 853ns+ (222-238mV). So EI un-mute is 213-427ns =>
**minimum EI-carved gap ~400ns: COMINIT/COMRESET window (175-525ns) is REACHABLE, COMWAKE
(55-175ns) is not.** ei_carve mode (LDR drives the whole sequence, EI carves gaps) added and
tested: every burst eaten, as predicted by the 400ns un-mute.

**FIRST SPEC-SHAPED OOB WAVEFORM OF THE CAMPAIGN.** Root cause of the previous "amplitude
starvation": with `--oob-config ei,...` the FFC_EI_EN port is wired and a momentary EI assert
between sequences mutes the LDR for ~400ns, eating the bursts. Build WITHOUT the ei port
(`--oob-config ldr_tx,ldr_rx`, bitstream D-gen2-noei) gives clean LDR OOB, wire-verified at
C113: bursts 176-408mV with gaps of **81-108ns (COMWAKE window!)** and, at wake_gap=48,
**270-297ns (COMINIT window)**. Burst-train amplitude sag is fixed by longer bursts
(new `_oob_burst_len` CSR: 427ns bursts give 174/142/138/134mV, uniform).

**New runtime TX controls** (`_oob_txctl`): FFC_TXPWDNB (pwdn), FFC_LANE_TX_RST (lane_rst),
lane_rst_auto (assert during OOB phase, release for data), pwdn_gap (gaps by power-down).
- pwdn=1: main driver down, LDR alive (224mV) -> LDR OOB with EI-free gaps.
- lane_rst=1: LDR carrier peaks at 402mV (best of campaign) but LDR *bursts* vanish.
- pwdn as a gap gate: produces a large ~406mV LOW-FREQUENCY transient, not clean gating.
- tx clock survives both (154MHz).

**THREE RETRACTIONS (instrument artifacts, all caught by controls):**
1. "chi2=3600 drive heard our COMRESET": with the LDR disabled and NOTHING emitted, the RX
   still reported the identical 0.52us fixed latency and identical chi2 => internal TX->RX
   self-coupling. **Present only in PCIE_MODE builds**; the D-gen2-noei build's RX is clean
   (600/s in every TX state). New `_oob_lat_holdoff` CSR rejects such artifacts by time.
2. Campaign 14's "drive reacts to our carrier with 655us events": these occur at the SAME rate
   on a SILENT line (4/8 windows silent vs 1/8 with carrier). No evidence the drive's receiver
   ever sees us. RETRACTED.
3. "PCIE_MODE/PCIE_EI_EN kills the serializer (8mV)": a healthy serializer carrier also reads
   only 20mV at C113 - the 100MHz scope simply cannot see 3Gbps content. Serializer amplitude
   claims from any campaign are void. (PCIE_EI_EN=1 killing the *LDR* is real and confirmed by
   SCI: channel reg 0x02 bit 6 = static force-idle; clearing it via SCI revives the carrier.)

**Drive still unresponsive** to: spec-shaped LDR COMWAKE (full amplitude, 81-108ns gaps),
serializer bursts with pwdn gaps (all lead/trail), paced and continuous storms. Beacon
unchanged at 600 bursts/s throughout, gaps always [310-330ns].

**Live hypothesis**: LDR content is a ~75MHz square - far below a SATA receiver's detection
band (CTLE/AGC optimized for 1.5-6Gbps, internally AC-coupled). If so, LDR-based OOB can never
be heard regardless of amplitude/timing, and the only viable transmitter is serializer content
(line-rate) with a gating mechanism. EI is the only clean gate we have, and at ~400ns it can
only make COMRESET/COMINIT-timed gaps - which IS spec-legal and worth a beacon-phase test.
