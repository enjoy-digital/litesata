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

### Campaign 16 addendum: cleanest deafness statement + SCI TX-driver register map

- **Beacon interval instrument, 250 intervals per condition: 10.0029ms, sd = 0.0000ms, zero
  outliers** - identical for silent line, spec-shaped LDR COMWAKE storm, LDR COMRESET storm,
  and long-burst (427ns) COMWAKE. The drive's beacon is a perfect free-running oscillator; our
  OOB does not perturb it by even one 10ns tick. This is the strongest deafness evidence yet.
- Full ctrl handshake with the clean LDR waveform (5 variants: 107/213ns bursts, 107/160ns
  gaps, main driver powered down, both RX detectors): no link, beacon unchanged.
- Serializer bursts + pwdn gaps, ctrl handshake, lead/trail swept 0-53ns: no link.
- **TN-02206 SCI TX register map recovered** (via external consult; addresses are direct, sel=0
  selects the channel space, our SCI CSRs already reach them):
  * CH_11: [4:0] rterm_tx (10011 = 50 ohm), [6:5] tx_cm_sel (doc: 00 = power down, 01 = 0.6V,
    10 = 0.55V, 11 = 0.5V). Read back 0x13 => rterm 50 ohm and tx_cm_sel ALREADY 00 while the
    TX works => the doc's "00 = power down" encoding does NOT match observed silicon.
  * CH_12: tdrv_slice0..3_sel (00 = power down per slice); CH_13/CH_14: slice currents.
  * CH_15: [3:0] lb_ctl (serial loopback, known nonfunctional), **[5:4] tdrv_dat_sel: 00 =
    serializer data to driver, 01 = DATA RATE CLOCK to driver**, 10/11 = loopback paths.
- `tdrv_dat_sel=01` is a promising untested burst source: a line-rate (1.5GHz at gen2) clock
  straight out of the main driver, i.e. exactly the GHz content a SATA squelch expects, with no
  datapath involvement. Combined with SCI-modulated TDRV slice power-down as the gap gate it
  could be a complete OOB transmitter - IF an SCI write completes fast enough (our SCI is a
  parallel fabric-side interface, a few sys cycles, NOT a slow serial bus: needs measuring).
- CAUTION: tonight's SCI-on-live-TX measurements were unreliable (LDR carrier amplitude wandered
  104-376mV across identical conditions - hand-placed single-ended probe drift, a hazard this
  journal has recorded before). Re-do them with a fixed probe before trusting any of it.

**Morning shortlist**: (1) measure SCI write latency in sys cycles; (2) tdrv_dat_sel=01 as burst
content + TDRV slice power-down as gap gate; (3) two-probe differential check of the LDR output
(is the LDR differential at all, or common-mode? - would explain a perfect single-ended
waveform that no device can hear); (4) 470-680pF cap swap remains the fallback.

### Campaign 16 addendum 2: SCI slice gate implemented; compliant-timing negative; aliasing trap

External review (3rd consult) delivered a decisive correction: **the OOB windows used all campaign
were the "may detect" windows, not the "shall detect" ones.**
  * COMWAKE: may 55-175ns, SHALL 101.3-112ns, compliant TX gap 103.5-109.9ns.
  * COMRESET/COMINIT: may 175-525ns, SHALL 304-336ns, compliant TX gap 310.4-329.6ns.
  * Burst duration must also be ~103.5-110ns (the 427ns anti-sag burst was non-compliant).
  * OOB content is specified as repeated D24.3 (or ALIGN) AT THE GEN1 RATE for every generation:
    D24.3 at 1.5Gb/s = a **375MHz** square. A drive is NOT required to detect our ~75MHz LDR
    square at any amplitude - the LDR's response at 75MHz is simply undefined by spec.
  * Also flagged: with EI unwired, dropping FFC_LDR_CORE2TX_EN does NOT idle the main serializer;
    unless pwdn=1 is held, the drive sees continuous 3Gb/s energy through our "gaps".

**Compliant COMRESET achieved and tested** (D-gen2-noei, pwdn=1 held, wake_gap=48, 16-cycle
bursts): wire-measured bursts 117ns, gaps 312ns, 4/5 gaps inside the 304-336ns SHALL-DETECT
window. Scored per the SATA interop procedure: (a) beacon suppression while COMRESET is
sustained - NONE (600 bursts/s throughout); (b) COMINIT phase reset after release, 39 trials -
UNIFORM (chi2=18.7). **Clean negative with genuinely compliant timing.**

**SCI TDRV-slice gate implemented** (Codex's top-ranked mechanism): a new state in the SCI
reconfig FSM writes CH_12 (tdrv_slice*_sel) on every OOB burst/gap transition - a 2-cycle
(~20ns) write at the driver's output-current stage, downstream of FIFO/gearbox/serializer, so
it bypasses the EI pipeline entirely. CSRs `_oob_txctl.sci_gate` + `_oob_sci_vals`
(burst/gap values). CH_12 reads 0x51 in the non-boost build (slices 0/2/3 on main data),
exactly as predicted. Gate + serializer D10.2 bursts vs the drive: no beacon suppression, no
link. NOTE: scope verification of the gate is still missing (probe drift left even the
reference carrier below trigger); the mechanism is implemented and sim-clean but electrically
UNVERIFIED.

**NEW MEASUREMENT TRAP (cost me a false positive tonight):** host-paced phase tests alias
against the 10.003ms beacon. A 30ms software loop produced chi2=39.6 with a striking
alternating histogram - but the control with the TX completely dead (gate burst value 0x00)
gave chi2=40.0, and merely changing the loop period to 37ms flipped which case looked
"clustered". **Only hardware-paced tests (free-running storm + seq_quiet) are valid for phase
analysis.** This is the same class of error as the beacon attribution collapse and the PCIE_MODE
self-coupling artifact - the third one this campaign.

**State of the diagnosis**: with compliant COMRESET timing, full amplitude, serializer silenced
during gaps, and three independent gating mechanisms tried, the drive shows zero reaction of any
kind. The two surviving explanations are (1) the ~75MHz LDR carrier is outside what any SATA
squelch is required to detect - the spec wants a 375MHz Gen1-rate carrier; (2) the LDR output is
not differential at the drive (a single-leg probe cannot tell). Both are addressed by the same
next step: **Gen1-rate D24.3 serializer content + the SCI slice gate**, plus a two-probe
differential check when hands are available.

### Campaign 16 addendum 3: Gen1-rate D24.3 carrier synthesized; spec-correct OOB still unheard

**Gen1-rate OOB carrier implemented** (`_oob_txctl.pat_alt`). SATA requires the OOB burst content
to be repeated D24.3 AT THE GEN1 RATE for every generation = a 375MHz square. A period-8 pattern
does not tile into the 20-bit raw word, so the pattern now alternates with its bitwise inverse
every tx word: 0xF0F0F / 0x0F0F0 concatenate into a continuous period-8 stream (proved
arithmetically; run lengths all exactly 4 bits). At 3.0Gb/s that is exactly 375MHz - the first
spec-correct OOB carrier of the campaign. Scope corroborates indirectly through its own roll-off:
150MHz=102mV, 375MHz(alt)=38mV, 750MHz(D10.2)=34mV, monotone in frequency as expected.

**Definitive test - spec-correct content AND compliant timing AND three gate mechanisms:**
  * 375MHz D24.3 + SCI slice gate, COMRESET 312ns gaps: beacon 600/s (no suppression).
  * 375MHz D24.3 + TX power-down gate: 600/s.
  * 750MHz D10.2 + SCI gate (content control): 600/s.
  * Hardware-paced phase test (seq_quiet, immune to the host-aliasing trap): **chi2=1.9, n=300,
    perfectly uniform** - and the control is equally uniform.
  => The drive does not react to spec-correct, compliant-timing, full-amplitude OOB.

**SCI slice gate is digitally CONFIRMED FIRING**: SCI interface busy 15.0% of samples during an
OOB storm with the gate enabled, and 0.0% in all three controls (park/gate-off, storm/gate-off,
park/gate-on). The RTL sequencer works; only the analog response of the TDRV slices is unproven.

**BLOCKER for the remaining verification: the probe.** C113 contact drifted from 102mV to 6mV
within minutes during this session (the campaign-long 30-300mV instability, now worse). No TX
gating mechanism can be electrically verified for serializer content until the probe is re-seated,
so "are our gaps real gaps at the drive?" remains formally open for every serializer-based mode.

**Where the diagnosis now stands.** Eliminated tonight: EI latching (false), OOB timing windows
(now compliant), OOB content (now spec-correct 375MHz), burst amplitude (176-408mV), gap presence
for LDR content (wire-verified), and three instrument artifacts. What remains:
  1. **Is the TX differential at the drive?** A single-leg probe cannot distinguish a correct
     differential waveform from a common-mode one that no receiver can hear. NEEDS TWO PROBES.
  2. **Are serializer-content gaps real?** SCI gate fires digitally but is analog-unverified;
     EI's minimum gap (~460ns) is ABOVE the 336ns shall-detect ceiling, so EI cannot make a
     compliant COMRESET gap either - only the SCI slice gate can, in principle.
  3. Cap swap to 470-680pF remains the fallback.

**Next session, in order**: (a) re-seat the probe, verify the SCI slice gate on a 150MHz carrier
(bursts 107ns / gaps 312ns at full amplitude = mechanism proven); (b) two-probe differential check
of both TX legs during an OOB burst; (c) if the gate proves out and the drive still ignores it,
the fault is differential/physical and the cap swap or an RF switch is the answer.

## Research: was SATA ever officially supported on Lattice SERDES? (ECP2M / ECP3 / ECP5)

Primary-source evidence, all found in the LOCAL Diamond 3.12 install and the Lattice usage guides
(no license needed - the template/model files are readable):

**ECP2M - SATA WAS offered in the tools.**
  * `/opt/diamond/3.12/module/pcs/latticeecp2m/gui/core_template.tcl` contains protocol entries
    `"SATA I"`, and in the (now commented-out) full protocol list `"SATA Type1"`, `"SATA Type2"`
    with mode codes `SATAT1`, `SATAT2`.
  * A dedicated `proc SATAISetting {}` exists, and the "SATA I" case sets the clock-tolerance
    compensation matcher to CC_MATCH1..4 = 0110111100 / 0001001010 / 0001001010 / 0001111011 -
    i.e. **K28.5, D10.2, D10.2, D27.3 = the SATA ALIGN primitive**.
  * IPexpress PCS changelog (docs/webhelp .../ipexpress/pcs_tab.htm): "**5.0: Added CPRI and
    SATA.**"
  * BUT the active list in the shipped 3.12 wizard is
    `{PCI-Express "Gigabit Ethernet" "Generic 8B10B" "10-bit SERDES Only" "8-bit SERDES Only"
    SD-SDI HD-SDI CPRI}` with the SATA-bearing list commented out next to
    `#ISPL_CR_32029 - only support pcie & pipe`. Lattice progressively WITHDREW the option.

**ECP3 - SATA is still a protocol in the PCS templates.**
  * `/opt/diamond/3.12/ispfpga/maco/data/pcs/PCSD.vhd` (`library ECP3;`) has **68** `"SATA"`
    conditionals. What SATA mode actually does:
      - `#if (_chX_protocol_new == "SATA" || _chX_protocol_new == "PCIE") && mode != "DISABLED"`
        -> `FFC_EI_EN_X => tx_idle_chX_c` : **it exposes the TX electrical-idle port** - exactly
        the same mechanism we drive on ECP5. SATA is grouped with PCIe for this and only this.
      - requires `_datarange == "HIGH"` (1.5/3.0Gbps range), plus the usual comma/CTC settings.

**ECP5 - SATA is GONE.**
  * `/opt/diamond/3.12/ispfpga/sa5p00/data/DCUA.v`: **zero** "SATA" occurrences. Its protocol
    enum is `10BSER, 8BSER, CPRI, EDP, G8B10B, JESD204, PCIE, SDI, SGMII, XAUI`. prjtrellis'
    fuzzer enum agrees exactly (and adds GBE/XAUI naming) - no SATA.
  * The string "SATA" appears **zero** times in the ECP2M, ECP3 (TN1176) and ECP5 (TN1261)
    SERDES/PCS usage guides. Their "OOB" feature is explicitly "Out-of-band (OOB) signal
    interface for low-speed inputs (**video application**)" - a <250Mbps LDR bypass path for
    SD-SDI / 100Mbps Ethernet. It is NOT SATA OOB signalling; the naming collision misled this
    campaign for a long time.

**NO Lattice family ever had a hardware SATA OOB engine.** `cominit|comwake|comreset|comsas`
appears **0 times** in the ECP3 PCS templates (and nowhere in ECP5). Even in the era when the
wizard said "SATA", the silicon contribution was only: ALIGN-matching CTC + a TX electrical-idle
port. The COM burst/gap sequencing was always expected to be soft logic in the FPGA fabric -
which is exactly what our COMGenerator does.

**Why it nevertheless works on Xilinx and not here.** Xilinx 7-series GTP/GTX implement OOB in
the transceiver: TXCOMINIT / TXCOMWAKE / TXCOMSAS / TXCOMFINISH generate spec-timed burst/gap
sequences in hardware, and RXCOMINITDET / RXCOMWAKEDET / RXELECIDLE detect them. LiteSATA's
Xilinx PHYs simply strobe those ports. On ECP5 the one primitive the fabric approach still needs
from the PCS - a TX electrical idle fast enough to carve a 106.7ns COMWAKE gap - is the thing
that does not exist: measured un-mute 213-427ns (2026, ECPIX-5), and a ~220ns minimum EI pulse
measured independently in **March 2022 on a Versa-ECP5** (commit 13dbd2a, "wip hacky code to
mesure minimal tx_elec_idle generated pulse (220ns...)"). Same wall, twice, four years apart,
two different boards.

**Conclusion.** SATA on ECP5 is unsupported by Lattice, was quietly dropped after ECP2M/ECP3
where it was only ever a PCS preset (never an OOB engine), and the ECP5 DCU lacks any TX idle
mechanism fast enough for SATA OOB gap timing. A working ECP5 SATA host therefore needs the gap
made OUTSIDE the DCU: small AC-coupling caps (470-680pF) so a driven-constant gap decays below
squelch, or an RF switch on the TX pair. That is a board-level fix, not a gateware one.

### Research addendum: ECP2M was the most SATA-aware generation; CertusPro-NX is the modern answer

- **ECP2M went furthest.** `/opt/diamond/3.12/ispfpga/maco/data/pcs/PCSC.v` has protocol values
  `SATA_I` and `SATA_II`, wires `FFC_EI_EN` for SATA_I alongside PCIE (line 164), and even emits
  a port literally named **`ffs_sata_oob_rx_chX`** (line 172) - emitted adjacent to
  `ffs_rlos_lo_chX`, i.e. the "SATA OOB receive detect" was simply the loss-of-signal detector
  brought out under a SATA name. So even the most SATA-aware Lattice generation offered
  {fabric-generated bursts + generic TX electrical idle + RX activity detector} - precisely the
  architecture we built on ECP5, minus a supported/characterized EI timing contract.
- **The lineage**: ECP2M (SATA_I/SATA_II + ffs_sata_oob_rx) -> ECP3 (SATA protocol retained in
  PCSD.vhd, EI port only) -> ECP5 (**removed entirely**). Diamond 3.12 even shows the withdrawal
  in progress: the ECP2M wizard's SATA entries are commented out behind
  `#ISPL_CR_32029 - only support pcie & pipe`.
- **CertusPro-NX is the family to use if this must work on Lattice.** Its PCS guide documents
  that deasserting `mpcs_txval_i` / `epcs_txval_i` produces Electrical Idle with **entry AND exit
  at 22 tx_pcs_clk cycles** (a real, specified timing contract - the thing ECP5 lacks), and
  `mpcs_rxoob_i` configures the activity detector specifically to detect OOB. The guide mentions
  SATA explicitly. Still no hard COMINIT/COMWAKE classifier, but a fabric OOB engine like ours
  would have a specified fast gate to drive.
- **No public end-to-end SATA link on ECP3 or ECP5 exists** (GitHub, Lattice community, EEVblog,
  Hackaday all searched): only unfinished branches and experiments - including this project's own
  2020-2022 history. LiteSATA's README still lists Lattice PHY support as a possible improvement.
- ECP5's only fast, *specified* idle path remains the PCIe per-word one (TN1261 Fig.16:
  idle entry 16UI, exit < 20UI). Everything we can reach outside PCIe protocol mode - async
  FFC_EI_EN, SCI tx_cm_sel, SCI TDRV slice select - has **no published register-to-pad latency
  at all**, so our measured 213-427ns is unspecified behaviour rather than a violated spec.

## Campaign 17: de-emphasis "data-driven idle" trick - tested and DISPROVEN (with control)

**The idea**: the one block that could produce a differential zero with zero latency is the TX
de-emphasis FIR. If the post-cursor tap is set equal to the main tap, a CONSTANT bit pattern
should cancel to ~0V differential while a TOGGLING pattern adds to full swing. That would give a
data-driven electrical idle switching in one word (6.7ns), with no EI pipeline and - crucially -
no dependence on the 100nF caps, because the driver would be actively holding differential zero.
Implemented as `_oob_txctl.deemph_gap` (bit 6) + `_oob_gap_pattern` CSR: OOB gaps transmit a
constant pattern, bursts transmit a toggling one. The TDRV taps are RUNTIME-writable via SCI
(CH_12 slice select, CH_13/CH_14 currents) - no rebuild needed to configure the canceller.

**Result: the post-cursor tap ADDS rather than subtracts.** Measured single-ended DC swing
between all-zeros and all-ones patterns (DC-coupled - see technique note below):
  * stock TDRV (slices 0/2/3 = main)                  : 56mV
  * "canceller" (slice0=main 800uA, slice2=post 800uA): 16mV
  * CONTROL, main-only at the same 800uA              :  8mV
The canceller is LARGER than main-only at identical current, so adding the post slice increased
the output. The 56->16mV reduction is purely less total drive current, exactly as the control
predicts. **No DC cancellation occurs.** Idea dead.

**METHODOLOGY NOTE - the instrument this campaign has been missing.** A constant differential is
DC, so an AC-coupled probe cannot see it: every "gap" amplitude measured with AC coupling this
campaign was blind to precisely the quantity that matters. The correct technique is
**DC-coupled single-leg probing**: when the driver holds a constant pattern the leg sits at a
rail; during a TRUE electrical idle both legs are pulled to common mode, so the leg's DC level
moves by half the swing. That is a low-frequency, easily measurable signature of electrical idle
on a 100MHz scope, and it works regardless of the 3Gbps content being invisible. **Any future gap
mechanism (PCIe word-sync EI, SCI slice gate, tx_cm_sel) should be validated this way**: slow the
burst/gap modulation to ~1.7us (burst_len/wake_gap = 255), DC-couple, and look for a square DC
wave. First attempt was botched by scope offset/scale handling (trace off-screen, then a failed
PAVA query) - fix the scope automation, it is a 10-minute test.

**Still untried after this**: (a) validate the PCIe word-sync EI with the DC technique - it is the
ONE path with a documented fast contract (<20UI entry AND exit) and it has never been verified
electrically; (b) SCI `tx_cm_sel` (CH_11[6:5]) as a gap gate, re-tested with correct decoding -
note CH_11 read 0x13 => tx_cm_sel=00 while the TX works, contradicting the documented
"00 = power down", so the encoding needs establishing before trusting it; (c) fabric-IO assisted
shorting of the pair during gaps (small board mod, cheaper than an RF switch).

## Campaign 18: PCIe word-sync EI PROVEN FUNCTIONAL - and quantitatively too slow

Using the scope that was available all along (no hands needed - my mistake to defer this), with
the `ei_carve` mode so the LDR toggles continuously through bursts AND gaps and the ONLY variable
is whether EI is requested:

  * CONTROL (ei_carve, trail>=wake_gap so EI is never asserted): continuous carrier,
    contrast 1.3x, envelope essentially flat.
  * TEST (ei_carve, EI asserted for the whole gap): carrier visibly chopped into ~1.6us blocks,
    contrast 2.1x (and up to 27x at other settings).
  **=> The PCIe word-synchronous EI flags (TX bus bits 11/23 in PCIE_MODE) DO work.** This closes
  the question left open since campaign 15: the mechanism is real, not misconfigured.

**But the gap width saturates.** Sweeping the requested gap with the burst held at 1.7us:
    requested 1700ns -> measured 1600ns
    requested  853ns -> measured 1100ns
    requested  427ns -> measured  700ns
    requested  213ns -> measured  625ns
    requested  160ns -> measured  650ns
    requested  107ns -> measured  675ns
  **Floor ~625-700ns, independent of request.** That is above even the COMINIT/COMRESET
  shall-detect ceiling (336ns) and ~6x the COMWAKE gap (101-112ns). So EI-carved OOB cannot be
  made spec-compliant on this path, no matter how the request is shaped.

Caveat on scope: this measures EI acting on the **LDR aux buffer** (the only carrier this 100MHz
probe can see at ~200mV). The documented <20UI figure applies to the **serializer** output, whose
envelope reads only 14mV here - below the probe's resolution - so the serializer path remains
unmeasured. However, the drive has already rejected serializer-burst + EI-flag OOB at every
timing (campaign 15/16), which is consistent with the same floor applying there.

**Net**: every ECP5 TX idle mechanism is now characterized rather than merely suspected -
async FFC_EI_EN (213-427ns un-mute), PCIe word-sync EI (functional, ~650ns gap floor), TX
power-down (large transients), SCI TDRV slice gate (fires digitally, analog effect unmeasurable),
de-emphasis cancellation (disproven by control). None reaches the ~110ns SATA needs.

## *** CAMPAIGN 19: BREAKTHROUGH - THE DRIVE ANSWERS. NO HARDWARE CHANGE NEEDED ***

**The trick: make the gaps out of DATA, not electrical idle.** The serializer switches between a
toggling pattern (burst) and a CONSTANT pattern (gap) in ONE WORD - 6.7ns, no EI pipeline, no
dependence on the coupling caps. The gap still carries differential voltage, but it has ZERO
transitions, and a real SATA squelch is transition/energy sensitive rather than a DC amplitude
detector - so it reads as idle. This is `_oob_txctl.deemph_gap` + `_oob_gap_pattern` (built
during the de-emphasis experiment, whose cancellation hypothesis failed - but the GAP MECHANISM
was the valuable part and had never been pointed at the drive).

**PROOF - the drive's beacon is suppressed, deterministically and gap-width selectively.**
Three independent passes, silent re-baseline between every point, bitstream H-gen2-deemph
(bypass, 375MHz Gen1-rate carrier via pat_alt, gap pattern 0x0000):

    gap(ns)   pass1  pass2  pass3
        53      600    600    600     (below COMWAKE window)
       107       78     26      6     SUPPRESSED  <- COMWAKE nominal 106.7ns
       160     1280   1280   1280     rate DOUBLES (drive adding its own bursts?)
       213      600    600    600
       320        0      0      0     FULLY SUPPRESSED <- COMRESET nominal 320ns
       427        0      0      0     FULLY SUPPRESSED <- in COMINIT window
       533      600    600    600     (above window)

**Controls rule out the obvious artifacts:**
  * continuous carrier, same continuous drive, NO OOB structure -> 600/s (NOT jamming our RX)
  * continuous carrier without pat_alt                          -> 600/s
  * OOB with real EI gaps (line actually goes idle)             -> 600/s (EI too slow, as measured)
  * silent before and after                                     -> 600/s
Only OOB-STRUCTURED data-gap transmission suppresses the beacon. Suppression tracks the SATA
detect windows. This is the spec-mandated behaviour of a device that has qualified a COMRESET
(it stops its autonomous COMINIT schedule while reset is asserted).

**THE FULL OOB HANDSHAKE NOW COMPLETES.** With ctrl live, the FSM traverses
RESET -> COMINIT -> AWAIT-COMINIT -> AWAIT-NO-COMINIT -> COMWAKE -> AWAIT-COMWAKE ->
AWAIT-NO-COMWAKE -> **AWAIT-ALIGN (86% occupancy)**, and the RX records **gap=[100-330ns]**:
100ns gaps are COMWAKE-class, i.e. **the drive is answering with its own COMWAKE**. After 19
campaigns this is the first two-way OOB exchange with a SATA device on ECP5.

**What remains: the ALIGN / speed-negotiation phase.** RX decodes all-zero dwords with no
notintable errors - consistent with the known gen2 RX word-clock collapse (documented in campaign
10) and/or SATA speed negotiation (the device steps its ALIGN bursts down through its supported
rates; our RX is fixed at 3Gbps and must lock during the Gen2 window). This is a RECEIVE-side
problem, entirely separate from OOB, and it is the last mile.

**Consequences:**
  * The cap swap (470-680pF), the RF switch and the fabric-IO mod are ALL UNNECESSARY.
  * The ~650ns EI floor no longer matters - we never use EI for gaps.
  * ECP5 CAN generate SATA-compliant OOB despite Lattice dropping SATA support: the burst/gap
    structure comes from the serializer datapath, which switches per word.
  * Best config so far: bypass PCS, `pat_alt` (375MHz Gen1-rate D24.3 carrier), `deemph_gap`
    with gap pattern 0x0000, burst_len=16 (107ns), COMRESET gaps 320ns / COMWAKE gaps 107ns.

NEXT: (1) fix the gen2 RX word clock so ALIGNs decode (rx_cdrhold handling, CDR lock on the
drive's ALIGN bursts, possibly force a rate/step); (2) speed negotiation - consider a gen1 RX or
sweeping the RX rate during the device's step-down; (3) then IDENTIFY + BIST (task #8).

### Campaign 19 addendum: the RX blocker, precisely located

With OOB now working, the remaining failure is fully characterized as a RECEIVE-side problem:
  * In AWAIT-ALIGN the datapath delivers valid dwords that are ALL ZERO, with zero notintable
    errors, and `ctrl_rx_idle` is stuck at 1 - so ctrl's `If(~trx.rx_idle, ...)` can never fire
    and the FSM cannot leave AWAIT-ALIGN regardless of what the drive sends.
  * The RX word-clock counter reads nonsense (full 32-bit wrap: "35724MHz", or 0.7/14.9MHz) =>
    the recovered clock is not running at 150MHz. This is the gen2 RX word-clock collapse first
    noted in campaign 10, now confirmed as THE blocker.
  * The CDR has nothing to lock to on an idle line, and the drive only transmits in short windows
    (beacon = 6x100ns every 10ms; ALIGN = ~54.6us per speed step during negotiation). The CDR must
    acquire inside one of those windows.
  * `rx_ready` does assert in the `align_force` configuration - worth pulling on.

Next session plan for the last mile:
  1. Instrument properly: the rx-domain cycle counter is unreliable when the recovered clock is
     unstable (counter wraps). Add a sys-domain edge-count/toggle-detect on FF_RX_PCLK instead.
  2. CDR acquisition: sweep CHx_RX_LOS_LVL (2 was clean), check FFC_SIGNAL_DETECT/rx_align gating,
     and try forcing an RX reset (ctrl.rx_reset / init restart) at the moment OOB completes, so the
     CDR re-acquires on a now-active line rather than a stale idle one.
  3. Speed negotiation: the device steps ALIGN bursts through its supported rates. Our RX is fixed
     at 3Gbps. Either lock fast enough to catch the Gen2 step, or fix gen1 (currently broken:
     tx_clk 87MHz instead of 75, rx_ready=0 - the CHx_DCO* tuning noted as unsolved).
  4. Note the data-gap OOB trick REQUIRES bypass PCS (raw patterns); in g8b10b `tx_produce_pattern`
     forces tx_bus=0, not our pattern. So 8b10b decode stays in fabric for now.

### Campaign 19 addendum 2: the drive is fully engaged - the blocker is our RX word clock

Corrected diagnosis (supersedes addendum 1's "speed negotiation / CDR can't lock" guess):

**The drive is now an active participant.** While ctrl sits in AWAIT-ALIGN transmitting D10.2, the
drive transmits **~10,400 bursts/s - 17x its 600/s idle beacon rate** - stable over six 1s windows,
with gap=[100-330ns] (COMWAKE-class 100ns gaps present) and burst=[100-655350ns], the max being
the recorder's saturation value i.e. **continuous transmission**. That is a device doing OOB
dialogue AND sending ALIGN-like continuous data, not a device ignoring us.

**Our RX rx-domain logic is frozen.** In AWAIT-ALIGN:
  * `rx_lol = 0` for 100% of samples - the DCU reports the **CDR IS LOCKED**.
  * `rx_los`/`phy_rx_idle` = 0 for ~2% of samples - RLOS does see the drive's activity.
  * `rx_comwake_stb` fires (62/2040) - our COMChecker classifies the drive's bursts as COMWAKE.
  * But the datapath delivers **all-zero dwords with notintable = 00**. All-zeros AND no error
    flags is the signature of frozen rx-domain registers: real garbage would raise notintable,
    and a stuck-zero 10-bit code is not valid 8b10b. Everything in the rx clock domain is holding
    its reset value.
  => **FF_RX_PCLK is not running even though the CDR is locked.** The rx-domain cycle counter
  agrees (wraps/nonsense). This is the campaign-10 gen2 collapse, and it is now the ONLY thing
  between us and link-up.

So the last mile is not OOB, not the drive, and not CDR acquisition - it is the RX PCS/gearbox
failing to produce a word clock. Leads to pull, in order:
  1. RX gearbox/PCS config in bypass mode: CHx_RX_GEAR_MODE / RX_GEAR_BYPASS / RX_SB_BYPASS and
     the D_RX/word-alignment settings vs the Diamond reference netlist (the same full-param-diff
     method that found D_SYNC_LOCAL_EN for the TX side - and note the TX had EXACTLY this failure
     mode: PLL locked, word clock dead, fixed by one parameter).
  2. `rx_ready` gating in our split SerdesInit: it toggles 0/1 between runs; the RX branch may be
     releasing the PCS reset before the CDR/gearbox is ready, or holding it after.
  3. Try pcs_mode=g8b10b for the RX side (DCU-internal decode + aligner) while keeping bypass-only
     raw patterns for TX OOB - currently impossible in one build because tx_produce_pattern forces
     tx_bus=0 in g8b10b; a small RTL change would let the raw-pattern OOB coexist with g8b10b RX.

### Campaign 19 addendum 3: hybrid PCS mode - RX now receives real data, alignment is the last bug

The bypass RX datapath (fabric 8b10b) had NEVER been proven to decode real data - the loopback
self link-up of campaign 12 used **g8b10b** (DCU-internal decode + aligner). But the data-driven
OOB gap trick needs RAW TX patterns, which g8b10b cannot produce (tx_produce_pattern forces
tx_bus=0 there). ENC_BYPASS and DEC_BYPASS are INDEPENDENT parameters, so a hybrid is possible:

**New `pcs_mode="hybrid"`**: PROTOCOL=G8B10B, UC_MODE=0, **ENC_BYPASS=1** (TX raw 10-bit words
from the fabric encoder, so data-gap OOB works) + **DEC_BYPASS=0** (RX decoded and word-aligned
by the DCU, the path proven in loopback). Fuses verified in the emitted .config. Bitstream
`I-gen2-hybrid`. Init clean: tx_ready=1, rx_ready=1, beacon still decoded at ~590/s.

**Result: the RX now receives REAL DATA where bypass gave pure zeros.** In AWAIT-ALIGN the DCU
decoder emits 92+ non-zero dwords instead of an all-zero stream. The remaining fault is bit
alignment: every symbol pairs a plausible byte with the DCU's 0xEE invalid marker -
`78EE78EE/k0101`, `E5EEE5EE/k0101`, `26EE26EE/k0101`, `0000EEEE/k0011` - i.e. one byte of each
pair decodes and the other does not. That is a half-symbol / gearbox phase offset, not a dead
receiver. Adding the g8b10b word-aligner re-arm (edge-pulsed FFC_ENABLE_CGALIGN on decode errors,
which hybrid initially lacked) did not converge it.

So the chain is now: OOB works -> drive answers and transmits continuously -> RX recovers the
clock and delivers symbols -> **only the word/bit alignment is wrong**. Leads:
  * RX_GEAR_MODE/RX_SB_BYPASS and the 1:2 gearbox phase: the 0xEE pairing suggests the aligner is
    locking on the wrong 10-bit boundary of the 20-bit word.
  * UDF_COMMA_A/B/MASK are configured for K28.5; verify they are honoured with ENC_BYPASS=1 (the
    TX side being raw may interact with the shared PCS comma configuration).
  * Try LSM_DISABLE=0 (let the link state machine drive alignment) and/or ENABLE_CG_ALIGN=1
    static instead of the edge-pulsed re-arm.
  * Compare the full RX parameter set against the Diamond reference netlist - the same method
    that found D_SYNC_LOCAL_EN for the TX.

### Campaign 19 addendum 4: alignment attempts, and what the decoded content actually says

Tried in hybrid mode:
  * Edge-pulsed FFC_ENABLE_CGALIGN re-arm (ported from g8b10b): real symbols, one lane of each
    word valid, the other 0xEE.
  * Static `ENABLE_CG_ALIGN=1` (as bypass uses): WORSE - mostly all-zero dwords again and
    rx_ready drops. Reverted; edge-pulsed is the better config and is what is committed.
  * ctrl timers checked and are NOT the problem: align_timer = 873us (the SATA host value) and
    retry_timer = 10ms, so we do hold long enough for several of the device's ~54.6us speed steps.

**What the decoded bytes tell us.** The valid lane decodes as **0x78 = D24.3**, which is the SATA
OOB BURST character - not ALIGN (K28.5/D10.2/D10.2/D27.3 = 7B4A4ABC). Across every capture we
have never once decoded an ALIGN primitive. Combined with the burst/gap statistics (gaps
100-330ns, i.e. COMWAKE- and COMINIT-class), the drive appears to be **cycling OOB retries rather
than progressing into speed negotiation**. So the outstanding question is not only "why is one
symbol lane misaligned" but "why does the device restart OOB instead of sending ALIGNs".

Two candidate explanations, both testable:
  1. The device completes OOB, transmits ALIGNs at a rate we cannot receive (Gen3 6Gbps, or Gen1
     1.5Gbps), gets no reply inside its window, and falls back to OOB. Our RX is hard-wired to
     Gen2. A real host tries each rate. Fixing gen1 (currently broken: tx_clk 87MHz instead of
     75, rx_ready=0, CHx_DCO* tuning unsolved) would let us cover the low step; covering Gen3 is
     out of reach for ECP5-5G at 6Gbps.
  2. Our post-COMWAKE behaviour is not what the device expects - e.g. the device requires the host
     to cease OOB and hold D10.2 with particular timing after ITS COMWAKE completes. Worth
     capturing the exact TX/RX sequence around AWAIT-NO-COMWAKE -> AWAIT-ALIGN with litescope at
     full rate and comparing against the SATA state diagram.

**Honest status: the PHY is not yet functional and IDENTIFY is not reachable.** What is solved
and reproducible: OOB generation the drive genuinely decodes (beacon suppression, gap-width
selective, controlled), a drive that answers with COMWAKE and then transmits ~17x its idle rate,
and an RX that recovers clock and delivers real 8b10b symbols. What is missing: symbol alignment
convergence, and the device progressing to ALIGN/speed negotiation.

## Campaign 20: the OOB dialogue is textbook, ALIGNs ARE received - the deadlock is post-COMWAKE

**Litescope timeline of the full handshake** (trigger = our COMWAKE, 64x subsampling), which
confirms the OOB exchange end to end:
    +105us  COMINIT          (we transmit COMRESET)
    +115us  AWAIT-NO-COMINIT  COMINIT-det   <- the drive's COMINIT received
    +120us  COMWAKE          (we transmit COMWAKE)
    +125us  AWAIT-NO-COMWAKE  COMWAKE-det   <- the drive's COMWAKE received
    +458us  AWAIT-ALIGN
Every OOB step works in both directions.

**WE RECEIVE AND CORRECTLY DECODE THE DRIVE'S ALIGN PRIMITIVES.** Triggering litescope on a
K-character yields `7B4A4ABC/k0001` - K28.5 + D10.2 + D10.2 + D27.3, the SATA ALIGN primitive -
**94 occurrences in a single capture**, alongside the DCU 0xEE invalid markers. So: the drive
reaches speed negotiation, transmits ALIGNs at Gen2, and our hybrid RX decodes them. Earlier
readings of only D24.3 were sampling windows where the device was still in OOB.

**The remaining fault is a post-COMWAKE deadlock.** AWAIT-NO-COMWAKE occupies 333us of the
timeline because ctrl waits for the device's COMWAKE detection to CLEAR, while the device keeps
repeating COMWAKE precisely because it has had no response. SATA requires the host to begin
transmitting D10.2 **within 533ns of the device's last COMWAKE burst**; we are three orders of
magnitude late, so the device times out and restarts OOB. Chicken-and-egg.

Changes made (committed):
  * ALIGN detection in AWAIT-ALIGN no longer gated on `~trx.rx_idle`: on ECP5 the RLOS-derived
    rx_idle stays asserted throughout the device's ALIGN bursts (verified: rx_idle=1 for 100% of
    samples while rx_lol=0, i.e. CDR locked), so the ALIGN comparison was never evaluated.
    Decoding a valid ALIGN primitive is itself proof of signal, so the gate was redundant.
  * `retry_timer` 10ms -> 50ms; `align_timeout_us` made a parameter (kept at the spec 873us -
    raising it to 5ms is counterproductive: ctrl then camps in AWAIT-ALIGN, stops retrying OOB,
    and the device gives up entirely).
  * `_oob_quiet` swept at runtime 360ns..2us to clear COMWAKE detection faster. No link yet -
    so the 533ns budget is not met by shortening the quiet threshold alone; AWAIT-NO-COMWAKE
    itself must be bounded (e.g. proceed on a timer after the first COMWAKE detection rather than
    waiting for it to stop).

**Next (highest value first):**
  1. Bound AWAIT-NO-COMWAKE: leave for AWAIT-ALIGN a fixed ~300-500ns after the FIRST COMWAKE
     detection instead of waiting for deassertion. This directly targets the 533ns budget and is
     a few lines in ctrl.py.
  2. Then the ALIGN qualification: we receive valid ALIGNs but must catch >=1 while in
     AWAIT-ALIGN. Consider latching "ALIGN seen" in the PHY (sticky, cleared on FSM reset) so a
     single correctly-decoded ALIGN cannot be missed between state transitions.
  3. Diagnostic build suggested by review: same raw-TX config with DEC_BYPASS=1 plus a 40-bit
     sliding ALIGN detector on the pre-decoder RX bus (all 10 serial offsets, both disparities,
     both polarities) to separate "PCS word-align problem" from "device back in OOB".

## *** CAMPAIGN 21: LINK NEGOTIATION REACHES SEND-ALIGN ***

Two fixes in ctrl.py, both derived from the campaign-20 measurements:

**1. Bounded AWAIT-NO-COMWAKE (`nocomwake_timeout_us`, default 400ns).** Waiting for the device's
COMWAKE detection to DEASSERT deadlocked: the device repeats COMWAKE because it has had no
response, and we wait because it is still transmitting. SATA requires the host to begin D10.2
within 533ns of the device's LAST COMWAKE burst, so we now leave on a timer instead. The state
went from **333us (12.7% occupancy) to 0.2%**.

**2. Sticky ALIGN/ALIGN_N latch.** The device's ALIGN bursts are short and were being missed
between FSM state transitions. ALIGN detection is now latched (cleared on entry to COMWAKE) and
the FSM acts on the latched flag. Combined with removing the redundant `~rx_idle` gate (campaign
20), AWAIT-ALIGN now qualifies.

**RESULT - the FSM now reaches SEND-ALIGN:**
    AWAIT-CRG        : 26.9%
    COMINIT..COMWAKE :  ~1%   (OOB, 2 retry cycles in the window)
    AWAIT-NO-COMWAKE :  0.2%  (was 12.7%)
    AWAIT-ALIGN      :  8.3%  <- the drive's ALIGNs qualify
    **SEND-ALIGN     : 63.4%  <- transmitting ALIGN, one step from READY**

So the chain is now: data-driven OOB -> device COMINIT -> our COMWAKE -> device COMWAKE ->
device ALIGNs received and qualified -> we transmit ALIGN. Everything except the final
handshake step works.

**What remains.** SEND-ALIGN exits to READY after four received dwords whose low byte is 0x7C
(SYNC, K28.3). We do not get them: a full-rate capture triggered on SEND-ALIGN returned NO valid
RX dwords at all, i.e. the receive path goes quiet exactly while we transmit continuous ALIGNs.
Raising the ALIGN window 873us -> 3ms did not help. Hypotheses, in order:
  1. **Our continuous ALIGN transmission is disturbing our own receiver** (the RX went silent only
     once we started transmitting continuously in SEND-ALIGN). Check TX->RX coupling/CDR pull with
     a full-rate rx-domain capture; try transmitting D10.2 instead of ALIGN in SEND-ALIGN, or
     briefly gapping our TX.
  2. The device expects SYNC only after IT sees our ALIGNs decoded at ITS rate - i.e. residual
     speed-negotiation mismatch; try a gen1 build once the gen1 PLL is fixed.
  3. The RX word aligner loses lock when the line content changes from the device's ALIGNs to a
     bidirectional exchange (re-arm the aligner on entry to SEND-ALIGN).

Config that gets here: bitstream `J-gen2-hybrid-los2` (pcs_mode=hybrid, rx_los_lvl=2),
txctl=pat_alt|deemph_gap, gap_pattern=0x0000, pattern=0xF0F0, wake_gap=16, burst_len=16,
_oob_quiet=50, control=ei_mode|ldr_timeout|burst_mode|zero_bus|cdrhold_dis.

### Campaign 21 addendum: SEND-ALIGN diagnosed - persistent word misalignment

Full-rate capture triggered on SEND-ALIGN (the state we now occupy 63% of the time):
  * The RX **is** alive there: 1438 of 1916 samples carry valid dwords (the earlier "no valid
    dwords" reading was a mistriggered capture).
  * The dominant dword is **`F0B5A487/k0000` (1434x)** - note the `B5` bytes: this is SYNC-like
    content (SYNC = K28.3 D21.5 D21.5 D21.5 = B5B5B57C) but with **no K flag set**, i.e. the comma
    is not being recognised and the stream is decoded at the wrong 10-bit boundary.
  * A correctly aligned `7B4A4ABC/k0001` (ALIGN) slips through only twice in the window.
  * rx_lol=0 throughout (CDR locked); misalign fires 36/2040.
  => The device IS transmitting continuous primitives at Gen2 and our receiver IS recovering
  them; the sole remaining fault is that the DCU word aligner sits on the wrong boundary.

Fixes attempted for this (committed, none sufficient yet):
  * SEND-ALIGN now accepts SYNC (0x7C) **or** ALIGN (0xBC) as the qualifying dword - some devices
    linger on ALIGN after negotiation, and waiting only for SYNC is over-strict.
  * Aligner re-arm no longer keyed solely on 0xEE decode errors: mis-aligned data often decodes to
    plausible symbols with no error marker (exactly the F0B5A487/k0000 case), so the re-arm never
    fired. It now also pulses FFC_ENABLE_CGALIGN when no K character has been decoded for 4096 rx
    cycles.

**Still to try (in order):**
  1. The re-arm holdoff is 255 rx cycles and the no-comma window 4096; sweep both - the aligner may
     need many more attempts, or a much shorter holdoff, to walk onto the right boundary.
  2. Make the re-arm a CSR-driven manual pulse so the boundary search can be driven from software
     and observed dword-by-dword, instead of guessing constants in RTL.
  3. Compare the full RX PCS parameter set against a Diamond-generated G8B10B design
     (WA_MODE, RX_SB_BYPASS, UDF_COMMA_A/B/MASK, LSM_DISABLE, ENABLE_CG_ALIGN) - the TX side had an
     identical class of bug (dead word clock) fixed by one parameter, D_SYNC_LOCAL_EN.
  4. As a fallback, do the 8b10b decode and comma alignment in fabric (bypass RX) with a soft
     barrel shifter we control, rather than relying on the DCU aligner.

## *** CAMPAIGN 22: WORD ALIGNMENT FIXED - ctrl REACHES READY ***

**The alignment fix: let the DCU link state machine do it.** Every attempt to drive alignment via
the edge-pulsed `FFC_ENABLE_CGALIGN` re-arm failed, including an exhaustive runtime sweep of the
re-arm holdoff and no-comma window (12 combinations from 1 to 128 rx cycles, exposed as the new
`_oob_align` CSR). Setting **`LSM_DISABLE=0` + `ENABLE_CG_ALIGN=1`** in hybrid mode fixed it
outright:

    before (edge-pulsed re-arm): F0B5A487/k0000 x1434  (SYNC content, comma never flagged)
    after  (LSM + static align): **7B4A4ABC/k0001 x1488**  (clean ALIGN primitives)
    1526 valid dwords, 1498 carrying a K character.

So the receive chain is now fully working: CDR locks, word aligner finds the comma, and we decode
the device's ALIGN primitives cleanly and continuously.

**ctrl now reaches READY.** With alignment fixed plus a leaky-bucket debounce on `misalign` (a
single mis-aligned dword during resync must not tear the link down - the RX converter self-resets
and recovers within a few words), the FSM reached **READY at 15.3% occupancy**, with READY dwell
times up to 123us.

**What still blocks `ready`:** READY must survive the stability timer before asserting. We are
evicted by `rx_idle`, which the datapath's ALIGN timer raises after 41us without an ALIGN - the
device's ALIGN bursts are not continuous during bring-up, so READY is repeatedly torn down.
Attempts to tune this did NOT converge and one regressed:
  * datapath ALIGN window 41us -> 327us: **worse** - ctrl then sits in AWAIT-ALIGN 99.6% with
    rx_idle permanently asserted and never enters READY. REVERTED.
  * stability timer 5ms -> 1ms -> 50us (now the `stability_us` parameter): still not asserting.
Something in the rx_idle/align-timer interaction is not yet understood; it needs a full-rate
capture of `align_timer`, `rx.source.valid/charisk` and the FSM together, which is the first job
next session.

**Parameters now exposed for tuning** (all in ctrl.py / CSR): `align_timeout_us` (3ms),
`nocomwake_timeout_us` (400ns), `stability_us` (50us), `_oob_align.holdoff/nocomma`.
Bitstream `K-gen2-link` is built **--with-bist** so IDENTIFY can be issued the moment `ready`
latches.

**Status: the PHY completes OOB, receives and decodes the device's ALIGNs, and reaches READY -
but does not yet hold it long enough to declare ready, so IDENTIFY is still not reachable.**

## ***** CAMPAIGN 23: PHY READY - FIRST SATA LINK-UP ON ECP5 *****

**The last bug: the ALIGN timer only reset on ALIGN primitives.** `LiteSATAPHYAlignTimer` keyed
`rx_idle` solely on receiving ALIGN. But once a link is established both ends switch from ALIGN to
**SYNC**, so ALIGNs stop, the 41us timer expires, `rx_idle` asserts, and READY is torn straight
back down to RESET. Fixed: any K-led primitive (`charisk == 0b0001`) now holds the timer, which is
what the signal actually means - "the far end is still transmitting".

**RESULT: `sata_phy_status = 0xf`** - ready, tx_ready, rx_ready and ctrl_ready all asserted.
**The complete SATA link comes up on ECP5 for the first time**, and reliably: across six
consecutive trials it reached READY after 0.2s, 1.5s, 1.7s, 2.2s, 3.5s and 8.8s.

Full working chain: data-driven OOB (no electrical idle anywhere) -> device COMINIT -> our COMWAKE
-> device COMWAKE -> device ALIGNs -> LSM word alignment -> ALIGN exchange -> READY.

**Remaining: the link does not HOLD.** It collapses within milliseconds of asserting, so
IDENTIFY (issued immediately on the ready edge, six attempts) always finds the link already gone -
`identify_done` stays 0 and status has fallen back to 0x2/0x6. Next session:
  1. Capture with litescope triggered on the FALLING edge of `ctrl_ready` to see exactly what
     evicts READY (rx_idle again? misalign? the retry/stability timers?).
  2. Suspect the link-layer side: once READY latches, the datapath demux switches from ctrl to the
     core, and the device may be sending link-layer traffic (SYNC/ALIGN/R_RDY) that our core must
     answer - if it does not, the device drops the link. Check the core is actually being fed and
     is emitting SYNC continuously.
  3. Only then retry IDENTIFY; the `K-gen2-link` bitstream is already built --with-bist and the
     identify CSRs are present (`sata_bist_identify_start/done/source_*`).

**Working configuration (bitstream `K-gen2-link`)**: pcs_mode=hybrid (ENC_BYPASS=1 raw TX +
DEC_BYPASS=0 DCU RX decode, LSM_DISABLE=0, ENABLE_CG_ALIGN=1), rx_los_lvl=2, --with-bist;
txctl = pat_alt|deemph_gap; pattern=0xF0F0, gap_pattern=0x0000, wake_gap=16, burst_len=16,
_oob_quiet=50; control = ei_mode|ldr_timeout(4)|burst_mode|zero_bus|cdrhold_dis.

### Campaign 23 addendum: why the link does not hold - the TX handover at ctrl.ready

Litescope triggered on the FALLING edge of `ctrl_ready`, full rate, 996 samples of READY captured:

     off   state            rx_idle mis  rx_dword
      -10  READY           0     0   00000000/k0000
       ...                                (all zeros throughout READY)
       -2  READY           1     0   00000000/k0000   <- align timer expires
       +0  RESET           1     0   00000000/k0000
       +4  COMINIT

**Throughout the whole READY period we receive nothing but all-zero dwords - not a single
primitive.** `rx_idle` then expires (41us with no K-led primitive) and READY drops to RESET.

Prime suspect: **the TX handover.** `LiteSATAPHYDatapath` switches both directions on
`ctrl.ready` - `mux.sel.eq(ctrl.ready)` hands TX from ctrl to the core, and
`demux.sel.eq(ctrl.ready)` routes RX to the core. The instant we declare ready we therefore STOP
sending ctrl's SYNC and hand the transmitter to `LiteSATACore`. If the core does not immediately
emit continuous SYNC (link-layer idle), the device sees the host go silent and stops transmitting -
which is exactly the all-zero RX we observe, and the link tears down ~41us later.

Next session, in order:
  1. Check what `LiteSATACore`'s link layer emits immediately after ready with a TX-side capture
     (`datapath.sink`/`trx.sink` data+charisk). If it is not SYNC, that is the bug.
  2. Simplest experiment: keep ctrl driving TX (force `mux.sel = 0`) even in READY and see whether
     the link then holds. If it does, the fault is entirely in the core handover and IDENTIFY can
     be attempted with a small mux change.
  3. Also verify the core is out of reset and its crossbar/BIST user port is idle-but-valid.

The PHY itself is now proven: OOB, alignment and the ALIGN exchange all work, and READY is
reached reliably (0.2-8.8s over six trials).

## Campaign 24: core always present, link holds intermittently; device never leaves ALIGN

**Phase 0 fixes (plan-driven):**
1. **`bench/ecpix5.py` now instantiates Core/Crossbar/BIST unconditionally** - it was the only bench
   in the repo doing it under `if with_bist:` (every Xilinx bench does it unconditionally). With no
   core, `sata_phy.sink` is a dangling endpoint and the PHY (which copies the bus ignoring
   `sink.valid`, like the Xilinx PHYs) transmitted **D0.0 for ever** the instant `ctrl.ready` flipped
   the datapath mux. `with_bist` now only gates the BIST CSRs. Added the acorn `min_sys_clk_freq`
   guard.
2. **Defensive `tx_idle_sync`** in `ecp5sataphy.py`: when `sink.valid` is low the PHY now transmits
   SYNC (as two alternating 16-bit halves) instead of the raw bus, so a starved sink can never again
   be link-fatal.
3. **Analyzer group 2** (link layer): `link.tx.fsm`, `link.rx.fsm`, `from_rx.idle/insert`,
   `tx_align.source`, `datapath.sink`, `ctrl.ready/rx_idle`.

**Verified: our transmitter is correct.** With the core present, group 2 shows `link.tx.fsm` and
`link.rx.fsm` both in IDLE, `from_rx.idle=1`, and `tx_align.source` = **`B5B5957C` / charisk=0b0001
/ valid=1** - continuous SYNC - carried identically on `datapath.sink`. The TX handover is no longer
the problem.

**Link now reaches READY in 0.1-0.5s but flaps.** Progress from parameterized tolerances:
    baseline (core fix only)                          :  1% ready
    + misalign_tolerance=512                          :  1%
    + ALIGN window 41us -> 655us                      : 20%
    + SCI background reconfig frozen (pause=1)        : 28-35%
Each is a real improvement but none is the cure. `_oob_align` holdoff/nocomma sweeps (12 settings,
1..65535) change nothing material - the CGALIGN re-arm is not the driver.

**Root-cause evidence for what remains.** Capturing the drop (trigger on falling `ctrl_ready`):
    -5 READY rdy=1 rx_idle=0 mis=1  0000EEEE/k0011
    -2 READY rdy=1 rx_idle=1 mis=0  E026D926/k0000     <- alignment lost, garbage
    +0 RESET rdy=0 rx_idle=1        0000EEEE/k0011
`rx_idle` (not misalign) is the evictor, and the RX content at that moment is garbage with no K
character: **the DCU word aligner drops lock mid-link.** Continuous alignment
(`ENABLE_CG_ALIGN=1`) acquires lock but is unstable once real data flows; switched to
**LSM maintenance (`LSM_DISABLE=0`) + pulse-on-error re-arm**, which is the committed config.

**The decisive observation: during READY the device transmits ALIGN continuously (628 of 742
dwords) and NEVER progresses to SYNC.** A device that had accepted our transmission would move to
SYNC. So the remaining suspect is **our TX 8b10b stream itself**: in `hybrid` mode
(`ENC_BYPASS=1`) the raw 10-bit words come from the *fabric* encoder, and that path has never been
validated against a real receiver - the loopback self link-up of campaign 12 used **g8b10b**, where
the *DCU* encodes. If our raw-word bit mapping into `tx_bus[0:10]/[12:22]` (the 10BSER layout) is
not what G8B10B+ENC_BYPASS expects, the device sees invalid symbols, never leaves speed
negotiation, and periodically resets us - exactly the observed flapping.

**Next, in order:**
  1. Validate the hybrid TX encoding: re-fit the loopback cable and check our own RX decodes our
     own fabric-encoded ALIGN/SYNC in `hybrid` mode. That isolates TX-mapping from device
     behaviour in one measurement.
  2. If the mapping is wrong, find the correct `ENC_BYPASS=1` TX bus layout (TN1261 Table 7.3 /
     Diamond reference), or move OOB into a `g8b10b` build so the DCU encodes both directions.
  3. Only then retry IDENTIFY - the link must hold for milliseconds, and 35% flapping is far short.

**Parameterization (keeps Xilinx behaviour byte-identical), per plan Phase 3:**
`LiteSATAPHYCtrl(misalign_tolerance=0)` default = original immediate RX reset; ECP5 passes 512.
`LiteSATAPHYDatapath(align_timeout=256*16)` default unchanged; ECP5 passes 256*16*16.
Both selected in the ECP5 arm of `litesata/phy/__init__.py`.
