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
