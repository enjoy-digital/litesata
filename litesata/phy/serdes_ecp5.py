#
# This file is part of LiteSATA.
#
# Vendored from LiteICLink @ 2da8e8b (liteiclink/serdes/serdes_ecp5.py) with SATA OOB support.
# Modifications are marked with "# OOB:" comments:
# - SerdesInit: split tx_ready/rx_ready, no rx_los exit from READY (line is idle during OOB).
# - TX electrical idle: direct FFC_EI_EN port (SCI pcie_ei_en path removed, too slow).
# - LDR (low data rate) direct-drive OOB path: TX burst generation / RX line observation.
#
# Copyright (c) 2019-2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

from migen import *
from migen.genlib.cdc import MultiReg, PulseSynchronizer
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import *
from litex.gen.genlib.misc import WaitTimer

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream
from litex.soc.cores.prbs import PRBSTX, PRBSRX
from litex.soc.cores.code_8b10b import Encoder, Decoder

# SerDesECP5PLL ------------------------------------------------------------------------------------

class SerDesECP5PLL(LiteXModule):
    def __init__(self, refclk, refclk_freq, linerate):
        self.refclk = refclk
        self.config = self.compute_config(refclk_freq, linerate)

    @staticmethod
    def compute_config(refclk_freq, linerate):
        multipliers = [8, 10, 16, 20]
        if refclk_freq == 100e6:
            multipliers.append(25) # x25 for 100MHz refclk only.
        dividers = [1, 2, 4, 8, 16, 32]
        for d in reversed(dividers): # Use highest possible VCO.
            for m in multipliers:
                vco_freq = refclk_freq*m
                current_linerate = vco_freq/d
                if current_linerate == linerate:
                    return {
                        "clkin": refclk_freq,
                        "m":               m,
                        "d":               d,
                        "vco_freq": vco_freq,
                        "linerate": linerate,
                    }
        msg = "No config found for {:3.4f} MHz refclk / {:3.4f} Gbps linerate.\n"
        msg += "Possible refclk frequencies:\n"
        refclks = []
        for d in dividers:
            for m in multipliers:
                f = linerate*d/m
                if f <= 250e6 and f not in refclks:
                    refclks.append(f)
                    msg += " - {:3.4f}MHz\n".format(linerate*d/m*1e-6)
        msg = msg[:-1]
        raise ValueError(msg.format(refclk_freq/1e6, linerate/1e9))

    def __repr__(self):
        config = self.config
        r = """
SerDesECP5PLL
==============
  overview:
  ---------
       +---------------------------+
       | +-----+  +-----+  +-----+ |
       | |     |  |     |  |     | |
CLKIN +-->  M  +--> VCO +--> /D  +--> LINERATE
       | |     |  |     |  |     | |
       | +-----+  +-----+  +-----+ |
       +---------------------------+

  config:
  -------
    CLKIN    = {clkin}MHz
    VCO      = CLKIN x M = {clkin}MHz x {m} = {vco_freq}GHz
    LINERATE = VCO / D   = {vco_freq}GHz / {d}
             = {linerate}GHz
""".format(clkin    = config["clkin"]/1e6,
           m        = config["m"],
           d        = config["d"],
           vco_freq = config["vco_freq"]/1e9,
           linerate = config["linerate"]/1e9)
        return r

# BypassWordAligner --------------------------------------------------------------------------------

class BypassWordAligner(Module):
    """Fabric word aligner for the raw 10BSER (bypass) datapath.

    The DCU comma aligner is a G8B10B PCS feature and does not operate on the raw 10-bit datapath
    (measured: against a real device ALIGN stream the raw-bus word boundary drifts freely and the
    decoded stream never contains a K character, with every DCU aligner knob neutral). This module
    aligns in the fabric: scan a 40-bit sliding window over consecutive raw words for the K28.5
    comma7 (serial 0011111 = 0x7C read LSB-first, or its complement 0x03) at all 20 bit offsets,
    and barrel-shift the datapath to the symbol boundary. Bit 0 of `sink` must be the earliest bit
    on the wire. Pipelined in three stages (comparators / priority encode / shift) to close timing
    in the 150MHz rx word-clock domain. Latency 3 cycles; `slip` is quasi-static once locked so
    the inter-stage vintage skew at re-lock only garbles the word in flight.
    """
    def __init__(self):
        self.enable  = Signal(reset=1) # i
        self.sink    = Signal(20)      # i: raw word, bit0 first on wire
        self.source  = Signal(20)      # o: comma-aligned word
        self.slip    = Signal(5)       # o (debug)
        self.slip_mv = Signal(8)       # o (debug): slip-change count

        # # #

        prev   = Signal(20)
        win    = Signal(40)
        win_r  = Signal(40)
        hits   = Signal(20)
        found  = Signal()
        slip_n = Signal(5)
        shift  = Signal(40)

        self.comb += win.eq(Cat(prev, self.sink)) # bit 0 = oldest on the wire
        # Stage A: 20 parallel comma comparators, registered.
        self.sync += [
            prev.eq(self.sink),
            win_r.eq(win),
            hits.eq(Cat(*[(win[k:k+7] == 0x7C) | (win[k:k+7] == 0x03) for k in range(20)])),
        ]
        # Stage B: priority encode (lowest offset wins), registered slip.
        self.comb += [
            found.eq(hits != 0),
            slip_n.eq(self.slip),
        ]
        for k in reversed(range(20)): # last match wins -> lowest offset
            self.comb += If(hits[k], slip_n.eq(k))
        # Slip-offset voting: scrambled payload and CONT junk contain comma7 LOOKALIKES at random
        # offsets, and a single false comma used to steal the boundary - which is unrecoverable
        # between the drive's 256-dword ALIGN beacons since only K28.5 carries the true comma7
        # (SYNC's K28.3 is 0011110, no match). Adopt a new slip only when two CONSECUTIVE comma
        # detections agree on the offset: the drive's ALIGN bursts put two back-to-back K28.5s at
        # the same offset (and repeat every 256 dwords), while junk commas land at random offsets,
        # so a 2-of-2 vote passes real ALIGNs and rejects junk. A detection at the current slip
        # clears the candidate, so an isolated junk comma cannot pair with a later unrelated one.
        cand    = Signal(5)
        cand_ok = Signal()
        # Stage C: barrel shift with the (quasi-static) slip, registered output.
        self.comb += shift.eq(win_r >> self.slip)
        self.sync += [
            If(self.enable & found,
                If(slip_n == self.slip,
                    cand_ok.eq(0),                     # confirmed at current offset
                ).Elif(cand_ok & (slip_n == cand),
                    self.slip.eq(slip_n),              # second consecutive vote -> adopt
                    self.slip_mv.eq(self.slip_mv + 1),
                    cand_ok.eq(0),
                ).Else(
                    cand.eq(slip_n),                   # first vote for a new offset
                    cand_ok.eq(1),
                ),
            ),
            self.source.eq(shift[0:20]),
        ]

# SerDesSCI ----------------------------------------------------------------------------------------

class SerDesECP5SCI(LiteXModule):
    def __init__(self, serdes):
        self.dual_sel = Signal()
        self.chan_sel = Signal()
        self.re       = Signal()
        self.we       = Signal()
        self.done     = Signal()
        self.adr      = Signal(6)
        self.dat_w    = Signal(8)
        self.dat_r    = Signal(8)

        # # #

        self.sci_rd    = sci_rd    = Signal()
        self.sci_wrn   = sci_wrn   = Signal(reset=1)
        self.sci_addr  = sci_addr  = Signal(6)
        self.sci_wdata = sci_wdata = Signal(8)
        self.sci_rdata = sci_rdata = Signal(8)

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.eq(1),
            If(self.we,
                NextState("WRITE")
            ).Elif(self.re,
                sci_rd.eq(1),
                NextState("READ")
            )
        )
        fsm.act("WRITE",
            sci_wrn.eq(0),
            NextState("IDLE")
        )
        fsm.act("READ",
            sci_rd.eq(1),
            NextValue(self.dat_r, sci_rdata),
            NextState("IDLE")
        )
        self.comb += [
            sci_addr.eq(self.adr),
            sci_wdata.eq(self.dat_w)
        ]

        serdes.serdes_params.update(
             **{"i_D_SCIWDATA%d"  % n: sci_wdata[n] for n in range(8)},
             **{"i_D_SCIADDR%d"   % n: sci_addr[n]  for n in range(6)},
             **{"o_D_SCIRDATA%d"  % n: sci_rdata[n] for n in range(8)},
             i_D_SCIENAUX  = self.dual_sel,
             i_D_SCISELAUX = self.dual_sel,
             i_CHX_SCIEN   = self.chan_sel,
             i_CHX_SCISEL  = self.chan_sel,
             i_D_SCIRD     = sci_rd,
             i_D_SCIWSTN   = sci_wrn,
        )

@ResetInserter()
class SerDesECP5SCIReconfig(LiteXModule):
    def __init__(self, serdes):
        self.loopback    = Signal()
        self.rx_polarity = Signal()
        self.tx_polarity = Signal()
        self.rx_cdr_hold = Signal()
        # OOB: tx_idle removed from SCI path; electrical idle now uses the direct FFC_EI_EN port.

        self.pause = CSRStorage(1, description="Pause Hardware re-configuration")
        self.sel   = CSRStorage(1, description="Channel/Dual selection: 0 Channel / 1 Dual")
        self.we    = CSRStorage(1, description="Do a Write access")
        self.re    = CSRStorage(1, description="Do a Read access")
        self.done  = CSRStatus(    description="Access is Done")
        self.adr   = CSRStorage(6, description="Access address")
        self.dat_w = CSRStorage(8, description="Access Write data")
        self.dat_r = CSRStatus(8,  description="Access Read data")

        # # #

        self.sci = sci = SerDesECP5SCI(serdes)

        # OOB slice gate (see PHY): when enabled, this FSM stops its background refresh loop and
        # instead writes CH_12 (tdrv_slice*_sel) on every burst/gap transition of the OOB
        # generator. A write is 2 SCI cycles (~20ns at 100MHz), i.e. fast enough for SATA OOB,
        # unlike FFC_EI_EN whose un-mute latency is 213-427ns (measured).
        self.oob_gate_en  = Signal()
        self.oob_gate_lvl = Signal()   # 1 = burst (slices on), 0 = gap (slices powered down)
        self.oob_burst_val = Signal(8)
        self.oob_gap_val   = Signal(8)
        oob_lvl_d = Signal(reset=1)

        first = Signal()
        data  = Signal(8)

        self.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.done.status.eq(1),
            If(self.oob_gate_en,
                If(oob_lvl_d != self.oob_gate_lvl,
                    NextValue(oob_lvl_d, self.oob_gate_lvl),
                    NextState("OOB-GATE")
                )
            ).Elif(self.pause.storage,
                If(self.we.wr_stb,
                    NextState("WRITE")
                ),
                If(self.re.wr_stb,
                    NextState("READ")
                )
            ).Else(
                NextState("READ-CH-01")
            )
        )
        fsm.act("OOB-GATE",
            sci.chan_sel.eq(1),
            sci.we.eq(1),
            sci.adr.eq(0x12),
            sci.dat_w.eq(Mux(oob_lvl_d, self.oob_burst_val, self.oob_gap_val)),
            If(~first & sci.done,
                sci.we.eq(0),
                NextState("IDLE")
            )
        )
        fsm.act("READ",
            sci.dual_sel.eq(self.sel.storage == 1),
            sci.chan_sel.eq(self.sel.storage == 0),
            sci.re.eq(1),
            sci.adr.eq(self.adr.storage),
            If(~first & sci.done,
                sci.re.eq(0),
                NextValue(self.dat_r.status, sci.dat_r),
                NextState("IDLE"),
            )
        )
        fsm.act("WRITE",
            sci.dual_sel.eq(self.sel.storage == 1),
            sci.chan_sel.eq(self.sel.storage == 0),
            sci.we.eq(1),
            sci.adr.eq(self.adr.storage),
            sci.dat_w.eq(self.dat_w.storage),
            If(~first & sci.done,
                sci.we.eq(0),
                NextState("IDLE")
            )
        )
        fsm.act("READ-CH-01",
            sci.chan_sel.eq(1),
            sci.re.eq(1),
            sci.adr.eq(0x01),
            If(~first & sci.done,
                sci.re.eq(0),
                NextValue(data, sci.dat_r),
                NextState("WRITE-CH-01"),
            )
        )
        fsm.act("WRITE-CH-01",
            sci.chan_sel.eq(1),
            sci.we.eq(1),
            sci.adr.eq(0x01),
            sci.dat_w.eq(data),
            sci.dat_w[0].eq(self.rx_polarity),
            sci.dat_w[1].eq(self.tx_polarity),
            If(~first & sci.done,
                sci.we.eq(0),
                # OOB: CH-02 (pcie_ei_en) states removed; EI is driven by the direct FFC_EI_EN port.
                NextState("READ-CH-15")
            )
        )
        fsm.act("READ-CH-15",
            sci.chan_sel.eq(1),
            sci.re.eq(1),
            sci.adr.eq(0x15),
            If(~first & sci.done,
                sci.re.eq(0),
                NextValue(data, sci.dat_r),
                NextState("WRITE-CH-15"),
            )
        )
        fsm.act("WRITE-CH-15",
            sci.chan_sel.eq(1),
            sci.we.eq(1),
            sci.adr.eq(0x15),
            sci.dat_w.eq(data),
            If(self.loopback,
                sci.dat_w[0:4].eq(0b0001) # lb_ctl
            ),
            If(~first & sci.done,
                sci.we.eq(0),
                NextState("READ-CH-18")
            )
        )
        fsm.act("READ-CH-18",
            sci.chan_sel.eq(1),
            sci.re.eq(1),
            sci.adr.eq(0x18),
            If(~first & sci.done,
                sci.re.eq(0),
                NextValue(data, sci.dat_r),
                NextState("WRITE-CH-18"),
            )
        )
        fsm.act("WRITE-CH-18",
            sci.chan_sel.eq(1),
            sci.we.eq(1),
            sci.adr.eq(0x18),
            sci.dat_w.eq(data),
            sci.dat_w[6].eq(self.rx_cdr_hold),
            If(~first & sci.done,
                sci.we.eq(0),
                NextState("IDLE")
            )
        )
        fsm.finalize()

        last_fsm_state = Signal(4)
        self.sync += last_fsm_state.eq(fsm.state)
        self.comb += first.eq(fsm.state != last_fsm_state)

# SerdesInit ---------------------------------------------------------------------------------------

class SerdesInit(LiteXModule):
    def __init__(self, tx_lol, rx_lol, rx_los):
        self.rst      = Signal()
        self.tx_rst   = Signal()
        self.rx_rst   = Signal()
        self.pcs_rst  = Signal()
        # OOB: split ready in tx_ready/rx_ready (registered/sticky): the TX side must be usable
        # (COMGenerator running, EI/LDR controllable) while RX has no lock/signal, since the SATA
        # OOB sequence precedes any RX alignment.
        self.tx_ready = Signal()
        self.rx_ready = Signal()

        # # #

        self.tx_lol = _tx_lol = Signal()
        self.rx_lol = _rx_lol = Signal()
        self.rx_los = _rx_los = Signal()
        self.specials += MultiReg(tx_lol, _tx_lol)
        self.specials += MultiReg(rx_lol, _rx_lol)
        self.specials += MultiReg(rx_los, _rx_los)

        timer = WaitTimer(1024)
        self.submodules += timer
        self.comb += timer.wait.eq(~self.rst)

        fsm = FSM(reset_state="RESET-ALL")
        fsm = ResetInserter()(fsm)
        self.fsm = fsm
        self.comb += fsm.reset.eq(self.rst)
        fsm.act("RESET-ALL",
            # Reset TX Serdes, RX Serdes and PCS.
            self.tx_rst.eq(1),
            self.rx_rst.eq(1),
            self.pcs_rst.eq(1),
            NextValue(self.tx_ready, 0),
            NextValue(self.rx_ready, 0),
            If(timer.done,
                timer.wait.eq(0),
                NextState("RESET-RX-PCS-WAIT-TX-PLL-LOCK")
            )
        )
        fsm.act("RESET-RX-PCS-WAIT-TX-PLL-LOCK",
            # Reset RX Serdes and PCS, wait for TX PLL lock.
            self.rx_rst.eq(1),
            self.pcs_rst.eq(1),
            NextValue(self.rx_ready, 0),
            If(timer.done & ~_tx_lol,
                timer.wait.eq(0),
                NextValue(self.tx_ready, 1),
                NextState("RESET-PCS-WAIT-RX-CDR-LOCK")
            )
        )
        fsm.act("RESET-PCS-WAIT-RX-CDR-LOCK",
            # Reset PCS, wait for RX CDR lock.
            self.pcs_rst.eq(1),
            If(timer.done & ~_rx_lol,
                timer.wait.eq(0),
                NextValue(self.rx_ready, 1),
                NextState("READY")
            )
        )
        fsm.act("READY",
            # OOB: no rx_los exit here: during OOB/idle phases the line is electrically idle by
            # design (rx_los asserted); leaving READY on rx_los would reset the RX path in a loop
            # and prevent the OOB sequence from ever completing. rx_los remains a status signal.
            If(_tx_lol,
                NextState("RESET-ALL")
            ),
            If(_rx_lol,
                NextState("RESET-RX-PCS-WAIT-TX-PLL-LOCK")
            )
        )

# SerDesECP5 ---------------------------------------------------------------------------------------

class SerDesECP5(LiteXModule):
    def __init__(self, pll, tx_pads, rx_pads,
        dual        = 0,
        channel     = 0,
        data_width  = 20,
        tx_polarity = 0,
        rx_polarity = 0,
        oob_config  = {"ei", "ldr_tx", "ldr_rx"},
        pcs_mode    = "bypass",
        tx_boost    = False,
        rx_los_lvl  = 4,
        rx_rate_mode = "0b0",
        tx_rate_mode = "0b0",
        pcie_mode   = False):
        assert dual       in [0, 1]
        assert channel    in [0, 1]
        assert data_width in [20]
        assert pcs_mode   in ["bypass", "g8b10b", "pcie", "pcie_bypass", "hybrid"]
        self.pcs_mode = pcs_mode
        self.dual       = dual
        self.channel    = channel
        self.oob_config = oob_config

        # TX controls.
        self.tx_enable              = Signal(reset=1)
        self.tx_ready               = Signal()
        self.tx_inhibit             = Signal() # FIXME
        self.tx_produce_square_wave = Signal()
        self.tx_produce_pattern     = Signal()
        self.tx_pattern             = Signal(data_width)
        self.tx_prbs_config         = Signal(2)
        self.tx_idle                = Signal()

        # RX controls.
        self.rx_enable              = Signal(reset=1)
        self.rx_ready               = Signal()
        self.rx_align               = Signal(reset=1)
        self.rx_prbs_config         = Signal(2)
        self.rx_prbs_pause          = Signal()
        self.rx_prbs_errors         = Signal(32)
        self.rx_idle                = Signal()
        self.rx_cdr_hold            = Signal()

        # OOB: SATA OOB controls (LDR direct-drive path, see TN1261/LUNA).
        # tx_oob_en/tx_oob_data/tx_oob_idle are expected to be driven from the "tx" clock domain
        # (burst timing is defined by tx_oob_en alone); rx_oob_data is a raw asynchronous view of
        # the RX differential pair, to be synchronized by the consumer.
        self.tx_oob_en              = Signal()
        self.rx_det_en   = Signal() # i: PCIe receiver-detect enable (TX in EI).
        self.rx_det_ct   = Signal() # i: receiver-detect trigger.
        self.rx_det_done = Signal() # o: detect sequence done.
        self.rx_det_con  = Signal() # o: far-end receiver termination present. # i, tx domain: drive pads via LDR (burst gate).
        self.tx_oob_data            = Signal() # i, tx domain: LDR level (square wave).
        self.tx_oob_idle            = Signal() # i, tx domain: extra EI request (OR'ed with tx_idle).
        self.tx_oob_active          = Signal() # i, tx domain: OOB sequence in progress.
        self.tx_oob_ei_req          = Signal() # i, tx domain: shaped EI request (lead/trail comp.).
        self.rx_oob_data            = Signal() # o, async   : raw LDR_RX2CORE line observation.
        # OOB: runtime silencing of the MAIN (serializer) TX driver while leaving the LDR aux
        # driver alive - gives EI-free OOB gaps that are releasable for the data phase (unlike
        # p_CHX_PCIE_MODE, which achieves the same silence but is a fuse).
        # OOB: Gen1-rate carrier synthesis. SATA specifies the OOB burst content as repeated
        # D24.3 AT THE GEN1 RATE for every generation (1.5Gb/s D24.3 = a 375MHz square). A
        # period-8 pattern does not tile into the 20-bit raw word, so alternate the word with its
        # bitwise inverse every tx cycle: 0xF0F0F / 0x0F0F0 concatenate into a continuous period-8
        # stream (verified). At 3.0Gb/s that is exactly 375MHz.
        self.tx_pattern_alt         = Signal() # i: alternate tx_pattern with its inverse.
        self.tx_pattern_gap         = Signal(20) # i: pattern sent during OOB gaps (DC = idle).
        self.tx_oob_gap             = Signal()   # i (tx domain): 1 = OOB gap in progress.
        self.tx_oob_deemph          = Signal()   # i (tx domain): data-driven gaps -> mask EI.
        # SATA speed negotiation: the DCU's dynamic half-rate divider. With the PLL at 3.0Gbps
        # (Gen2), asserting the RX rate mode receives 1.5Gbps (Gen1) without touching the PLL -
        # which is how a real SATA host hunts the device's rate during speed negotiation. Ports
        # confirmed present in Diamond's DCUA.v (CH0_FFC_RATE_MODE_RX/TX).
        self.rate_mode_tx           = Signal()
        self.rate_mode_rx           = Signal()
        self.sci_oob_gate_en   = Signal() # i: SCI slice gate enable.
        self.sci_oob_gate_lvl  = Signal() # i: 1 = burst, 0 = gap.
        self.sci_oob_burst_val = Signal(8)
        self.sci_oob_gap_val   = Signal(8)
        self.tx_pwdn                = Signal() # i, quasi-static: power down the TX driver.
        self.tx_lane_rst            = Signal() # i, quasi-static: hold the TX lane in reset.
        self.ei_mode                = Signal() # i, quasi-static: 0 = EI masked during LDR drive
                                               #                  (LUNA-style), 1 = shaped EI
                                               #                  (COMGenerator lead/trail comp.).

        # Loopback.
        self.loopback               = Signal() # FIXME: reconfigure lb_ctl to 0b0001 but does not seem enough

        # # #

        self.nwords = nwords = data_width//10

        if pcs_mode in ["bypass", "pcie_bypass", "hybrid"]:
            self.encoder  = ClockDomainsRenamer("tx")(Encoder(nwords, True))
            self.decoders = [ClockDomainsRenamer("rx")(Decoder(True)) for _ in range(nwords)]
            # A plain LIST attribute is NOT auto-registered by LiteXModule (verified: a module
            # holding a list of Decoders elaborates to zero statements from them), so the fabric
            # decoders were silently absent from the netlist - d/k/invalid hardwired to 0, which
            # is exactly the "decoded output always 00000000/k0000, notintable always 0" measured
            # against the drive. Register them explicitly.
            self.submodules += self.decoders
        else:
            # OOB: G8B10B mode uses the DCU-internal 8b10b; invalid received symbols are decoded
            # as 0xEE with the K flag set (see LUNA/TN-02206).
            self.rx_errs = Signal(nwords)

        # Transceiver direct clock outputs (useful to specify clock constraints).
        self.txoutclk = Signal()
        self.rxoutclk = Signal()

        self.tx_clk_freq = pll.config["linerate"]/data_width
        self.rx_clk_freq = pll.config["linerate"]/data_width

        # Internal signals -------------------------------------------------------------------------
        rx_los     = Signal()
        rx_lol     = Signal()
        rx_lsm     = Signal()
        rx_align   = Signal()
        cg_align_pulse = Signal() # OOB: g8b10b word-aligner re-arm pulse (edge-sensitive input).
        rx_data    = Signal(20)
        rx_bus     = Signal(24)

        tx_lol     = Signal()
        self.rx_bus_dbg = Signal(24)   # raw DCU RX parallel bus (debug observation)
        self.rx_lol_dbg = Signal()     # CDR loss-of-lock (debug observation)
        tx_data    = Signal(20)
        tx_data_r  = Signal(20) # tx_data registered in the tx domain (DCU TX bus setup, see below).
        tx_bus     = Signal(24)

        # Control/Status CDC -----------------------------------------------------------------------
        tx_produce_square_wave = Signal()
        tx_produce_pattern     = Signal()
        tx_pattern             = Signal(20)
        self.align_holdoff     = Signal(16, reset=64)   # rx cycles to wait after a re-arm pulse
        self.align_nocomma     = Signal(16, reset=64)   # re-arm after this many cycles with no K
        self.align_cont        = Signal()               # 1 = continuous CG align, 0 = re-arm on error
        align_holdoff_rx       = Signal(16, reset=64)
        align_nocomma_rx       = Signal(16, reset=64)
        align_cont_rx          = Signal()
        pattern_alt_tx         = Signal()
        pattern_toggle         = Signal()
        tx_prbs_config         = Signal(2)

        rx_prbs_config         = Signal(2)
        rx_prbs_pause          = Signal()
        rx_prbs_errors         = Signal(32)

        self.specials += [
            MultiReg(self.tx_produce_square_wave, tx_produce_square_wave, "tx"),
            MultiReg(self.tx_produce_pattern, tx_produce_pattern, "tx"),
            MultiReg(self.tx_pattern_alt,     pattern_alt_tx,     "tx"),
            MultiReg(self.align_holdoff, align_holdoff_rx, "rx"),
            MultiReg(self.align_nocomma, align_nocomma_rx, "rx"),
            MultiReg(self.align_cont,    align_cont_rx,    "rx"),
            MultiReg(self.tx_pattern, tx_pattern, "tx"),
            MultiReg(self.tx_prbs_config, tx_prbs_config, "tx"),
        ]

        # OOB: Electrical idle control (tx domain). tx_idle comes from sys (level, held for entire
        # OOB phases so MultiReg latency is harmless); tx_oob_* come from the tx domain directly.
        tx_idle_tx = Signal()
        ei_mode_tx = Signal()
        ei_en      = Signal()
        self.specials += [
            MultiReg(self.tx_idle, tx_idle_tx, "tx"),
            MultiReg(self.ei_mode, ei_mode_tx, "tx"),
        ]
        ei_legacy = Signal()
        ei_shaped = Signal()
        self.comb += [
            # LUNA-style: EI whenever idle is requested, masked while LDR is driving.
            ei_legacy.eq((tx_idle_tx | self.tx_oob_idle) & ~self.tx_oob_en),
            # Shaped: during an OOB sequence the COMGenerator emits an EI request with lead/trail
            # compensation for the slow FFC_EI_EN response; outside sequences ctrl's tx_idle rules.
            ei_shaped.eq(Mux(self.tx_oob_active, self.tx_oob_ei_req, tx_idle_tx)),
            # Data-driven gaps: the gap is made by the serializer holding a constant, transition-free
            # pattern while the driver keeps driving, so electrical idle must be kept out of the
            # whole sequence - any EI assertion mutes the driver and the gap width then follows the
            # EI un-mute latency (213-427ns measured), which cannot reach the 101.3-112ns COMWAKE
            # window. Masked here, in the tx domain, so it holds for both EI modes.
            ei_en.eq(Mux(ei_mode_tx, ei_shaped, ei_legacy)
                     & ~(self.tx_oob_deemph & self.tx_oob_active)),
        ]

        self.specials += [
            MultiReg(self.rx_align, rx_align, "rx"),
            MultiReg(self.rx_prbs_config, rx_prbs_config, "rx"),
            MultiReg(self.rx_prbs_pause, rx_prbs_pause, "rx"),
            MultiReg(rx_los, self.rx_idle, "sys"),
            MultiReg(rx_prbs_errors, self.rx_prbs_errors, "sys"),
        ]

        # DCU init ---------------------------------------------------------------------------------
        self.init = init = SerdesInit(tx_lol, rx_lol, rx_los)

        # Clocking ---------------------------------------------------------------------------------
        # OOB: tx/rx domains released independently (split init readies): the TX domain must run
        # (COMGenerator) while RX is still unlocked during the OOB sequence.
        self.cd_tx = ClockDomain()
        self.comb += self.cd_tx.clk.eq(self.txoutclk)
        self.specials += AsyncResetSynchronizer(self.cd_tx, ~init.tx_ready)
        self.comb += self.tx_ready.eq(init.tx_ready)

        self.cd_rx = ClockDomain()
        self.comb += self.cd_rx.clk.eq(self.rxoutclk)
        self.specials += AsyncResetSynchronizer(self.cd_rx, ~init.rx_ready)
        self.comb += self.rx_ready.eq(init.rx_ready)

        # DCU instance -----------------------------------------------------------------------------
        self.serdes_params = dict(
            # ECP5's DCU parameters/signals/instance have been documented by whitequark as part of
            #             Yumewatari project: https://github.com/whitequark/Yumewatari
            #                  Copyright (C) 2018 whitequark@whitequark.org
            # DCU ----------------------------------------------------------------------------------
            # DCU — power management
            p_D_MACROPDB            = "0b1",
            p_D_IB_PWDNB            = "0b1",    # undocumented (required for RX)
            p_D_TXPLL_PWDNB         = "0b1",
            i_D_FFC_MACROPDB        = 1,

            # DCU — reset
            i_D_FFC_MACRO_RST       = ResetSignal("sys"),
            i_D_FFC_DUAL_RST        = ResetSignal("sys"),

            # DCU — clocking
            i_D_REFCLKI             = pll.refclk,
            o_D_FFS_PLOL            = tx_lol,
            p_D_REFCK_MODE          = {
                25: "0b100",
                20: "0b000",
                16: "0b010",
                10: "0b001",
                 8: "0b011"}[pll.config["m"]],
            p_D_TX_MAX_RATE         = "5.0",    # 5.0 Gbps
            p_D_TX_VCO_CK_DIV       = {
                32: "0b111",
                16: "0b110",
                 8: "0b101",
                 4: "0b100",
                 2: "0b010",
                 1: "0b000"}[pll.config["d"]],
            p_D_BITCLK_LOCAL_EN     = "0b1",    # Use clock from local PLL
            # OOB: local TX sync enable (Diamond/Clarity sets it; without it the TX gearbox sync
            # never starts and FF_TX_PCLK stays dead while the TX PLL still reports lock).
            p_D_SYNC_LOCAL_EN       = "0b1",

            # DCU ­— unknown
            p_D_CMUSETBIASI         = "0b00",   # begin undocumented (10BSER sample code used)
            p_D_CMUSETI4CPP         = "0d3",
            p_D_CMUSETI4CPZ         = "0d3",
            p_D_CMUSETI4VCO         = "0b00",
            p_D_CMUSETICP4P         = "0b01",
            p_D_CMUSETICP4Z         = "0b101",
            p_D_CMUSETINITVCT       = "0b00",
            p_D_CMUSETISCL4VCO      = "0b000",
            p_D_CMUSETP1GM          = "0b000",
            p_D_CMUSETP2AGM         = "0b000",
            p_D_CMUSETZGM           = "0b000",
            p_D_SETIRPOLY_AUX       = "0b01",
            p_D_SETICONST_AUX       = "0b01",
            p_D_SETIRPOLY_CH        = "0b01",
            p_D_SETICONST_CH        = "0b10",
            p_D_SETPLLRC            = "0d1",
            p_D_RG_EN               = "0b0",
            p_D_RG_SET              = "0b00",
            p_D_REQ_ISET            = "0b011",
            p_D_PD_ISET             = "0b11",   # end undocumented

            # DCU — FIFOs
            p_D_LOW_MARK            = "0d4",    # Clock compensation FIFO low  water mark (mean=8)
            p_D_HIGH_MARK           = "0d12",   # Clock compensation FIFO high water mark (mean=8)

            # CHX common ---------------------------------------------------------------------------
            # CHX — protocol
            p_CHX_PROTOCOL          = "10BSER",
            p_CHX_UC_MODE           = "0b1",
            p_CHX_ENC_BYPASS        = "0b1",    # Bypass 8b10b encoder
            p_CHX_DEC_BYPASS        = "0b1",    # Bypass 8b10b encoder

            # CHX receive --------------------------------------------------------------------------
            # CHX RX — power management
            p_CHX_RPWDNB            = "0b1",
            i_CHX_FFC_RXPWDNB       = 1,

            # CHX RX — reset
            i_CHX_FFC_RRST          = ~self.rx_enable | init.rx_rst,
            i_CHX_FFC_LANE_RX_RST   = ~self.rx_enable | init.pcs_rst,

            # CHX RX — input
            i_CHX_HDINP             = rx_pads.p,
            i_CHX_HDINN             = rx_pads.n,

            # RX equalizer: LUNA enables it for its 5Gbps USB3 link on this same DCU
            # (luna/gateware/interface/serdes_phy/ecp5.py: REQ_EN=1, REQ_LVL_SET=0b01, 9dB).
            # We had it OFF while receiving a real drive over a SATA cable.
            p_CHX_REQ_EN            = "0b1",    # Enable equalizer
            p_CHX_REQ_LVL_SET       = "0b01",   # Equalizer attenuation, 9 dB (LUNA value)
            p_CHX_RX_RATE_SEL       = "0d10",   # Equalizer  pole position
            p_CHX_RTERM_RX          = {
                "5k-ohms": "0d00",
                "80-ohms": "0d01",
                "75-ohms": "0d04",
                "70-ohms": "0d06",
                "60-ohms": "0d11",
                "50-ohms": "0d19",
                "46-ohms": "0d25"}["50-ohms"],
            p_CHX_RXIN_CM           = "0b11",   # CMFB (wizard value used)
            p_CHX_RXTERM_CM         = "0b10",   # Terminate RX to GND (LUNA value)
            p_CHX_RX_LOS_HYST_EN    = "0b0",    # (LUNA value)
            p_D_CDR_LOL_SET         = "0b10",   # +-4000ppm lock / +-7000ppm unlock (LUNA)

            # CHX RX ­— clocking
            i_CHX_RX_REFCLK         = pll.refclk,
            o_CHX_FF_RX_PCLK        = self.rxoutclk,
            i_CHX_FF_RXI_CLK        = ClockSignal("rx"),

            p_CHX_CDR_MAX_RATE      = "5.0",    # 5.0 Gbps
            p_CHX_RX_DCO_CK_DIV     = {
                32: "0b111",
                16: "0b110",
                 8: "0b101",
                 4: "0b100",
                 2: "0b010",
                 1: "0b000"}[pll.config["d"]],
            p_CHX_RX_GEAR_MODE      = "0b1",    # 1:2 gearbox
            p_CHX_FF_RX_H_CLK_EN    = "0b1",    # enable  DIV/2 output clock
            p_CHX_FF_RX_F_CLK_DIS   = "0b1",    # disable DIV/1 output clock
            p_CHX_SEL_SD_RX_CLK     = "0b1",    # FIFO driven by recovered clock

            p_CHX_AUTO_FACQ_EN      = "0b1",    # undocumented (wizard value used)
            p_CHX_AUTO_CALIB_EN     = "0b1",    # undocumented (wizard value used)
            # PDEN_SEL=1 disables the CDR phase detector whenever RLOS asserts. With a squelch
            # that reads "idle" on the device's post-OOB stream this deadlocks: LOS -> phase
            # detector off -> CDR never locks -> no data -> LOS stays asserted. Symptom is exactly
            # all-zero dwords with ZERO not-in-table errors, which is what we measure. Decoupled.
            p_CHX_PDEN_SEL          = "0b0",    # phase detector NOT gated by LOS

            p_CHX_DCOATDCFG         = "0b00",   # begin undocumented (sample code used)
            p_CHX_DCOATDDLY         = "0b00",
            p_CHX_DCOBYPSATD        = "0b1",
            p_CHX_DCOCALDIV         = "0b000",
            p_CHX_DCOCTLGI          = "0b011",
            p_CHX_DCODISBDAVOID     = "0b0",
            p_CHX_DCOFLTDAC         = "0b00",
            p_CHX_DCOFTNRG          = "0b001",
            p_CHX_DCOIOSTUNE        = "0b010",
            p_CHX_DCOITUNE          = "0b00",
            p_CHX_DCOITUNE4LSB      = "0b010",
            p_CHX_DCOIUPDNX2        = "0b1",
            p_CHX_DCONUOFLSB        = "0b100",
            p_CHX_DCOSCALEI         = "0b01",
            p_CHX_DCOSTARTVAL       = "0b010",
            p_CHX_DCOSTEP           = "0b11",   # end undocumented

            # CHX RX — loss of signal
            o_CHX_FFS_RLOS          = rx_los,
            p_CHX_RLOS_SEL          = "0b1",
            p_CHX_RX_LOS_EN         = "0b1",
            p_CHX_RX_LOS_LVL        = "0b{:03b}".format(rx_los_lvl), # Lattice "TBD" (wizard value 0b100;
                                                # lower = more sensitive, used for crosstalk hunting)
            p_CHX_RX_LOS_CEQ        = "0b11",   # Lattice "TBD" (wizard value used)

            # CHX RX — loss of lock
            o_CHX_FFS_RLOL          = rx_lol,

            # CHx_RXLSM? CHx_RXWA?

            # CHX RX — link state machine
            i_CHX_FFC_SIGNAL_DETECT = rx_align & (self.rx_prbs_config == 0),
            # RATE_MODE_RX/TX are 1-bit fuses that nextpnr DOES emit (see nextpnr
            # ecp5/dcu_bitstream.h "DCU.CH0_RATE_MODE_RX"), defaulting to 0 = full rate.
            # The FFC_ ports give dynamic control on top; the TX one is proven to change
            # the wire, the RX one appears to need the fuse set to take effect.
            p_CHX_RATE_MODE_RX      = rx_rate_mode,
            p_CHX_RATE_MODE_TX      = tx_rate_mode,
            i_CHX_FFC_RATE_MODE_TX  = self.rate_mode_tx,
            i_CHX_FFC_RATE_MODE_RX  = self.rate_mode_rx,
            o_CHX_FFS_LS_SYNC_STATUS= rx_lsm,
            p_CHX_ENABLE_CG_ALIGN   = "0b1",
            p_CHX_UDF_COMMA_MASK    = "0x3ff",  # compare all 10 bits
            p_CHX_UDF_COMMA_A       = "0x283",  # K28.5 inverted
            p_CHX_UDF_COMMA_B       = "0x17C",  # K28.5

            p_CHX_CTC_BYPASS        = "0b1",    # bypass CTC FIFO
            p_CHX_MIN_IPG_CNT       = "0b11",   # minimum interpacket gap of 4
            p_CHX_MATCH_2_ENABLE    = "0b0",    # 2 character skip matching
            p_CHX_MATCH_4_ENABLE    = "0b0",    # 4 character skip matching
            p_CHX_CC_MATCH_1        = "0x000",
            p_CHX_CC_MATCH_2        = "0x000",
            p_CHX_CC_MATCH_3        = "0x000",
            p_CHX_CC_MATCH_4        = "0x000",

            # CHX RX — data
            **{"o_CHX_FF_RX_D_%d" % n: rx_bus[n] for n in range(rx_bus.nbits)},

            # CHX transmit -------------------------------------------------------------------------
            # CHX TX — power management
            p_CHX_TPWDNB            = "0b1",
            i_CHX_FFC_TXPWDNB       = ~self.tx_pwdn,

            # CHX TX — reset
            i_D_FFC_TRST            = ~self.tx_enable | init.tx_rst,
            i_CHX_FFC_LANE_TX_RST   = ~self.tx_enable | init.pcs_rst | self.tx_lane_rst,

            # CHX TX - output
            o_CHX_HDOUTP            = tx_pads.p,
            o_CHX_HDOUTN            = tx_pads.n,

            p_CHX_TXAMPLITUDE       = "0d1000",  # 1000 mV
            p_CHX_RTERM_TX          = {
                "5k-ohms": "0d00",
                "80-ohms": "0d01",
                "75-ohms": "0d04",
                "70-ohms": "0d06",
                "60-ohms": "0d11",
                "50-ohms": "0d19",
                "46-ohms": "0d25"}["50-ohms"],

            p_CHX_TDRV_SLICE0_CUR   = "0b011",  # 400 uA
            p_CHX_TDRV_SLICE0_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE1_CUR   = "0b000",  # 100 uA
            p_CHX_TDRV_SLICE1_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE2_CUR   = "0b11",   # 3200 uA
            p_CHX_TDRV_SLICE2_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE3_CUR   = "0b10",   # 2400 uA
            p_CHX_TDRV_SLICE3_SEL   = "0b01",   # main data
            p_CHX_TDRV_SLICE4_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_SLICE4_SEL   = "0b00",   # power down
            p_CHX_TDRV_SLICE5_CUR   = "0b00",   # 800 uA
            p_CHX_TDRV_SLICE5_SEL   = "0b00",   # power down

            # CHX TX — clocking
            o_CHX_FF_TX_PCLK        = self.txoutclk,
            i_CHX_FF_TXI_CLK        = ClockSignal("tx"),

            p_CHX_TX_GEAR_MODE      = "0b1",    # 1:2 gearbox
            p_CHX_FF_TX_H_CLK_EN    = "0b1",    # enable  DIV/2 output clock
            p_CHX_FF_TX_F_CLK_DIS   = "0b1",    # disable DIV/1 output clock

            # CHX TX — data
            **{"i_CHX_FF_TX_D_%d" % n: tx_bus[n] for n in range(tx_bus.nbits)}
        )

        # OOB: optional DCU OOB hookups (separable for hardware debug/bisect).
        if "ei" in oob_config:
            # CHX TX - electrical idle (direct port; idle requests < ~220ns are swallowed entirely,
            # measured on hardware -> unusable for COMWAKE gaps, kept for long idle phases).
            self.serdes_params.update(
                i_CHX_FFC_EI_EN = ei_en,
            )
        if "pcie_ct" in oob_config:
            # CHX TX - PCIe electrical idle (FFC_PCIE_CT). Tested on hardware as a fast-EI
            # alternative for short OOB gaps: kills TX entirely in this 10BSER/bypassed-PCS
            # config, with or without p_CHX_PCIE_MODE=0b1. Kept for documentation only.
            self.serdes_params.update(
                i_CHX_FFC_PCIE_CT = ei_en,
            )
        else:
            # CHX TX - PCIe receiver detect: senses far-end RX termination through the AC coupling
            # caps (TX must be in electrical idle). Used to test TX-path continuity to the drive.
            self.serdes_params.update(
                i_CHX_FFC_PCIE_DET_EN = self.rx_det_en,
                i_CHX_FFC_PCIE_CT     = self.rx_det_ct,
                o_CHX_FFS_PCIE_DONE   = self.rx_det_done,
                o_CHX_FFS_PCIE_CON    = self.rx_det_con,
            )
        if "ldr_tx" in oob_config:
            # CHX TX - LDR direct pad drive (out-of-band burst generation, LUNA-style).
            self.serdes_params.update(
                p_CHX_LDR_CORE2TX_SEL    = "0b0", # Use FFC_LDR_CORE2TX_EN to enable OOB output.
                i_CHX_LDR_CORE2TX        = self.tx_oob_data,
                i_CHX_FFC_LDR_CORE2TX_EN = self.tx_oob_en,
            )
        if "ldr_rx" in oob_config:
            # CHX RX - LDR low-speed line observation (raw digitized view of the RX pair, usable
            # for fabric-side OOB burst/idle detection as an alternative to RLOS).
            self.serdes_params.update(
                p_CHX_LDR_RX2CORE_SEL = "0b1", # Enable low-speed out-of-band input.
                o_CHX_LDR_RX2CORE     = self.rx_oob_data,
            )

        # OOB: G8B10B PCS mode (DCU-internal 8b10b). Per TN-02206 8.25 the electrical idle enable
        # is a pipelined word-synchronous control in PCS-managed modes (idle achieved <20 UI after
        # the designated word) - unlike the slow asynchronous behavior measured in the 10BSER/UC
        # bypass configuration. Alignment: the link state machine must be disabled and the
        # edge-sensitive FFC_ENABLE_CGALIGN input pulsed to re-arm the word aligner (per LUNA).
        if pcs_mode == "bypass":
            # Word alignment for the raw 10BSER datapath. The bypass arm previously inherited only
            # the base parameters (ENABLE_CG_ALIGN=1, LSM left at its default, FFC_ENABLE_CGALIGN
            # undriven), i.e. it never got the campaign-26 configuration that made the aligner
            # actually lock in hybrid. With the link state machine enabled, the LSM owns the barrel
            # shifter and nothing else can move it; disabling it and re-arming on decode errors is
            # the arrangement proven to hold for 60s of continuous SYNC/ALIGN traffic.
            self.serdes_params.update(
                p_CHX_LSM_DISABLE        = "0b1",
                p_CHX_ENABLE_CG_ALIGN    = "0b1",
                i_CHX_FFC_ENABLE_CGALIGN = Mux(align_cont_rx, rx_align, cg_align_pulse),
            )
        if pcs_mode == "pcie_bypass":
            # OOB: EI-flag feature on top of the raw 10BSER bypass datapath: bits 11/23 are
            # unused in bypass gearing, so the per-byte EI flags can ride them if the feature
            # samples the bus in this mode (experiment: raw patterns give scope-visible content).
            self.serdes_params.update(
                p_CHX_PCIE_EI_EN = "0b1",
            )
        if pcs_mode == "hybrid":
            self.serdes_params.update(
                p_CHX_PROTOCOL           = "G8B10B",
                p_CHX_UC_MODE            = "0b0",
                p_CHX_ENC_BYPASS         = "0b1",  # TX: raw 10-bit words from fabric
                p_CHX_DEC_BYPASS         = "0b0",  # RX: DCU 8b10b decode + aligner
                # Word alignment: the DCU link state machine maintains it (LSM_DISABLE=0). The
                # static ENABLE_CG_ALIGN fuse must be set for the aligner to work at all: with it
                # cleared, pulsing the edge-sensitive FFC_ENABLE_CGALIGN input does nothing and the
                # barrel shifter stays wherever it landed. Loopback proof: transmitting a known-good
                # ALIGN stream came back decoded as FC35B5EE/k0001, which is bit-exactly a correct
                # 7B4A4ABC/k0001 read 3 bits off the word boundary (see bench/BRINGUP.md, and the
                # per-phase decode table there). align_cont selects continuous alignment (the
                # configuration that first reached READY) vs error-driven re-arm at runtime.
                # LSM_DISABLE=1 is what actually matters: with the link state machine ENABLED
                # (LSM_DISABLE=0) the LSM owns the barrel shifter and neither the static
                # ENABLE_CG_ALIGN fuse nor an FFC_ENABLE_CGALIGN pulse can move it - measured on a
                # clean continuous-ALIGN loopback, the aligner sat stably 3 bits off (FC35B5EE) for
                # five consecutive captures while the comma occurs, uniquely, only at phase 0.
                # LSM_DISABLE=1 is also what `g8b10b` uses, the mode that achieved the campaign-12
                # 3Gbps loopback self link-up, and liteiclink leaves the LSM at its default with
                # ENABLE_CG_ALIGN=1.
                p_CHX_LSM_DISABLE        = "0b1",
                p_CHX_ENABLE_CG_ALIGN    = "0b1",
                i_CHX_FFC_ENABLE_CGALIGN = Mux(align_cont_rx, rx_align, cg_align_pulse),
            )
        if pcs_mode in ["g8b10b", "pcie"]:
            self.serdes_params.update(
                p_CHX_PROTOCOL           = "G8B10B",
                p_CHX_UC_MODE            = "0b0",
                p_CHX_ENC_BYPASS         = "0b0",
                p_CHX_DEC_BYPASS         = "0b0",
                p_CHX_LSM_DISABLE        = "0b1",
                p_CHX_ENABLE_CG_ALIGN    = "0b0",
                i_CHX_FFC_ENABLE_CGALIGN = cg_align_pulse,
            )
            if pcs_mode == "pcie":
                # OOB: true PCIe protocol mode - the ONLY documented word-synchronous TX
                # electrical idle: per-byte EI flags ride the TX bus (bits 11/23, TN-02206
                # Table 7.3, <20UI to reach EI). FFC_PCIE_CT is NOT this (it is the receiver
                # detect strobe) and FFC_EI_EN is the slow asynchronous path.
                # Diamond PCIe reference recipe (pcie_2p5_100mhzrefclk.v): PCIE_MODE=1 with
                # PCIE_EI_EN=0. PCIE_EI_EN is NOT a feature enable - it is a STATIC force-idle
                # (the fuse image of SCI CH_02 bit 6, hardware-verified: setting it kills the TX,
                # clearing it via SCI revives it). The per-byte EI flags (bits 11/23) are the
                # dynamic mechanism and only have EI semantics in PCIe mode.
                self.serdes_params.update(
                    p_CHX_PCIE_MODE  = "0b1",
                    p_CHX_PCIE_EI_EN = "0b0",
                )
                # Keep the slow asynchronous FFC_EI_EN path OUT of the picture in this mode:
                # idle is requested exclusively through the word-synchronous flags.
                self.serdes_params["i_CHX_FFC_EI_EN"] = 0
            if pcie_mode:
                self.serdes_params.update(p_CHX_PCIE_MODE = "0b1")
            del self.serdes_params["i_CHX_FFC_SIGNAL_DETECT"]

        # OOB: TX driver boost (max slice currents, hearing-margin experiment: the drive's OOB
        # squelch shows marginal detection of our bursts at nominal amplitude).
        if tx_boost:
            self.serdes_params.update(
                p_CHX_TDRV_SLICE0_CUR = "0b111",
                p_CHX_TDRV_SLICE0_SEL = "0b01",
                p_CHX_TDRV_SLICE1_CUR = "0b111",
                p_CHX_TDRV_SLICE1_SEL = "0b01",
                p_CHX_TDRV_SLICE2_CUR = "0b11",
                p_CHX_TDRV_SLICE2_SEL = "0b01",
                p_CHX_TDRV_SLICE3_CUR = "0b11",
                p_CHX_TDRV_SLICE3_SEL = "0b01",
                p_CHX_TDRV_SLICE4_CUR = "0b11",
                p_CHX_TDRV_SLICE4_SEL = "0b01",
                p_CHX_TDRV_SLICE5_CUR = "0b11",
                p_CHX_TDRV_SLICE5_SEL = "0b01",
            )

        # SCI Reconfiguration ----------------------------------------------------------------------
        # OOB: reset released with tx_ready (not full init.ready) so polarity/cdr_hold writes work
        # while the RX side is still unlocked (i.e. during the OOB sequence).
        self.sci_reconfig = sci_reconfig = SerDesECP5SCIReconfig(self)
        self.comb += sci_reconfig.reset.eq(~self.init.tx_ready)
        self.comb += sci_reconfig.sci.dual_sel.eq(dual)
        self.comb += sci_reconfig.loopback.eq(self.loopback)
        self.comb += sci_reconfig.oob_gate_en.eq(self.sci_oob_gate_en)
        self.comb += sci_reconfig.oob_gate_lvl.eq(self.sci_oob_gate_lvl)
        self.comb += sci_reconfig.oob_burst_val.eq(self.sci_oob_burst_val)
        self.comb += sci_reconfig.oob_gap_val.eq(self.sci_oob_gap_val)
        self.comb += sci_reconfig.rx_polarity.eq(rx_polarity)
        self.comb += sci_reconfig.tx_polarity.eq(tx_polarity)
        self.comb += sci_reconfig.rx_cdr_hold.eq(self.rx_cdr_hold)

        # Debug observation taps (see bench analyzer group 0).
        self.comb += self.rx_bus_dbg.eq(rx_bus)
        self.comb += self.rx_lol_dbg.eq(rx_lol)

        # TX/RX Datapaths (and PRBS in bypass mode) ------------------------------------------------
        if pcs_mode in ["bypass", "pcie_bypass", "hybrid"]:
            self.tx_prbs = ClockDomainsRenamer("tx")(PRBSTX(data_width, reverse=True))
            self.comb += self.tx_prbs.config.eq(tx_prbs_config)
            self.comb += [
                self.tx_prbs.i.eq(Cat(*[self.encoder.output[i] for i in range(nwords)])),
                If(tx_produce_square_wave,
                    # square wave @ linerate/data_width for scope observation
                    tx_data.eq(Signal(data_width, reset=(1<<(data_width//2))-1))
                ).Elif(tx_produce_pattern,
                    tx_data.eq(Mux(self.tx_oob_gap, self.tx_pattern_gap,
                        Mux(pattern_alt_tx & pattern_toggle, ~tx_pattern, tx_pattern)))
                ).Else(
                    tx_data.eq(self.tx_prbs.o)
                ),
                tx_bus[ 0:10].eq(tx_data_r[ 0:10]),
                tx_bus[12:22].eq(tx_data_r[10:20]),
            ]
            # Pipeline the TX word: tx_data is the output of a deep combinational cone (square-wave
            # / OOB pattern / gap / pat_alt / PRBS muxing, several of whose selects come out of CDC
            # synchronizers). Driving DCUA.CH0_FF_TX_D_* straight from that cone missed setup at
            # 150MHz (nextpnr: 8.8ns against a 6.66ns budget, txoutclk capped at 127.75MHz), which
            # intermittently corrupts transmitted words. One tx-domain register costs a uniform
            # 6.7ns of TX latency - no relative skew between pattern, gap and data - and hands the
            # DCU a flop output.
            self.sync.tx += [
                tx_data_r.eq(tx_data),
                pattern_toggle.eq(~pattern_toggle),
            ]

            if pcs_mode == "hybrid":
                # RX comes from the DCU 8b10b decoder (the path proven to decode real 3Gbps data
                # during the loopback self link-up); only the TX side is raw.
                self.rx_word_data = Signal(nwords*8)
                self.rx_word_ctrl = Signal(nwords)
                self.rx_errs      = Signal(nwords)
                self.comb += [
                    self.rx_word_data[0: 8].eq(rx_bus[ 0: 8]),
                    self.rx_word_data[8:16].eq(rx_bus[12:20]),
                    self.rx_word_ctrl[0].eq(rx_bus[ 8]),
                    self.rx_word_ctrl[1].eq(rx_bus[20]),
                    self.rx_errs[0].eq(rx_bus[ 8] & (rx_bus[ 0: 8] == 0xEE)),
                    self.rx_errs[1].eq(rx_bus[20] & (rx_bus[12:20] == 0xEE)),
                ]
            else:
                # Raw RX word source. For bypass, run it through a FABRIC word aligner: the DCU
                # comma aligner does not operate on the raw 10BSER datapath - measured against a
                # real device ALIGN stream, the raw-bus word boundary drifts freely (B5/4A phase
                # mix with occasional clean K28.5s) and the decoded stream never contains a single
                # K character, while every DCU aligner knob (ENABLE_CG_ALIGN, LSM_DISABLE, FFC
                # pulse, continuous mode) measures neutral. Comma alignment is a G8B10B PCS
                # feature; in 10BSER it has to be done here. Scan a 40-bit sliding window for the
                # K28.5 comma7 (serial 0011111 = 0x7C LSB-first, or its complement 0x03) and
                # barrel-shift the datapath to the symbol boundary. Pipelined (window and slip
                # registered) so the 40->20 dynamic shift gets a full rx cycle.
                rx_raw_al = Signal(20)
                if pcs_mode == "bypass":
                    self.bp_aligner = bp_aligner = ClockDomainsRenamer("rx")(BypassWordAligner())
                    # Re-arm policy: scrambled payloads contain comma-like bit patterns, so an
                    # always-tracking aligner false-slips mid-frame (measured: starting IDENTIFY
                    # drops rx_ready - the drive's scrambled response garbles the boundary and
                    # ctrl tears the RX down). Freeze the slip once locked; re-arm only under
                    # sustained decode errors (leaky bucket) or a genuinely comma-free stretch
                    # (align_nocomma, runtime CSR - set it well above the 512-word device ALIGN
                    # period). A wrong slip floods invalids, so re-lock is self-healing.
                    bp_inv    = Signal()
                    bp_kseen  = Signal()
                    bp_nocom  = Signal(16)
                    bp_invcnt = Signal(6)
                    self.comb += [
                        bp_inv.eq(Cat(*[d.invalid for d in self.decoders]) != 0),
                        bp_kseen.eq(Cat(*[d.k for d in self.decoders]) != 0),
                    ]
                    # bp_invcnt decrement-on-valid was a trap: a MISALIGNED ALIGN stream decodes
                    # 3 plausible symbols per 1 invalid (e.g. FC,35,B5,EE), so a +1/-1 bucket never
                    # reaches the threshold and the aligner stays frozen at the wrong offset - the
                    # exact situation it must recover from. Saturate up on invalids and reset ONLY
                    # on a decoded K28.5 (the one symbol that proves the boundary is right).
                    bp_k285 = Signal()
                    self.comb += bp_k285.eq(
                        ((self.decoders[0].k == 1) & (self.decoders[0].d == 0xBC)) |
                        ((self.decoders[1].k == 1) & (self.decoders[1].d == 0xBC)))
                    # K28.5-age gate: the two failure modes pull opposite ways. A MISALIGNED
                    # ALIGN stream (link-up) decodes no true K28.5 and needs fast re-arm; the
                    # frame-exchange CONT junk (valid scrambled symbols, occasional invalids, no
                    # K28.5 between the drive's 256-dword ALIGN beacons ~3.4us apart) must NOT
                    # re-arm or the boundary is stolen mid-exchange and ctrl tears the link down
                    # (measured: identify -> status 0xa -> drive back to OOB). Gate every re-arm
                    # on "no K28.5 for >=1024 rx cycles (~6.8us)": genuine traffic refreshes the
                    # age via ALIGN beacons and stays locked; a wrong boundary never decodes
                    # K28.5, ages out in 7us, and unlocks the aligner.
                    bp_k285_age = Signal(11)
                    self.sync.rx += [
                        If(bp_kseen,
                            bp_nocom.eq(0)
                        ).Elif(bp_nocom != 0xFFFF,
                            bp_nocom.eq(bp_nocom + 1)
                        ),
                        If(bp_k285,
                            bp_k285_age.eq(0),
                            bp_invcnt.eq(0),
                        ).Else(
                            If(bp_k285_age != 2**11-1,
                                bp_k285_age.eq(bp_k285_age + 1)
                            ),
                            If(bp_inv & (bp_invcnt != 63),
                                bp_invcnt.eq(bp_invcnt + 1)
                            ),
                        ),
                    ]
                    self.comb += [
                        bp_aligner.enable.eq(rx_align & (bp_k285_age == 2**11-1) &
                            ((bp_invcnt >= 8) | (bp_nocom >= align_nocomma_rx))),
                        bp_aligner.sink.eq(Cat(rx_bus[0:10], rx_bus[12:22])),
                        rx_raw_al.eq(bp_aligner.source),
                    ]
                    # Debug observation (CSR-polled): slip, slip-change count, and a rolling count
                    # of rx words whose fabric decode contains a K character.
                    self.bp_slip_dbg   = Signal(5)
                    self.bp_slipmv_dbg = Signal(8)
                    self.bp_kcnt_dbg   = Signal(16)
                    self.comb += [
                        self.bp_slip_dbg.eq(bp_aligner.slip),
                        self.bp_slipmv_dbg.eq(bp_aligner.slip_mv),
                    ]
                    self.sync.rx += If(Cat(*[d.k for d in self.decoders]) != 0,
                        self.bp_kcnt_dbg.eq(self.bp_kcnt_dbg + 1)
                    )
                    # Stage-by-stage analyzer taps: aligned word, decoder outputs.
                    self.bp_src_dbg = Signal(20)
                    self.bp_dec_d   = Signal(16)
                    self.bp_dec_k   = Signal(2)
                    self.bp_dec_inv = Signal(2)
                    self.comb += [
                        self.bp_src_dbg.eq(bp_aligner.source),
                        self.bp_dec_d.eq(Cat(self.decoders[0].d, self.decoders[1].d)),
                        self.bp_dec_k.eq(Cat(self.decoders[0].k, self.decoders[1].k)),
                        self.bp_dec_inv.eq(Cat(self.decoders[0].invalid, self.decoders[1].invalid)),
                    ]
                else:
                    self.comb += rx_raw_al.eq(Cat(rx_bus[0:10], rx_bus[12:22]))
                self.rx_prbs = ClockDomainsRenamer("rx")(PRBSRX(data_width, reverse=True))
                self.comb += [
                    self.rx_prbs.config.eq(rx_prbs_config),
                    self.rx_prbs.pause.eq(rx_prbs_pause),
                    rx_prbs_errors.eq(self.rx_prbs.errors),
                    rx_data[ 0:10].eq(rx_raw_al[ 0:10]),
                    rx_data[10:20].eq(rx_raw_al[10:20]),
                ]
            if pcs_mode == "hybrid":
                # The DCU word aligner is edge-triggered via FFC_ENABLE_CGALIGN. Re-arming only on
                # 0xEE decode errors is not enough: mis-aligned data frequently decodes to
                # plausible symbols with no error marker (observed: a continuous stream of
                # F0B5A487/k0000, i.e. SYNC content with the comma never flagged), so the aligner
                # never gets re-armed and stays locked to the wrong boundary. Also re-arm
                # periodically until a real comma (K character) is actually being decoded.
                holdoff_h  = Signal(16)
                nocomma    = Signal(16)
                self.sync.rx += [
                    cg_align_pulse.eq(0),
                    If(self.rx_word_ctrl != 0,
                        nocomma.eq(0)
                    ).Else(
                        nocomma.eq(nocomma + 1)
                    ),
                    If(holdoff_h != 0,
                        holdoff_h.eq(holdoff_h - 1)
                    ).Elif(rx_align & ((self.rx_errs != 0) | (nocomma >= align_nocomma_rx)),
                        cg_align_pulse.eq(1),
                        holdoff_h.eq(align_holdoff_rx),
                    )
                ]
            if pcs_mode == "pcie_bypass":
                self.comb += [
                    tx_bus[11].eq(ei_en),
                    tx_bus[23].eq(ei_en),
                ]
            if pcs_mode != "hybrid":
                for i in range(nwords):
                    self.sync.rx += self.decoders[i].input.eq(rx_data[10*i:10*(i+1)])
                self.sync.rx += self.rx_prbs.i.eq(rx_data)

            if pcs_mode == "bypass":
                # Word-aligner re-arm for the raw datapath, mirroring the hybrid arm but sourced
                # from the FABRIC decoders (there is no DCU decode here, so no 0xEE marker): pulse
                # on any invalid symbol, and also after a spell with no K character at all, since
                # a wrong boundary frequently decodes to plausible symbols with nothing flagged.
                bp_holdoff = Signal(16)
                bp_nocomma = Signal(16)
                bp_invalid = Signal()
                bp_kseen   = Signal()
                self.comb += [
                    bp_invalid.eq(Cat(*[d.invalid for d in self.decoders]) != 0),
                    bp_kseen.eq(  Cat(*[d.k       for d in self.decoders]) != 0),
                ]
                self.sync.rx += [
                    cg_align_pulse.eq(0),
                    If(bp_kseen,
                        bp_nocomma.eq(0)
                    ).Else(
                        bp_nocomma.eq(bp_nocomma + 1)
                    ),
                    If(bp_holdoff != 0,
                        bp_holdoff.eq(bp_holdoff - 1)
                    ).Elif(rx_align & (bp_invalid | (bp_nocomma >= align_nocomma_rx)),
                        cg_align_pulse.eq(1),
                        bp_holdoff.eq(align_holdoff_rx),
                    )
                ]
        else:
            # OOB: G8B10B datapaths: 8-bit data + K flag per byte on the DCU bus (disparity bits
            # left at 0 = automatic running disparity).
            self.tx_word_data = Signal(nwords*8)
            self.tx_word_ctrl = Signal(nwords)
            self.rx_word_data = Signal(nwords*8)
            self.rx_word_ctrl = Signal(nwords)
            self.comb += [
                If(tx_produce_pattern,
                    # zero_bus support: raw zeros (D0.0, K=0) during electrical idle.
                    tx_bus.eq(0),
                ).Else(
                    tx_bus[ 0: 8].eq(self.tx_word_data[0: 8]),
                    tx_bus[    8].eq(self.tx_word_ctrl[0]),
                    tx_bus[12:20].eq(self.tx_word_data[8:16]),
                    tx_bus[   20].eq(self.tx_word_ctrl[1]),
                ),
                self.rx_word_data[0: 8].eq(rx_bus[ 0: 8]),
                self.rx_word_data[8:16].eq(rx_bus[12:20]),
                self.rx_word_ctrl[0].eq(rx_bus[ 8]),
                self.rx_word_ctrl[1].eq(rx_bus[20]),
                self.rx_errs[0].eq(rx_bus[ 8] & (rx_bus[ 0: 8] == 0xEE)),
                self.rx_errs[1].eq(rx_bus[20] & (rx_bus[12:20] == 0xEE)),
            ]
            if pcs_mode == "pcie":
                # Word-synchronous electrical idle request per geared byte (tx domain, same
                # expression that drives FFC_EI_EN in the other modes).
                self.comb += [
                    tx_bus[11].eq(ei_en),
                    tx_bus[23].eq(ei_en),
                ]
            # Word-aligner re-arm: pulse the edge-sensitive CGALIGN input on decode errors,
            # with a holdoff to let the barrel shifter settle.
            holdoff = Signal(8)
            self.sync.rx += [
                cg_align_pulse.eq(0),
                If(holdoff != 0,
                    holdoff.eq(holdoff - 1)
                ).Elif(rx_align & (self.rx_errs != 0),
                    cg_align_pulse.eq(1),
                    holdoff.eq(255),
                )
            ]

    def add_stream_endpoints(self):
        self.sink   =   sink = stream.Endpoint([("data", self.nwords*8), ("ctrl", self.nwords)])
        self.source = source = stream.Endpoint([("data", self.nwords*8), ("ctrl", self.nwords)])

        self.comb += sink.ready.eq(1)
        self.comb += source.valid.eq(1)
        if self.pcs_mode == "hybrid":
            for i in range(self.nwords):
                self.comb += [
                    self.encoder.k[i].eq(sink.ctrl[i]),
                    self.encoder.d[i].eq(sink.data[8*i:8*(i+1)]),
                ]
            self.comb += [
                source.data.eq(self.rx_word_data),
                source.ctrl.eq(self.rx_word_ctrl),
            ]
        elif self.pcs_mode in ["bypass", "pcie_bypass"]:
            for i in range(self.nwords):
                self.comb += [
                    self.encoder.k[i].eq(sink.ctrl[i]),
                    self.encoder.d[i].eq(sink.data[8*i:8*(i+1)]),
                    source.ctrl[i].eq(self.decoders[i].k),
                    source.data[8*i:8*(i+1)].eq(self.decoders[i].d),
                ]
        else:
            self.comb += [
                self.tx_word_data.eq(sink.data),
                self.tx_word_ctrl.eq(sink.ctrl),
                source.data.eq(self.rx_word_data),
                source.ctrl.eq(self.rx_word_ctrl),
            ]

    def add_base_control(self, auto_enable=True):
        self._tx_enable = CSRStorage(fields=[
                CSRField("enable", size=1, values=[
                    ("``0b0``", "TX disabled."),
                    ("``0b1``", "TX enabled.")
                ], reset=int(auto_enable))
            ])
        self._tx_ready = CSRStatus(fields=[
                CSRField("ready", size=1, values=[
                    ("``0b0``", "TX not initialized."),
                    ("``0b1``", "TX initialized and ready.")
                ])
            ])
        self._tx_inhibit = CSRStorage(fields=[
                CSRField("inhibit", size=1, values=[
                    ("``0b0``", "Normal operation."),
                    ("``0b1``", "TX inhibited.")
                ])
            ])
        self._tx_produce_square_wave = CSRStorage(fields=[
                CSRField("enable", size=1, values=[
                    ("``0b0``", "Normal operation."),
                    ("``0b1``", "TX square wave generation enabled (linerate observation/checks).")
                ])
            ])
        self._rx_enable = CSRStorage(fields=[
                CSRField("enable", size=1, values=[
                    ("``0b0``", "RX disabled."),
                    ("``0b1``", "RX enabled.")
                ], reset=int(auto_enable))
            ])
        self._rx_ready = CSRStatus(fields=[
                CSRField("ready", size=1, values=[
                    ("``0b0``", "RX not initialized."),
                    ("``0b1``", "RX initialized and ready.")
                ])
            ])
        self.comb += [
            self.tx_enable.eq(self._tx_enable.fields.enable),
            self._tx_ready.fields.ready.eq(self.tx_ready),
            self.tx_inhibit.eq(self._tx_inhibit.fields.inhibit),
            self.tx_produce_square_wave.eq(self._tx_produce_square_wave.fields.enable),
            self.rx_enable.eq(self._rx_enable.fields.enable),
            self._rx_ready.fields.ready.eq(self.rx_ready),
        ]

    def add_prbs_control(self, rx_errors_width=32):
        self._tx_prbs_config = CSRStorage(fields=[
            CSRField("config", size=2, values=[
                ("``0b00``", "PRBS   Disabled."),
                ("``0b01``", "PRBS7  Enabled."),
                ("``0b10``", "PRBS15 Enabled."),
                ("``0b11``", "PRBS31 Enabled."),
            ])
        ])
        self._rx_prbs_config = CSRStorage(fields=[
            CSRField("config", size=2, values=[
                ("``0b00``", "PRBS   Disabled."),
                ("``0b01``", "PRBS7  Enabled."),
                ("``0b10``", "PRBS15 Enabled."),
                ("``0b11``", "PRBS31 Enabled."),
            ]),
            CSRField("pause", size=1, description="Pause RX PRBS."),
        ])
        self._rx_prbs_errors = CSRStatus(rx_errors_width, description="RX PRBS errors.")
        self.comb += [
            self.tx_prbs_config.eq(self._tx_prbs_config.fields.config),
            self.rx_prbs_config.eq(self._rx_prbs_config.fields.config),
            self.rx_prbs_pause.eq(self._rx_prbs_config.fields.pause),
            self._rx_prbs_errors.status.eq(self.rx_prbs_errors),
        ]

    def add_loopback_control(self):
        self._loopback = CSRStorage()
        self.comb += self.loopback.eq(self._loopback.storage)


    def add_controls(self, auto_enable=True, rx_prbs_errors_width=32):
        self.add_base_control(auto_enable)
        self.add_prbs_control(rx_errors_width=rx_prbs_errors_width)
        self.add_loopback_control()

    def add_clock_cycles(self):
        self.clock_latch    = CSRStorage(description="Write to latch TX/RX clock cycles")
        self.clock_tx_cycles = CSRStorage(32, description="TX clock cycles")
        self.clock_rx_cycles = CSRStorage(32, description="RX clock cycles")

        tx_cycles = Signal(32)
        rx_cycles = Signal(32)
        self.sync.tx += tx_cycles.eq(tx_cycles + 1)
        self.sync.rx += rx_cycles.eq(rx_cycles + 1)

        self.sync += If(self.clock_latch.wr_stb,
            self.clock_tx_cycles.storage.eq(tx_cycles),
            self.clock_rx_cycles.storage.eq(rx_cycles),
        )

    def do_finalize(self):
        serdes_params = dict()
        for k, v in self.serdes_params.items():
            k = k.replace("CHX", "CH{}".format(self.channel))
            serdes_params[k] = v
        self.specials.dcu0 = Instance("DCUA", **serdes_params)
        self.dcu0.attr.add(("LOC", "DCU{}".format(self.dual)))
        self.dcu0.attr.add(("CHAN", "CH{}".format(self.channel)))
