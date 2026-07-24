// Auto-generated DCUA testbench (g8b10b mode) - LiteSATA ECP5 OOB investigation.
`timescale 1ps/1ps
module tb_g8b10b;

  reg  refclk = 0;
  reg  rst = 1, tx_rst = 1, rx_rst = 1, pcs_rst = 1;
  reg  ei_en = 1, ldr_en = 0, ldr_data = 0;
  reg  [23:0] tx_d = 0;
  wire [23:0] rx_d;
  wire tx_pclk, rx_pclk, hdoutp, hdoutn, ldr_rx, rlos, rlol, plol, lsm;
  reg  hdinp = 0, hdinn = 1;
  reg  [3:0] phase = 0;

  // 150MHz refclk.
  always #3333 refclk = ~refclk;

  DCUA #(
        .CH0_AUTO_CALIB_EN("0b1"),
        .CH0_AUTO_FACQ_EN("0b1"),
        .CH0_CC_MATCH_1("0x000"),
        .CH0_CC_MATCH_2("0x000"),
        .CH0_CC_MATCH_3("0x000"),
        .CH0_CC_MATCH_4("0x000"),
        .CH0_CDR_MAX_RATE("5.0"),
        .CH0_CTC_BYPASS("0b1"),
        .CH0_DCOATDCFG("0b00"),
        .CH0_DCOATDDLY("0b00"),
        .CH0_DCOBYPSATD("0b1"),
        .CH0_DCOCALDIV("0b000"),
        .CH0_DCOCTLGI("0b011"),
        .CH0_DCODISBDAVOID("0b0"),
        .CH0_DCOFLTDAC("0b00"),
        .CH0_DCOFTNRG("0b001"),
        .CH0_DCOIOSTUNE("0b010"),
        .CH0_DCOITUNE("0b00"),
        .CH0_DCOITUNE4LSB("0b010"),
        .CH0_DCOIUPDNX2("0b1"),
        .CH0_DCONUOFLSB("0b100"),
        .CH0_DCOSCALEI("0b01"),
        .CH0_DCOSTARTVAL("0b010"),
        .CH0_DCOSTEP("0b11"),
        .CH0_DEC_BYPASS("0b0"),
        .CH0_ENABLE_CG_ALIGN("0b0"),
        .CH0_ENC_BYPASS("0b0"),
        .CH0_FF_RX_F_CLK_DIS("0b1"),
        .CH0_FF_RX_H_CLK_EN("0b1"),
        .CH0_FF_TX_F_CLK_DIS("0b1"),
        .CH0_FF_TX_H_CLK_EN("0b1"),
        .CH0_LDR_CORE2TX_SEL("0b0"),
        .CH0_LDR_RX2CORE_SEL("0b1"),
        .CH0_LSM_DISABLE("0b1"),
        .CH0_MATCH_2_ENABLE("0b0"),
        .CH0_MATCH_4_ENABLE("0b0"),
        .CH0_MIN_IPG_CNT("0b11"),
        .CH0_PDEN_SEL("0b1"),
        .CH0_PROTOCOL("G8B10B"),
        .CH0_REQ_EN("0b0"),
        .CH0_RLOS_SEL("0b1"),
        .CH0_RPWDNB("0b1"),
        .CH0_RTERM_RX("0d19"),
        .CH0_RTERM_TX("0d19"),
        .CH0_RXIN_CM("0b11"),
        .CH0_RXTERM_CM("0b11"),
        .CH0_RX_DCO_CK_DIV("0b000"),
        .CH0_RX_GEAR_MODE("0b1"),
        .CH0_RX_LOS_CEQ("0b11"),
        .CH0_RX_LOS_EN("0b1"),
        .CH0_RX_LOS_LVL("0b100"),
        .CH0_RX_RATE_SEL("0d10"),
        .CH0_SEL_SD_RX_CLK("0b1"),
        .CH0_TDRV_SLICE0_CUR("0b011"),
        .CH0_TDRV_SLICE0_SEL("0b01"),
        .CH0_TDRV_SLICE1_CUR("0b000"),
        .CH0_TDRV_SLICE1_SEL("0b00"),
        .CH0_TDRV_SLICE2_CUR("0b11"),
        .CH0_TDRV_SLICE2_SEL("0b01"),
        .CH0_TDRV_SLICE3_CUR("0b10"),
        .CH0_TDRV_SLICE3_SEL("0b01"),
        .CH0_TDRV_SLICE4_CUR("0b00"),
        .CH0_TDRV_SLICE4_SEL("0b00"),
        .CH0_TDRV_SLICE5_CUR("0b00"),
        .CH0_TDRV_SLICE5_SEL("0b00"),
        .CH0_TPWDNB("0b1"),
        .CH0_TXAMPLITUDE("0d1000"),
        .CH0_TX_GEAR_MODE("0b1"),
        .CH0_UC_MODE("0b0"),
        .CH0_UDF_COMMA_A("0x283"),
        .CH0_UDF_COMMA_B("0x17C"),
        .CH0_UDF_COMMA_MASK("0x3ff"),
        .D_BITCLK_LOCAL_EN("0b1"),
        .D_CMUSETBIASI("0b00"),
        .D_CMUSETI4CPP("0d3"),
        .D_CMUSETI4CPZ("0d3"),
        .D_CMUSETI4VCO("0b00"),
        .D_CMUSETICP4P("0b01"),
        .D_CMUSETICP4Z("0b101"),
        .D_CMUSETINITVCT("0b00"),
        .D_CMUSETISCL4VCO("0b000"),
        .D_CMUSETP1GM("0b000"),
        .D_CMUSETP2AGM("0b000"),
        .D_CMUSETZGM("0b000"),
        .D_HIGH_MARK("0d12"),
        .D_IB_PWDNB("0b1"),
        .D_LOW_MARK("0d4"),
        .D_MACROPDB("0b1"),
        .D_PD_ISET("0b11"),
        .D_REFCK_MODE("0b000"),
        .D_REQ_ISET("0b011"),
        .D_RG_EN("0b0"),
        .D_RG_SET("0b00"),
        .D_SETICONST_AUX("0b01"),
        .D_SETICONST_CH("0b10"),
        .D_SETIRPOLY_AUX("0b01"),
        .D_SETIRPOLY_CH("0b01"),
        .D_SETPLLRC("0d1"),
        .D_SYNC_LOCAL_EN("0b1"),
        .D_TXPLL_PWDNB("0b1"),
        .D_TX_MAX_RATE("5.0"),
        .D_TX_VCO_CK_DIV("0b000")
  ) dut (
        .CH0_FFC_EI_EN(ei_en),
        .CH0_FFC_ENABLE_CGALIGN(1'b0),
        .CH0_FFC_LANE_RX_RST(pcs_rst),
        .CH0_FFC_LANE_TX_RST(pcs_rst),
        .CH0_FFC_LDR_CORE2TX_EN(ldr_en),
        .CH0_FFC_RRST(rx_rst),
        .CH0_FFC_RXPWDNB(1'b1),
        .CH0_FFC_TXPWDNB(1'b1),
        .CH0_FF_RXI_CLK(rx_pclk),
        .CH0_FF_TXI_CLK(tx_pclk),
        .CH0_FF_TX_D_0(tx_d[0]),
        .CH0_FF_TX_D_1(tx_d[1]),
        .CH0_FF_TX_D_10(tx_d[10]),
        .CH0_FF_TX_D_11(tx_d[11]),
        .CH0_FF_TX_D_12(tx_d[12]),
        .CH0_FF_TX_D_13(tx_d[13]),
        .CH0_FF_TX_D_14(tx_d[14]),
        .CH0_FF_TX_D_15(tx_d[15]),
        .CH0_FF_TX_D_16(tx_d[16]),
        .CH0_FF_TX_D_17(tx_d[17]),
        .CH0_FF_TX_D_18(tx_d[18]),
        .CH0_FF_TX_D_19(tx_d[19]),
        .CH0_FF_TX_D_2(tx_d[2]),
        .CH0_FF_TX_D_20(tx_d[20]),
        .CH0_FF_TX_D_21(tx_d[21]),
        .CH0_FF_TX_D_22(tx_d[22]),
        .CH0_FF_TX_D_23(tx_d[23]),
        .CH0_FF_TX_D_3(tx_d[3]),
        .CH0_FF_TX_D_4(tx_d[4]),
        .CH0_FF_TX_D_5(tx_d[5]),
        .CH0_FF_TX_D_6(tx_d[6]),
        .CH0_FF_TX_D_7(tx_d[7]),
        .CH0_FF_TX_D_8(tx_d[8]),
        .CH0_FF_TX_D_9(tx_d[9]),
        .CH0_HDINN(hdinn),
        .CH0_HDINP(hdinp),
        .CH0_LDR_CORE2TX(ldr_data),
        .CH0_RX_REFCLK(refclk),
        .CH0_SCIEN(1'b0),
        .CH0_SCISEL(1'b0),
        .D_FFC_DUAL_RST(rst),
        .D_FFC_MACROPDB(1'b1),
        .D_FFC_MACRO_RST(rst),
        .D_FFC_TRST(tx_rst),
        .D_REFCLKI(refclk),
        .D_SCIADDR0(1'b0),
        .D_SCIADDR1(1'b0),
        .D_SCIADDR2(1'b0),
        .D_SCIADDR3(1'b0),
        .D_SCIADDR4(1'b0),
        .D_SCIADDR5(1'b0),
        .D_SCIENAUX(1'b0),
        .D_SCIRD(1'b0),
        .D_SCISELAUX(1'b0),
        .D_SCIWDATA0(1'b0),
        .D_SCIWDATA1(1'b0),
        .D_SCIWDATA2(1'b0),
        .D_SCIWDATA3(1'b0),
        .D_SCIWDATA4(1'b0),
        .D_SCIWDATA5(1'b0),
        .D_SCIWDATA6(1'b0),
        .D_SCIWDATA7(1'b0),
        .D_SCIWSTN(1'b1),
        .CH0_FFS_LS_SYNC_STATUS(lsm),
        .CH0_FFS_RLOL(rlol),
        .CH0_FFS_RLOS(rlos),
        .CH0_FF_RX_D_0(rx_d[0]),
        .CH0_FF_RX_D_1(rx_d[1]),
        .CH0_FF_RX_D_10(rx_d[10]),
        .CH0_FF_RX_D_11(rx_d[11]),
        .CH0_FF_RX_D_12(rx_d[12]),
        .CH0_FF_RX_D_13(rx_d[13]),
        .CH0_FF_RX_D_14(rx_d[14]),
        .CH0_FF_RX_D_15(rx_d[15]),
        .CH0_FF_RX_D_16(rx_d[16]),
        .CH0_FF_RX_D_17(rx_d[17]),
        .CH0_FF_RX_D_18(rx_d[18]),
        .CH0_FF_RX_D_19(rx_d[19]),
        .CH0_FF_RX_D_2(rx_d[2]),
        .CH0_FF_RX_D_20(rx_d[20]),
        .CH0_FF_RX_D_21(rx_d[21]),
        .CH0_FF_RX_D_22(rx_d[22]),
        .CH0_FF_RX_D_23(rx_d[23]),
        .CH0_FF_RX_D_3(rx_d[3]),
        .CH0_FF_RX_D_4(rx_d[4]),
        .CH0_FF_RX_D_5(rx_d[5]),
        .CH0_FF_RX_D_6(rx_d[6]),
        .CH0_FF_RX_D_7(rx_d[7]),
        .CH0_FF_RX_D_8(rx_d[8]),
        .CH0_FF_RX_D_9(rx_d[9]),
        .CH0_FF_RX_PCLK(rx_pclk),
        .CH0_FF_TX_PCLK(tx_pclk),
        .CH0_HDOUTN(hdoutn),
        .CH0_HDOUTP(hdoutp),
        .CH0_LDR_RX2CORE(ldr_rx),
        .D_FFS_PLOL(plol),
        .D_SCIRDATA0(),
        .D_SCIRDATA1(),
        .D_SCIRDATA2(),
        .D_SCIRDATA3(),
        .D_SCIRDATA4(),
        .D_SCIRDATA5(),
        .D_SCIRDATA6(),
        .D_SCIRDATA7()
  );

  // D10.2-style TX content (0x4A bytes, K=0).
  always @(posedge tx_pclk) begin
    tx_d <= {3'b000, 1'b0, 8'h4A, 3'b000, 1'b0, 8'h4A};
  end

  // LDR 75MHz square (toggle every tx_pclk at 150MHz).
  always @(posedge tx_pclk) ldr_data <= ~ldr_data;

  task gap(input integer ns);  // EI-request gap of <ns>, then 107ns burst window (EI off).
    begin
      ei_en = 1; #(ns*1000);
      ei_en = 0; #107000;
    end
  endtask

  initial begin
    $dumpfile("tb_g8b10b.vcd");
    $dumpvars(1, tb_g8b10b);
    // Reset sequence (mimics SerdesInit).
    #2000000  rst = 0; tx_rst = 0;
    #2000000  rx_rst = 0;
    #2000000  pcs_rst = 0;
    #20000000 ;                 // 20us settle.

    // S1: EI step response.
    $display("S1 EI step @%0t", $time);
    phase = 1; ei_en = 0; #2000000 ei_en = 1; #2000000;

    // S2: gap-request sweep 53/107/160/213/320ns between 107ns EI-off windows.
    $display("S2 gap sweep @%0t", $time);
    phase = 2; ei_en = 0; #1000000;
    gap(53); gap(107); gap(160); gap(213); gap(320);
    ei_en = 1; #1000000;

    // S3: LDR bursts through held EI (mute test).
    $display("S3 LDR vs held EI @%0t", $time);
    phase = 3; ei_en = 1; #500000;
    repeat (6) begin ldr_en = 1; #107000 ldr_en = 0; #107000; end
    #1000000;

    // S4: masked-EI COMWAKE (ei_en = ~ldr_en pattern).
    $display("S4 masked COMWAKE @%0t", $time);
    phase = 4;
    repeat (6) begin ei_en = 0; ldr_en = 1; #107000 ldr_en = 0; ei_en = 1; #107000; end
    #2000000;
    $display("DONE @%0t", $time);
    $finish;
  end
endmodule
