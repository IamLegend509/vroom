// Replays ctrl_model.py ticks through rl_ctrl and logs every output and every
// actor-input word; run.py compares them with the model (bit-exact).
`timescale 1ns/1ps
module tb_rl_ctrl;
  reg clk = 0, rst = 1; always #5 clk = ~clk;
  reg tick_go = 0, init = 0;
  reg [11:0] th; reg signed [31:0] w, rf; reg [15:0] adc;
  reg dneg, sneg; reg [15:0] off, zero; reg [16:0] gain; reg [1:0] ph;
  wire busy, done, overrun; wire [12:0] da, db, dc;
  wire signed [31:0] ia, id, iq, iqr; wire [15:0] e16;
  rl_ctrl dut (.clk(clk), .rst(rst), .tick_go(tick_go), .th(th), .w_in(w), .adc_sum(adc), .ref_in(rf),
    .cal_dir_neg(dneg), .cal_sign_neg(sneg), .cal_off(off), .cal_zero(zero), .cal_gain(gain), .cal_phase(ph), .cal_mode(1'b0), .cal_e16(16'd0), .cal_n(16'sd0),
    .init(init), .init_ref(32'sd0), .busy(busy), .done(done), .duty_a(da), .duty_b(db), .duty_c(dc),
    .overrun(overrun), .dbg_ia(ia), .dbg_id(id), .dbg_iq(iq), .dbg_iqref(iqr), .dbg_e16(e16));
  integer fi, fo, fx, fy, n, k, a0, a1, a2, a3, a4, a5, nt, maxc, c0;
  always @(posedge clk) begin
    if (dut.iwe) $fdisplay(fx, "%0d", $signed(dut.xdata));
    if (dut.owe) $fdisplay(fy, "%0d", $signed(dut.xdata));
  end
  initial begin
    fi = $fopen("ctrl_cal.txt", "r"); k = $fscanf(fi, "%d %d %d %d %d %d %d", a0, a1, a2, a3, a4, nt, a5); $fclose(fi); ph = a5;
    dneg = a0; off = a1; zero = a2; sneg = a3; gain = a4;
    fi = $fopen("ctrl_in.txt", "r"); fo = $fopen("rtl_ctrl_out.txt", "w");
    fx = $fopen("rtl_ctrl_xi.txt", "w"); fy = $fopen("rtl_ctrl_xo.txt", "w");
    repeat (4) @(posedge clk); rst = 0;
    @(negedge clk) init = 1; @(negedge clk) init = 0;
    maxc = 0;
    for (n = 0; n < nt; n = n + 1) begin
      k = $fscanf(fi, "%d %d %d %d", a0, a1, a2, a3);
      @(negedge clk); th = a0; w = a1; adc = a2; rf = a3; tick_go = 1; c0 = $time;
      @(negedge clk); tick_go = 0;
      while (!done) @(posedge clk);
      if (($time - c0) / 10 > maxc) maxc = ($time - c0) / 10;
      $fdisplay(fo, "%0d %0d %0d %0d %0d %0d %0d %0d", e16, ia, id, iq, iqr, da, db, dc);
    end
    $fclose(fo); $fclose(fx); $fclose(fy);
    $display("CTRL_TB_DONE ticks=%0d max_cycles=%0d overrun=%0d", nt, maxc, overrun);
    $finish;
  end
endmodule
