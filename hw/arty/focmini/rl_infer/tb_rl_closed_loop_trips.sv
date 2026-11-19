// Top-level trip test for rl_closed_loop_top (iverilog), with stub XADC/AS5600 models.
//
// Purpose: exercise the ARM -> calibration -> RUN path with the real rl_ctrl/rl_dual_mlp and
// check calibration (direction measured, stuck rotor refused with q=14) and the run-time trips: no false trip while healthy, observer overcurrent -> q=12,
// overspeed -> q=13, keepalive loss -> q=8, and that PWM/enable drop in FAULT.
// Timing parameters are scaled down (1 ms = 1,000 cycles, keepalive 30,000 cycles); the RL
// tick stays at 10,000 cycles so the controller's real cycle budget is preserved.
// Needs the generated weight/sin/tanh memories in the working directory (run_closed_loop.py).
`timescale 1ns/1ps

module diag_xadc_a0 (input wire clk, rst, vaux4_p, vaux4_n,
                     output reg [11:0] raw = 12'd1185, output reg fresh = 1'b1, output reg clipped = 1'b0);
endmodule

module as5600_reader #(parameter integer CLK_HZ = 100000000, parameter integer I2C_HZ = 400000) (
  input wire clk, rst, start, scl_in, sda_in, output wire scl_low, sda_low,
  output reg busy = 0, done = 0, ok = 1, output reg [3:0] error = 0, output reg [7:0] status = 8'h20,
  output reg [11:0] angle = 12'd2000, output wire [1:0] bus_level);
  assign scl_low = 0; assign sda_low = 0; assign bus_level = 2'b11;
  // Rotor model: during calibration the rotor follows the commanded electrical angle
  // (motor_mode 0: counts decrease with e16, 1: counts increase, 2: rotor stuck).
  // Outside calibration it holds its position. 112 e16 per count (7 pole pairs).
  reg [15:0] last_e16 = 0;
  reg signed [31:0] acc = 0;
  always @(posedge clk) begin
    done <= start;
    if (start) begin
      if (tb_rl_closed_loop_trips.dut.cal_mode && tb_rl_closed_loop_trips.dut.state == 5) begin
        acc = acc + $signed(16'(tb_rl_closed_loop_trips.dut.cal_e16 - last_e16));
        last_e16 <= tb_rl_closed_loop_trips.dut.cal_e16;
        case (tb_rl_closed_loop_trips.motor_mode)
          0: angle <= 12'(2000 - acc / 112);
          1: angle <= 12'(2000 + acc / 112);
          default: angle <= 12'd2000;
        endcase
      end else begin acc = 0; last_e16 <= 0; end
    end
  end
endmodule

module tb_rl_closed_loop_trips;
  reg clk = 0; always #5 clk = ~clk;
  reg btn = 0;
  wire pwm_a, pwm_b, pwm_c, enable, reset_n, sleep_n, uart_tx;
  wire [3:0] led;
  wire scl, sda;
  rl_closed_loop_top #(.POLL_CYCLES(2000), .TELEMETRY_CYCLES(1000000), .WAKE_CYCLES(100),
                       .MS_CYCLES(1000), .KEEPALIVE_CYCLES(30000), .ARM_CYCLES(100000)) dut (
    .clk(clk), .reset_button(btn), .fault_n(1'b1), .vaux4_p(1'b0), .vaux4_n(1'b0), .uart_rx(1'b1),
    .as5600_scl(scl), .as5600_sda(sda), .pwm_a(pwm_a), .pwm_b(pwm_b), .pwm_c(pwm_c),
    .enable(enable), .reset_n(reset_n), .sleep_n(sleep_n), .uart_tx(uart_tx), .led(led));

  integer fails = 0, i;
  reg [1:0] motor_mode = 0;
  reg pinging = 0;
  // host keepalive every 10,000 cycles while `pinging`
  always begin
    #(100000);
    if (pinging) begin force dut.ping = 1'b1; @(posedge clk); #1 release dut.ping; end
  end

  task pulse_arm; begin force dut.arm = 1'b1; @(posedge clk); #1 release dut.arm; end endtask
  task pulse_sweep; begin force dut.sweep = 1'b1; @(posedge clk); #1 release dut.sweep; end endtask
  task do_reset; begin btn = 1; repeat (4) @(posedge clk); btn = 0; repeat (70000) @(posedge clk); end endtask

  task start_run; begin
    pinging = 1;
    pulse_arm;
    repeat (400) @(posedge clk);
    if (dut.state != 3) begin $display("FAIL: not ARMED (state=%0d q=%0d)", dut.state, dut.result); fails = fails + 1; end
    pulse_sweep;
    // calibration completes at elapsed_ms = 3990 (3,990,000 cycles scaled)
    while (dut.state == 4) @(posedge clk);
    while (!dut.calibrated && dut.state == 5) @(posedge clk);
    if (!dut.calibrated && motor_mode != 2) begin $display("FAIL: calibration not reached (state=%0d q=%0d)", dut.state, dut.result); fails = fails + 1; end
    repeat (50000) @(posedge clk);  // five RL ticks after calibration
  end endtask

  task expect_state(input [2:0] st, input [3:0] q, input [8*24-1:0] what); begin
    if (dut.state !== st || dut.result !== q) begin
      $display("FAIL %0s: state=%0d q=%0d (expected state=%0d q=%0d)", what, dut.state, dut.result, st, q);
      fails = fails + 1;
    end else $display("PASS %0s: state=%0d q=%0d enable=%0d pwm=%0d%0d%0d", what, dut.state, dut.result,
                      enable, pwm_a, pwm_b, pwm_c);
  end endtask

  initial begin
    do_reset;
    // A: healthy run, no false trip, PWM enabled
    start_run;
    expect_state(5, 0, "healthy RUN");
    if (!enable) begin $display("FAIL: enable low in RUN"); fails = fails + 1; end
    // A2: a dead/clipped ADC input must not trip (sensor unused)
    force dut.adc_clip = 1'b1; force dut.adc_fresh = 1'b0; repeat (20000) @(posedge clk);
    release dut.adc_clip; release dut.adc_fresh;
    expect_state(5, 0, "clipped ADC ignored");
    // B: observer overcurrent
    force dut.dbg_iq = 32'sd16385; repeat (5) @(posedge clk); release dut.dbg_iq;
    expect_state(6, 12, "observer overcurrent");
    if (enable || pwm_a || pwm_b || pwm_c) begin $display("FAIL: outputs active in FAULT"); fails = fails + 1; end
    // C: overspeed
    do_reset; start_run;
    force dut.w_f = -32'sd22757; repeat (5) @(posedge clk); release dut.w_f;
    expect_state(6, 13, "overspeed");
    // D: at-limit values must not trip
    do_reset; start_run;
    force dut.dbg_id = -32'sd16384; force dut.w_f = 32'sd22756; repeat (20) @(posedge clk);
    release dut.dbg_id; release dut.w_f;
    expect_state(5, 0, "at-limit no trip");
    // E: keepalive loss
    pinging = 0; repeat (40000) @(posedge clk);
    expect_state(6, 8, "keepalive loss");
    // F: calibration measures direction (motor_mode 0 -> dir_neg 1, travel ~ -585)
    if (dut.cal_dir_neg !== 1'b1 || dut.cal_travel > -16'sd560 || dut.cal_travel < -16'sd610) begin
      $display("FAIL cal dir: dir_neg=%0d travel=%0d", dut.cal_dir_neg, dut.cal_travel); fails = fails + 1;
    end else $display("PASS cal forward-negative: dir_neg=%0d travel=%0d", dut.cal_dir_neg, dut.cal_travel);
    // G: reversed rotor -> dir_neg 0, runs
    motor_mode = 1; do_reset; start_run;
    expect_state(5, 0, "reversed motor RUN");
    if (dut.cal_dir_neg !== 1'b0 || dut.cal_travel < 16'sd560) begin
      $display("FAIL reversed cal: dir_neg=%0d travel=%0d", dut.cal_dir_neg, dut.cal_travel); fails = fails + 1;
    end else $display("PASS reversed cal: dir_neg=%0d travel=%0d", dut.cal_dir_neg, dut.cal_travel);
    // H: stuck rotor -> refuse (q=14), outputs off
    motor_mode = 2; do_reset; start_run;
    expect_state(6, 14, "stuck rotor refused");
    if (enable || pwm_a || pwm_b || pwm_c) begin $display("FAIL: outputs active after cal refusal"); fails = fails + 1; end
    $display("TRIP_TB_DONE fails=%0d overrun=%0d", fails, dut.ctrl_overrun);
    $finish;
  end
endmodule
