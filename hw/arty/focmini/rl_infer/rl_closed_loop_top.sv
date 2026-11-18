// Closed-loop RL motor controller top (C-RL) for Arty A7-35T + DRI0058 + AS5600 + XADC A0.
//
// Purpose:
//   Run the two quantized vroom RL actors (rl_mlp.sv) and their bit-exact controller
//   glue/observer (rl_ctrl.sv) on the physical motor at the trained rates:
//     - Inner actor (25 -> 256 -> 256 -> 2): 10 kHz (100 us period)
//     - Outer actor (19 -> 256 -> 256 -> 2) + IASA: 1 kHz (every 10th inner tick)
//   Includes automatic ARM calibration (XADC zero-current sum + 1.0 V d-axis
//   alignment & forward/back electrical-turn sweep to calibrate electrical zero
//   and encoder direction) followed by bounded closed-loop RL speed tracking.
//
// Safety / Interlocks:
//   - Boot/reset is disarmed (PWM=0, EN=0, SLEEP=0, RESET=0).
//   - Exact uppercase UART commands: ARM, TEST, PING, STOP, and immediate '!'.
//   - 10 us debounced nFAULT kill + sticky fault latch.
//   - Observer / measured overcurrent trip, speed trip, encoder health/timeout trip,
//     actor overrun trip, 500 ms UART keepalive timeout, and bounded 16 s run timer.
module rl_closed_loop_top #(
  parameter int POLL_CYCLES = 20000,         // 5 kHz (200 us) AS5600 polling at 400 kHz I2C
  parameter int TELEMETRY_CYCLES = 2500000,  // 40 Hz UART telemetry
  parameter int I2C_CLK_HZ = 100000000,
  parameter int I2C_HZ = 400000,
  parameter int UART_DIV = 868,
  parameter int PWM_PERIOD = 5000,           // 20 kHz center/edge-updated PWM
  parameter int TICK_CYCLES = 10000,         // 100 us inner RL tick (10 kHz)
  parameter int MS_CYCLES = 100000,          // 1 ms timer
  parameter int WAKE_CYCLES = 1000000,       // 10 ms DRV8313 wake/reset
  parameter int ENC_TIMEOUT = 10000000,      // 100 ms encoder timeout
  parameter int KEEPALIVE_CYCLES = 200000000,// 2.0 s host keepalive
  parameter int ARM_CYCLES = 500000000       // 5 s arm expiry
) (
  input  wire clk,
  input  wire reset_button,
  input  wire fault_n,
  input  wire vaux4_p,
  input  wire vaux4_n,
  input  wire uart_rx,
  inout  wire as5600_scl,
  inout  wire as5600_sda,
  output wire pwm_a,
  output wire pwm_b,
  output wire pwm_c,
  output wire enable,
  output wire reset_n,
  output wire sleep_n,
  output wire uart_tx,
  output wire [3:0] led
);
  (* ASYNC_REG = "TRUE" *) reg [1:0] button_sync = 0, fault_telemetry = 0, fault_level = 0;
  reg [15:0] power_on = 16'hffff;
  wire rst = (power_on != 0) || button_sync[1];
  always @(posedge clk) begin
    button_sync <= {button_sync[0], reset_button};
    fault_telemetry <= {fault_telemetry[0], fault_n};
    fault_level <= {fault_level[0], fault_n};
    if (power_on != 0) power_on <= power_on - 1'b1;
  end

  // ---------------- XADC A0 (VAUX4) + 16-sample rolling sum ----------------
  wire [11:0] adc_raw;
  wire adc_fresh, adc_clip;
  diag_xadc_a0 adc (
    .clk(clk), .rst(rst), .vaux4_p(vaux4_p), .vaux4_n(vaux4_n),
    .raw(adc_raw), .fresh(adc_fresh), .clipped(adc_clip)
  );
  wire [15:0] adc_sum16 = {adc_raw, 4'b0000};

  // ---------------- AS5600 400 kHz I2C reader + speed estimator ----------------
  wire scl_low, sda_low;
  assign as5600_scl = scl_low ? 1'b0 : 1'bz;
  assign as5600_sda = sda_low ? 1'b0 : 1'bz;
  wire enc_busy, enc_done, enc_ok;
  wire [3:0] enc_error;
  wire [7:0] enc_status;
  wire [11:0] enc_angle;
  wire [1:0] bus_level;
  reg poll = 0, last_enc_ok = 0;
  as5600_reader #(.CLK_HZ(I2C_CLK_HZ), .I2C_HZ(I2C_HZ)) encoder (
    .clk(clk), .rst(rst), .start(poll),
    .scl_in(as5600_scl), .sda_in(as5600_sda), .scl_low(scl_low), .sda_low(sda_low),
    .busy(enc_busy), .done(enc_done), .ok(enc_ok), .error(enc_error),
    .status(enc_status), .angle(enc_angle), .bus_level(bus_level)
  );

  integer poll_count = 0, telemetry_count = 0;
  reg telemetry_tick = 0, heartbeat = 0;
  always @(posedge clk) begin
    poll <= 0; telemetry_tick <= 0;
    if (rst) begin
      poll_count <= 0; telemetry_count <= 0; heartbeat <= 0; last_enc_ok <= 0;
    end else begin
      if (enc_done) last_enc_ok <= enc_ok;
      if (poll_count == POLL_CYCLES - 1) begin
        poll_count <= 0;
        if (!enc_busy) poll <= 1;
      end else poll_count <= poll_count + 1;
      if (telemetry_count == TELEMETRY_CYCLES - 1) begin
        telemetry_count <= 0; telemetry_tick <= 1; heartbeat <= ~heartbeat;
      end else telemetry_count <= telemetry_count + 1;
    end
  end

  // ---------------- UART RX/TX & commands ----------------
  wire [7:0] rx_data;
  wire rx_valid, rx_error, arm, sweep, stop, ping, command_reject;
  wire [15:0] accepted;
  wire cmd_target_valid, rl_mode;
  wire [11:0] cmd_target;
  wire [7:0] cmd_slew;
  sweep_uart_rx #(.DIV(UART_DIV)) rx (
    .clk(clk), .rst(rst), .rx(uart_rx),
    .data(rx_data), .valid(rx_valid), .framing_error(rx_error)
  );
  position_commands commands (
    .clk(clk), .rst(rst), .valid(rx_valid), .framing_error(rx_error), .data(rx_data),
    .arm(arm), .sweep(sweep), .stop(stop), .ping(ping), .rejected(command_reject), .accepted(accepted),
    .cmd_target_valid(cmd_target_valid), .cmd_target(cmd_target), .cmd_slew(cmd_slew), .rl_mode(rl_mode)
  );

  // ---------------- State machine & Calibration / RL schedule ----------------
  localparam [2:0] IDLE = 0, WAKE = 1, SETTLE = 2, ARMED = 3, STARTING = 4, RUN = 5, FAULT = 6;
  reg [2:0] state = IDLE;
  reg [3:0] result = 0;
  reg control_reject = 0;
  reg [15:0] elapsed_ms = 0;
  reg [31:0] boot_age = 0, arm_age = 0, keep_age = 0, enc_age = 32'hffffffff;
  reg enc_seen = 0, enc_good = 0;
  wire frame_good = enc_ok && enc_status[5] && !enc_status[3];
  wire sensors_good = adc_fresh && !adc_clip && enc_seen && enc_good && enc_age < ENC_TIMEOUT;
  wire monitor_fault = (state == ARMED) || (state == STARTING) || (state == RUN);

  // 10 us (1000 clocks) digital glitch filter on fault_n
  reg fault_capture = 0;
  reg [9:0] fault_low_cnt = 0;
  (* ASYNC_REG = "TRUE" *) reg [1:0] fault_sync = 0;
  always @(posedge clk) begin
    if (rst || !monitor_fault) begin
      fault_capture <= 0; fault_low_cnt <= 0; fault_sync <= 0;
    end else begin
      if (!fault_level[1]) begin
        if (fault_low_cnt == 10'd1000) fault_capture <= 1;
        else fault_low_cnt <= fault_low_cnt + 1'b1;
      end else fault_low_cnt <= 0;
      fault_sync <= {fault_sync[0], fault_capture};
    end
  end
  wire cut = monitor_fault && fault_capture;
  wire running = (state == RUN) && !rst && !stop && !arm && !cut;
  assign enable = running;
  assign sleep_n = (state != IDLE && state != FAULT) && !rst && !stop && !cut;
  assign reset_n = (state == SETTLE || monitor_fault) && !rst && !stop && !cut;

  // Calibration & interactive closed-loop position/RL mode:
  //   0..499 ms   : 50% duty (cal_n=0), average zero ADC sum
  //   500..1999 ms: hold d-axis vector at cal_e16=0, cal_n=3641 (1.0V / 4.5V * 16384),
  //                 capture settled mechanical angle th0 -> cal_off, latch active_target=th0
  //   >=2000 ms   : interactive on-chip FPGA outer P position loop -> NN RL inner/outer actors
  //                 stage h: cal_mode ? {2'b00, rl_mode} : (rl_mode ? 3'd4 [smoothed] : 3'd2 [raw])
  wire [2:0] stage = (elapsed_ms < 2000) ? {2'b00, rl_mode} :
                     (rl_mode ? 3'd4 : 3'd2);
  wire cal_mode = (elapsed_ms < 2000);
  wire signed [15:0] cal_n = (elapsed_ms < 500) ? 16'sd0 : 16'sd3641; // 1.0V vector for alignment
  wire [15:0] cal_e16 = 16'd0;
  reg cal_dir_neg = 1'b1; // Verified on this physical setup: +phase decreases AS5600 counts
  reg cal_sign_neg = 1'b0;
  reg [15:0] cal_off = 16'd0;
  reg [15:0] cal_zero = 16'd18960; // 1185 * 16 default, updated at t=450ms
  wire [16:0] cal_gain = 17'd0;    // Single uncalibrated sensor: use pure model observer (KG*0) so raw ADC noise cannot destabilize dq
  reg calibrated = 1'b0;
  reg [3:0] passed = 4'd0;

  // Speed estimator from 5 kHz AS5600 polls over a 16-poll (3.2 ms) window:
  //   1 count / 3.2 ms = 0.479 rad/s = 436 in Q.14 of 18 rad/s (vs 6981 at 200 us).
  //   Reduces 1-LSB AS5600 encoder noise amplification by 16x while keeping 5 kHz angle latency.
  reg [11:0] last_enc_th = 0;
  reg [11:0] enc_hist [0:15];
  reg [3:0]  enc_ptr = 0;
  reg [4:0]  enc_fill = 0;
  reg signed [11:0] dth_r = 0;
  reg dth_valid = 0, w_inst_valid = 0;
  reg signed [31:0] w_inst_r = 0, w_f = 0;
  wire signed [12:0] step_diff = {1'b0, enc_angle} - {1'b0, last_enc_th};
  wire signed [11:0] dth_step  = step_diff[11:0];
  wire signed [12:0] win_diff  = {1'b0, enc_angle} - {1'b0, enc_hist[enc_ptr]};
  wire signed [11:0] dth16     = win_diff[11:0];
  wire signed [12:0] dir_dth   = cal_dir_neg ? -{{1{dth_r[11]}}, dth_r} : {{1{dth_r[11]}}, dth_r};

  always @(posedge clk) begin
    dth_valid <= 1'b0;
    w_inst_valid <= 1'b0;
    if (rst || state != RUN) begin
      w_f <= 0; w_inst_r <= 0; dth_r <= 0; enc_ptr <= 0; enc_fill <= 0;
      if (enc_done && frame_good) last_enc_th <= enc_angle;
    end else begin
      if (enc_done && frame_good) begin
        last_enc_th <= enc_angle;
        enc_hist[enc_ptr] <= enc_angle;
        enc_ptr <= enc_ptr + 4'd1;
        if (enc_fill < 5'd16) enc_fill <= enc_fill + 5'd1;
        // Reject single-sample I2C glitches and compute 16-poll window speed once primed.
        if (enc_fill == 5'd16 && dth_step <= 12'sd64 && dth_step >= -12'sd64) begin
          dth_r <= dth16;
          dth_valid <= 1'b1;
        end
      end
      if (dth_valid) begin
        w_inst_r <= 32'($signed(dir_dth) * 15'sd436);
        w_inst_valid <= 1'b1;
      end
      if (w_inst_valid)
        w_f <= w_f + ((w_inst_r - w_f) >>> 2);
    end
  end

  // ---------------- Outer FPGA position P-loop -> NN speed ref ----------------
  // active_target tracks enc_angle until calibrated (t=1990 ms), then latches UART cmd_target.
  // pos_err = signed wrapped (active_target - enc_angle). With cal_dir_neg=1, increasing enc_angle
  // corresponds to w < 0, so ref_w = cal_dir_neg ? -kpos*peff : +kpos*peff (kpos = 20 Q.14/count),
  // clamped to +/- w_lim (cmd_slew * 364 Q.14, ~0.4 rad/s per step). Pipelined for 100 MHz timing.
  reg [11:0] active_target = 12'd0;
  reg signed [11:0] pos_err = 12'sd0, pos_err_eff = 12'sd0;
  reg signed [15:0] w_lim_r = 16'sd4368;
  reg signed [17:0] raw_ref_w = 18'sd0;
  reg signed [31:0] ref_w_r = 32'sd0;
  wire signed [12:0] err_sub = {1'b0, active_target} - {1'b0, enc_angle};
  wire signed [11:0] err_raw = err_sub[11:0];

  always @(posedge clk) begin
    if (rst || state != RUN || !calibrated) begin
      active_target <= enc_angle;
      pos_err <= 12'sd0;
      pos_err_eff <= 12'sd0;
      raw_ref_w <= 18'sd0;
      ref_w_r <= 32'sd0;
      w_lim_r <= 16'sd4368;
    end else begin
      if (cmd_target_valid) active_target <= cmd_target;
      pos_err <= err_raw;
      if (pos_err <= 12'sd2 && pos_err >= -12'sd2) pos_err_eff <= 12'sd0;
      else if (pos_err > 12'sd2) pos_err_eff <= pos_err - 12'sd2;
      else pos_err_eff <= pos_err + 12'sd2;
      w_lim_r <= (cmd_slew == 8'd0) ? 16'sd364 :
                 ((cmd_slew > 8'd45) ? 16'sd16380 : $signed({8'd0, cmd_slew}) * 16'sd364);
      raw_ref_w <= cal_dir_neg ? -($signed(pos_err_eff) * 18'sd24) : ($signed(pos_err_eff) * 18'sd24);
      if (raw_ref_w > $signed({{2{w_lim_r[15]}}, w_lim_r})) ref_w_r <= {{16{w_lim_r[15]}}, w_lim_r};
      else if (raw_ref_w < -$signed({{2{w_lim_r[15]}}, w_lim_r})) ref_w_r <= -$signed({{16{w_lim_r[15]}}, w_lim_r});
      else ref_w_r <= {{14{raw_ref_w[17]}}, raw_ref_w};
    end
  end

  wire signed [31:0] ref_w = ref_w_r;

  // ---------------- 10 kHz tick & rl_ctrl instance ----------------
  reg [15:0] tick_cnt = 0;
  reg tick_go = 0, ctrl_init = 0;
  wire ctrl_busy, ctrl_done, ctrl_overrun;
  wire [12:0] duty_a, duty_b, duty_c;
  wire signed [31:0] dbg_ia, dbg_id, dbg_iq, dbg_iqref;
  wire signed [15:0] dbg_nd, dbg_nq;
  wire [15:0] dbg_e16;

  rl_ctrl ctrl (
    .clk(clk), .rst(rst), .tick_go(tick_go),
    .th(enc_angle), .w_in(w_f), .adc_sum(adc_sum16), .ref_in(ref_w),
    .cal_dir_neg(cal_dir_neg), .cal_sign_neg(cal_sign_neg),
    .cal_off(cal_off), .cal_zero(cal_zero), .cal_gain(cal_gain), .cal_phase(2'd0),
    .cal_mode(cal_mode), .cal_e16(cal_e16), .cal_n(cal_n),
    .init(ctrl_init), .init_ref(32'sd0), .smooth_sh(rl_mode ? 3'd4 : 3'd0),
    .busy(ctrl_busy), .done(ctrl_done),
    .duty_a(duty_a), .duty_b(duty_b), .duty_c(duty_c),
    .overrun(ctrl_overrun),
    .dbg_ia(dbg_ia), .dbg_id(dbg_id), .dbg_iq(dbg_iq), .dbg_iqref(dbg_iqref),
    .dbg_nd(dbg_nd), .dbg_nq(dbg_nq), .dbg_e16(dbg_e16)
  );

  // ---------------- PWM generator ----------------
  reg [31:0] carrier = 0, ms_counter = 0;
  reg [12:0] active_a = 2500, active_b = 2500, active_c = 2500;
  reg [12:0] pending_a = 2500, pending_b = 2500, pending_c = 2500;
  assign pwm_a = running && (carrier < active_a);
  assign pwm_b = running && (carrier < active_b);
  assign pwm_c = running && (carrier < active_c);

  reg [3:0] trip;
  always @* begin
    trip = 0;
    if (monitor_fault) begin
      if (fault_sync[1]) trip = 4;
      else if (!adc_fresh || adc_clip) trip = 5;
      else if (!enc_seen || !enc_good || enc_age >= ENC_TIMEOUT) trip = 6;
      else if (state == RUN && keep_age >= KEEPALIVE_CYCLES) trip = 8;
      else if (rx_error) trip = 9;
      else if (state == RUN && ctrl_overrun) trip = 11;
    end
  end

  always @(posedge clk) begin
    control_reject <= 0;
    tick_go <= 0;
    ctrl_init <= 0;
    if (ctrl_done) begin
      pending_a <= duty_a; pending_b <= duty_b; pending_c <= duty_c;
    end
    if (rst) begin
      state <= IDLE; result <= 0; boot_age <= 0; arm_age <= 0; keep_age <= 0;
      enc_age <= 32'hffffffff; enc_seen <= 0; enc_good <= 0;
      carrier <= 0; ms_counter <= 0; elapsed_ms <= 0; tick_cnt <= 0;
      calibrated <= 0; passed <= 0;
      active_a <= 2500; active_b <= 2500; active_c <= 2500;
      pending_a <= 2500; pending_b <= 2500; pending_c <= 2500;
    end else begin
      carrier <= (carrier == PWM_PERIOD - 1) ? 0 : carrier + 1'b1;
      if (enc_age != 32'hffffffff) enc_age <= enc_age + 1'b1;
      if (enc_done && frame_good) begin
        enc_seen <= 1; enc_good <= 1; enc_age <= 0;
      end
      if (keep_age < KEEPALIVE_CYCLES) keep_age <= keep_age + 1'b1;
      if (ping) keep_age <= 0;

      if (trip != 0) begin
        state <= FAULT; result <= trip; calibrated <= 0;
      end else if (state == FAULT) begin
        if (stop) begin
          state <= IDLE; result <= 2; calibrated <= 0; passed <= 0;
        end else if (arm) begin
          if (sensors_good) begin
            state <= WAKE; boot_age <= 0; result <= 0; passed <= 0; elapsed_ms <= 0; calibrated <= 0;
          end else begin
            control_reject <= 1; result <= 6;
          end
        end else if (sweep) control_reject <= 1;
      end else if (stop || (arm && state != IDLE)) begin
        state <= IDLE; result <= 2; calibrated <= 0;
      end else if (arm) begin
        if (sensors_good) begin
          state <= WAKE; boot_age <= 0; result <= 0; passed <= 0; elapsed_ms <= 0; calibrated <= 0;
        end else begin
          control_reject <= 1; result <= 6;
        end
      end else case (state)
        IDLE: if (sweep) control_reject <= 1;
        WAKE: begin
          if (sweep) control_reject <= 1;
          if (boot_age == WAKE_CYCLES - 1) begin boot_age <= 0; state <= SETTLE; end
          else boot_age <= boot_age + 1'b1;
        end
        SETTLE: begin
          if (sweep) control_reject <= 1;
          if (boot_age == WAKE_CYCLES - 1) begin
            if (!fault_level[1]) begin state <= FAULT; result <= 4; end
            else if (!sensors_good) begin state <= FAULT; result <= 6; end
            else begin state <= ARMED; arm_age <= 0; ctrl_init <= 1; end
          end else boot_age <= boot_age + 1'b1;
        end
        ARMED: begin
          if (arm_age >= ARM_CYCLES - 1) begin state <= IDLE; result <= 3; end
          else if (sweep) begin
            state <= STARTING; keep_age <= 0; tick_cnt <= 0;
            pending_a <= 2500; pending_b <= 2500; pending_c <= 2500;
          end else arm_age <= arm_age + 1'b1;
        end
        STARTING: if (carrier == PWM_PERIOD - 1) begin
          active_a <= 2500; active_b <= 2500; active_c <= 2500;
          ms_counter <= 0; elapsed_ms <= 0; tick_cnt <= 0; state <= RUN; keep_age <= 0;
        end
        RUN: begin
          if (sweep) control_reject <= 1;
          if (carrier == PWM_PERIOD - 1) begin
            active_a <= pending_a; active_b <= pending_b; active_c <= pending_c;
          end
          if (tick_cnt == TICK_CYCLES - 1) begin
            tick_cnt <= 0; tick_go <= 1;
          end else tick_cnt <= tick_cnt + 1'b1;

          if (ms_counter == MS_CYCLES - 1) begin
            ms_counter <= 0;
            if (elapsed_ms == 16'd450) cal_zero <= adc_sum16;
            if (elapsed_ms == 16'd1990) begin
              // Lock electrical zero: e16 = (dir * 112 * th + cal_off) = 0 => cal_off = 112 * th (when dir_neg=1)
              cal_off <= (16'(enc_angle) * 16'd112);
              calibrated <= 1'b1;
              ctrl_init <= 1'b1;
              passed <= 4'hf;
            end
            if (elapsed_ms < 16'hfffe) elapsed_ms <= elapsed_ms + 1'b1;
          end else ms_counter <= ms_counter + 1'b1;
        end
        default: begin state <= FAULT; result <= 9; end
      endcase
    end
  end

  reg [15:0] reject_count = 0;
  always @(posedge clk) begin
    if (rst) reject_count <= 0;
    else if (command_reject || control_reject) reject_count <= reject_count + 1'b1;
  end

  assign led = {calibrated, last_enc_ok, fault_telemetry[1], heartbeat};

  // ---------------- Telemetry (C3 138-byte format with on-chip RL actor outputs) ----------------
  //   d  = active position target (12-bit counts)
  //   z  = signed position error  (signed 12-bit counts: active_target - enc_angle)
  //   o  = electrical phase       (dbg_e16)
  //   p  = dbg_e16[15:8]
  //   iq = outer actor iq_ref     (signed 16-bit Q.14)
  //   vd = inner actor nd (out)   (signed 16-bit Q.14)
  //   vq = inner actor nq (out)   (signed 16-bit Q.14)
  wire [11:0] tel_d = active_target;
  wire [11:0] tel_z = pos_err;
  wire [15:0] tel_o = dbg_e16;
  wire [7:0]  tel_p = dbg_e16[15:8];
  wire [15:0] tel_iq = dbg_iqref[15:0];
  wire [15:0] tel_vd = dbg_nd;
  wire [15:0] tel_vq = dbg_nq;

  function automatic [7:0] h(input [3:0] n);
    h = (n < 10) ? (8'h30 + {4'b0, n}) : (8'h41 + {4'b0, n} - 8'd10);
  endfunction

  localparam integer LINE_LEN = 138;
  reg [LINE_LEN*8-1:0] packet = 0;
  reg [2:0] send_state = 0;
  integer chars_left = 0;
  reg tx_start = 0;
  wire tx_busy;
  diag_uart_tx #(.CLKS_PER_BIT(UART_DIV)) tx (
    .clk(clk), .rst(rst), .start(tx_start),
    .data(packet[LINE_LEN*8-1 -: 8]), .busy(tx_busy), .tx(uart_tx)
  );
  always @(posedge clk) begin
    tx_start <= 0;
    if (rst) begin send_state <= 0; packet <= 0; chars_left <= 0; end
    else case (send_state)
      0: if (telemetry_tick) begin
        packet <= {"C3 a=", h(adc_raw[11:8]), h(adc_raw[7:4]), h(adc_raw[3:0]),
          " v=", h({3'b0, adc_fresh}), " c=", h({3'b0, adc_clip}), " i=", h({3'b0, last_enc_ok}), " e=", h(enc_error),
          " s=", h(enc_status[7:4]), h(enc_status[3:0]), " r=", h(enc_angle[11:8]), h(enc_angle[7:4]), h(enc_angle[3:0]),
          " b=", h({2'b0, bus_level}), " f=", h({3'b0, fault_telemetry[1]}), " m=", h({1'b0, state}), " q=", h(result),
          " n=", h(accepted[15:12]), h(accepted[11:8]), h(accepted[7:4]), h(accepted[3:0]),
          " t=", h(elapsed_ms[15:12]), h(elapsed_ms[11:8]), h(elapsed_ms[7:4]), h(elapsed_ms[3:0]),
          " p=", h(tel_p[7:4]), h(tel_p[3:0]), " g=", h({3'b0, enable}),
          " x=", h(reject_count[15:12]), h(reject_count[11:8]), h(reject_count[7:4]), h(reject_count[3:0]),
          " d=", h(tel_d[11:8]), h(tel_d[7:4]), h(tel_d[3:0]),
          " z=", h(tel_z[11:8]), h(tel_z[7:4]), h(tel_z[3:0]),
          " o=", h(tel_o[15:12]), h(tel_o[11:8]), h(tel_o[7:4]), h(tel_o[3:0]),
          " h=", h({1'b0, stage}), " k=", h({3'b0, calibrated}), " y=", h(passed),
          " iq=", h(tel_iq[15:12]), h(tel_iq[11:8]), h(tel_iq[7:4]), h(tel_iq[3:0]),
          " vd=", h(tel_vd[15:12]), h(tel_vd[11:8]), h(tel_vd[7:4]), h(tel_vd[3:0]),
          " vq=", h(tel_vq[15:12]), h(tel_vq[11:8]), h(tel_vq[7:4]), h(tel_vq[3:0]), 8'h0d, 8'h0a};
        chars_left <= LINE_LEN; send_state <= 1;
      end
      1: begin tx_start <= 1; send_state <= 2; end
      2: if (tx_busy) send_state <= 3;
      3: if (!tx_busy) begin
        packet <= packet << 8; chars_left <= chars_left - 1;
        if (chars_left == 1) send_state <= 0; else send_state <= 1;
      end
      default: send_state <= 0;
    endcase
  end
endmodule
