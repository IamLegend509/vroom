// C-RL controller glue around the two quantized actors.
//
// Purpose: bit-exact RTL of ctrl_model.py Controller.step. Per 100 us tick it
// forms the electrical angle, converts the phase-A ADC sum, runs the declared
// single-sensor dq observer, (every 10th tick) builds the outer observation and
// runs the outer actor + IASA adapter, builds the inner observation, runs the
// inner actor, applies the 0.9 scale / circle projection and the inverse
// Park/Clarke transform, and returns three PWM compare values.
// Units/formats: see ctrl_model.py header (i_n Q.14 of 0.27 A, w_n Q.14 of
// 18 rad/s, e16 65536/electrical turn, duty counts of a 5000-count period).
// Arithmetic: one registered 36x18 multiplier; floor (arithmetic) shifts.
// Timing: ~150 glue cycles + inner 2.5k (+ outer 4.7k every 10th tick) < 10k.
// Limits: tick_go while busy sets overrun (sticky) and is otherwise ignored.
module rl_ctrl #(
  parameter string SINF = "sin.mem",
  parameter string TANHF = "tanh.mem"
) (
  input  logic clk, rst,
  input  logic tick_go,                   // start one inner tick (inputs sampled)
  input  logic [11:0] th,                 // AS5600 counts
  input  logic signed [31:0] w_in,        // speed w_n (clamped internally)
  input  logic [15:0] adc_sum,            // 16-sample XADC sum
  input  logic signed [31:0] ref_in,      // speed reference w_n
  input  logic cal_dir_neg, cal_sign_neg, // encoder reversed / current sign negative
  input  logic [15:0] cal_off,            // e16 offset
  input  logic [15:0] cal_zero,           // zero ADC sum
  input  logic [16:0] cal_gain,           // i_n per sum LSB, Q.8 (< 2^17)
  input  logic [1:0] cal_phase,           // sensed motor phase: 0 A, 1 B, 2 C
  input  logic cal_mode,                  // tick outputs a fixed vector (calibration): no actors/state
  input  logic [15:0] cal_e16,            // calibration vector electrical angle
  input  logic signed [15:0] cal_n,       // calibration vector amplitude (Q.14 of Vdc/2)
  input  logic init,                      // reset controller state (ARM); needs !busy
  input  logic signed [31:0] init_ref,    // OuterIO initial reference
  output logic busy,
  output logic done,                      // duties valid (one-cycle pulse)
  output logic [12:0] duty_a, duty_b, duty_c,
  output logic overrun,
  output logic signed [31:0] dbg_ia, dbg_id, dbg_iq, dbg_iqref,
  output logic [15:0] dbg_e16
);
  localparam int ONE = 16384;
  localparam int KV = 22756, KW = 688, KE = 3104, KA = 12138, KG = 8192, K09 = 14746, KS3 = 14189;
  localparam int KC = 197, ADCSH = 8;
  localparam int IASA_LIM = 1 << 23;

  // ---------------- tables ----------------
  (* rom_style = "block" *) logic signed [15:0] sint [1024];
  (* rom_style = "block" *) logic [15:0] tht [513];
  initial begin $readmemh(SINF, sint); $readmemh(TANHF, tht); end

  function automatic logic signed [31:0] clipv(input logic signed [63:0] x, input int lim);
    if (x > lim) return lim;
    if (x < -lim) return -lim;
    return x[31:0];
  endfunction

  function automatic logic signed [31:0] tanh14(input logic signed [63:0] x);
    logic signed [31:0] z; logic [31:0] m; logic [31:0] i; logic [7:0] f;
    logic signed [31:0] t0, t1, t;
    z = clipv(x, (1 << 24) - 1);
    m = z[31] ? -z : z;
    i = m >> 8; f = m[7:0];
    if (i >= 512) t = 32767;
    else begin
      t0 = tht[i[9:0]]; t1 = tht[i[9:0] + 10'd1];
      t = t0 + (((t1 - t0) * $signed({1'b0, f}) + 128) >>> 8);
    end
    if (z[31]) t = -t;
    return t >>> 1;
  endfunction

  // ---------------- actors ----------------
  logic iwe, owe, istart, ostart, ibusy, obusy, idone, odone;
  logic [4:0] xaddr; logic signed [15:0] xdata;
  logic signed [15:0] iy0, iy1, oy0, oy1;
  rl_mlp #(.NIN(25), .LANES(32), .W1B(16)) inner (.clk, .rst, .x_we(iwe), .x_addr(xaddr),
    .x_data(xdata), .start(istart), .busy(ibusy), .done(idone), .y0(iy0), .y1(iy1), .z0(), .z1());
  rl_mlp #(.NIN(19), .LANES(16), .W1B(9),
    .W0F("outer_w0.mem"), .W1F("outer_w1.mem"), .W2F("outer_w2.mem"),
    .B0F("outer_bs0.mem"), .B1F("outer_bs1.mem"), .B2F("outer_bs2.mem")) outer (.clk, .rst,
    .x_we(owe), .x_addr(xaddr), .x_data(xdata), .start(ostart), .busy(obusy), .done(odone),
    .y0(oy0), .y1(oy1), .z0(), .z1());

  // ---------------- shared multiplier ----------------
  logic signed [35:0] ma; logic signed [17:0] mb; logic signed [53:0] p;
  always_ff @(posedge clk) p <= ma * mb;
  wire signed [53:0] p14 = p >>> 14;

  // ---------------- state ----------------
  logic signed [31:0] d, q, napp0, napp1, nprev0, nprev1, prev0, prev1, iqref;
  logic signed [31:0] ihd [5], ihq [5];
  logic signed [31:0] ohw [20], ohe [20], ohc [20];
  logic [4:0] owp;                        // next write index of the outer ring
  logic signed [31:0] bias, cmd;          // Q.24
  logic [3:0] div10;
  logic first_tick;

  // per-tick temporaries
  logic [11:0] th_l; logic signed [31:0] w, rf; logic [15:0] adc_l;
  logic signed [31:0] ea, s, co, ia, m1, ssd, m3, m4, ssq, d1, q1, t1, innov, gi, d2;
  logic [15:0] e16;
  logic cm_l; logic signed [31:0] sp, cp;
  wire [15:0] e16p = e16 - ((cal_phase == 2'd1) ? 16'd21845 : (cal_phase == 2'd2) ? 16'd43691 : 16'd0);
  logic signed [31:0] err, ap, ai, trial, req, lim;
  logic signed [31:0] r0, r1, qd, qq, n2, sc, pal, al, be, tt, hb;
  logic signed [31:0] feat [25];
  logic [4:0] fi, fn;
  logic which;                            // 0 inner, 1 outer
  // isqrt / divide
  logic [31:0] sq_n, sq_r, sq_b, dv_num, dv_rem, dv_q;
  logic [5:0] it;

  typedef enum logic [6:0] {
    IDLE, MW, E1, E2, E3, E4, E5, C1, C2,
    O1, O2, O3, O4, O5, O6, O7, O8, O9, O10, O11, O12, O13, O14,
    U1, U2, U3, U4, U5, U6, U7, U8,
    I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, I16, I17, I18,
    WR, RUNA, WAITA, FIN
  } st_t;
  st_t st, ret, after_wr;

  task automatic mul(input logic signed [63:0] a, input logic signed [63:0] b, input st_t nxt);
    ma <= a[35:0]; mb <= b[17:0]; ret <= nxt; st <= MW;
  endtask

  assign busy = (st != IDLE);
  assign dbg_iqref = iqref;

  always_ff @(posedge clk) begin
    done <= 1'b0; iwe <= 0; owe <= 0; istart <= 0; ostart <= 0;
    if (rst) begin
      st <= IDLE; overrun <= 0;
    end else begin
      if (tick_go && st != IDLE) overrun <= 1'b1;
      case (st)
        IDLE: begin
          if (init) begin
            d <= 0; q <= 0; napp0 <= 0; napp1 <= 0; nprev0 <= 0; nprev1 <= 0;
            prev0 <= 0; prev1 <= 0; iqref <= 0; bias <= 0; cmd <= 0; div10 <= 0; first_tick <= 1;
            for (int k = 0; k < 5; k++) begin ihd[k] <= 0; ihq[k] <= 0; end
            for (int k = 0; k < 20; k++) begin ohw[k] <= 0; ohe[k] <= clipv(init_ref, (1 << 17) - 1); ohc[k] <= 0; end
            owp <= 0;
          end else if (tick_go) begin
            th_l <= th; adc_l <= adc_sum; cm_l <= cal_mode;
            w <= clipv(w_in, (1 << 17) - 1); rf <= clipv(ref_in, (1 << 17) - 1);
            mul(th, 112, E2);
          end
        end
        MW: st <= ret;                     // product registered this cycle
        E2: begin ea <= cal_dir_neg ? -p : p; mul(w, KC, E3); end
        E3: begin
          e16 <= cm_l ? cal_e16 : ea[15:0] + cal_off + p14[15:0];
          st <= E4;
        end
        E4: begin
          s <= sint[e16[15:6]]; co <= sint[(e16[15:6] + 10'd256) & 10'h3ff];
          sp <= sint[e16p[15:6]]; cp <= sint[(e16p[15:6] + 10'd256) & 10'h3ff];
          dbg_e16 <= e16;
          mul(cal_sign_neg ? -($signed({16'd0, adc_l}) - $signed({16'd0, cal_zero}))
                           :  ($signed({16'd0, adc_l}) - $signed({16'd0, cal_zero})), $signed({1'b0, cal_gain}), E5);
        end
        E5: begin
          ia <= p >>> ADCSH;
          if (cm_l) st <= C1; else mul(napp0, KV, O1);
        end
        C1: begin dbg_ia <= ia; qd <= cal_n; qq <= 0; st <= C2; end   // fixed vector, no state change
        C2: mul(qd, co, I14);
        O1: begin m1 <= p14; mul(q, w, O2); end
        O2: begin mul(p14, KW, O3); end
        O3: begin ssd <= m1 + p14; mul(napp1, KV, O4); end
        O4: begin m3 <= p14; mul(d, w, O5); end
        O5: begin mul(p14, KW, O6); end
        O6: begin m4 <= p14; mul(w, KE, O7); end
        O7: begin ssq <= m3 - m4 - p14; mul(d - ssd, KA, O8); end
        O8: begin d1 <= ssd + p14; mul(q - ssq, KA, O9); end
        O9: begin q1 <= ssq + p14; mul(d1, cp, O10); end
        O10: begin t1 <= p14; mul(q1, sp, O11); end
        O11: begin innov <= ia - (t1 - p14); mul(ia - (t1 - p14), KG, O12); end
        O12: begin gi <= p14; mul(p14, cp, O13); end
        O13: begin d2 <= clipv(d1 + p14, 2 * ONE); mul(gi, sp, O14); end
        O14: begin
          d <= d2; q <= clipv(q1 - p14, 2 * ONE);
          dbg_ia <= ia; dbg_id <= d2; dbg_iq <= clipv(q1 - p14, 2 * ONE);
          if (div10 == 0) begin err <= rf - w; mul(rf - w, 360, U1); end
          else st <= I1;
        end
        // ---------------- outer ----------------
        U1: begin
          feat[0] <= clipv(w, ONE); feat[1] <= clipv(rf, ONE); feat[2] <= clipv(err, ONE);
          feat[3] <= clipv(tanh14(p), ONE); feat[4] <= clipv(d, ONE); feat[5] <= clipv(q, ONE);
          feat[6] <= clipv((bias >>> 10) * 2, ONE); feat[7] <= clipv((cmd >>> 10) * 2, ONE);
          feat[8] <= clipv(s, ONE); feat[9] <= clipv(co, ONE);
          feat[10] <= clipv(ohw[(owp + 15) % 20], ONE); feat[11] <= clipv(ohe[(owp + 15) % 20], ONE); feat[12] <= clipv(ohc[(owp + 15) % 20], ONE);
          feat[13] <= clipv(ohw[(owp + 10) % 20], ONE); feat[14] <= clipv(ohe[(owp + 10) % 20], ONE); feat[15] <= clipv(ohc[(owp + 10) % 20], ONE);
          feat[16] <= clipv(ohw[owp], ONE); feat[17] <= clipv(ohe[owp], ONE); feat[18] <= clipv(ohc[owp], ONE);
          if (!first_tick) begin
            ohw[owp] <= w; ohe[owp] <= err; ohc[owp] <= (cmd >>> 10) * 2;
            owp <= (owp == 19) ? 5'd0 : owp + 1'b1;
          end
          which <= 1; fi <= 0; fn <= 19; after_wr <= U2; st <= WR;
        end
        U2: begin                                    // outer outputs ready
          ap <= oy0 >>> 1; ai <= oy1 >>> 1;
          mul(oy1 >>> 1, 82, U3);
        end
        U3: begin trial <= bias + p[31:0]; req <= (ap <<< 9) + bias + p[31:0]; st <= U4; end
        U4: begin lim <= clipv(req, IASA_LIM); mul(clipv(req, IASA_LIM) - req, 1638, U5); end
        U5: begin
          bias <= clipv(trial + p14, IASA_LIM);
          cmd <= (cmd >>> 1) + (lim >>> 1);
          iqref <= ((cmd >>> 1) + (lim >>> 1)) >>> 10;
          st <= I1;
        end
        // ---------------- inner ----------------
        I1: begin
          feat[0] <= clipv(d, ONE); feat[1] <= clipv(q, ONE); feat[2] <= 0; feat[3] <= clipv(iqref, ONE);
          feat[4] <= clipv((0 - d) >>> 1, ONE); feat[5] <= clipv((iqref - q) >>> 1, ONE);
          feat[6] <= clipv(prev0, ONE); feat[7] <= clipv(prev1, ONE); feat[8] <= 0; feat[9] <= 0;
          for (int k = 0; k < 5; k++) begin feat[10 + 2*k] <= clipv(ihd[k], ONE); feat[11 + 2*k] <= clipv(ihq[k], ONE); end
          feat[20] <= clipv(w, ONE); feat[21] <= clipv(s, ONE); feat[22] <= clipv(co, ONE); feat[23] <= 0; feat[24] <= 0;
          for (int k = 0; k < 4; k++) begin ihd[k] <= ihd[k+1]; ihq[k] <= ihq[k+1]; end
          ihd[4] <= d; ihq[4] <= q;
          which <= 0; fi <= 0; fn <= 25; after_wr <= I2; st <= WR;
        end
        I2: begin
          r0 <= iy0 >>> 1; r1 <= iy1 >>> 1; prev0 <= iy0 >>> 1; prev1 <= iy1 >>> 1;
          mul(iy0 >>> 1, K09, I3);
        end
        I3: begin qd <= p14; mul(r1, K09, I4); end
        I4: begin qq <= p14; mul(qd, qd, I5); end
        I5: begin n2 <= p; mul(qq, qq, I6); end
        I6: begin
          n2 <= n2 + p;
          if (n2 + p > K09 * K09) begin
            sq_n <= n2 + p; sq_r <= 0; sq_b <= 32'h1000_0000; it <= 0; st <= I7;
          end else st <= I12;
        end
        I7: begin                                     // floor sqrt, 15 digit steps
          if (sq_n >= sq_r + sq_b) begin sq_n <= sq_n - (sq_r + sq_b); sq_r <= (sq_r >> 1) + sq_b; end
          else sq_r <= sq_r >> 1;
          sq_b <= sq_b >> 2; it <= it + 1'b1;
          if (it == 14) begin st <= I8; end
        end
        I8: begin dv_num <= K09 << 14; dv_rem <= 0; dv_q <= 0; it <= 27; st <= I9; end
        I9: begin : div_step                          // restoring division, 28 bits
          logic [31:0] rr;
          rr = {dv_rem[30:0], dv_num[it]};
          if (rr >= sq_r) begin dv_rem <= rr - sq_r; dv_q[it] <= 1'b1; end
          else dv_rem <= rr;
          if (it == 0) st <= I10; else it <= it - 1'b1;
        end
        I10: begin sc <= dv_q; mul(qd < 0 ? -qd : qd, dv_q, I11); end
        I11: begin qd <= (qd < 0) ? -p14 : p14; mul(qq < 0 ? -qq : qq, sc, I13); end
        I13: begin qq <= (qq < 0) ? -p14 : p14; st <= I12; end
        I12: begin
          napp0 <= nprev0; napp1 <= nprev1; nprev0 <= qd; nprev1 <= qq;
          mul(qd, co, I14);
        end
        I14: begin pal <= p; mul(qq, s, I15); end
        I15: begin al <= (pal - p) >>> 14; mul(qd, s, I16); end
        I16: begin pal <= p; mul(qq, co, I17); end
        I17: begin be <= (pal + p) >>> 14; mul((pal + p) >>> 14, KS3, I18); end
        I18: begin
          tt <= p14; hb <= (-al) >>> 1;
          mul(al, 2500, FIN);
        end
        FIN: begin
          // duty_a from al; b/c from hb +/- tt (two more products)
          duty_a <= 13'(2500 + p14);
          mul(hb + tt, 2500, U6);
        end
        U6: begin duty_b <= 13'(2500 + p14); mul(hb - tt, 2500, U7); end
        U7: begin
          duty_c <= 13'(2500 + p14);
          if (!cm_l) begin div10 <= (div10 == 9) ? 4'd0 : div10 + 1'b1; first_tick <= 0; end
          done <= 1'b1; st <= IDLE;
        end
        // ---------------- write features to an actor, run it ----------------
        WR: begin
          xaddr <= fi; xdata <= feat[fi][15:0];
          if (which) owe <= 1; else iwe <= 1;
          if (fi == fn - 1) st <= RUNA; else fi <= fi + 1'b1;
        end
        RUNA: begin if (which) ostart <= 1; else istart <= 1; st <= WAITA; end
        WAITA: if ((which && odone) || (!which && idone)) st <= after_wr;
        default: st <= IDLE;
      endcase
    end
  end
endmodule
