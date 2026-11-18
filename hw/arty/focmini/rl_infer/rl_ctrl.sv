// C-RL controller glue around the shared 16-lane dual-actor MLP engine.
//
// Purpose: bit-exact RTL of ctrl_model.py Controller.step. Per 100 us tick it
// forms the electrical angle, converts the phase-A ADC sum, runs the declared
// single-sensor dq observer, (every 10th tick) builds the outer observation and
// runs the outer actor + IASA adapter, builds the inner observation, runs the
// inner actor, applies the 0.9 scale / circle projection and the inverse
// Park/Clarke transform, and returns three PWM compare values.
// Area optimizations (100% bit-exact):
//   - Single shared 16-lane rl_dual_mlp (16 DSP48 MACs, <9.7k cycles worst-case
//     10th tick < 10k cycles @ 100 MHz).
//   - Synchronous TDP sin BRAM (1 RAMB18) and shared tanh BRAM port.
//   - Outer 20-entry history ring stored in 48b distributed RAM with 20b valid
//     mask (single-cycle init, zero %20 divider mux tree).
//   - 16b clipped feature register file.
module rl_ctrl #(
  parameter string SINF = "sin.mem"
) (
  input  logic               clk, rst,
  input  logic               tick_go,                   // start one inner tick (inputs sampled)
  input  logic [11:0]        th,                        // AS5600 counts
  input  logic signed [31:0] w_in,                      // speed w_n (clamped internally)
  input  logic [15:0]        adc_sum,                   // 16-sample XADC sum
  input  logic signed [31:0] ref_in,                    // speed reference w_n
  input  logic               cal_dir_neg, cal_sign_neg, // encoder reversed / current sign negative
  input  logic [15:0]        cal_off,                   // e16 offset
  input  logic [15:0]        cal_zero,                  // zero ADC sum
  input  logic [16:0]        cal_gain,                  // i_n per sum LSB, Q.8 (< 2^17)
  input  logic [1:0]         cal_phase,                 // sensed motor phase: 0 A, 1 B, 2 C
  input  logic               cal_mode,                  // tick outputs a fixed vector (calibration)
  input  logic [15:0]        cal_e16,                   // calibration vector electrical angle
  input  logic signed [15:0] cal_n,                     // calibration vector amplitude (Q.14 of Vdc/2)
  input  logic               init,                      // reset controller state (ARM); needs !busy
  input  logic signed [31:0] init_ref,                  // OuterIO initial reference
  input  logic [2:0]         smooth_sh,                 // 0=raw bit-exact; >0 EMA shift on applied dq voltage
  output logic               busy,
  output logic               done,                      // duties valid (one-cycle pulse)
  output logic [12:0]        duty_a, duty_b, duty_c,
  output logic               overrun,
  output logic signed [31:0] dbg_ia, dbg_id, dbg_iq, dbg_iqref,
  output logic signed [15:0] dbg_nd, dbg_nq,
  output logic [15:0]        dbg_e16
);
  localparam int ONE = 16384;
  localparam int KV = 22756, KW = 688, KE = 3104, KA = 12138, KG = 8192, K09 = 14746, KS3 = 14189;
  localparam int KC = 197, ADCSH = 8;
  localparam int IASA_LIM = 1 << 23;
  localparam int ACT_MAX = (1 << 24) - 1;

  // ---------------- synchronous TDP sine table (1 RAMB18) ----------------
  (* rom_style = "block" *) logic signed [15:0] sint [1024];
  initial $readmemh(SINF, sint);
  logic [9:0] sint_aa, sint_ab;
  logic signed [15:0] sint_qa, sint_qb;
  always_ff @(posedge clk) begin
    sint_qa <= sint[sint_aa];
    sint_qb <= sint[sint_ab];
  end

  function automatic logic signed [31:0] clipv(input logic signed [63:0] x, input int lim);
    if (x > lim) return lim;
    if (x < -lim) return -lim;
    return x[31:0];
  endfunction

  // ---------------- shared dual actor + shared tanh BRAM ----------------
  logic        xwe, mstart, mbusy, mdone;
  logic        which;                         // 0 inner, 1 outer
  logic        iwe, owe;                      // exposed for tb_rl_ctrl logging
  logic [4:0]  xaddr;
  logic signed [15:0] xdata;
  logic signed [15:0] y0, y1;
  logic        ext_th_we;
  logic [9:0]  ext_th_a, ext_th_b;
  logic [15:0] ext_t0, ext_t1;

  assign iwe = xwe && !which;
  assign owe = xwe &&  which;

  rl_dual_mlp actors (
    .clk, .rst,
    .x_we(xwe), .x_actor(which), .x_addr(xaddr), .x_data(xdata),
    .start(mstart), .actor(which),
    .ext_th_we(ext_th_we), .ext_th_a(ext_th_a), .ext_th_b(ext_th_b),
    .ext_t0(ext_t0), .ext_t1(ext_t1),
    .busy(mbusy), .done(mdone),
    .y0(y0), .y1(y1), .z0(), .z1()
  );

  // ---------------- shared multiplier ----------------
  logic signed [35:0] ma; logic signed [17:0] mb; logic signed [53:0] p;
  always_ff @(posedge clk) p <= ma * mb;
  wire signed [53:0] p14 = p >>> 14;

  // ---------------- controller state ----------------
  logic signed [31:0] d, q, napp0, napp1, nprev0, nprev1, prev0, prev1, iqref;
  logic signed [31:0] s_qd, s_qq;
  logic signed [15:0] ihd [5], ihq [5];
  (* ram_style = "distributed" *) logic [47:0] oh_ram [32];
  logic [19:0]        oh_valid;
  logic signed [15:0] init_ohe16;
  logic [4:0]         owp, oh_raddr;
  logic [47:0]        oh_15, oh_10, oh_00;
  wire [4:0]  owp_p15 = (owp >= 5'd5)  ? (owp - 5'd5)  : (owp + 5'd15);
  wire [4:0]  owp_p10 = (owp >= 5'd10) ? (owp - 5'd10) : (owp + 5'd10);
  wire [47:0] oh_rdata = oh_valid[oh_raddr] ? oh_ram[oh_raddr] : {16'sd0, init_ohe16, 16'sd0};

  logic signed [31:0] bias, cmd;              // Q.24
  logic [3:0]         div10;
  logic               first_tick;

  // per-tick temporaries
  logic [11:0] th_l; logic signed [31:0] w, rf; logic [15:0] adc_l;
  logic signed [31:0] ea, s, co, ia, m1, ssd, m3, m4, ssq, d1, q1, t1, innov, gi, d2;
  logic [15:0] e16;
  logic cm_l; logic signed [31:0] sp, cp;
  wire [15:0] e16p = e16 - ((cal_phase == 2'd1) ? 16'd21845 : (cal_phase == 2'd2) ? 16'd43691 : 16'd0);
  logic signed [31:0] err, ap, ai, trial, req, lim;
  logic signed [31:0] r0, r1, qd, qq, n2, sc, pal, al, be, tt, hb;
  logic signed [15:0] feat [25];
  logic [4:0] fi, fn;
  // tanh14 helpers
  logic        th_neg, th_sat;
  logic [7:0]  th_f;
  logic [15:0] th_t0;
  logic signed [31:0] tanh_err;
  // isqrt / divide
  logic [31:0] sq_n, sq_r, sq_b, dv_num, dv_rem, dv_q;
  logic [5:0] it;

  typedef enum logic [6:0] {
    IDLE, MW, E2, E3, E4, E4a, E4b, E4c, E5, C1, C2,
    O1, O2, O3, O4, O5, O6, O7, O8, O9, O10, O11, O12, O13, O14,
    U0a, U0b, U0c, U0d, U0e, U1, U2, U3, U4, U5, U6, U7,
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
    done <= 1'b0; xwe <= 1'b0; mstart <= 1'b0; ext_th_we <= 1'b0;
    if (rst) begin
      st <= IDLE; overrun <= 0; oh_valid <= 20'd0;
    end else begin
      if (tick_go && st != IDLE) overrun <= 1'b1;
      case (st)
        IDLE: begin
          if (init) begin
            d <= 0; q <= 0; napp0 <= 0; napp1 <= 0; nprev0 <= 0; nprev1 <= 0;
            s_qd <= 0; s_qq <= 0; dbg_nd <= 0; dbg_nq <= 0;
            prev0 <= 0; prev1 <= 0; iqref <= 0; bias <= 0; cmd <= 0; div10 <= 0; first_tick <= 1;
            for (int k = 0; k < 5; k++) begin ihd[k] <= 0; ihq[k] <= 0; end
            oh_valid <= 20'd0;
            init_ohe16 <= 16'(clipv(init_ref, ONE));
            owp <= 0;
          end else if (tick_go) begin
            th_l <= th; adc_l <= adc_sum; cm_l <= cal_mode;
            w <= clipv(w_in, (1 << 17) - 1); rf <= clipv(ref_in, (1 << 17) - 1);
            mul(th, 112, E2);
          end
        end
        MW: st <= ret;
        E2: begin ea <= cal_dir_neg ? -p : p; mul(w, KC, E3); end
        E3: begin
          e16 <= cm_l ? cal_e16 : ea[15:0] + cal_off + p14[15:0];
          st <= E4;
        end
        E4: begin
          sint_aa <= e16[15:6];
          sint_ab <= (e16[15:6] + 10'd256) & 10'h3ff;
          st <= E4a;
        end
        E4a: begin
          sint_aa <= e16p[15:6];
          sint_ab <= (e16p[15:6] + 10'd256) & 10'h3ff;
          st <= E4b;
        end
        E4b: begin
          s  <= sint_qa;
          co <= sint_qb;
          st <= E4c;
        end
        E4c: begin
          sp <= sint_qa;
          cp <= sint_qb;
          dbg_e16 <= e16;
          mul(cal_sign_neg ? -($signed({16'd0, adc_l}) - $signed({16'd0, cal_zero}))
                           :  ($signed({16'd0, adc_l}) - $signed({16'd0, cal_zero})),
              $signed({1'b0, cal_gain}), E5);
        end
        E5: begin
          ia <= p >>> ADCSH;
          if (cm_l) st <= C1; else mul(napp0, KV, O1);
        end
        C1: begin dbg_ia <= ia; qd <= cal_n; qq <= 0; st <= C2; end
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
          if (div10 == 0) begin err <= rf - w; mul(rf - w, 360, U0a); end
          else st <= I1;
        end
        // ---------------- outer (shared tanh BRAM + distributed history RAM) ----------------
        U0a: begin : u0a_blk
          logic signed [31:0] z; logic [31:0] m, idx;
          z = clipv(p, ACT_MAX);
          m = z[31] ? -z : z;
          idx = m >> 8;
          th_neg <= z[31];
          th_sat <= (idx >= 512);
          th_f   <= m[7:0];
          ext_th_we <= 1'b1;
          ext_th_a  <= (idx >= 512) ? 10'd0 : idx[9:0];
          ext_th_b  <= (idx >= 512) ? 10'd1 : (idx[9:0] + 10'd1);
          oh_raddr  <= owp_p15;
          st        <= U0b;
        end
        U0b: begin
          oh_15    <= oh_rdata;
          oh_raddr <= owp_p10;
          st       <= U0c;
        end
        U0c: begin
          oh_10    <= oh_rdata;
          oh_raddr <= owp;
          st       <= U0d;
        end
        U0d: begin
          oh_00 <= oh_rdata;
          th_t0 <= ext_t0;
          mul($signed({16'd0, ext_t1}) - $signed({16'd0, ext_t0}), $signed({24'd0, th_f}), U0e);
        end
        U0e: begin : u0e_blk
          logic signed [31:0] tval;
          tval = th_sat ? 32'sd32767 : ($signed({16'd0, th_t0}) + ((p[31:0] + 32'sd128) >>> 8));
          tanh_err <= (th_neg ? -tval : tval) >>> 1;
          st       <= U1;
        end
        U1: begin
          feat[0]  <= 16'(clipv(w, ONE));
          feat[1]  <= 16'(clipv(rf, ONE));
          feat[2]  <= 16'(clipv(err, ONE));
          feat[3]  <= 16'(clipv(tanh_err, ONE));
          feat[4]  <= 16'(clipv(d, ONE));
          feat[5]  <= 16'(clipv(q, ONE));
          feat[6]  <= 16'(clipv((bias >>> 10) * 2, ONE));
          feat[7]  <= 16'(clipv((cmd >>> 10) * 2, ONE));
          feat[8]  <= 16'(clipv(s, ONE));
          feat[9]  <= 16'(clipv(co, ONE));
          feat[10] <= $signed(oh_15[47:32]);
          feat[11] <= $signed(oh_15[31:16]);
          feat[12] <= $signed(oh_15[15:0]);
          feat[13] <= $signed(oh_10[47:32]);
          feat[14] <= $signed(oh_10[31:16]);
          feat[15] <= $signed(oh_10[15:0]);
          feat[16] <= $signed(oh_00[47:32]);
          feat[17] <= $signed(oh_00[31:16]);
          feat[18] <= $signed(oh_00[15:0]);
          if (!first_tick) begin
            oh_ram[owp]   <= {16'(clipv(w, ONE)), 16'(clipv(err, ONE)), 16'(clipv((cmd >>> 10) * 2, ONE))};
            oh_valid[owp] <= 1'b1;
            owp           <= (owp == 5'd19) ? 5'd0 : (owp + 5'd1);
          end
          which <= 1; fi <= 0; fn <= 19; after_wr <= U2; st <= WR;
        end
        U2: begin
          ap <= y0 >>> 1; ai <= y1 >>> 1;
          mul(y1 >>> 1, 82, U3);
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
          feat[0]  <= 16'(clipv(d, ONE));
          feat[1]  <= 16'(clipv(q, ONE));
          feat[2]  <= 16'sd0;
          feat[3]  <= 16'(clipv(iqref, ONE));
          feat[4]  <= 16'(clipv((0 - d) >>> 1, ONE));
          feat[5]  <= 16'(clipv((iqref - q) >>> 1, ONE));
          feat[6]  <= 16'(clipv(prev0, ONE));
          feat[7]  <= 16'(clipv(prev1, ONE));
          feat[8]  <= 16'sd0;
          feat[9]  <= 16'sd0;
          for (int k = 0; k < 5; k++) begin
            feat[10 + 2*k] <= ihd[k];
            feat[11 + 2*k] <= ihq[k];
          end
          feat[20] <= 16'(clipv(w, ONE));
          feat[21] <= 16'(clipv(s, ONE));
          feat[22] <= 16'(clipv(co, ONE));
          feat[23] <= 16'sd0;
          feat[24] <= 16'sd0;
          for (int k = 0; k < 4; k++) begin ihd[k] <= ihd[k+1]; ihq[k] <= ihq[k+1]; end
          ihd[4] <= 16'(clipv(d, ONE)); ihq[4] <= 16'(clipv(q, ONE));
          which <= 0; fi <= 0; fn <= 25; after_wr <= I2; st <= WR;
        end
        I2: begin
          r0 <= y0 >>> 1; r1 <= y1 >>> 1; prev0 <= y0 >>> 1; prev1 <= y1 >>> 1;
          mul(y0 >>> 1, K09, I3);
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
        I7: begin
          if (sq_n >= sq_r + sq_b) begin sq_n <= sq_n - (sq_r + sq_b); sq_r <= (sq_r >> 1) + sq_b; end
          else sq_r <= sq_r >> 1;
          sq_b <= sq_b >> 2; it <= it + 1'b1;
          if (it == 14) begin st <= I8; end
        end
        I8: begin dv_num <= K09 << 14; dv_rem <= 0; dv_q <= 0; it <= 27; st <= I9; end
        I9: begin : div_step
          logic [31:0] rr;
          rr = {dv_rem[30:0], dv_num[it]};
          if (rr >= sq_r) begin dv_rem <= rr - sq_r; dv_q[it] <= 1'b1; end
          else dv_rem <= rr;
          if (it == 0) st <= I10; else it <= it - 1'b1;
        end
        I10: begin sc <= dv_q; mul(qd < 0 ? -qd : qd, dv_q, I11); end
        I11: begin qd <= (qd < 0) ? -p14 : p14; mul(qq < 0 ? -qq : qq, sc, I13); end
        I13: begin qq <= (qq < 0) ? -p14 : p14; st <= I12; end
        I12: begin : i12_blk
          logic signed [31:0] qd_out, qq_out;
          qd_out = (smooth_sh == 3'd0) ? qd : (s_qd + ((qd - s_qd) >>> smooth_sh));
          qq_out = (smooth_sh == 3'd0) ? qq : (s_qq + ((qq - s_qq) >>> smooth_sh));
          s_qd <= qd_out; s_qq <= qq_out;
          qd <= qd_out; qq <= qq_out;
          dbg_nd <= qd_out[15:0]; dbg_nq <= qq_out[15:0];
          napp0 <= nprev0; napp1 <= nprev1; nprev0 <= qd_out; nprev1 <= qq_out;
          mul(qd_out, co, I14);
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
          duty_a <= 13'(2500 + p14);
          mul(hb + tt, 2500, U6);
        end
        U6: begin duty_b <= 13'(2500 + p14); mul(hb - tt, 2500, U7); end
        U7: begin
          duty_c <= 13'(2500 + p14);
          if (!cm_l) begin div10 <= (div10 == 9) ? 4'd0 : div10 + 1'b1; first_tick <= 0; end
          done <= 1'b1; st <= IDLE;
        end
        // ---------------- write features to shared actor, run it ----------------
        WR: begin
          xwe <= 1; xaddr <= fi; xdata <= feat[fi];
          if (fi == fn - 1) st <= RUNA; else fi <= fi + 1'b1;
        end
        RUNA: begin mstart <= 1; st <= WAITA; end
        WAITA: if (mdone) st <= after_wr;
        default: st <= IDLE;
      endcase
    end
  end
endmodule
