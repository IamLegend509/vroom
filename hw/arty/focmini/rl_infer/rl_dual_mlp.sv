// Shared 16-lane fixed-point dual-actor MLP engine (inner + outer vroom actors).
//
// Purpose: run both quantized 256x256 MLPs (Linear/ReLU x2, Linear/tanh) in a
// single time-shared 16-lane DSP48 MAC datapath sized to fit XC7A35T cleanly
// without overflowing BRAM or LUT budgets while remaining bit-exact with quant.py.
// Architecture:
//   - actor=0 (inner): 25 -> 256 -> 256 -> 2, W1=16b (~4,790 cycles = 47.9 us)
//   - actor=1 (outer): 19 -> 256 -> 256 -> 2, W1=9b  (~4,690 cycles = 46.9 us)
//   - External tanh BRAM port allows rl_ctrl to share the 513x16 tanh RAMB18E1
//     while the MLP engine is idle.
module rl_dual_mlp #(
  parameter string IW0F = "inner_w0.mem",
  parameter string OW0F = "outer_w0.mem",
  parameter string IW1F = "inner_w1.mem",
  parameter string OW1F = "outer_w1.mem",
  parameter string W2F  = "w2_comb.mem",
  parameter string BSF  = "bs_comb.mem",
  parameter string BS2F = "bs2_comb.mem",
  parameter string TF   = "tanh.mem"
) (
  input  logic               clk,
  input  logic               rst,
  input  logic               x_we,
  input  logic               x_actor,   // 0=inner slot (0..24), 1=outer slot (32..50)
  input  logic [4:0]         x_addr,
  input  logic signed [15:0] x_data,
  input  logic               start,
  input  logic               actor,     // 0=inner, 1=outer (latched on start)
  input  logic               ext_th_we, // latch addresses into shared tanh BRAM while idle
  input  logic [9:0]         ext_th_a,
  input  logic [9:0]         ext_th_b,
  output logic [15:0]        ext_t0,
  output logic [15:0]        ext_t1,
  output logic               busy,
  output logic               done,      // one-cycle pulse; y/z valid until next run
  output logic signed [15:0] y0,
  output logic signed [15:0] y1,
  output logic signed [24:0] z0,
  output logic signed [24:0] z1
);
  localparam int LANES = 16;
  localparam int H     = 256;
  localparam int NPL   = H / LANES;     // 16
  localparam int AB    = 25;
  localparam int BSB   = 30;            // {shm1[4:0], bias[24:0]}
  localparam signed [48:0] ACT_MAX_49 = 49'((1 << 24) - 1);
  localparam int ACT_MAX = (1 << 24) - 1;

  // ---------------- memories ----------------
  // BRAM budget on XC7A35T (100 RAMB18):
  //   iw1m (4096x256): 57 RAMB18
  //   ow1m (4096x144): 32 RAMB18
  //   iw0m (400x256) :  8 RAMB18
  //   tm   (513x16)  :  1 RAMB18 (TDP shared with rl_ctrl)
  // Distributed LUT/ROM: ow0m (304x256), w2m (512x32), bsm (64x480), ha/hb (16x400)
  (* rom_style = "block" *)       logic [LANES*16-1:0] iw0m [400];
  (* rom_style = "distributed" *) logic [LANES*16-1:0] ow0m [304];
  (* rom_style = "block" *)       logic [LANES*16-1:0] iw1m [4096];
  (* rom_style = "block" *)       logic [LANES*9-1:0]  ow1m [4096];
  (* rom_style = "distributed" *) logic [31:0]         w2m  [512];
  (* rom_style = "distributed" *) logic [LANES*BSB-1:0] bsm [64];
  logic [2*BSB-1:0] bs2m [2];
  (* rom_style = "block" *)       logic [15:0]         tm   [513];

  initial begin
    $readmemh(IW0F, iw0m); $readmemh(OW0F, ow0m);
    $readmemh(IW1F, iw1m); $readmemh(OW1F, ow1m);
    $readmemh(W2F,  w2m);
    $readmemh(BSF,  bsm);  $readmemh(BS2F, bs2m);
    $readmemh(TF,   tm);
  end

  logic signed [15:0] xm [64];
  (* ram_style = "distributed" *) logic [LANES*AB-1:0] ha [NPL];
  (* ram_style = "distributed" *) logic [LANES*AB-1:0] hb [NPL];
  always_ff @(posedge clk) if (x_we) xm[{x_actor, x_addr}] <= x_data;

  // ---------------- issue counters ----------------
  typedef enum logic [2:0] {IDLE, RUN, DRAIN, TANH_RD0, TANH_RD1, TANH_C0, TANH_C1} st_t;
  st_t st;
  logic act_r;
  logic [1:0] layer;
  logic [11:0] addr;
  logic [8:0] k, kmax;
  logic [3:0] row, rmax;
  logic [3:0] drain;
  logic iv;

  assign busy = (st != IDLE);
  assign kmax = (layer == 2'd0) ? (act_r ? 9'd18 : 9'd24) : 9'd255;
  assign rmax = (layer == 2'd2) ? 4'd0 : 4'd15;
  assign iv   = (st == RUN);

  // ---------------- pipeline stage 1 & 2 (ROM + activation) ----------------
  logic [LANES*16-1:0] iw0q, iw0q2, ow0q, ow0q2, iw1q, iw1q2;
  logic [LANES*9-1:0]  ow1q, ow1q2;
  logic [31:0]         w2q, w2q2;
  logic signed [AB-1:0] a1, a2, asel;
  logic [LANES*AB-1:0]  hw;

  logic act1, act2, act3, act4;
  logic [1:0] l1, l2, l3, l4, l5, l6;
  logic v1, v2, v3, v4;
  logic f1, f2, f3, e1, e2, e3, e4, e5, e6;
  logic [3:0] r1, r2, r3, r4, r5, r6;

  always_comb begin : act_select
    hw = (layer == 2'd1) ? ha[k[7:4]] : hb[k[7:4]];
    if (layer == 2'd0) asel = AB'(xm[{act_r, k[4:0]}]);
    else               asel = hw[k[3:0] * AB +: AB];
  end

  logic [LANES*BSB-1:0] bsw_q;
  logic [2*BSB-1:0]     bs2_q;

  always_ff @(posedge clk) begin
    iw0q  <= iw0m[addr[8:0]];
    ow0q  <= ow0m[addr[8:0]];
    iw1q  <= iw1m[addr];
    ow1q  <= ow1m[addr];
    w2q   <= w2m[{act_r, addr[7:0]}];
    a1    <= asel;
    act1  <= act_r; v1 <= iv; l1 <= layer; f1 <= (k == 0); e1 <= (k == kmax); r1 <= row;

    iw0q2 <= iw0q; ow0q2 <= ow0q; iw1q2 <= iw1q; ow1q2 <= ow1q; w2q2 <= w2q;
    a2    <= a1;
    act2  <= act1; v2 <= v1; l2 <= l1; f2 <= f1; e2 <= e1; r2 <= r1;

    act3  <= act2; v3 <= v2; l3 <= l2; f3 <= f2; e3 <= e2; r3 <= r2;

    // Synchronous bias+shift read overlaps stage 3 -> stage 4
    bsw_q <= bsm[{act3, l3[0], r3}];
    bs2_q <= bs2m[act3];
    act4  <= act3; v4 <= v3; l4 <= l3; e4 <= v3 & e3; r4 <= r3;

    e5    <= e4; l5 <= l4; r5 <= r4;
    e6    <= e5; l6 <= l5; r6 <= r5;

    if (rst) begin
      v1 <= 0; v2 <= 0; v3 <= 0; v4 <= 0;
      e4 <= 0; e5 <= 0; e6 <= 0;
    end
  end

  // ---------------- 16 DSP48 MAC lanes + pipelined 2-step shifter ----------------
  logic [LANES*AB-1:0] hword;
  logic signed [AB-1:0] zl [2];

  genvar g;
  generate for (g = 0; g < LANES; g++) begin : lane
    logic signed [17:0] w;
    logic signed [42:0] prod;
    logic signed [47:0] acc, acc_c, s;
    logic [BSB-1:0]     bs_lane;
    logic [4:0]         shm1;
    logic [2:0]         sh_lo;
    logic signed [AB-1:0] b1, b2;
    logic signed [48:0] v;

    always_comb begin
      if (g < 2) begin
        case (l2)
          2'd0:    w = act2 ? 18'(signed'(ow0q2[g*16 +: 16])) : 18'(signed'(iw0q2[g*16 +: 16]));
          2'd1:    w = act2 ? 18'(signed'(ow1q2[g*9  +:  9])) : 18'(signed'(iw1q2[g*16 +: 16]));
          default: w = 18'(signed'(w2q2[(g&1)*16 +: 16]));
        endcase
        bs_lane = (l4 == 2'd2) ? bs2_q[(g&1)*BSB +: BSB] : bsw_q[g*BSB +: BSB];
      end else begin
        case (l2)
          2'd0:    w = act2 ? 18'(signed'(ow0q2[g*16 +: 16])) : 18'(signed'(iw0q2[g*16 +: 16]));
          2'd1:    w = act2 ? 18'(signed'(ow1q2[g*9  +:  9])) : 18'(signed'(iw1q2[g*16 +: 16]));
          default: w = 18'sd0;
        endcase
        bs_lane = (l4 == 2'd2) ? BSB'(0) : bsw_q[g*BSB +: BSB];
      end
      shm1 = bs_lane[AB +: 5];
    end

    always_ff @(posedge clk) begin
      prod <= a2 * w;                                         // stage 3 (DSP M)
      if (v3) acc <= (f3 ? 48'sd0 : acc) + 48'(prod);         // stage 4 (DSP P)

      // stage 5a: coarse shift by {0, 8, 16, 24} (single LUT6 4:1 mux)
      case (shm1[4:3])
        2'd0: acc_c <= acc;
        2'd1: acc_c <= acc >>> 8;
        2'd2: acc_c <= acc >>> 16;
        2'd3: acc_c <= acc >>> 24;
      endcase
      sh_lo <= shm1[2:0];
      b1    <= bs_lane[AB-1:0];

      // stage 5b: fine shift by 0..7 + round-half-up
      s     <= ((acc_c >>> sh_lo) + 48'sd1) >>> 1;
      b2    <= b1;
    end

    assign v = 49'(s) + 49'(b2);
    always_comb begin
      if (l6 == 2'd2)
        hword[g*AB +: AB] = (v > ACT_MAX_49) ? AB'(ACT_MAX) : (v < -ACT_MAX_49) ? AB'(-ACT_MAX) : AB'(v);
      else
        hword[g*AB +: AB] = (v < 49'sd0) ? '0 : (v > ACT_MAX_49) ? AB'(ACT_MAX) : AB'(v);
    end
    if (g < 2) begin : outz
      assign zl[g] = hword[g*AB +: AB];
    end
  end endgenerate

  always_ff @(posedge clk) begin
    if (e6 && l6 == 2'd0) ha[r6] <= hword;
    if (e6 && l6 == 2'd1) hb[r6] <= hword;
    if (e6 && l6 == 2'd2) begin z0 <= zl[0]; z1 <= zl[1]; end
  end

  // ---------------- synchronous TDP tanh BRAM (shared with rl_ctrl) ----------------
  logic [9:0]  ta0, ta1;
  logic [15:0] tq0, tq1, t0_r;
  always_ff @(posedge clk) begin
    tq0 <= tm[ta0];
    tq1 <= tm[ta1];
  end
  assign ext_t0 = tq0;
  assign ext_t1 = tq1;

  wire [AB-1:0]  z0_mag = z0[AB-1] ? AB'(-z0) : AB'(z0);
  wire [AB-9:0]  z0_idx = z0_mag[AB-1:8];
  wire [AB-1:0]  z1_mag = z1[AB-1] ? AB'(-z1) : AB'(z1);
  wire [AB-9:0]  z1_idx = z1_mag[AB-1:8];

  logic [7:0]  th_f0, th_f1;
  logic        th_s0, th_s1, th_sat0, th_sat1;
  logic signed [26:0] tp;
  logic signed [17:0] t_sum;

  assign t_sum = $signed({2'b0, t0_r}) + 18'(tp >>> 8);

  // ---------------- FSM ----------------
  always_ff @(posedge clk) begin
    done <= 1'b0;
    if (rst) begin
      st <= IDLE; act_r <= 0; layer <= 0; addr <= 0; k <= 0; row <= 0; drain <= 0;
      ta0 <= 0; ta1 <= 0;
    end else case (st)
      IDLE: begin
        if (start) begin
          st <= RUN; act_r <= actor; layer <= 0; addr <= 0; k <= 0; row <= 0;
        end else if (ext_th_we) begin
          ta0 <= ext_th_a; ta1 <= ext_th_b;
        end
      end
      RUN: begin
        addr <= addr + 1'b1;
        if (k == kmax) begin
          k <= 0;
          if (row == rmax) begin st <= DRAIN; drain <= 0; end
          else row <= row + 1'b1;
        end else k <= k + 1'b1;
      end
      DRAIN: begin
        drain <= drain + 1'b1;
        if (drain == 4'd6) begin
          if (layer == 2'd2) begin
            th_sat0 <= (z0_idx >= 512);
            th_f0   <= z0_mag[7:0];
            th_s0   <= z0[AB-1];
            ta0     <= (z0_idx >= 512) ? 10'd0 : z0_idx[9:0];
            ta1     <= (z0_idx >= 512) ? 10'd1 : (z0_idx[9:0] + 10'd1);
            st      <= TANH_RD0;
          end else begin
            layer <= layer + 1'b1; st <= RUN; addr <= 0; k <= 0; row <= 0;
          end
        end
      end
      TANH_RD0: begin
        th_sat1 <= (z1_idx >= 512);
        th_f1   <= z1_mag[7:0];
        th_s1   <= z1[AB-1];
        ta0     <= (z1_idx >= 512) ? 10'd0 : z1_idx[9:0];
        ta1     <= (z1_idx >= 512) ? 10'd1 : (z1_idx[9:0] + 10'd1);
        st      <= TANH_RD1;
      end
      TANH_RD1: begin
        tp   <= 27'((18'($signed({1'b0, tq1})) - 18'($signed({1'b0, tq0}))) * 18'($signed({1'b0, th_f0}))) + 27'sd128;
        t0_r <= tq0;
        st   <= TANH_C0;
      end
      TANH_C0: begin
        y0   <= th_s0 ? 16'(-(th_sat0 ? 18'sd32767 : t_sum)) : 16'(th_sat0 ? 18'sd32767 : t_sum);
        tp   <= 27'((18'($signed({1'b0, tq1})) - 18'($signed({1'b0, tq0}))) * 18'($signed({1'b0, th_f1}))) + 27'sd128;
        t0_r <= tq0;
        st   <= TANH_C1;
      end
      TANH_C1: begin
        y1   <= th_s1 ? 16'(-(th_sat1 ? 18'sd32767 : t_sum)) : 16'(th_sat1 ? 18'sd32767 : t_sum);
        done <= 1'b1;
        st   <= IDLE;
      end
      default: st <= IDLE;
    endcase
  end
endmodule
