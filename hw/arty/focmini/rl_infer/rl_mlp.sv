// Fixed-point inference engine for one vroom actor (Linear/ReLU x2, Linear/tanh).
//
// Purpose: execute the integer contract defined in quant.py bit-exactly.
// Architecture: LANES lock-step MAC lanes (one DSP48 each). Lane l owns hidden
// rows r with r % LANES == l (local row r / LANES). Every cycle all lanes read
// one wide weight word (lane l in bits [l*WB +: WB]) and multiply it by one
// broadcast activation. Layer 2 (two outputs) uses lanes 0 and 1, local row 0.
// Pipeline: issue -> mem read -> mem output reg -> multiply -> accumulate ->
// rounded per-row shift -> bias + ReLU/saturate -> write wide hidden word.
// Units: activations Q.14 in 25-bit signed; outputs Q.15 int16.
// Latency (100 MHz): NPL*NIN + NPL*256 + 256 issue cycles + 3 drains + tanh,
// ~2.5k cycles (25 us) for inner (LANES=32), ~4.7k (47 us) for outer (LANES=16).
// Limits: start is ignored while busy; x must not be written while busy.
module rl_mlp #(
  parameter int NIN = 25,
  parameter int LANES = 32,
  parameter int W1B = 16,
  parameter string W0F = "inner_w0.mem",
  parameter string W1F = "inner_w1.mem",
  parameter string W2F = "inner_w2.mem",
  parameter string B0F = "inner_bs0.mem",
  parameter string B1F = "inner_bs1.mem",
  parameter string B2F = "inner_bs2.mem",
  parameter string TF  = "tanh.mem"
) (
  input  logic clk,
  input  logic rst,
  input  logic x_we,
  input  logic [$clog2(NIN)-1:0] x_addr,
  input  logic signed [15:0] x_data,
  input  logic start,
  output logic busy,
  output logic done,                 // one-cycle pulse; y valid until next done
  output logic signed [15:0] y0,
  output logic signed [15:0] y1,
  output logic signed [24:0] z0,     // saturated pre-tanh values (debug)
  output logic signed [24:0] z1
);
  localparam int H = 256;
  localparam int NPL = H / LANES;
  localparam int LG = $clog2(LANES);
  localparam int AB = 25, BSB = 31;
  localparam int ACT_MAX = (1 << 24) - 1;
  localparam int D0 = NPL * NIN, D1 = NPL * H, D2 = H;
  localparam int AW = $clog2(D1);
  localparam int RW = $clog2(NPL + 1);

  // ---------------- memories ----------------
  (* rom_style = "block" *) logic [LANES*16-1:0]  w0m [D0];
  (* rom_style = "block" *) logic [LANES*W1B-1:0] w1m [D1];
  (* rom_style = "block" *) logic [LANES*16-1:0]  w2m [D2];
  logic [LANES*BSB-1:0] b0m [NPL];
  logic [LANES*BSB-1:0] b1m [NPL];
  logic [LANES*BSB-1:0] b2m [1];
  logic [15:0] tm [513];
  initial begin
    $readmemh(W0F, w0m); $readmemh(W1F, w1m); $readmemh(W2F, w2m);
    $readmemh(B0F, b0m); $readmemh(B1F, b1m); $readmemh(B2F, b2m);
    $readmemh(TF, tm);
  end
  logic signed [15:0] xm [NIN];
  logic [LANES*AB-1:0] ha [NPL];     // layer-0 output
  logic [LANES*AB-1:0] hb [NPL];     // layer-1 output
  always_ff @(posedge clk) if (x_we) xm[x_addr] <= x_data;

  // ---------------- issue counters ----------------
  typedef enum logic [2:0] {IDLE, RUN, DRAIN, TANH0, TANH1, FIN} st_t;
  st_t st;
  logic [1:0] layer;
  logic [AW-1:0] addr;
  logic [8:0] k, kmax;
  logic [RW-1:0] row, rmax;
  logic [3:0] drain;
  logic iv;                          // issue valid this cycle
  assign busy = (st != IDLE);
  assign kmax = (layer == 2'd0) ? 9'(NIN - 1) : 9'(H - 1);
  assign rmax = (layer == 2'd2) ? '0 : RW'(NPL - 1);
  assign iv = (st == RUN);

  // stage 1: memory read + activation select
  logic [LANES*16-1:0] w0q, w2q, w0q2, w2q2;
  logic [LANES*W1B-1:0] w1q, w1q2;
  logic signed [AB-1:0] a1, a2;
  logic [1:0] l1, l2, l3, l4, l5;
  logic v1, v2, v3, v4, v5;
  logic f1, f2, f3, e1, e2, e3, e4, e5;
  logic [RW-1:0] r1, r2, r3, r4, r5;
  logic signed [AB-1:0] asel;
  logic [LANES*AB-1:0] hw;
  always_comb begin : act_select
    hw = (layer == 2'd1) ? ha[k >> LG] : hb[k >> LG];
    if (layer == 2'd0) asel = AB'(xm[k[$clog2(NIN)-1:0]]);
    else               asel = hw[(k & (LANES - 1)) * AB +: AB];
  end
  always_ff @(posedge clk) begin
    w0q <= w0m[addr[$clog2(D0)-1:0]];
    w1q <= w1m[addr];
    w2q <= w2m[addr[7:0]];
    a1 <= asel; v1 <= iv; l1 <= layer; f1 <= (k == 0); e1 <= (k == kmax); r1 <= row;
    w0q2 <= w0q; w1q2 <= w1q; w2q2 <= w2q;
    a2 <= a1; v2 <= v1; l2 <= l1; f2 <= f1; e2 <= e1; r2 <= r1;
    v3 <= v2; l3 <= l2; f3 <= f2; e3 <= e2; r3 <= r2;
    v4 <= v3; l4 <= l3; e4 <= v3 & e3; r4 <= r3;
    v5 <= v4; l5 <= l4; e5 <= e4; r5 <= r4;
    if (rst) begin v1 <= 0; v2 <= 0; v3 <= 0; v4 <= 0; v5 <= 0; e4 <= 0; e5 <= 0; end
  end

  // ---------------- lanes ----------------
  logic [LANES*BSB-1:0] bsw;
  always_comb case (l4)
    2'd0: bsw = b0m[r4];
    2'd1: bsw = b1m[r4];
    default: bsw = b2m[0];
  endcase
  logic [LANES*AB-1:0] hword;
  logic signed [AB-1:0] zl [2];
  genvar g;
  generate for (g = 0; g < LANES; g++) begin : lane
    logic signed [17:0] w;
    logic signed [42:0] prod;
    logic signed [47:0] acc, s;
    logic signed [AB-1:0] b;
    logic signed [48:0] v;
    always_comb case (l2)
      2'd0: w = 18'(signed'(w0q2[g*16 +: 16]));
      2'd1: w = 18'(signed'(w1q2[g*W1B +: W1B]));
      default: w = 18'(signed'(w2q2[g*16 +: 16]));
    endcase
    always_ff @(posedge clk) begin
      prod <= a2 * w;                                     // stage 3 (DSP M)
      if (v3) acc <= (f3 ? 48'sd0 : acc) + 48'(prod);     // stage 4 (DSP P, UG901 MACC form)
      b  <= bsw[g*BSB +: AB];                             // stage 5
      s  <= ((acc >>> (bsw[g*BSB + AB +: 6] - 6'd1)) + 48'sd1) >>> 1;
    end
    assign v = 49'(s) + 49'(b);
    always_comb begin
      if (l5 == 2'd2)
        hword[g*AB +: AB] = (v > ACT_MAX) ? AB'(ACT_MAX) : (v < -ACT_MAX) ? AB'(-ACT_MAX) : AB'(v);
      else
        hword[g*AB +: AB] = (v < 0) ? '0 : (v > ACT_MAX) ? AB'(ACT_MAX) : AB'(v);
    end
    if (g < 2) begin : outz
      assign zl[g] = hword[g*AB +: AB];
    end
  end endgenerate

  always_ff @(posedge clk) begin                         // stage 6: write back
    if (e5 && l5 == 2'd0) ha[r5] <= hword;
    if (e5 && l5 == 2'd1) hb[r5] <= hword;
    if (e5 && l5 == 2'd2) begin z0 <= zl[0]; z1 <= zl[1]; end
  end

  // ---------------- tanh (table + linear interpolation) ----------------
  function automatic logic signed [15:0] tanh_q(input logic signed [AB-1:0] z);
    logic [AB-1:0] m;
    logic [AB-9:0] i;
    logic [7:0] f;
    logic signed [17:0] t0, t1, d;
    logic signed [26:0] p;
    logic signed [17:0] t;
    m = z[AB-1] ? AB'(-z) : AB'(z);
    i = m[AB-1:8];
    f = m[7:0];
    if (i >= 512) t = 18'sd32767;
    else begin
      t0 = 18'(tm[i[9:0]]);
      t1 = 18'(tm[i[9:0] + 10'd1]);
      d = t1 - t0;
      p = d * $signed({1'b0, f}) + 27'sd128;
      t = t0 + 18'(p >>> 8);
    end
    return z[AB-1] ? 16'(-t) : 16'(t);
  endfunction

  // ---------------- control ----------------
  always_ff @(posedge clk) begin
    done <= 1'b0;
    if (rst) begin
      st <= IDLE; layer <= 0; addr <= 0; k <= 0; row <= 0; drain <= 0;
    end else case (st)
      IDLE: if (start) begin st <= RUN; layer <= 0; addr <= 0; k <= 0; row <= 0; end
      RUN: begin
        addr <= addr + 1'b1;
        if (k == kmax) begin
          k <= 0;
          if (row == rmax) begin st <= DRAIN; drain <= 0; end
          else row <= row + 1'b1;
        end else k <= k + 1'b1;
      end
      DRAIN: begin                                       // let writes land
        drain <= drain + 1'b1;
        if (drain == 4'd8) begin
          if (layer == 2'd2) st <= TANH0;
          else begin layer <= layer + 1'b1; st <= RUN; addr <= 0; k <= 0; row <= 0; end
        end
      end
      TANH0: begin y0 <= tanh_q(z0); st <= TANH1; end
      TANH1: begin y1 <= tanh_q(z1); st <= FIN; end
      FIN: begin done <= 1'b1; st <= IDLE; end
      default: st <= IDLE;
    endcase
  end
endmodule
