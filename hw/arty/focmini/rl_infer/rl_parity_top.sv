// On-silicon inference parity image (P1). Motor interface is held inert.
//
// Purpose: run the two quantized actors on the Arty A7-35T with host-supplied
// observations so the integer outputs of the real device can be compared with
// quant.py. No sensor, PWM or driver activity: pwm/enable/reset_n/sleep_n are
// constant LOW (DRI0058 asleep, gates off).
// Protocol (115200 8-N-1):
//   host -> FPGA: A5, actor (0 inner / 1 outer), N x int16 LE (N = 25 / 19), xor
//   FPGA -> host: 5A, actor, y0 i16, y1 i16, z0 i32, z1 i32, cycles u16, xor  (17 bytes)
//                 (all LE; xor covers every preceding byte of the frame)
//   on bad actor or checksum: EE, code (1 actor, 2 checksum), xor
// Repro: python3 hw/arty/focmini/rl_infer/run.py [--build]
module rl_parity_top #(parameter integer DIV = 868) (
  input  wire clk,
  input  wire reset_button,
  input  wire uart_rx,
  output wire uart_tx,
  output wire pwm_a, pwm_b, pwm_c, enable, reset_n, sleep_n,
  output wire [3:0] led
);
  assign {pwm_a, pwm_b, pwm_c, enable, reset_n, sleep_n} = 6'b0;

  (* ASYNC_REG = "TRUE" *) reg [1:0] button_sync = 2'b00;
  always @(posedge clk) button_sync <= {button_sync[0], reset_button};
  wire rst = button_sync[1];

  wire [7:0] rx_data; wire rx_valid, rx_ferr;
  sweep_uart_rx #(.DIV(DIV)) rx (.clk(clk), .rst(rst), .rx(uart_rx), .data(rx_data), .valid(rx_valid), .framing_error(rx_ferr));

  // ---------------- actors ----------------
  logic iwe, owe, istart, ostart, ibusy, obusy, idone, odone;
  logic [4:0] xaddr;
  logic signed [15:0] xdata, iy0, iy1, oy0, oy1;
  logic signed [24:0] iz0, iz1, oz0, oz1;
  rl_mlp #(.NIN(25), .LANES(32), .W1B(16)) inner (.clk, .rst, .x_we(iwe), .x_addr(xaddr),
    .x_data(xdata), .start(istart), .busy(ibusy), .done(idone), .y0(iy0), .y1(iy1), .z0(iz0), .z1(iz1));
  rl_mlp #(.NIN(19), .LANES(16), .W1B(9),
    .W0F("outer_w0.mem"), .W1F("outer_w1.mem"), .W2F("outer_w2.mem"),
    .B0F("outer_bs0.mem"), .B1F("outer_bs1.mem"), .B2F("outer_bs2.mem")) outer (.clk, .rst,
    .x_we(owe), .x_addr(xaddr), .x_data(xdata), .start(ostart), .busy(obusy), .done(odone),
    .y0(oy0), .y1(oy1), .z0(oz0), .z1(oz1));

  // ---------------- frame receive / reply ----------------
  typedef enum logic [2:0] {HDR, ACT, LO, HI, CK, RUNNING, SEND} st_t;
  st_t st;
  logic actor;
  logic [7:0] lo, x, txx;
  logic [4:0] nlast;
  logic [15:0] cycles;
  logic [7:0] tbuf [18];
  logic [4:0] tlen, tidx;
  logic tstart, tbusy; logic [7:0] tdata;
  logic err_led, ok_led;
  diag_uart_tx #(.CLKS_PER_BIT(DIV)) tx (.clk(clk), .rst(rst), .start(tstart), .data(tdata), .busy(tbusy), .tx(uart_tx));

  wire signed [15:0] ry0 = actor ? oy0 : iy0, ry1 = actor ? oy1 : iy1;
  wire signed [31:0] rz0 = 32'(actor ? oz0 : iz0), rz1 = 32'(actor ? oz1 : iz1);
  wire adone = actor ? odone : idone;

  always_ff @(posedge clk) begin
    {iwe, owe, istart, ostart, tstart} <= '0;
    if (iwe || owe) xaddr <= xaddr + 1'b1;               // after each x write
    if (rst) begin
      st <= HDR; tlen <= 0; tidx <= 0; err_led <= 0; ok_led <= 0; xaddr <= 0;
    end else case (st)
      HDR: if (rx_valid && rx_data == 8'hA5) begin x <= 8'hA5; st <= ACT; end
      ACT: if (rx_valid) begin
        x <= x ^ rx_data; xaddr <= 0; actor <= rx_data[0];
        nlast <= rx_data[0] ? 5'd18 : 5'd24;
        if (rx_data[7:1] != 0) begin                     // unknown actor: reject now
          tbuf[0] <= 8'hEE; tbuf[1] <= 8'h01; tlen <= 3; tidx <= 0; txx <= 0;
          err_led <= 1; st <= SEND;
        end else st <= LO;
      end
      LO: if (rx_valid) begin lo <= rx_data; x <= x ^ rx_data; st <= HI; end
      HI: if (rx_valid) begin
        x <= x ^ rx_data; xdata <= {rx_data, lo};
        if (actor) owe <= 1; else iwe <= 1;
        if (xaddr == nlast) st <= CK; else st <= LO;
      end
      CK: if (rx_valid) begin
        if (rx_data != x) begin
          tbuf[0] <= 8'hEE; tbuf[1] <= 8'h02; tlen <= 3; tidx <= 0; txx <= 0;
          err_led <= 1; st <= SEND;
        end else begin
          if (actor) ostart <= 1; else istart <= 1;
          cycles <= 0; st <= RUNNING;
        end
      end
      RUNNING: begin
        cycles <= cycles + 1'b1;
        if (adone) begin
          tbuf[0] <= 8'h5A; tbuf[1] <= {7'd0, actor};
          tbuf[2] <= ry0[7:0]; tbuf[3] <= ry0[15:8]; tbuf[4] <= ry1[7:0]; tbuf[5] <= ry1[15:8];
          tbuf[6] <= rz0[7:0]; tbuf[7] <= rz0[15:8]; tbuf[8] <= rz0[23:16]; tbuf[9] <= rz0[31:24];
          tbuf[10] <= rz1[7:0]; tbuf[11] <= rz1[15:8]; tbuf[12] <= rz1[23:16]; tbuf[13] <= rz1[31:24];
          tbuf[14] <= cycles[7:0]; tbuf[15] <= cycles[15:8];
          tlen <= 17; tidx <= 0; txx <= 0; ok_led <= ~ok_led; st <= SEND;
        end
      end
      SEND: begin
        if (tidx == tlen) st <= HDR;
        else if (!tbusy && !tstart) begin
          tstart <= 1; tidx <= tidx + 1'b1;
          if (tidx == tlen - 1'b1) tdata <= txx;
          else begin tdata <= tbuf[tidx]; txx <= txx ^ tbuf[tidx]; end
        end
      end
      default: st <= HDR;
    endcase
  end

  reg [26:0] hb = 0;
  always @(posedge clk) hb <= hb + 1'b1;
  assign led = {ibusy | obusy, err_led, ok_led, hb[26]};
endmodule
