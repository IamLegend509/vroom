// Drives every golden vector through both actors; writes outputs for run.py
// to compare bit-exactly against quant.py's integer model.
`timescale 1ns/1ps
module tb_rl_mlp;
  parameter int NV = 518;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;
  logic [15:0] xi [NV*25];
  logic [15:0] xo [NV*19];
  logic iwe, owe, istart, ostart, ibusy, obusy, idone, odone;
  logic [4:0] iaddr, oaddr;
  logic signed [15:0] idata, odata, iy0, iy1, oy0, oy1;
  logic signed [24:0] iz0, iz1, oz0, oz1;
  rl_mlp #(.NIN(25), .LANES(32), .W1B(16)) inner (.clk, .rst, .x_we(iwe), .x_addr(iaddr),
    .x_data(idata), .start(istart), .busy(ibusy), .done(idone), .y0(iy0), .y1(iy1), .z0(iz0), .z1(iz1));
  rl_mlp #(.NIN(19), .LANES(16), .W1B(9),
    .W0F("outer_w0.mem"), .W1F("outer_w1.mem"), .W2F("outer_w2.mem"),
    .B0F("outer_bs0.mem"), .B1F("outer_bs1.mem"), .B2F("outer_bs2.mem")) outer (.clk, .rst,
    .x_we(owe), .x_addr(oaddr), .x_data(odata), .start(ostart), .busy(obusy), .done(odone),
    .y0(oy0), .y1(oy1), .z0(oz0), .z1(oz1));
  integer fi, fo, n, j, t0, ic, oc;
  initial begin
    $readmemh("inner_x.mem", xi); $readmemh("outer_x.mem", xo);
    fi = $fopen("rtl_inner_y.txt", "w"); fo = $fopen("rtl_outer_y.txt", "w");
    {iwe, owe, istart, ostart} = 0;
    repeat (4) @(posedge clk); rst = 0;
    for (n = 0; n < NV; n++) begin
      for (j = 0; j < 25; j++) begin
        @(negedge clk); iwe = 1; iaddr = j; idata = xi[n*25+j];
        if (j < 19) begin owe = 1; oaddr = j; odata = xo[n*19+j]; end else owe = 0;
      end
      @(negedge clk); iwe = 0; owe = 0; istart = 1; ostart = 1; t0 = $time;
      @(negedge clk); istart = 0; ostart = 0;
      ic = 0; oc = 0;
      while (!(ic && oc)) begin
        @(posedge clk);
        if (idone) begin ic = ($time - t0) / 10; $fdisplay(fi, "%0d %0d", iy0, iy1); end
        if (odone) begin oc = ($time - t0) / 10; $fdisplay(fo, "%0d %0d", oy0, oy1); end
      end
      if (n == 0) $display("CYCLES inner=%0d outer=%0d", ic, oc);
    end
    $fclose(fi); $fclose(fo);
    $display("TB_DONE vectors=%0d", NV);
    $finish;
  end
  initial begin #2_000_000_000; $display("TIMEOUT"); $fatal(1); end
endmodule
