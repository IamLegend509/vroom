// Drives every golden vector through the shared dual-actor MLP engine; writes
// outputs for run.py to compare bit-exactly against quant.py's integer model.
`timescale 1ns/1ps
module tb_rl_mlp;
  parameter int NV = 518;
  logic clk = 0, rst = 1;
  always #5 clk = ~clk;
  logic [15:0] xi [NV*25];
  logic [15:0] xo [NV*19];
  logic x_we, x_actor, start, actor, busy, done;
  logic [4:0] x_addr;
  logic signed [15:0] x_data, y0, y1;
  logic signed [24:0] z0, z1;

  rl_dual_mlp dut (
    .clk, .rst, .x_we, .x_actor, .x_addr, .x_data,
    .start, .actor,
    .ext_th_we(1'b0), .ext_th_a(10'd0), .ext_th_b(10'd0),
    .ext_t0(), .ext_t1(),
    .busy, .done, .y0, .y1, .z0, .z1
  );

  integer fi, fo, n, j, t0, ic, oc;
  initial begin
    $readmemh("inner_x.mem", xi); $readmemh("outer_x.mem", xo);
    fi = $fopen("rtl_inner_y.txt", "w"); fo = $fopen("rtl_outer_y.txt", "w");
    {x_we, x_actor, start, actor, x_addr, x_data} = 0;
    repeat (4) @(posedge clk); rst = 0;
    for (n = 0; n < NV; n++) begin
      // Run inner
      for (j = 0; j < 25; j++) begin
        @(negedge clk); x_we = 1; x_actor = 0; x_addr = j; x_data = xi[n*25+j];
      end
      @(negedge clk); x_we = 0; actor = 0; start = 1; t0 = $time;
      @(negedge clk); start = 0;
      while (!done) @(posedge clk);
      ic = ($time - t0) / 10;
      $fdisplay(fi, "%0d %0d", y0, y1);

      // Run outer
      for (j = 0; j < 19; j++) begin
        @(negedge clk); x_we = 1; x_actor = 1; x_addr = j; x_data = xo[n*19+j];
      end
      @(negedge clk); x_we = 0; actor = 1; start = 1; t0 = $time;
      @(negedge clk); start = 0;
      while (!done) @(posedge clk);
      oc = ($time - t0) / 10;
      $fdisplay(fo, "%0d %0d", y0, y1);

      if (n == 0) $display("CYCLES inner=%0d outer=%0d", ic, oc);
    end
    $fclose(fi); $fclose(fo);
    $display("TB_DONE vectors=%0d", NV);
    $finish;
  end
  initial begin #2_000_000_000; $display("TIMEOUT"); $fatal(1); end
endmodule
