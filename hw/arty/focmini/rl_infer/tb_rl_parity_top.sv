// UART-level test of the parity image: valid inner/outer frames return the
// integer-model outputs; a corrupted checksum returns EE 02; motor pins stay LOW.
`timescale 1ns/1ps
module tb_rl_parity_top;
  localparam integer DIV = 16;
  reg clk = 0, btn = 1, rx = 1;
  wire tx, pa, pb, pc, en, rn, sn; wire [3:0] led;
  always #5 clk = ~clk;
  rl_parity_top #(.DIV(DIV)) dut (.clk(clk), .reset_button(btn), .uart_rx(rx), .uart_tx(tx),
    .pwm_a(pa), .pwm_b(pb), .pwm_c(pc), .enable(en), .reset_n(rn), .sleep_n(sn), .led(led));
  reg [15:0] xi [0:518*25-1]; reg [15:0] xo [0:518*19-1];
  integer yi [0:518*2-1]; integer yo [0:518*2-1];
  integer fails = 0, i, j, f, a, b, nrx = 0;
  reg [7:0] rq [0:63];
  reg [7:0] x, c, rb;
  reg [15:0] v;
  integer got0, got1, cyc;
  always @(posedge clk) if ({pa, pb, pc, en, rn, sn} !== 6'b0) begin $display("FAIL motor pin active"); $finish; end
  // free-running receiver
  always begin
    @(negedge tx); repeat (DIV + DIV/2) @(posedge clk);
    for (i = 0; i < 8; i = i + 1) begin rb[i] = tx; repeat (DIV) @(posedge clk); end
    rq[nrx] = rb; nrx = nrx + 1;
  end
  task send(input [7:0] bt);
    integer k;
    begin
      rx = 0; repeat (DIV) @(posedge clk);
      for (k = 0; k < 8; k = k + 1) begin rx = bt[k]; repeat (DIV) @(posedge clk); end
      rx = 1; repeat (DIV) @(posedge clk);
    end
  endtask
  task frame(input integer actor, input integer n, input integer corrupt);
    integer nin, k, want;
    begin
      nin = actor ? 19 : 25; x = 8'hA5; nrx = 0;
      send(8'hA5); send(actor); x = x ^ actor;
      for (k = 0; k < nin; k = k + 1) begin
        v = actor ? xo[n*19+k] : xi[n*25+k];
        send(v[7:0]); send(v[15:8]); x = x ^ v[7:0] ^ v[15:8];
      end
      send(corrupt ? ~x : x);
      want = corrupt ? 3 : 17;
      while (nrx < want) @(posedge clk);
      if (corrupt) begin
        if (rq[0] != 8'hEE || rq[1] != 8'h02 || rq[2] != (8'hEE ^ 8'h02)) begin
          $display("FAIL error frame %h %h %h", rq[0], rq[1], rq[2]); fails = fails + 1; end
        else $display("PASS corrupted checksum rejected (EE 02)");
      end else begin
        c = 0; for (k = 0; k < 16; k = k + 1) c = c ^ rq[k];
        got0 = $signed({rq[3], rq[2]}); got1 = $signed({rq[5], rq[4]}); cyc = {rq[15], rq[14]};
        if (rq[0] != 8'h5A || rq[1] != actor || rq[16] != c ||
            got0 != (actor ? yo[2*n] : yi[2*n]) || got1 != (actor ? yo[2*n+1] : yi[2*n+1])) begin
          $display("FAIL actor %0d vec %0d got %0d %0d", actor, n, got0, got1); fails = fails + 1;
        end else $display("PASS actor %0d vec %0d y=(%0d,%0d) cycles=%0d", actor, n, got0, got1, cyc);
      end
    end
  endtask
  initial begin
    $readmemh("inner_x.mem", xi); $readmemh("outer_x.mem", xo);
    f = $fopen("inner_y.txt", "r"); for (i = 0; i < 518; i = i + 1) begin j = $fscanf(f, "%d %d", a, b); yi[2*i] = a; yi[2*i+1] = b; end $fclose(f);
    f = $fopen("outer_y.txt", "r"); for (i = 0; i < 518; i = i + 1) begin j = $fscanf(f, "%d %d", a, b); yo[2*i] = a; yo[2*i+1] = b; end $fclose(f);
    repeat (5) @(posedge clk); btn = 0; repeat (5) @(posedge clk);
    frame(0, 0, 0); frame(1, 0, 0); frame(0, 517, 0); frame(1, 300, 0); frame(0, 1, 1); frame(1, 7, 0); frame(0, 42, 0);
    if (fails == 0) $display("PASS PARITY_TOP: frames bit-exact, checksum error path, motor pins LOW");
    else $display("FAIL PARITY_TOP fails=%0d", fails);
    $finish;
  end
  initial begin #50_000_000; $display("FAIL TIMEOUT"); $finish; end
endmodule
