`timescale 1ns/1ps
module tb_uart;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,rx=1;
 wire [7:0] data;wire valid,error,arm,sweep,stop,ping,rejected;
 wire [15:0] accepted;
 sweep_uart_rx #(.DIV(16)) receiver(.clk(clk),.rst(rst),.rx(rx),.data(data),.valid(valid),.framing_error(error));
 position_commands #(.GAP_CYCLES(4000)) parser(.clk(clk),.rst(rst),.valid(valid),.framing_error(error),.data(data),
  .arm(arm),.sweep(sweep),.stop(stop),.ping(ping),.rejected(rejected),.accepted(accepted));
 integer arms=0,sweeps=0,stops=0,pings=0,errors=0;
 always @(posedge clk) begin
  if(arm) arms=arms+1;if(sweep) sweeps=sweeps+1;if(stop) stops=stops+1;if(ping) pings=pings+1;
  if(error) errors=errors+1;
 end
 task byte_tx(input [7:0] b,input bit good_stop);
  integer j;
  begin
   @(negedge clk);rx=0;repeat(16) @(negedge clk);
   for(j=0;j<8;j=j+1) begin rx=b[j];repeat(16) @(negedge clk);end
   rx=good_stop;repeat(16) @(negedge clk);
   if(good_stop) repeat(3) @(negedge clk);
  end
 endtask
 task send(input string s);
  integer j;
  begin for(j=0;j<s.len();j=j+1) byte_tx(s[j],1);repeat(5) @(negedge clk);end
 endtask
 // A second parser exercises a newline arriving exactly at the expiry boundary.
 reg pv=0;reg [7:0] pd=0;wire pa;wire [15:0] pc;
 position_commands #(.GAP_CYCLES(20)) expiry(.clk(clk),.rst(rst),.valid(pv),.framing_error(1'b0),.data(pd),
  .arm(pa),.sweep(),.stop(),.ping(),.rejected(),.accepted(pc));
 task direct(input [7:0] b);
  begin @(negedge clk);pd=b;pv=1;@(negedge clk);pv=0;end
 endtask
 initial begin
  repeat(5) @(negedge clk);rst=0;
  send("TEST\nARM\nPING\nSTOP\n");
  if({arms,sweeps,pings,stops}!=={32'd1,32'd1,32'd1,32'd1}) $fatal(1,"exact commands failed");
  send("xARM\n12345ARM\narm\n AR M\n");
  byte_tx(0,1);send("ARM\n");
  send("AR");repeat(4100) @(negedge clk);send("M\n");
  if(accepted!==4) $fatal(1,"invalid suffix or timeout accepted");
  send("ARM");byte_tx(8'h0d,1);byte_tx(8'h0a,1);
  if(arms!=2 || accepted!=5) $fatal(1,"CRLF duplicated command");
  send("xxx!ARM\n");if(stops!=2 || arms!=2) $fatal(1,"immediate stop/suffix");
  byte_tx(8'h00,0);repeat(1000) @(negedge clk);
  if(errors!=1) $fatal(1,"BREAK error must be one-shot");
  rx=1;repeat(100) @(negedge clk);send("ARM\n");
  if(arms!=2) $fatal(1,"framing error suffix accepted");
  send("ARM\n");if(arms!=3) $fatal(1,"framing recovery failed");
  // Short idle-line glitch must not become a byte.
  rx=0;repeat(3) @(negedge clk);rx=1;repeat(180) @(negedge clk);
  if(arms!=3 || errors!=1) $fatal(1,"start glitch accepted");
  direct("A");direct("R");direct("M");
  wait(expiry.age==19);@(negedge clk);pd=10;pv=1;@(negedge clk);pv=0;
  repeat(3) @(negedge clk);if(pa || pc!=0) $fatal(1,"newline at expiry accepted");
  $display("PASS UART: exact commands, CRLF, overflow, nonprintable, gap boundary, suffix rejection, immediate stop, BREAK and recovery");$finish;
 end
 initial begin #2000000;$fatal(1,"UART timeout");end
endmodule
