`timescale 1ns/1ps
module tb_driver_diag_top;
 reg clk=0;always #5 clk=~clk;
 reg reset_button=0,rx=1,fault_n=0;
 tri1 scl,sda;
 reg slave_low=0;assign sda=slave_low?1'b0:1'bz;
 wire [5:0] motor;wire tx;wire [3:0] led;
 reg [7:0] status=8'h17;
 driver_diag_top #(.POLL_CYCLES(10000),.TELEMETRY_CYCLES(20000),.I2C_CLK_HZ(4000000),.UART_DIV(16),
  .MS_CYCLES(100),.WAKE_CYCLES(100),.KEEPALIVE_CYCLES(500000)) dut(
  .clk(clk),.reset_button(reset_button),.fault_n(fault_n),.vaux4_p(1'b0),.vaux4_n(1'b0),.uart_rx(rx),
  .as5600_scl(scl),.as5600_sda(sda),.pwm_a(motor[5]),.pwm_b(motor[4]),.pwm_c(motor[3]),
  .enable(motor[2]),.reset_n(motor[1]),.sleep_n(motor[0]),.uart_tx(tx),.led(led));
 always @(negedge clk) if(motor[5:2]!==0) $fatal(1,"Diagnostic motor controls not constant LOW");
 task get_byte(input [7:0] expected);
  reg [7:0] b;integer i;
  begin
   for(i=7;i>=0;i=i-1) begin @(posedge scl);#1;b[i]=sda;end
   if(b!==expected) $fatal(1,"I2C address/register mismatch");
   @(negedge scl);slave_low=1;@(posedge scl);@(negedge scl);slave_low=0;
  end
 endtask
 task put_byte(input [7:0] b,input bit ack);
  integer i;
  begin
   for(i=7;i>=0;i=i-1) begin slave_low=~b[i];@(posedge scl);@(negedge scl);end
   slave_low=0;@(posedge scl);#1;if(sda!==!ack) $fatal(1,"I2C read ACK");@(negedge scl);
  end
 endtask
 initial begin
  #1; // Let initialized open-drain outputs settle before recognizing START.
  forever begin
  @(negedge sda);if(scl!==1) $fatal(1,"I2C START");
  get_byte(8'h6c);get_byte(8'h0b);
  @(negedge sda);if(scl!==1) $fatal(1,"I2C RESTART");
  get_byte(8'h6d);put_byte(status,1);put_byte(8'h01,1);put_byte(8'h20,0);
  @(posedge sda);if(scl!==1) $fatal(1,"I2C STOP");
  end
 end
 task send(input string s);
  integer i,j;
  begin
   for(i=0;i<s.len();i=i+1) begin
    @(negedge clk);rx=0;repeat(16) @(negedge clk);
    for(j=0;j<8;j=j+1) begin rx=(s[i]>>j)&1;repeat(16) @(negedge clk);end
    rx=1;repeat(20) @(negedge clk);
   end
   repeat(5) @(negedge clk);
  end
 endtask
 reg [7:0] ch;reg [663:0] line=0;integer i,j;
 initial begin
  for(i=0;i<83;i=i+1) begin
   @(negedge tx);repeat(24) @(posedge clk);
   for(j=0;j<8;j=j+1) begin ch[j]=tx;repeat(16) @(posedge clk);end
   if(tx!==1) $fatal(1,"TX stop");line={line[655:0],ch};
  end
  if(line!=={"D1 a=654 v=1 c=0 i=1 e=0 s=17 r=120 b=3 f=0 m=0 q=0 n=0000 t=0000 p=00 g=0 x=0000",8'h0d,8'h0a}) $fatal(1,"D1 packet mismatch: %s",line);
  send("SWEEP\n");if(dut.state!=0 || dut.accepted!=1 || dut.reject_count!=1) $fatal(1,"diagnostic SWEEP accepted");
  // Fault low and magnet missing deliberately do not prevent a no-drive wake probe.
  send("ARM\n");wait(dut.state==2);#1;if(motor[5:2]!==0 || motor[1:0]!==3) $fatal(1,"wake controls");
  if(dut.enc_status[5]) $fatal(1,"fixture must lack magnet");
  send("SWEEP\n");if(dut.state!=2 || dut.reject_count!=2 || motor[5:2]!==0) $fatal(1,"SWEEP changed diagnostic state");
  fault_n=1;repeat(10) @(negedge clk);if(!dut.fault_telemetry[1] || !led[2]) $fatal(1,"live FAULT high missed");
  fault_n=0;repeat(10) @(negedge clk);if(dut.fault_telemetry[1] || led[2]) $fatal(1,"live FAULT low missed");
  send("!");if(motor!==0 || dut.state!=0 || dut.result!=2) $fatal(1,"UART ! failed");
  send("\nARM\n");wait(dut.state==2);
  @(negedge clk);reset_button=1;repeat(10) @(negedge clk);reset_button=0;
  repeat(10) @(negedge clk);if(motor!==0 || dut.state!=0) $fatal(1,"button reset");
  $display("PASS DIAG TOP: no-drive invariant, D1 UART/I2C, missing magnet and low nFAULT permitted only for hard-disabled wake probe, SWEEP rejection, live FAULT LED, STOP/button");$finish;
 end
 initial begin #5000000;$fatal(1,"top timeout state=%d",dut.state);end
endmodule
