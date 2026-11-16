`timescale 1ns/1ps
module tb_top;
 reg clk=0; always #5 clk=~clk;
 reg reset_button=0;
 tri1 scl,sda;
 wire [5:0] motor;
 wire tx;wire [3:0] led;
 sensor_diag_top #(.POLL_CYCLES(10000),.I2C_CLK_HZ(4000000),.UART_DIV(16)) dut(
 .clk(clk),.reset_button(reset_button),.fault_n(1'b0),.vaux4_p(1'b0),.vaux4_n(1'b0),
 .as5600_scl(scl),.as5600_sda(sda),.pwm_a(motor[5]),.pwm_b(motor[4]),.pwm_c(motor[3]),
 .enable(motor[2]),.reset_n(motor[1]),.sleep_n(motor[0]),.uart_tx(tx),.led(led));
 always @(posedge clk) if(motor!==0) $fatal(1,"motor output not LOW");
 reg [7:0] ch;reg [359:0] line;integer i,j;
 task get_line;
 begin
  for(i=0;i<45;i=i+1) begin
   @(negedge tx); repeat(24) @(posedge clk);
   for(j=0;j<8;j=j+1) begin ch[j]=tx;repeat(16) @(posedge clk);end
   if(tx!==1) $fatal(1,"UART stop bit");
   line={line[351:0],ch};
  end
 end endtask
 initial begin
  get_line;
  if(line!=={"S1 a=654 v=1 c=0 i=0 e=2 s=00 r=000 b=3 f=0",8'h0d,8'h0a}) $fatal(1,"bad packet: %s",line);
  // No encoder slave attached => address NACK. Raw data still useful.
  dut.adc.adc.sample=12'hfff;
  repeat(100) @(posedge clk); if(!dut.adc_clip) $fatal(1,"clip missed");
  dut.adc.adc.CHANNEL=5'h15;dut.adc.adc.sample=12'h123;
  repeat(100) @(posedge clk); if(dut.adc_raw!==12'hfff) $fatal(1,"wrong channel accepted");
  dut.adc.adc.run_adc=0;
  repeat(1000100) @(posedge clk); if(dut.adc_fresh) $fatal(1,"stale ADC accepted");
  reset_button=1;repeat(10) @(posedge clk); reset_button=0;
  repeat(10) @(posedge clk); if(dut.adc_fresh) $fatal(1,"reset freshness");
  $display("PASS TOP: full UART packet decoded; motor pins always LOW; ADC channel/clip/stale/reset");$finish;
 end
 initial begin #20000000;$fatal(1,"timeout");end
endmodule
