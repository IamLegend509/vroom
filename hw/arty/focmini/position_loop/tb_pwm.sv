`timescale 1ns/1ps
module tb_pwm;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,arm=0,start=0,stop=0,done=0;
 wire a,b,c,en;wire [2:0] state;wire [12:0] da,db,dc;
 // Production PWM period and millisecond divisor, only wake delay accelerated.
 position_controller #(.WAKE_CYCLES(20)) dut(.clk(clk),.rst(rst),.fault_n(1'b1),
  .arm(arm),.sweep(start),.stop(stop),.ping(1'b1),.uart_error(1'b0),
  .adc_fresh(1'b1),.adc_clip(1'b0),.enc_done(done),.enc_ok(1'b1),.enc_status(8'h37),.enc_angle(12'd100),
  .pwm_a(a),.pwm_b(b),.pwm_c(c),.enable(en),.sleep_n(),.reset_n(),.state(state),.result(),
  .rejected(),.elapsed_ms(),.phase_index(),.active_a(da),.active_b(db),.active_c(dc),
  .target(),.position_error(),.phase_offset(),.stage(),.calibrated(),.passed());
 integer i,j,na,nb,nc;
 initial begin
  repeat(5) @(negedge clk);rst=0;done=1;@(negedge clk);done=0;
  arm=1;@(negedge clk);arm=0;wait(state==3);
  @(negedge clk);start=1;@(negedge clk);start=0;wait(en);@(negedge clk);
  for(j=0;j<4;j=j+1) begin
   na=0;nb=0;nc=0;
   for(i=0;i<5000;i=i+1) begin
    if(dut.carrier!=i) $fatal(1,"carrier not 5000 clocks");
    na=na+a;nb=nb+b;nc=nc+c;@(negedge clk);
   end
   if(na!=2500 || nb!=2211 || nc!=2789) $fatal(1,"production PWM widths %d %d %d",na,nb,nc);
  end
  repeat(800) @(negedge clk);stop=1;#1;
  if(en || a || b || c) $fatal(1,"mid-period stop delayed");
  $display("PASS PWM: production 100MHz / 5000 = 20kHz; exact three phase high times; immediate mid-period stop");$finish;
 end
 initial begin #1000000;$fatal(1,"PWM timeout");end
endmodule
