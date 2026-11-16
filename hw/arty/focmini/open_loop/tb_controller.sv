`timescale 1ns/1ps
module tb_controller;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,fault_n=1,arm=0,sweep=0,stop=0,ping=0,uart_error=0;
 reg adc_fresh=1,adc_clip=0,enc_done=0,enc_ok=1;
 reg [7:0] enc_status=8'h37;
 reg [11:0] enc_angle=100;
 reg feed_encoder=1,feed_ping=1;
 wire a,b,c,en,sl,re;wire [2:0] state;wire [3:0] result;wire rejected;
 wire [13:0] elapsed;wire [7:0] phase;wire [12:0] da,db,dc;
 sweep_controller #(.PWM_PERIOD(50),.MS_CYCLES(100),.WAKE_CYCLES(20),.ENC_TIMEOUT(300),
  .KEEPALIVE_CYCLES(500),.ARM_CYCLES(1000)) dut(
  .clk(clk),.rst(rst),.fault_n(fault_n),.arm(arm),.sweep(sweep),.stop(stop),.ping(ping),.uart_error(uart_error),
  .adc_fresh(adc_fresh),.adc_clip(adc_clip),.enc_done(enc_done),.enc_ok(enc_ok),.enc_status(enc_status),.enc_angle(enc_angle),
  .pwm_a(a),.pwm_b(b),.pwm_c(c),.enable(en),.sleep_n(sl),.reset_n(re),.state(state),.result(result),
  .rejected(rejected),.elapsed_ms(elapsed),.phase_index(phase),.active_a(da),.active_b(db),.active_c(dc));
 integer count=0,run_cycles=0;
 integer trace,previous_elapsed=-1,previous_state=-1;
 realtime enabled_at;
 initial begin
  trace=$fopen("sweep_trace.csv","w");
  $fdisplay(trace,"sim_ns,state,result,elapsed_ms,phase_index,duty_a_50,duty_b_50,duty_c_50,enable,angle_raw");
 end
 always @(posedge en) enabled_at=$realtime;
 always @(negedge en) if(result==1 && $realtime-enabled_at!=10000000.0)
  $fatal(1,"actual enable duration differs from 1000000 clocks");
 reg [38:0] previous_duties=0;
 always @(negedge clk) begin
  count=count+1;
  enc_done=feed_encoder && count%50==0;
  ping=feed_ping && count%100==0;
  if(en) begin
   if(!sl || !re || state!=5) $fatal(1,"enable before awake/run");
   if(da<21 || da>29 || db<21 || db>29 || dc<21 || dc>29) $fatal(1,"duty out of bound");
   if({da,db,dc}!=previous_duties && dut.carrier!=0) $fatal(1,"non-atomic duty update");
   if(a!==(dut.carrier<da) || b!==(dut.carrier<db) || c!==(dut.carrier<dc)) $fatal(1,"PWM duty wrong");
  end else if({a,b,c}!==0) $fatal(1,"PWM while disabled");
  previous_duties={da,db,dc};
  if(elapsed!=previous_elapsed || state!=previous_state) begin
   $fdisplay(trace,"%0.0f,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",$realtime,state,result,elapsed,phase,da,db,dc,en,enc_angle);
   previous_elapsed=elapsed;previous_state=state;
  end
 end
 task reset_all;
  begin
   @(negedge clk);rst=1;arm=0;sweep=0;stop=0;fault_n=1;adc_fresh=1;adc_clip=0;
   enc_ok=1;enc_status=8'h37;enc_angle=100;feed_encoder=1;feed_ping=1;uart_error=0;
   repeat(10) @(negedge clk);rst=0;repeat(60) @(negedge clk);
   if(state!=0 || {a,b,c,en,sl,re}!==0) $fatal(1,"reset not disarmed");
  end
 endtask
 task arm_only;
  begin
   @(negedge clk);arm=1;@(negedge clk);arm=0;
   wait(state==3 || state==6);@(negedge clk);
   if(state!=3 || en || !sl || !re) $fatal(1,"arm sequence failed result=%d",result);
  end
 endtask
 task launch;
  begin arm_only;@(negedge clk);sweep=1;@(negedge clk);sweep=0;wait(state==5);end
 endtask
 task expect_fault(input [3:0] code);
  begin
   wait(state==6);@(negedge clk);
   if(result!=code || {a,b,c,en,sl,re}!==0) $fatal(1,"fault expected=%d actual=%d",code,result);
   arm=1;sweep=1;repeat(10) @(negedge clk);arm=0;sweep=0;
   if(state!=6 || en) $fatal(1,"fault auto-rearmed");
   stop=1;@(negedge clk);stop=0;if(state!=6) $fatal(1,"STOP cleared fault");
  end
 endtask
 initial begin
  reset_all;
  sweep=1;@(negedge clk);sweep=0;if(state!=0 || en) $fatal(1,"unarmed start");
  adc_fresh=0;arm=1;@(negedge clk);arm=0;repeat(5) @(negedge clk);
  if(state!=0 || en) $fatal(1,"bad ADC armed");
  reset_all;arm_only;wait(state==0);if(result!=3) $fatal(1,"arm expiry");
  reset_all;fault_n=0;arm=1;@(negedge clk);arm=0;expect_fault(4);
  reset_all;launch;
  // Entire 10-second profile, with repeated start attempts during motion.
  run_cycles=0;
  while(state==5) begin
   @(negedge clk);run_cycles=run_cycles+1;
   sweep=elapsed==2000;
   if(elapsed==500 && phase!=0) $fatal(1,"initial hold");
   if(elapsed==2750 && (phase<127 || phase>129)) $fatal(1,"forward midpoint");
   if(elapsed==5000 && phase!=0) $fatal(1,"middle hold");
   if(elapsed==7250 && (phase<127 || phase>129)) $fatal(1,"reverse midpoint");
   if(elapsed==9500 && phase!=0) $fatal(1,"final hold");
  end
  sweep=0;
  if(result!=1 || elapsed!=10000 || run_cycles!=1000001) $fatal(1,"one-shot duration %d elapsed %d result %d",run_cycles,elapsed,result);
  $display("Full sweep clocks=%0d (monitor includes initial RUN edge); elapsed_ms=%0d",run_cycles,elapsed);
  reset_all;launch;@(negedge clk);stop=1;#1;if(en) $fatal(1,"STOP gate delay");@(negedge clk);stop=0;
  if(state!=0 || result!=2) $fatal(1,"STOP state");
  reset_all;launch;@(negedge clk);arm=1;#1;if(en) $fatal(1,"ARM during run not disarmed");@(negedge clk);arm=0;
  if(state!=0) $fatal(1,"ARM restarted motion");
  reset_all;launch;@(negedge clk);#2;fault_n=0;#1;if(en) $fatal(1,"raw fault kill");#1;fault_n=1;#1;if(en) $fatal(1,"short fault not latched");expect_fault(4);
  reset_all;launch;adc_fresh=0;expect_fault(5);
  reset_all;launch;adc_clip=1;expect_fault(5);
  reset_all;launch;enc_ok=0;expect_fault(6);
  reset_all;launch;enc_status=8'h17;expect_fault(6);
  reset_all;launch;enc_status=8'h28;expect_fault(6);
  reset_all;launch;feed_encoder=0;expect_fault(6);
  reset_all;launch;enc_angle=500;expect_fault(7);
  reset_all;launch;feed_ping=0;expect_fault(8);
  reset_all;launch;uart_error=1;expect_fault(9);
  reset_all;enc_angle=4090;repeat(60) @(negedge clk);launch;enc_angle=5;repeat(60) @(negedge clk);
  if(state!=5) $fatal(1,"valid wrap rejected");
  repeat(14) begin enc_angle=enc_angle+100;repeat(60) @(negedge clk);end
  expect_fault(7);
  reset_all;launch;rst=1;#1;if({a,b,c,en,sl,re}!==0) $fatal(1,"reset kill");
  $display("PASS CONTROL: weak-field allowed, missing magnet rejected; full schedule, bounded atomic PWM, startup/arm expiry, repeated commands, stop/reset, short nFAULT, ADC, encoder, wrap/travel, keepalive, UART trips");$finish;
 end
 initial begin #20000000;$fatal(1,"controller timeout state=%d result=%d",state,result);end
endmodule
