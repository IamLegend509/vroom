`timescale 1ns/1ps
// Closed-loop RTL with independent nonlinear damped rotor, 1 ms integration.
// Assumed plant only. Accelerated PWM=50, ms=50 gives 10 PWM periods per
// 10ms encoder sample; physical hardware has 200.
// All ms/profile/controller gains are unchanged; hardware PWM checked separately.
module tb_controller;
 reg clk=0;always #5 clk=~clk;
 reg rst=1,fault_n=1,arm=0,sweep=0,stop=0,ping=0,uart_error=0;
 reg adc_fresh=1,adc_clip=0,enc_done=0,enc_ok=1;
 reg [7:0] enc_status=8'h37;
 reg [11:0] enc_angle=100;
 reg feed_encoder=1,feed_ping=1,plant=1,stuck=0;
 integer direction=-1,scenario=0;
 wire a,b,c,en,sl,re;wire [2:0] state,stage;wire [3:0] result,passed;wire rejected,calibrated;
 wire [15:0] elapsed;wire [7:0] phase;wire [12:0] da,db,dc;wire [11:0] target;
 wire signed [11:0] error;wire signed [15:0] offset;
 position_controller #(.PWM_PERIOD(50),.MS_CYCLES(50),.WAKE_CYCLES(20),.ENC_TIMEOUT(10000),
  .KEEPALIVE_CYCLES(50000),.ARM_CYCLES(1000)) dut(
  .clk(clk),.rst(rst),.fault_n(fault_n),.arm(arm),.sweep(sweep),.stop(stop),.ping(ping),.uart_error(uart_error),
  .adc_fresh(adc_fresh),.adc_clip(adc_clip),.enc_done(enc_done),.enc_ok(enc_ok),.enc_status(enc_status),.enc_angle(enc_angle),
  .pwm_a(a),.pwm_b(b),.pwm_c(c),.enable(en),.sleep_n(sl),.reset_n(re),.state(state),.result(result),
  .rejected(rejected),.elapsed_ms(elapsed),.phase_index(phase),.active_a(da),.active_b(db),.active_c(dc),
  .target(target),.position_error(error),.phase_offset(offset),.stage(stage),.calibrated(calibrated),.passed(passed));
 integer count=0,trace,previous_elapsed=-1,previous_state=-1;
 real rotor=100.0,velocity=0.0,base=100.0,effort,accel,field,friction=300.0,load=0.0;
 reg [38:0] previous_duties=0;
 initial begin
  trace=$fopen("position_trace.csv","w");
  $fdisplay(trace,"scenario,elapsed_ms,state,result,angle_raw,target_raw,error_raw,phase_offset,enable,passed,rotor_counts");
 end
 always @(negedge clk) begin
  count=count+1;
  ping=feed_ping && count%5000==0;
  enc_done=feed_encoder && count%500==0;
  if(plant && en && count%50==0) begin
   field=(phase<128?integer'(phase):integer'(phase)-256)*6.283185307179586/256.0;
   effort=direction*6000.0*$sin(field-direction*7.0*(rotor-base)*6.283185307179586/4096.0)+load;
   if(velocity>0.01) accel=effort-30.0*velocity-friction;
   else if(velocity < -0.01) accel=effort-30.0*velocity+friction;
   else if(effort>friction) accel=effort-friction;
   else if(effort < -friction) accel=effort+friction;
   else begin accel=0;velocity=0;end
   velocity=velocity+accel*0.001;rotor=rotor+velocity*0.001;
  end
  if(plant && enc_done && !stuck) enc_angle=12'($rtoi(rotor+0.5));
  if(en) begin
   if(!sl || !re || state!=5) $fatal(1,"enable before awake/run");
   if(da<21 || da>29 || db<21 || db>29 || dc<21 || dc>29) $fatal(1,"duty bound");
   if({da,db,dc}!=previous_duties && dut.carrier!=0) $fatal(1,"non-atomic duties");
   if(a!==(dut.carrier<da) || b!==(dut.carrier<db) || c!==(dut.carrier<dc)) $fatal(1,"PWM comparator");
   if(offset>21845 || offset < -21845) $fatal(1,"phase bound");
  end else if({a,b,c}!==0) $fatal(1,"disabled PWM");
  previous_duties={da,db,dc};
  if((elapsed!=previous_elapsed && elapsed%10==0) || state!=previous_state) begin
   $fdisplay(trace,"%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%f",scenario,elapsed,state,result,enc_angle,target,error,offset,en,passed,rotor);
   previous_elapsed=elapsed;previous_state=state;
  end
 end
 task reset_all;
  begin
   @(negedge clk);rst=1;arm=0;sweep=0;stop=0;fault_n=1;adc_fresh=1;adc_clip=0;
   enc_ok=1;enc_status=8'h37;enc_angle=100;feed_encoder=1;feed_ping=1;uart_error=0;
   plant=1;stuck=0;direction=-1;base=100;rotor=100;velocity=0;friction=300;load=0;
   repeat(10) @(negedge clk);rst=0;repeat(1100) @(negedge clk);
   if(state!=0 || {a,b,c,en,sl,re}!==0) $fatal(1,"reset not disarmed");
  end
 endtask
 task arm_only;
  begin
   @(negedge clk);arm=1;@(negedge clk);arm=0;
   wait(state==3 || state==6);@(negedge clk);
   if(state!=3 || en || !sl || !re) $fatal(1,"arm failed %d",result);
  end
 endtask
 task launch;
  begin arm_only;@(negedge clk);sweep=1;@(negedge clk);sweep=0;wait(state==5);end
 endtask
 task expect_fault(input [3:0] code);
  begin
   wait(state==6);@(negedge clk);
   if(result!=code || {a,b,c,en,sl,re}!==0) $fatal(1,"expected fault %d actual %d",code,result);
   arm=1;sweep=1;repeat(10) @(negedge clk);arm=0;sweep=0;
   if(state!=6 || en) $fatal(1,"fault rearmed");
   stop=1;@(negedge clk);stop=0;if(state!=6) $fatal(1,"STOP cleared fault");
  end
 endtask
 task successful_profile;
  integer cycles;
  begin
   launch;cycles=0;
   while(state==5) begin
    @(negedge clk);cycles=cycles+1;
    sweep=elapsed==4000; // repeated starts cannot extend the run
    if(elapsed==9000) load=600; // modest unmeasured disturbance; defined before tests
    if(elapsed==10000) load=0;
   end
   sweep=0;
   if(result!=1 || elapsed!=51000 || passed!=15 || cycles!=2550001)
    $fatal(1,"profile failed scenario=%d result=%d elapsed=%d passed=%h cycles=%d",scenario,result,elapsed,passed,cycles);
   $display("PASS PLANT scenario=%0d full 51s profile; four settling checks; load disturbance; wrap; return error <=1.055deg",scenario);
  end
 endtask
 initial begin
  reset_all;scenario=1;successful_profile;
  reset_all;scenario=2;base=4090;rotor=4090;enc_angle=4090;
  repeat(1100) @(negedge clk);successful_profile;
  reset_all;scenario=7;friction=3000;successful_profile;
  reset_all;scenario=3;stuck=1;launch;expect_fault(11);
  reset_all;scenario=4;direction=1;launch;
  wait(state==6);@(negedge clk);#1;if(result!=7 && result!=11) $fatal(1,"wrong direction not contained");
  reset_all;scenario=5;plant=0;launch;wait(elapsed==1600);enc_angle=120;
  expect_fault(10);
  reset_all;scenario=6;sweep=1;@(negedge clk);sweep=0;if(state!=0 || en) $fatal(1,"unarmed start");
  adc_fresh=0;arm=1;@(negedge clk);arm=0;repeat(5) @(negedge clk);
  if(state!=0 || en) $fatal(1,"bad ADC armed");
  reset_all;arm_only;wait(state==0);if(result!=3) $fatal(1,"arm expiry");
  reset_all;fault_n=0;arm=1;@(negedge clk);arm=0;expect_fault(4);
  reset_all;launch;@(negedge clk);stop=1;#1;if(en) $fatal(1,"STOP gate");@(negedge clk);stop=0;
  if(state!=0 || result!=2) $fatal(1,"STOP state");
  reset_all;launch;@(negedge clk);arm=1;#1;if(en) $fatal(1,"ARM gate");@(negedge clk);arm=0;
  if(state!=0) $fatal(1,"ARM restarted");
  reset_all;launch;@(negedge clk);#2;fault_n=0;#1;if(en) $fatal(1,"raw fault kill");#1;fault_n=1;#1;if(en) $fatal(1,"fault latch");expect_fault(4);
  reset_all;launch;adc_fresh=0;expect_fault(5);
  reset_all;launch;adc_clip=1;expect_fault(5);
  reset_all;launch;enc_ok=0;expect_fault(6);
  reset_all;launch;enc_status=8'h17;expect_fault(6);
  reset_all;launch;enc_status=8'h28;expect_fault(6);
  reset_all;launch;feed_encoder=0;expect_fault(6);
  reset_all;plant=0;launch;enc_angle=500;expect_fault(7);
  reset_all;launch;feed_ping=0;expect_fault(8);
  reset_all;launch;uart_error=1;expect_fault(9);
  reset_all;launch;rst=1;#1;if({a,b,c,en,sl,re}!==0) $fatal(1,"reset kill");
  $display("PASS CONTROL: encoder feedback, all trip sources, no auto-retry, fault pulse, atomic bounded PWM, full duration and reset");$finish;
 end
 initial begin #200000000;$fatal(1,"controller timeout state=%d result=%d",state,result);end
endmodule
