// Slow nominal 90-degree mechanical sweep assuming seven pole pairs; not closed-loop positioning.
// Hold3s, positive-encoder direction12s, hold5s, return12s, hold3s. Fixed 0.6V.
module sweep_controller #(
 parameter integer PWM_PERIOD=5000, MS_CYCLES=100000,
 parameter integer WAKE_CYCLES=1000000, ENC_TIMEOUT=10000000,
 parameter integer KEEPALIVE_CYCLES=50000000, ARM_CYCLES=500000000,
 parameter ROM_FILE="source/open_loop_90/sweep_duty.mem"
)(
 input wire clk,rst,fault_n,arm,sweep,stop,ping,uart_error,
 input wire adc_fresh,adc_clip,
 input wire enc_done,enc_ok,input wire [7:0] enc_status,input wire [11:0] enc_angle,
 output wire pwm_a,pwm_b,pwm_c,enable,sleep_n,reset_n,
 output reg [2:0] state=0,output reg [3:0] result=0,
 output reg rejected=0, output reg [15:0] elapsed_ms=0,
 output wire [7:0] phase_index,
 output reg [12:0] active_a=0,active_b=0,active_c=0
);
 localparam [2:0] IDLE=0,WAKE=1,SETTLE=2,ARMED=3,STARTING=4,RUN=5,FAULT=6;
 reg [31:0] boot_age=0,arm_age=0,keep_age=0,enc_age=32'hffffffff;
 reg enc_seen=0,enc_good=0;
 reg [11:0] last_angle=0;
 reg signed [16:0] travel=0;
 reg signed [11:0] step=0;
 reg step_valid=0;
 wire signed [16:0] next_travel=travel+{{5{step[11]}},step};
 wire angle_bad=step>12'sd228 || step < -12'sd228 || next_travel>17'sd1303 || next_travel < -17'sd1303;
 wire frame_good=enc_ok && enc_status[5] && !enc_status[3];
 wire sensors_good=adc_fresh && !adc_clip && enc_seen && enc_good && enc_age<32'(ENC_TIMEOUT);
 wire monitor_fault=state==ARMED || state==STARTING || state==RUN;
 reg fault_capture=0;
 (* ASYNC_REG="TRUE" *) reg [1:0] fault_sync=0;
 (* ASYNC_REG="TRUE" *) reg [1:0] fault_level=0;
 always @(posedge clk) fault_level<={fault_level[0],fault_n};
 // Capture even sub-clock nFAULT pulses. Clear only while disarmed/starting up.
 always @(posedge clk or negedge fault_n) begin
  if(!fault_n) fault_capture<=1;
  else if(rst || !monitor_fault) fault_capture<=0;
 end
 always @(posedge clk) begin
  if(rst || !monitor_fault) fault_sync<=0;
  else fault_sync<={fault_sync[0],fault_capture};
 end
 wire cut=monitor_fault && (!fault_n || fault_capture);
 wire running=state==RUN && !rst && !stop && !arm && !cut;
 assign enable=running;
 assign sleep_n=(state!=IDLE && state!=FAULT) && !rst && !stop && !cut;
 assign reset_n=(state==SETTLE || monitor_fault) && !rst && !stop && !cut;
 reg [31:0] carrier=0,ms_counter=0,phase=0;
 assign phase_index=phase[31:24];
 reg [38:0] duties[0:255];
 reg [38:0] duty_word=0;
 initial $readmemh(ROM_FILE,duties);
 always @(posedge clk) duty_word<=duties[phase_index];
 function automatic [12:0] scaled(input [12:0] d);
  if(PWM_PERIOD==5000) scaled=d;
  else scaled=13'((32'(d)*32'(PWM_PERIOD))/5000);
 endfunction
 assign pwm_a=running && carrier<32'(active_a);
 assign pwm_b=running && carrier<32'(active_b);
 assign pwm_c=running && carrier<32'(active_c);
 reg [3:0] trip;
 always @* begin
  trip=0;
  if(monitor_fault) begin
   // Raw fault only gates outputs; state decisions use the synchronized latch.
   if(fault_sync[1]) trip=4;
   else if(!adc_fresh || adc_clip) trip=5;
   else if(!enc_seen || !enc_good || enc_age>=32'(ENC_TIMEOUT) || (enc_done && !frame_good)) trip=6;
   else if(state==RUN && step_valid && angle_bad) trip=7;
   else if(state==RUN && keep_age>=32'(KEEPALIVE_CYCLES)) trip=8;
   else if(uart_error) trip=9;
  end
 end
 always @(posedge clk) begin
  rejected<=0;
  if(rst) begin
   state<=IDLE;result<=0;boot_age<=0;arm_age<=0;keep_age<=0;
   enc_age<=32'hffffffff;enc_seen<=0;enc_good<=0;last_angle<=0;travel<=0;
   carrier<=0;ms_counter<=0;phase<=0;elapsed_ms<=0;step<=0;step_valid<=0;
   active_a<=0;active_b<=0;active_c<=0;
  end else begin
   carrier<=carrier==32'(PWM_PERIOD-1)?0:carrier+1'b1;
   step_valid<=state==RUN && enc_done && frame_good;
   if(enc_age!=32'hffffffff) enc_age<=enc_age+1'b1;
   if(enc_done) begin
    enc_seen<=1;enc_good<=frame_good;enc_age<=0;last_angle<=enc_angle;
    step<=enc_angle-last_angle;
   end
   if(state==RUN && step_valid) travel<=next_travel;
   if(keep_age<32'(KEEPALIVE_CYCLES)) keep_age<=keep_age+1'b1;
   if(ping) keep_age<=0;
   if(trip!=0) begin state<=FAULT;result<=trip;end
   else if(state==FAULT) begin if(arm||sweep) rejected<=1;end
   else if(stop || (arm && state!=IDLE)) begin state<=IDLE;result<=2;end
   else if(arm) begin
    if(sensors_good && !(enc_done && !frame_good)) begin
     state<=WAKE;boot_age<=0;result<=0;phase<=0;elapsed_ms<=0;travel<=0;
    end else begin rejected<=1;result<=6;end
   end else case(state)
    IDLE: begin phase<=0;if(sweep) rejected<=1;end
    WAKE: begin
     if(sweep) rejected<=1;
     if(boot_age==32'(WAKE_CYCLES-1)) begin boot_age<=0;state<=SETTLE;end
     else boot_age<=boot_age+1'b1;
    end
    SETTLE: begin
     if(sweep) rejected<=1;
     if(boot_age==32'(WAKE_CYCLES-1)) begin
      if(!fault_level[1]) begin state<=FAULT;result<=4;end
      else if(!sensors_good || (enc_done && !frame_good)) begin state<=FAULT;result<=6;end
      else begin state<=ARMED;arm_age<=0;end
     end else boot_age<=boot_age+1'b1;
    end
    ARMED: begin
     if(arm_age>=32'(ARM_CYCLES-1)) begin state<=IDLE;result<=3;end
     else if(sweep) begin state<=STARTING;phase<=0;travel<=0;keep_age<=0;end
     else arm_age<=arm_age+1'b1;
    end
    STARTING: if(carrier==32'(PWM_PERIOD-1)) begin
     active_a<=scaled(duty_word[38:26]);active_b<=scaled(duty_word[25:13]);active_c<=scaled(duty_word[12:0]);
     ms_counter<=0;elapsed_ms<=0;state<=RUN;travel<=0;keep_age<=0;
    end
    RUN: begin
     if(sweep) rejected<=1; // cannot restart/extend the 35-second timer
     if(carrier==32'(PWM_PERIOD-1)) begin
      active_a<=scaled(duty_word[38:26]);active_b<=scaled(duty_word[25:13]);active_c<=scaled(duty_word[12:0]);
     end
     if(ms_counter==32'(MS_CYCLES-1)) begin
      ms_counter<=0;
      if(elapsed_ms==16'd34999) begin elapsed_ms<=35000;state<=IDLE;result<=1;end
      else elapsed_ms<=elapsed_ms+1'b1;
      if(elapsed_ms<3000 || elapsed_ms>=32000) phase<=0;
      else if(elapsed_ms<15000) phase<=phase-32'd626349;
      else if(elapsed_ms<20000) phase<=32'h40000000;
      else phase<=phase+32'd626349;
     end else ms_counter<=ms_counter+1'b1;
    end
    default: begin state<=FAULT;result<=9;end
   endcase
  end
 end
 initial begin
  if(PWM_PERIOD<10 || PWM_PERIOD>8191 || MS_CYCLES<PWM_PERIOD || MS_CYCLES%PWM_PERIOD!=0 ||
     WAKE_CYCLES<2 || ENC_TIMEOUT<2 || KEEPALIVE_CYCLES<2 || ARM_CYCLES<2)
   $fatal(1,"Invalid sweep timing");
 end
endmodule
