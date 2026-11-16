module position_controller #(
 parameter integer PWM_PERIOD=5000, MS_CYCLES=100000,
 parameter integer WAKE_CYCLES=1000000, ENC_TIMEOUT=10000000,
 parameter integer KEEPALIVE_CYCLES=100000000, ARM_CYCLES=500000000,
 parameter ROM_FILE="sweep_duty.mem"
)(
 input wire clk,rst,fault_n,arm,sweep,stop,ping,uart_error,
 input wire cmd_target_valid,input wire [11:0] cmd_target,input wire [7:0] cmd_slew,input wire rl_mode,
 input wire adc_fresh,adc_clip,
 input wire enc_done,enc_ok,input wire [7:0] enc_status,input wire [11:0] enc_angle,
 output wire pwm_a,pwm_b,pwm_c,enable,sleep_n,reset_n,
 output reg [2:0] state,output reg [3:0] result,
 output reg rejected, output reg [15:0] elapsed_ms,
 output wire [7:0] phase_index,
 output reg [12:0] active_a,active_b,active_c,
 output wire [11:0] target,output wire signed [11:0] position_error,
 output wire signed [15:0] phase_offset,output wire [2:0] stage,
 output wire calibrated,output reg [3:0] passed
);
 initial begin state=0; result=0; rejected=0; elapsed_ms=0; active_a=0; active_b=0; active_c=0; passed=0; end
 localparam [2:0] IDLE=0,WAKE=1,SETTLE=2,ARMED=3,STARTING=4,RUN=5,FAULT=6;
 localparam [31:0] KEEP_LIM = 32'd200000000;
 reg [31:0] boot_age=0,arm_age=0,keep_age=0,enc_age=32'hffffffff;
 reg enc_seen=0,enc_good=0;
 wire frame_good=enc_ok && enc_status[5] && !enc_status[3];
 wire sensors_good=adc_fresh && !adc_clip && enc_seen && enc_good && enc_age<(ENC_TIMEOUT);
 wire monitor_fault=state==ARMED || state==STARTING || state==RUN;
 reg fault_capture=0;
 reg [9:0] fault_low_cnt=0;
 (* ASYNC_REG="TRUE" *) reg [1:0] fault_sync=0;
 (* ASYNC_REG="TRUE" *) reg [1:0] fault_level=0;
 always @(posedge clk) fault_level<={fault_level[0],fault_n};
 always @(posedge clk) begin
  if(rst || !monitor_fault) begin fault_capture<=0; fault_low_cnt<=0; fault_sync<=0; end
  else begin
   if(!fault_level[1]) begin
    if(fault_low_cnt==10'd1000) fault_capture<=1;
    else fault_low_cnt<=fault_low_cnt+1'b1;
   end else fault_low_cnt<=0;
   fault_sync<={fault_sync[0],fault_capture};
  end
 end
 wire cut=monitor_fault && fault_capture;
 wire running=state==RUN && !rst && !stop && !arm && !cut;
 assign enable=running;
 assign sleep_n=(state!=IDLE && state!=FAULT) && !rst && !stop && !cut;
 assign reset_n=(state==SETTLE || monitor_fault) && !rst && !stop && !cut;
 reg [31:0] carrier=0,ms_counter=0;
 wire [3:0] servo_fault,servo_passed;
 wire [1:0] amp_sel;
 position_servo servo(.clk(clk),.rst(rst),.active(state==RUN),.enc_done(enc_done && frame_good),
  .angle(enc_angle),.elapsed_ms(elapsed_ms),.cmd_target_valid(cmd_target_valid),.cmd_target(cmd_target),.cmd_slew(cmd_slew),.rl_mode(rl_mode),
  .calibrated(calibrated),.fault(servo_fault),
  .origin(),.target(target),.error(position_error),.phase_offset(phase_offset),.amp_sel(amp_sel),.stage(stage),.passed(servo_passed));
 assign phase_index=phase_offset[15:8];
 reg [29:0] sin3[0:255];
 reg [29:0] sin_word=0;
 initial $readmemh(ROM_FILE,sin3);
 always @(posedge clk) sin_word<=sin3[phase_index];
 wire signed [9:0] sa = sin_word[29:20];
 wire signed [9:0] sb = sin_word[19:10];
 wire signed [9:0] sc = sin_word[9:0];
 // Amplitude scaling from signed [-512..+512] sine:
 //   amp_sel=0 (0.45 V, RL settled): * 250 / 512  (sa >>> 1) -> +/-256 counts around 2500
 //   amp_sel=1 (0.70 V, RL medium) : * 384 / 512  ((sa >>> 1) + (sa >>> 2)) -> +/-384 counts
 //   amp_sel=2 (0.92 V, Classical) : * 512 / 512  (sa) -> +/-512 counts
 //   amp_sel=3 (1.15 V, RL boost)  : * 640 / 512  (sa + (sa >>> 2)) -> +/-640 counts
 function automatic [12:0] calc_duty(input signed [9:0] s, input [1:0] sel);
  reg signed [12:0] delta;
  begin
   case(sel)
    2'd0: delta = (s >>> 1);
    2'd1: delta = (s >>> 1) + (s >>> 2);
    2'd2: delta = s;
    default: delta = s + (s >>> 2);
   endcase
   calc_duty = 2500 + delta;
  end
 endfunction
 wire [38:0] duty_word = {calc_duty(sa, amp_sel), calc_duty(sb, amp_sel), calc_duty(sc, amp_sel)};
 assign pwm_a=running && carrier<(active_a);
 assign pwm_b=running && carrier<(active_b);
 assign pwm_c=running && carrier<(active_c);
 reg [3:0] trip;
 always @* begin
  trip=0;
  if(monitor_fault) begin
   if(fault_sync[1]) trip=4;
   else if(!adc_fresh || adc_clip) trip=5;
   else if(!enc_seen || !enc_good || enc_age>=(ENC_TIMEOUT)) trip=6;
   else if(state==RUN && keep_age>=KEEP_LIM) trip=8;
   else if(uart_error) trip=9;
   else if(state==RUN && servo_fault!=0) trip=servo_fault;
  end
 end
 always @(posedge clk) begin
  rejected<=0;
  if(state==RUN) passed<=servo_passed;
  if(rst) begin
   state<=IDLE;result<=0;boot_age<=0;arm_age<=0;keep_age<=0;
   enc_age<=32'hffffffff;enc_seen<=0;enc_good<=0;
   carrier<=0;ms_counter<=0;elapsed_ms<=0;passed<=0;
   active_a<=0;active_b<=0;active_c<=0;
  end else begin
   carrier<=carrier==(PWM_PERIOD-1)?0:carrier+1'b1;
   if(enc_age!=32'hffffffff) enc_age<=enc_age+1'b1;
   if(enc_done) begin
    enc_seen<=1;enc_good<=frame_good;enc_age<=0;
   end
   if(keep_age<KEEP_LIM) keep_age<=keep_age+1'b1;
   if(ping) keep_age<=0;
   if(trip!=0) begin state<=FAULT;result<=trip;end
   else if(state==FAULT) begin if(stop) begin state<=IDLE;result<=2;end else if(arm) begin state<=WAKE;boot_age<=0;result<=0;passed<=0;elapsed_ms<=0;end else if(sweep) rejected<=1;end
   else if(stop || (arm && state!=IDLE)) begin state<=IDLE;result<=2;end
   else if(arm) begin
    if(sensors_good) begin
     state<=WAKE;boot_age<=0;result<=0;passed<=0;elapsed_ms<=0;
    end else begin rejected<=1;result<=6;end
   end else case(state)
    IDLE: begin if(sweep) rejected<=1;end
    WAKE: begin
     if(sweep) rejected<=1;
     if(boot_age==(WAKE_CYCLES-1)) begin boot_age<=0;state<=SETTLE;end
     else boot_age<=boot_age+1'b1;
    end
    SETTLE: begin
     if(sweep) rejected<=1;
     if(boot_age==(WAKE_CYCLES-1)) begin
      if(!fault_level[1]) begin state<=FAULT;result<=4;end
      else if(!sensors_good) begin state<=FAULT;result<=6;end
      else begin state<=ARMED;arm_age<=0;end
     end else boot_age<=boot_age+1'b1;
    end
    ARMED: begin
     if(arm_age>=(ARM_CYCLES-1)) begin state<=IDLE;result<=3;end
     else if(sweep) begin state<=STARTING;keep_age<=0;end
     else arm_age<=arm_age+1'b1;
    end
    STARTING: if(carrier==(PWM_PERIOD-1)) begin
     active_a<=duty_word[38:26];active_b<=duty_word[25:13];active_c<=duty_word[12:0];
     ms_counter<=0;elapsed_ms<=0;state<=RUN;keep_age<=0;
    end
    RUN: begin
     if(carrier==(PWM_PERIOD-1)) begin
      active_a<=duty_word[38:26];active_b<=duty_word[25:13];active_c<=duty_word[12:0];
     end
     if(ms_counter==(MS_CYCLES-1)) begin
      ms_counter<=0;
      if(elapsed_ms < 16'hfffe) elapsed_ms<=elapsed_ms+1'b1;
     end else ms_counter<=ms_counter+1'b1;
    end
    default: begin state<=FAULT;result<=9;end
   endcase
  end
 end
endmodule
