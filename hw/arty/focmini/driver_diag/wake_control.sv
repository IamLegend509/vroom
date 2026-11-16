// Diagnostic wake only. No motor PWM/enable ports exist in this controller.
// Top ties IN1/2/3 and EN to GND, regardless of any command or sensor reading.
module wake_control #(
 parameter integer MS_CYCLES=100000,WAKE_CYCLES=1000000,
 parameter integer KEEPALIVE_CYCLES=50000000,DURATION_MS=10000
)(
 input wire clk,rst,arm,sweep,stop,ping,uart_error,
 output wire sleep_n,reset_n,
 output reg [2:0] state=0,output reg [3:0] result=0,
 output reg [13:0] elapsed_ms=0,output reg rejected=0
);
 localparam [2:0] IDLE=0,WAKE=1,AWAKE=2;
 reg [31:0] wake_age=0,keep_age=0,ms_count=0;
 wire active=state==WAKE || state==AWAKE;
 assign sleep_n=active && !rst && !stop && !arm && !uart_error;
 assign reset_n=state==AWAKE && !rst && !stop && !arm && !uart_error;
 always @(posedge clk) begin
  rejected<=0;
  if(rst) begin state<=IDLE;result<=0;elapsed_ms<=0;wake_age<=0;keep_age<=0;ms_count<=0;end
  else if(stop || (arm && active)) begin state<=IDLE;result<=2;end
  else if(active && uart_error) begin state<=IDLE;result<=9;end
  else if(active && keep_age>=32'(KEEPALIVE_CYCLES)) begin state<=IDLE;result<=8;end
  else if(active) begin
   if(sweep) rejected<=1;
   if(ping) keep_age<=0;else keep_age<=keep_age+1'b1;
   if(state==WAKE) begin
    if(wake_age==32'(WAKE_CYCLES-1)) state<=AWAKE;
    else wake_age<=wake_age+1'b1;
   end
   if(ms_count==32'(MS_CYCLES-1)) begin
    ms_count<=0;elapsed_ms<=elapsed_ms+1'b1;
    if(elapsed_ms==14'(DURATION_MS-1)) begin state<=IDLE;result<=1;end
   end else ms_count<=ms_count+1'b1;
  end else if(arm) begin
   state<=WAKE;result<=0;elapsed_ms<=0;wake_age<=0;keep_age<=0;ms_count<=0;
  end else begin
   if(sweep) rejected<=1;
   if(state!=IDLE) begin state<=IDLE;result<=9;end
  end
 end
 initial if(MS_CYCLES<2 || WAKE_CYCLES<2 || KEEPALIVE_CYCLES<2 ||
  DURATION_MS<1 || DURATION_MS>16000 || WAKE_CYCLES>=MS_CYCLES*DURATION_MS)
  $fatal(1,"Invalid diagnostic timing");
endmodule
