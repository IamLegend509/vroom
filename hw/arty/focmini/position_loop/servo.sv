// Bounded mechanical position feedback via stator-field phase. NOT dq FOC.
// 100 Hz encoder updates; 0.6 V fixed modulation at an assumed 9 V bus.
// Phase direction -1 comes from the observed open-loop forward/back trial.
module position_servo(
 input wire clk,rst,active,enc_done,input wire [11:0] angle,
 input wire [15:0] elapsed_ms,
 output reg calibrated=0,output reg [3:0] fault=0,
 output reg [11:0] origin=0,output wire [11:0] target,
 output wire signed [11:0] error,
 output reg signed [15:0] phase_offset=0,
 output wire [2:0] stage,output reg [3:0] passed=0
);
 reg [11:0] anchor=0,last_angle=0;
 reg [6:0] stable_samples=0,settled_samples=0;
 reg unstable=0;
 reg signed [11:0] reference=0;
 reg [15:0] previous_ms=0;
 reg [6:0] saturated_samples=0;
 assign stage=elapsed_ms<2000?0:elapsed_ms<3000?1:elapsed_ms<15000?2:
              elapsed_ms<27000?3:elapsed_ms<39000?4:5;
 wire signed [11:0] goal=stage==2?12'sd57:stage==4?-12'sd57:12'sd0;
 wire signed [11:0] next_reference=reference<goal?reference+12'sd1:
                                      reference>goal?reference-12'sd1:reference;
 assign target=origin+reference;
 assign error=target-angle; // signed shortest difference, modulo 4096
 // Pipeline one 100Hz sample through target, subtraction, correction and phase
 // addition. No multicycle timing exceptions: every stage meets the 100MHz clock.
 reg [4:0] pending=0;
 reg [11:0] sampled_target=0,sampled_angle=0;
 reg signed [11:0] next_error=0;
 reg signed [11:0] measured_step=0;
 reg signed [17:0] correction=0;
 reg signed [16:0] phase_step=0,proposed_phase=0;
 wire signed [11:0] alignment_delta=angle-anchor;
 wire signed [11:0] displacement=angle-origin;

 wire signed [16:0] limited_step=correction>192?17'sd192:correction < -192?-17'sd192:correction[16:0];
 wire in_window=(elapsed_ms>=14500 && elapsed_ms<15000) ||
                (elapsed_ms>=26500 && elapsed_ms<27000) ||
                (elapsed_ms>=38500 && elapsed_ms<39000) ||
                (elapsed_ms>=50500 && elapsed_ms<51000);
 wire checkpoint=elapsed_ms==14999 || elapsed_ms==26999 || elapsed_ms==38999 || elapsed_ms==50999;
 always @(posedge clk) begin
  if(rst || !active) begin
   calibrated<=0;fault<=0;origin<=0;anchor<=0;last_angle<=0;stable_samples<=0;
   unstable<=0;reference<=0;phase_offset<=0;previous_ms<=0;
   settled_samples<=0;saturated_samples<=0;passed<=0;
   pending<=0;sampled_target<=0;sampled_angle<=0;next_error<=0;phase_step<=0;proposed_phase<=0;measured_step<=0;correction<=0;
  end else begin
   previous_ms<=elapsed_ms;
   pending<={pending[3:0],enc_done && calibrated && fault==0};
   if(pending[0]) next_error<=sampled_target-sampled_angle;
   if(pending[1]) correction<=((next_error>4 || next_error < -4)?-$signed({{6{next_error[11]}},next_error}):18'sd0)
                                   + ($signed({{6{measured_step[11]}},measured_step}) <<< 6);
   if(pending[2]) phase_step<=limited_step;
   if(pending[3]) proposed_phase<=$signed({phase_offset[15],phase_offset})+phase_step;
   if(pending[4] && fault==0) begin
    if(proposed_phase>21845) phase_offset<=16'sd21845;
    else if(proposed_phase < -21845) phase_offset<=-16'sd21845;
    else phase_offset<=proposed_phase[15:0];
   end
   if(enc_done) begin
    last_angle<=angle;
    if(elapsed_ms>=1500 && elapsed_ms<2000) begin
     if(stable_samples==0) anchor<=angle;
     else if(alignment_delta>12 || alignment_delta < -12) unstable<=1;
     if(stable_samples<127) stable_samples<=stable_samples+1'b1;
    end
    if(calibrated && fault==0) begin
     reference<=next_reference;
     sampled_target<=origin+next_reference;sampled_angle<=angle;measured_step<=angle-last_angle;
     if(displacement>228 || displacement < -228) fault<=7; // 20 degrees from origin
     if((phase_offset==21845 || phase_offset==-21845) && (error>12 || error < -12)) begin
      if(saturated_samples>=49) fault<=11;
      else saturated_samples<=saturated_samples+1'b1;
     end else saturated_samples<=0;
     if(in_window && error<=12 && error>=-12 && reference==goal && settled_samples<127)
      settled_samples<=settled_samples+1'b1;
    end
   end
   if(elapsed_ms!=previous_ms) begin
    if(elapsed_ms==2000) begin
     if(stable_samples<40 || unstable) fault<=10;
     else begin calibrated<=1;origin<=last_angle;end
    end
    if(!in_window && !checkpoint) settled_samples<=0;
    if(checkpoint) begin
     if(settled_samples<40) fault<=11;
     else case(elapsed_ms)
      14999:passed[0]<=1;
      26999:passed[1]<=1;
      38999:passed[2]<=1;
      50999:passed[3]<=1;
     endcase
    end
   end
  end
 end
endmodule
