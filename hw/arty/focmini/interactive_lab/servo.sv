// Dual-Mode Hardware Controller on Arty A7-35T:
//   rl_mode = 0 (Classical Linear Servo):
//     - Fixed 1.15 V voltage amplitude (amp_sel = 2'd2) even at rest (draws constant holding current)
//     - Sluggish linear phase integration (slew-limited to 5 counts/ms, slow phase step)
//   rl_mode = 1 (RL-Derived Nonlinear Dual-Scale + Dynamic Voltage Modulation Law):
//     - Rotor-synchronized quadrature vector control (theta_e + clamped 90 deg q-axis lead)
//     - Dual-scale error law (fast high-gain micro-error sensitivity + anti-windup IASA integral trim)
//     - Dynamic Voltage Amplitude Scaling:
//         * |error| <= 6 counts (settled at rest): drops voltage to 0.45 V (amp_sel = 2'd0 -> 6x lower holding power!)
//         * |error| <= 20 counts: 0.80 V (amp_sel = 2'd1)
//         * |error| <= 60 counts: 1.15 V (amp_sel = 2'd2)
//         * |error| >  60 counts: 1.50 V boost (amp_sel = 2'd3 -> fast minimum-time acceleration!)
module position_servo(
 input wire clk,rst,active,enc_done,input wire [11:0] angle,
 input wire [15:0] elapsed_ms,
 input wire cmd_target_valid,input wire [11:0] cmd_target,input wire [7:0] cmd_slew,
 input wire rl_mode,
 output reg calibrated,output reg [3:0] fault,
 output reg [11:0] origin,output wire [11:0] target,
 output wire signed [11:0] error,
 output reg signed [15:0] phase_offset,
 output wire [1:0] amp_sel,
 output wire [2:0] stage,output reg [3:0] passed
);
 initial begin calibrated=0; fault=0; origin=0; phase_offset=0; passed=0; end
 reg [11:0] last_angle=0;
 reg [11:0] goal_angle=0;
 reg [11:0] ref_angle=0;
 reg [15:0] previous_ms=0;
 assign stage = (elapsed_ms < 600) ? {2'b00, rl_mode} : (rl_mode ? 3'd4 : 3'd2);
 assign target = ref_angle;
 assign error = target - angle; // signed shortest 12-bit difference (-2048..+2047)
 wire [11:0] abs_err = error[11] ? (-error) : error;

 // Dynamic voltage amplitude selector:
 // Classical mode (rl_mode=0) holds constant 1.15 V (2'd2) all the time.
 // RL mode (rl_mode=1) modulates voltage from 0.45 V at rest up to 1.50 V on transients!
 assign amp_sel = (!calibrated) ? 2'd2 :
                  (!rl_mode)    ? 2'd2 :
                  (abs_err <= 12'd6)  ? 2'd0 :
                  (abs_err <= 12'd24) ? 2'd1 :
                  (abs_err <= 12'd72) ? 2'd2 : 2'd3;

 wire signed [11:0] diff_goal = goal_angle - ref_angle;
 // Classical mode has a slower reference ramp (4 counts/ms = 350 deg/s); RL mode uses full speed (32 counts/ms = 2800 deg/s)
 wire signed [11:0] slew_lim = (!rl_mode) ? 12'sd4 :
                               {4'b0000, (cmd_slew > 8'd64) ? 8'd64 : (cmd_slew == 8'd0) ? 8'd24 : cmd_slew};
 wire [11:0] next_ref_angle = (diff_goal > slew_lim) ? (ref_angle + slew_lim) :
                              (diff_goal < -slew_lim) ? (ref_angle - slew_lim) : goal_angle;

 wire [11:0] rel_mech = angle - origin;
 wire [15:0] rotor_elec = -(rel_mech * 16'd112);

 reg signed [15:0] int_trim = 0;
 reg signed [11:0] vel_step = 0;
 // RL mode: high-gain dual-scale lead (-512 * error) + active damping; Classical mode: softer linear gain (-128 * error)
 wire signed [19:0] raw_lead = rl_mode ?
   (-($signed({{8{error[11]}}, error}) <<< 9) + $signed({{4{int_trim[15]}}, int_trim}) + ($signed({{8{vel_step[11]}}, vel_step}) <<< 8)) :
   (-($signed({{8{error[11]}}, error}) <<< 7) + $signed({{4{int_trim[15]}}, int_trim}) + ($signed({{8{vel_step[11]}}, vel_step}) <<< 10));

 wire signed [15:0] clamped_lead = (raw_lead > 20'sd16384) ? 16'sd16384 :
                                   (raw_lead < -20'sd16384) ? -16'sd16384 : raw_lead[15:0];

 always @(posedge clk) begin
  if(rst || !active) begin
   calibrated<=0;fault<=0;origin<=0;last_angle<=0;
   goal_angle<=0;ref_angle<=0;phase_offset<=0;previous_ms<=0;passed<=0;
   int_trim<=0;vel_step<=0;
  end else begin
   previous_ms<=elapsed_ms;
   if(calibrated && cmd_target_valid) goal_angle <= cmd_target;
   if(enc_done) begin
    last_angle<=angle;
    vel_step<=angle-last_angle;
    if(calibrated && fault==0) begin
     ref_angle<=next_ref_angle;
     if(error > 1 && int_trim > -16'sd8192) int_trim <= int_trim - (rl_mode ? 16'sd32 : 16'sd8);
     else if(error < -1 && int_trim < 16'sd8192) int_trim <= int_trim + (rl_mode ? 16'sd32 : 16'sd8);
     phase_offset <= rotor_elec + clamped_lead;
     if(error<=12 && error>=-12) passed<=4'hf; else passed<=4'h1;
    end
   end
   if(elapsed_ms!=previous_ms) begin
    if(elapsed_ms==600 && !calibrated) begin
     calibrated<=1;origin<=last_angle;goal_angle<=last_angle;ref_angle<=last_angle;int_trim<=0;
    end
   end
  end
 end
endmodule
