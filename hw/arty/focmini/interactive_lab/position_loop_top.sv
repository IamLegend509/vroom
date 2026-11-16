// Bounded encoder position test. Reset/boot is disarmed; exact UART commands only.
module position_loop_top #(
 parameter integer POLL_CYCLES=100000, TELEMETRY_CYCLES=2500000,
 parameter integer I2C_CLK_HZ=100000000, UART_DIV=868,
 parameter integer PWM_PERIOD=5000,MS_CYCLES=100000,WAKE_CYCLES=1000000,
 parameter integer ENC_TIMEOUT=10000000,KEEPALIVE_CYCLES=50000000,ARM_CYCLES=500000000,
 parameter ROM_FILE="sweep_duty.mem"
)(
 input wire clk,reset_button,fault_n,vaux4_p,vaux4_n,uart_rx,
 inout wire as5600_scl,as5600_sda,
 output wire pwm_a,pwm_b,pwm_c,enable,reset_n,sleep_n,uart_tx,
 output wire [3:0] led
);
 (* ASYNC_REG="TRUE" *) reg [1:0] button_sync=0,fault_telemetry=0;
 reg [15:0] power_on=16'hffff;
 wire rst=(power_on!=0)||button_sync[1];
 always @(posedge clk) begin
  button_sync<={button_sync[0],reset_button};
  fault_telemetry<={fault_telemetry[0],fault_n};
  if(power_on!=0) power_on<=power_on-1'b1;
 end
 wire [11:0] adc_raw;
 wire adc_fresh,adc_clip;
 diag_xadc_a0 adc(.clk(clk),.rst(rst),.vaux4_p(vaux4_p),.vaux4_n(vaux4_n),
  .raw(adc_raw),.fresh(adc_fresh),.clipped(adc_clip));
 wire scl_low,sda_low;
 assign as5600_scl=scl_low?1'b0:1'bz;
 assign as5600_sda=sda_low?1'b0:1'bz;
 wire enc_busy,enc_done,enc_ok;
 wire [3:0] enc_error;
 wire [7:0] enc_status;
 wire [11:0] enc_angle;
 wire [1:0] bus_level;
 reg poll=0,last_enc_ok=0;
 as5600_reader #(.CLK_HZ(I2C_CLK_HZ),.I2C_HZ(400000)) encoder(.clk(clk),.rst(rst),.start(poll),
  .scl_in(as5600_scl),.sda_in(as5600_sda),.scl_low(scl_low),.sda_low(sda_low),
  .busy(enc_busy),.done(enc_done),.ok(enc_ok),.error(enc_error),
  .status(enc_status),.angle(enc_angle),.bus_level(bus_level));
 integer poll_count=0,telemetry_count=0;
 reg telemetry_tick=0,heartbeat=0;
 always @(posedge clk) begin
  poll<=0;telemetry_tick<=0;
  if(rst) begin poll_count<=0;telemetry_count<=0;heartbeat<=0;last_enc_ok<=0;end
  else begin
   if(enc_done) last_enc_ok<=enc_ok;
   if(poll_count==POLL_CYCLES-1) begin poll_count<=0;if(!enc_busy) poll<=1;end
   else poll_count<=poll_count+1;
   if(telemetry_count==TELEMETRY_CYCLES-1) begin
    telemetry_count<=0;telemetry_tick<=1;heartbeat<=~heartbeat;
   end else telemetry_count<=telemetry_count+1;
  end
 end
 wire [7:0] rx_data;
 wire rx_valid,rx_error,arm,sweep,stop,ping,command_reject,control_reject,cmd_target_valid,rl_mode;
 wire [11:0] cmd_target;
 wire [7:0] cmd_slew;
 wire [15:0] accepted;
 sweep_uart_rx #(.DIV(UART_DIV)) rx(.clk(clk),.rst(rst),.rx(uart_rx),
  .data(rx_data),.valid(rx_valid),.framing_error(rx_error));
 position_commands commands(.clk(clk),.rst(rst),.valid(rx_valid),.framing_error(rx_error),.data(rx_data),
  .arm(arm),.sweep(sweep),.stop(stop),.ping(ping),.rejected(command_reject),.accepted(accepted),.cmd_target_valid(cmd_target_valid),.cmd_target(cmd_target),.cmd_slew(cmd_slew),.rl_mode(rl_mode),.rl_mode(rl_mode));
 wire [2:0] state;
 wire [3:0] result;
 wire [15:0] elapsed_ms;
 wire [7:0] phase_index;
 wire [11:0] target;wire signed [11:0] position_error;
 wire signed [15:0] phase_offset;wire [2:0] stage;wire calibrated;wire [3:0] passed;
 wire [12:0] active_a,active_b,active_c;
 position_controller #(.PWM_PERIOD(PWM_PERIOD),.MS_CYCLES(MS_CYCLES),.WAKE_CYCLES(WAKE_CYCLES),
  .ENC_TIMEOUT(ENC_TIMEOUT),.KEEPALIVE_CYCLES(KEEPALIVE_CYCLES),.ARM_CYCLES(ARM_CYCLES),.ROM_FILE(ROM_FILE)) control(
  .clk(clk),.rst(rst),.fault_n(fault_n),.arm(arm),.sweep(sweep),.stop(stop),.ping(ping),.uart_error(rx_error),.cmd_target_valid(cmd_target_valid),.cmd_target(cmd_target),.cmd_slew(cmd_slew),.rl_mode(rl_mode),
  .adc_fresh(adc_fresh),.adc_clip(adc_clip),.enc_done(enc_done),.enc_ok(enc_ok),.enc_status(enc_status),.enc_angle(enc_angle),
  .pwm_a(pwm_a),.pwm_b(pwm_b),.pwm_c(pwm_c),.enable(enable),.sleep_n(sleep_n),.reset_n(reset_n),
  .state(state),.result(result),.rejected(control_reject),.elapsed_ms(elapsed_ms),.phase_index(phase_index),
  .active_a(active_a),.active_b(active_b),.active_c(active_c),
  .target(target),.position_error(position_error),.phase_offset(phase_offset),.stage(stage),.calibrated(calibrated),.passed(passed));
 reg [15:0] reject_count=0;
 always @(posedge clk) begin
  if(rst) reject_count<=0;
  else if(command_reject || control_reject) reject_count<=reject_count+1'b1;
 end
 // LD4 heartbeat, LD5 ADC fresh, LD6 I2C OK, LD7 magnet healthy (not merely detected).
 assign led={last_enc_ok && enc_status[5] && !enc_status[4] && !enc_status[3],last_enc_ok,adc_fresh,heartbeat};
 function automatic [7:0] h(input [3:0] n);
  h=n<10 ? (8'h30+{4'b0,n}) : (8'h41+{4'b0,n}-8'd10);
 endfunction
 localparam integer LINE_LEN=114;
 reg [LINE_LEN*8-1:0] packet=0;
 reg [2:0] send_state=0;
 integer chars_left=0;
 reg tx_start=0;
 wire tx_busy;
 diag_uart_tx #(.CLKS_PER_BIT(UART_DIV)) tx(.clk(clk),.rst(rst),.start(tx_start),
  .data(packet[LINE_LEN*8-1 -: 8]),.busy(tx_busy),.tx(uart_tx));
 always @(posedge clk) begin
  tx_start<=0;
  if(rst) begin send_state<=0;packet<=0;chars_left<=0;end
  else case(send_state)
   0: if(telemetry_tick) begin
    packet<={"C3 a=",h(adc_raw[11:8]),h(adc_raw[7:4]),h(adc_raw[3:0]),
     " v=",h({3'b0,adc_fresh})," c=",h({3'b0,adc_clip})," i=",h({3'b0,last_enc_ok})," e=",h(enc_error),
     " s=",h(enc_status[7:4]),h(enc_status[3:0])," r=",h(enc_angle[11:8]),h(enc_angle[7:4]),h(enc_angle[3:0]),
     " b=",h({2'b0,bus_level})," f=",h({3'b0,fault_telemetry[1]})," m=",h({1'b0,state})," q=",h(result),
     " n=",h(accepted[15:12]),h(accepted[11:8]),h(accepted[7:4]),h(accepted[3:0]),
     " t=",h(elapsed_ms[15:12]),h(elapsed_ms[11:8]),h(elapsed_ms[7:4]),h(elapsed_ms[3:0]),
     " p=",h(phase_index[7:4]),h(phase_index[3:0])," g=",h({3'b0,enable}),
     " x=",h(reject_count[15:12]),h(reject_count[11:8]),h(reject_count[7:4]),h(reject_count[3:0]),
     " d=",h(target[11:8]),h(target[7:4]),h(target[3:0]),
     " z=",h(position_error[11:8]),h(position_error[7:4]),h(position_error[3:0]),
     " o=",h(phase_offset[15:12]),h(phase_offset[11:8]),h(phase_offset[7:4]),h(phase_offset[3:0]),
     " h=",h({1'b0,stage})," k=",h({3'b0,calibrated})," y=",h(passed),8'h0d,8'h0a};
    chars_left<=LINE_LEN;send_state<=1;
   end
   1: begin tx_start<=1;send_state<=2;end
   2: if(tx_busy) send_state<=3;
   3: if(!tx_busy) begin
    packet<=packet<<8;chars_left<=chars_left-1;
    if(chars_left==1) send_state<=0;else send_state<=1;
   end
   default:send_state<=0;
  endcase
 end
endmodule
