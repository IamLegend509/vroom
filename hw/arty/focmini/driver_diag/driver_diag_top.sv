// Driver-wake diagnostic only. PWM and EN are constant LOW; no motor motion path.
module driver_diag_top #(
 parameter integer POLL_CYCLES=1000000, TELEMETRY_CYCLES=10000000,
 parameter integer I2C_CLK_HZ=100000000, UART_DIV=868,
 parameter integer MS_CYCLES=100000,WAKE_CYCLES=1000000,KEEPALIVE_CYCLES=50000000
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
 as5600_reader #(.CLK_HZ(I2C_CLK_HZ)) encoder(.clk(clk),.rst(rst),.start(poll),
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
 wire rx_valid,rx_error,arm,sweep,stop,ping,command_reject,control_reject;
 wire [15:0] accepted;
 sweep_uart_rx #(.DIV(UART_DIV)) rx(.clk(clk),.rst(rst),.rx(uart_rx),
  .data(rx_data),.valid(rx_valid),.framing_error(rx_error));
 sweep_commands commands(.clk(clk),.rst(rst),.valid(rx_valid),.framing_error(rx_error),.data(rx_data),
  .arm(arm),.sweep(sweep),.stop(stop),.ping(ping),.rejected(command_reject),.accepted(accepted));
 wire [2:0] state;
 wire [3:0] result;
 wire [13:0] elapsed_ms;
 // Structural separation: even erroneous commands cannot drive these outputs.
 assign {pwm_a,pwm_b,pwm_c,enable}=4'b0000;
 wire [7:0] phase_index=0;
 wake_control #(.MS_CYCLES(MS_CYCLES),.WAKE_CYCLES(WAKE_CYCLES),.KEEPALIVE_CYCLES(KEEPALIVE_CYCLES)) control(
  .clk(clk),.rst(rst),.arm(arm),.sweep(sweep),.stop(stop),.ping(ping),.uart_error(rx_error),
  .sleep_n(sleep_n),.reset_n(reset_n),.state(state),.result(result),.elapsed_ms(elapsed_ms),.rejected(control_reject));
 reg [15:0] reject_count=0;
 always @(posedge clk) begin
  if(rst) reject_count<=0;
  else if(command_reject || control_reject) reject_count<=reject_count+1'b1;
 end
 // LD4 heartbeat, LD5 awake sequence active, LD6 live fault_n, LD7 last I2C OK.
 assign led={last_enc_ok,fault_telemetry[1],sleep_n,heartbeat};
 function automatic [7:0] h(input [3:0] n);
  h=n<10 ? (8'h30+{4'b0,n}) : (8'h41+{4'b0,n}-8'd10);
 endfunction
 localparam integer LINE_LEN=83;
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
    packet<={"D1 a=",h(adc_raw[11:8]),h(adc_raw[7:4]),h(adc_raw[3:0]),
     " v=",h({3'b0,adc_fresh})," c=",h({3'b0,adc_clip})," i=",h({3'b0,last_enc_ok})," e=",h(enc_error),
     " s=",h(enc_status[7:4]),h(enc_status[3:0])," r=",h(enc_angle[11:8]),h(enc_angle[7:4]),h(enc_angle[3:0]),
     " b=",h({2'b0,bus_level})," f=",h({3'b0,fault_telemetry[1]})," m=",h({1'b0,state})," q=",h(result),
     " n=",h(accepted[15:12]),h(accepted[11:8]),h(accepted[7:4]),h(accepted[3:0]),
     " t=",h({2'b0,elapsed_ms[13:12]}),h(elapsed_ms[11:8]),h(elapsed_ms[7:4]),h(elapsed_ms[3:0]),
     " p=",h(phase_index[7:4]),h(phase_index[3:0])," g=",h({3'b0,enable}),
     " x=",h(reject_count[15:12]),h(reject_count[11:8]),h(reject_count[7:4]),h(reject_count[3:0]),8'h0d,8'h0a};
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
