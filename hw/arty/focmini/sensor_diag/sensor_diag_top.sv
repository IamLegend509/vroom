// Sensor-only physical board top. All six motor-drive controls hard tied LOW.
// No control core, motion state, UART RX, current conversion, or A1 input.
module sensor_diag_top #(
    parameter integer POLL_CYCLES=10000000,
    parameter integer I2C_CLK_HZ=100000000,
    parameter integer UART_DIV=868
)(
    input wire clk,reset_button,fault_n,vaux4_p,vaux4_n,
    inout wire as5600_scl,as5600_sda,
    output wire pwm_a,pwm_b,pwm_c,enable,reset_n,sleep_n,uart_tx,
    output wire [3:0] led
);
    assign {pwm_a,pwm_b,pwm_c,enable,reset_n,sleep_n}=6'b0;
    (* ASYNC_REG="TRUE" *) reg [1:0] button_sync=0, fault_sync=0;
    reg [15:0] power_on=16'hffff;
    wire rst=(power_on!=0) || button_sync[1];
    always @(posedge clk) begin
        button_sync<={button_sync[0],reset_button};
        fault_sync<={fault_sync[0],fault_n};
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
    reg poll=0;
    as5600_reader #(.CLK_HZ(I2C_CLK_HZ)) encoder(.clk(clk),.rst(rst),.start(poll),
        .scl_in(as5600_scl),.sda_in(as5600_sda),.scl_low(scl_low),.sda_low(sda_low),
        .busy(enc_busy),.done(enc_done),.ok(enc_ok),.error(enc_error),
        .status(enc_status),.angle(enc_angle),.bus_level(bus_level));
    integer interval_count=0;
    reg heartbeat=0;
    always @(posedge clk) begin
        poll<=0;
        if(rst) begin interval_count<=0; heartbeat<=0; end
        else if(interval_count==POLL_CYCLES-1) begin
            interval_count<=0; if(!enc_busy) poll<=1; heartbeat<=~heartbeat;
        end else interval_count<=interval_count+1;
    end
    assign led={enc_ok && enc_status[5] && !enc_status[4] && !enc_status[3],enc_ok,adc_fresh,heartbeat};
    function automatic [7:0] hexchar(input [3:0] n);
        hexchar=n<10 ? (8'h30+{4'b0,n}) : (8'h41+{4'b0,n}-8'd10);
    endfunction
    localparam integer LINE_LEN=45;
    reg [LINE_LEN*8-1:0] packet=0;
    reg [2:0] send_state=0;
    integer chars_left=0;
    reg tx_start=0;
    wire tx_busy;
    diag_uart_tx #(.CLKS_PER_BIT(UART_DIV)) uart(.clk(clk),.rst(rst),.start(tx_start),
        .data(packet[LINE_LEN*8-1 -: 8]),.busy(tx_busy),.tx(uart_tx));
    always @(posedge clk) begin
        tx_start<=0;
        if(rst) begin send_state<=0; packet<=0; chars_left<=0; end
        else case(send_state)
            0: if(enc_done) begin
                packet<={"S1 a=",hexchar(adc_raw[11:8]),hexchar(adc_raw[7:4]),hexchar(adc_raw[3:0]),
                    " v=",hexchar({3'b0,adc_fresh})," c=",hexchar({3'b0,adc_clip}),
                    " i=",hexchar({3'b0,enc_ok})," e=",hexchar(enc_error),
                    " s=",hexchar(enc_status[7:4]),hexchar(enc_status[3:0]),
                    " r=",hexchar(enc_angle[11:8]),hexchar(enc_angle[7:4]),hexchar(enc_angle[3:0]),
                    " b=",hexchar({2'b0,bus_level})," f=",hexchar({3'b0,fault_sync[1]}),8'h0d,8'h0a};
                chars_left<=LINE_LEN; send_state<=1;
            end
            1: begin tx_start<=1; send_state<=2; end
            2: if(tx_busy) send_state<=3;
            3: if(!tx_busy) begin
                packet<=packet << 8; chars_left<=chars_left-1;
                if(chars_left==1) send_state<=0; else send_state<=1;
            end
            default: send_state<=0;
        endcase
    end
endmodule
