// Read-only AS5600 transaction: START 6C 0B RESTART 6D STATUS ANGLE_H ANGLE_L STOP.
// Only the register pointer is written; never writes configuration or OTP.
// Open-drain outputs; synchronized inputs; actual SCL-high wait with timeout.
module as5600_reader #(
    parameter integer CLK_HZ = 100000000,
    parameter integer I2C_HZ = 100000,
    parameter integer TIMEOUT_CYCLES = CLK_HZ/100 // 10ms whole transaction
)(
    input wire clk, rst, start,
    input wire scl_in, sda_in,
    output reg scl_low, sda_low,
    output wire busy,
    output reg done, ok,
    output reg [3:0] error,
    output reg [7:0] status,
    output reg [11:0] angle,
    output wire [1:0] bus_level
);
    localparam integer Q = CLK_HZ/(I2C_HZ*4);
    (* ASYNC_REG = "TRUE" *) reg [1:0] scl_sync = 2'b11;
    (* ASYNC_REG = "TRUE" *) reg [1:0] sda_sync = 2'b11;
    always @(posedge clk) begin
        scl_sync <= {scl_sync[0], scl_in};
        sda_sync <= {sda_sync[0], sda_in};
    end
    assign bus_level = {scl_sync[1],sda_sync[1]};
    localparam [4:0] IDLE=0, START_HOLD=1, SEND_LOW=2, SEND_RISE=3,
        SEND_HIGH=4, SEND_FALL=5, ACK_LOW=6, ACK_RISE=7, ACK_HIGH=8,
        ACK_FALL=9, RESTART_LOW=10, RESTART_RISE=11, RESTART_HIGH=12,
        READ_LOW=13, READ_RISE=14, READ_HIGH=15, READ_FALL=16,
        MACK_LOW=17, MACK_RISE=18, MACK_HIGH=19, MACK_FALL=20,
        STOP_LOW=21, STOP_RISE=22, STOP_HIGH=23, STOP_FREE=24,
        RECOVER_LOW=25, RECOVER_HIGH=26;
    reg [4:0] state = IDLE;
    integer timer=0, watchdog=0;
    reg [2:0] bit_index=7;
    reg [3:0] recover_count=0;
    reg [1:0] tx_index=0, rx_index=0;
    reg [7:0] tx_byte=0, rx_byte=0, pending_status=0, pending_high=0, pending_low=0;
    reg ack=0;
    assign busy = state != IDLE;
    always @(posedge clk) begin
        done <= 0;
        if (rst) begin
            state<=IDLE; scl_low<=0; sda_low<=0; timer<=0; watchdog<=0;
            done<=0; ok<=0; error<=0; status<=0; angle<=0;
            bit_index<=7; tx_index<=0; rx_index<=0; tx_byte<=0; rx_byte<=0;
            pending_status<=0; pending_high<=0; pending_low<=0; ack<=0;
        end else if (busy && watchdog >= TIMEOUT_CYCLES-1) begin
            state<=IDLE; scl_low<=0; sda_low<=0; done<=1; ok<=0; error<=5;
        end else begin
            if (busy) watchdog<=watchdog+1;
            if (timer != 0) timer<=timer-1;
            else case(state)
                IDLE: if (start) begin
                    ok<=0; error<=0; watchdog<=0;
                    if (bus_level == 2'b10) begin
                        sda_low<=0; scl_low<=1; timer<=Q-1; recover_count<=0; state<=RECOVER_LOW;
                    end else if (bus_level != 2'b11) begin
                        done<=1; error<=1;
                    end else begin
                        sda_low<=1; scl_low<=0; timer<=Q*2-1; state<=START_HOLD;
                        tx_index<=0; tx_byte<=8'h6c; bit_index<=7; rx_index<=0;
                    end
                end
                RECOVER_LOW: begin scl_low<=0; if (scl_sync[1]) begin timer<=Q*2-1; state<=RECOVER_HIGH; end end
                RECOVER_HIGH: begin
                    scl_low<=1; timer<=Q-1;
                    if (recover_count==8) begin error<=1; state<=STOP_LOW; end
                    else begin recover_count<=recover_count+1'b1; state<=RECOVER_LOW; end
                end
                START_HOLD: begin scl_low<=1; timer<=Q-1; state<=SEND_LOW; end
                SEND_LOW: begin sda_low<=~tx_byte[bit_index]; timer<=Q-1; state<=SEND_RISE; end
                SEND_RISE: begin scl_low<=0; if (scl_sync[1]) begin timer<=Q*2-1; state<=SEND_HIGH; end end
                SEND_HIGH: begin scl_low<=1; timer<=Q-1; state<=SEND_FALL; end
                SEND_FALL: if (bit_index==0) begin sda_low<=0; timer<=Q-1; state<=ACK_RISE; end
                    else begin bit_index<=bit_index-1'b1; state<=SEND_LOW; end
                ACK_RISE: begin scl_low<=0; if (scl_sync[1]) begin timer<=Q-1; state<=ACK_HIGH; end end
                ACK_HIGH: begin ack<=~sda_sync[1]; timer<=Q-1; state<=ACK_FALL; end
                ACK_FALL: begin
                    scl_low<=1; timer<=Q-1;
                    if (!ack) begin error<={2'b00,tx_index}+4'd2; state<=STOP_LOW; end
                    else if (tx_index==0) begin tx_index<=1; tx_byte<=8'h0b; bit_index<=7; state<=SEND_LOW; end
                    else if (tx_index==1) begin state<=RESTART_LOW; end
                    else begin bit_index<=7; rx_byte<=0; state<=READ_LOW; end
                end
                RESTART_LOW: begin sda_low<=0; timer<=Q-1; state<=RESTART_RISE; end
                RESTART_RISE: begin scl_low<=0; if (scl_sync[1]) begin timer<=Q*2-1; state<=RESTART_HIGH; end end
                RESTART_HIGH: begin
                    sda_low<=1; timer<=Q*2-1; state<=START_HOLD;
                    tx_index<=2; tx_byte<=8'h6d; bit_index<=7;
                end
                READ_LOW: begin sda_low<=0; timer<=Q-1; state<=READ_RISE; end
                READ_RISE: begin scl_low<=0; if (scl_sync[1]) begin timer<=Q-1; state<=READ_HIGH; end end
                READ_HIGH: begin rx_byte[bit_index]<=sda_sync[1]; timer<=Q-1; state<=READ_FALL; end
                READ_FALL: begin
                    scl_low<=1; timer<=Q-1;
                    if (bit_index==0) begin
                        case(rx_index)
                            0: pending_status<=rx_byte;
                            1: pending_high<=rx_byte;
                            2: pending_low<=rx_byte;
                            default: ;
                        endcase
                        state<=MACK_LOW;
                    end else begin bit_index<=bit_index-1'b1; state<=READ_LOW; end
                end
                MACK_LOW: begin sda_low <= rx_index!=2; timer<=Q-1; state<=MACK_RISE; end
                MACK_RISE: begin scl_low<=0; if (scl_sync[1]) begin timer<=Q*2-1; state<=MACK_HIGH; end end
                MACK_HIGH: begin scl_low<=1; timer<=Q-1; state<=MACK_FALL; end
                MACK_FALL: begin
                    if (rx_index==2) state<=STOP_LOW;
                    else begin rx_index<=rx_index+1'b1; bit_index<=7; rx_byte<=0; state<=READ_LOW; end
                end
                STOP_LOW: begin scl_low<=1; sda_low<=1; timer<=Q*2-1; state<=STOP_RISE; end
                STOP_RISE: begin scl_low<=0; if (scl_sync[1]) begin timer<=Q*2-1; state<=STOP_HIGH; end end
                STOP_HIGH: begin sda_low<=0; timer<=Q*2-1; state<=STOP_FREE; end
                STOP_FREE: begin
                    state<=IDLE; done<=1;
                    if (error==0 && bus_level==2'b11) begin
                        ok<=1; status<=pending_status; angle<={pending_high[3:0],pending_low};
                    end else if(error==0) error<=1;
                end
                default: begin state<=IDLE; scl_low<=0; sda_low<=0; ok<=0; error<=6; done<=1; end
            endcase
        end
    end
endmodule
