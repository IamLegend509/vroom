// A0=VAUX4. Single unipolar channel, extended acquisition, 100MHz/100=1MHz
// ADCCLK; ~31.25ksps. Slow diagnostic only, no PWM-synchronous acquisition.
// Output is raw 12-bit XADC code, NOT amperes or ESP32 ADC millivolts.
module diag_xadc_a0(input wire clk,rst,vaux4_p,vaux4_n,
    output reg [11:0] raw=0, output wire fresh, output reg clipped=0);
    wire [15:0] dout;
    wire drdy,eoc;
    wire [4:0] channel;
    wire den = eoc && channel==5'h14;
    reg pending=0,seen=0;
    reg [23:0] age=24'hffffff;
    assign fresh=seen && age<24'd1000000; // <10ms
    always @(posedge clk) begin
        if(rst) begin raw<=0; pending<=0; seen<=0; age<=24'hffffff; clipped<=0; end
        else begin
            if(age!=24'hffffff) age<=age+1'b1;
            if(den) pending<=1;
            if(drdy && pending) begin
                raw<=dout[15:4]; age<=0; seen<=1; pending<=0;
                clipped<=dout[15:4]<=12'd4 || dout[15:4]>=12'd4091;
            end
        end
    end
    // UG480: 40h[8] ACQ, [4:0] channel; 41h[15:12]=0011 single-channel.
    // 3FFF disables alarms and enables ADC offset/gain correction. 42h[15:8]=100.
    XADC #(.INIT_40(16'h0114),.INIT_41(16'h3fff),.INIT_42(16'h6400)) adc (
        .DCLK(clk),.RESET(rst),.DADDR(7'h14),.DEN(den),.DWE(1'b0),.DI(16'd0),
        .DO(dout),.DRDY(drdy),.EOC(eoc),.CHANNEL(channel),
        .VAUXP({11'd0,vaux4_p,4'd0}),.VAUXN({11'd0,vaux4_n,4'd0}),
        .VP(1'b0),.VN(1'b0),.CONVST(1'b0),.CONVSTCLK(1'b0),
        .ALM(),.OT(),.BUSY(),.EOS(),.JTAGBUSY(),.JTAGLOCKED(),.JTAGMODIFIED(),.MUXADDR()
    );
endmodule
