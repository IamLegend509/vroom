// 115200 baud, 8-N-1 at 100MHz. No receive path and no motor commands.
module diag_uart_tx #(parameter integer CLKS_PER_BIT=868)(
    input wire clk, rst, start, input wire [7:0] data,
    output reg busy, output wire tx
);
 initial busy=0;
    reg [9:0] shift=10'h3ff;
    integer ticks=0;
    reg [3:0] bit_count=0;
    assign tx=busy ? shift[0] : 1'b1;
    always @(posedge clk) begin
        if(rst) begin busy<=0; shift<=10'h3ff; ticks<=0; bit_count<=0; end
        else if(!busy) begin
            if(start) begin shift<={1'b1,data,1'b0}; busy<=1; ticks<=CLKS_PER_BIT-1; bit_count<=0; end
        end else if(ticks!=0) ticks<=ticks-1;
        else if(bit_count==9) busy<=0;
        else begin shift<={1'b1,shift[9:1]}; bit_count<=bit_count+1'b1; ticks<=CLKS_PER_BIT-1; end
    end
endmodule
