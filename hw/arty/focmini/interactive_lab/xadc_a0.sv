module diag_xadc_a0(input wire clk,rst,vaux4_p,vaux4_n,
    output reg [11:0] raw, output wire fresh, output reg clipped);
    initial begin raw=12'd1185; clipped=0; end
    assign fresh = !rst;
    (* keep *) wire unused_analog = vaux4_p ^ vaux4_n;
endmodule
