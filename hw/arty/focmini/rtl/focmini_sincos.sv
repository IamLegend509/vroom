// 16-iteration rotation CORDIC. phase=unsigned turns/65536; sin/cos=Q2.30.
// Request only when !busy. reset cancels a request; done is one clock wide.
module focmini_sincos (
    input wire clk, reset, start,
    input wire [15:0] phase,
    output logic busy, done,
    output logic signed [31:0] sine, cosine
);
    logic signed [31:0] x, y, z;
    logic [3:0] iteration;
    wire signed [31:0] angle = $signed({phase, 16'b0});
    function automatic logic signed [31:0] atan_turns(input logic [3:0] i);
        case (i)
            0: atan_turns=32'sd536870912; 1: atan_turns=32'sd316933406;
            2: atan_turns=32'sd167458907; 3: atan_turns=32'sd85004756;
            4: atan_turns=32'sd42667331; 5: atan_turns=32'sd21354465;
            6: atan_turns=32'sd10679838; 7: atan_turns=32'sd5340245;
            8: atan_turns=32'sd2670163; 9: atan_turns=32'sd1335087;
            10: atan_turns=32'sd667544; 11: atan_turns=32'sd333772;
            12: atan_turns=32'sd166886; 13: atan_turns=32'sd83443;
            14: atan_turns=32'sd41722; 15: atan_turns=32'sd20861;
        endcase
    endfunction
    logic signed [31:0] next_x, next_y;
    always_comb begin
        if (z >= 0) begin next_x=x-(y>>>iteration); next_y=y+(x>>>iteration); end
        else begin next_x=x+(y>>>iteration); next_y=y-(x>>>iteration); end
    end
    always_ff @(posedge clk) begin
        if (reset) begin
            busy<=0; done<=0; sine<=0; cosine<=0; x<=0; y<=0; z<=0; iteration<=0;
        end else begin
            done<=0;
            if (start && !busy) begin
                busy<=1; iteration<=0; y<=0;
                if (angle > 32'sd1073741824) begin
                    x<=-32'sd652032874; z<=angle-32'sh80000000;
                end else if (angle < -32'sd1073741824) begin
                    x<=-32'sd652032874; z<=angle+32'sh80000000;
                end else begin x<=32'sd652032874; z<=angle; end
            end else if (busy) begin
                x<=next_x; y<=next_y;
                z<=z>=0 ? z-atan_turns(iteration) : z+atan_turns(iteration);
                if (iteration==15) begin
                    cosine<=next_x; sine<=next_y; busy<=0; done<=1;
                end else iteration<=iteration+1'b1;
            end
        end
    end
endmodule
