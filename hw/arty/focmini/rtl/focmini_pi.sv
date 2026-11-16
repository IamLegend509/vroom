// Trapezoidal PI, bounded integral and bounded output, as SimpleFOC PID D=0.
// Q16.16 input/output; KI_HALF_DT = Ki * sample_period / 2 in Q16.16.
module focmini_pi #(
    parameter logic signed [31:0] KP = 32'sd196608,
    parameter logic signed [31:0] KI_HALF_DT = 32'sd983,
    parameter logic signed [31:0] LIMIT = 32'sd98304,
    parameter integer ERROR_BITS=22
) (
    input wire clk, reset, clear, update,
    input wire signed [ERROR_BITS-1:0] error,
    output logic signed [31:0] value,
    output logic done
);
    import focmini_math::*;
    logic signed [31:0] integral;
    logic signed [ERROR_BITS-1:0] previous_error, captured_error;
    logic signed [31:0] error_sum;
    logic signed [31:0] proportional, increment, trial, total;
    logic [2:0] stage;
    always_ff @(posedge clk) begin
        if (reset || clear) begin
            integral<=0; previous_error<=0; value<=0; captured_error<=0;
            error_sum<=0; proportional<=0; increment<=0; trial<=0; total<=0;
            stage<=0; done<=0;
        end else begin
            done<=0;
            case(stage)
                0: if(update) begin captured_error<=ERROR_BITS'(error); stage<=1; end
                1: begin error_sum<=addq(32'(captured_error),32'(previous_error)); stage<=2; end
                2: begin proportional<=mulq(KP,32'(captured_error)); increment<=mulq(KI_HALF_DT,error_sum); stage<=3; end
                3: begin trial<=addq(integral,increment); stage<=4; end
                4: begin integral<=clipq(trial,LIMIT); stage<=5; end
                5: begin total<=addq(proportional,integral); stage<=6; end
                6: begin value<=clipq(total,LIMIT); previous_error<=captured_error; done<=1; stage<=0; end
                default: stage<=0;
            endcase
        end
    end
endmodule
