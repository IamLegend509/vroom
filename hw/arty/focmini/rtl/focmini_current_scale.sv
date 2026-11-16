// ACS724-05AB 400mV/A; 10k/18k divider => 1,800,000/7 microvolts/A.
// zero_uv is a measured, independently stored offset for THIS ADC channel.
// adc_uv must already be ADC-calibrated node voltage, not an unscaled ADC code.
// The divider is an ESP32 reference circuit, not an Arty XADC input circuit.
module focmini_current_scale (
    input wire [23:0] adc_uv, zero_uv,
    output wire signed [31:0] current_q16
);
    wire signed [31:0] delta = $signed({8'b0,adc_uv})-$signed({8'b0,zero_uv});
    wire signed [63:0] scaled = 64'(delta)*64'sd458752;
    assign current_q16 = 32'(scaled / 64'sd1800000);
endmodule
