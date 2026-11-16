// Physical control arithmetic only; this does not change RL actor precision.
package focmini_math;
    function automatic logic signed [31:0] sat32(input logic signed [63:0] x);
        if (x > 64'sd2147483647) sat32 = 32'sh7fffffff;
        else if (x < -64'sd2147483648) sat32 = 32'sh80000000;
        else sat32 = x[31:0];
    endfunction
    function automatic logic signed [31:0] mulq(
        input logic signed [31:0] a, b);
        logic signed [63:0] p;
        begin p = a * b; mulq = sat32(p >>> 16); end
    endfunction
    // Physical transform/filter path: value in [-8,8), factor in [-2,2).
    // The driver enforces <=2.5A raw phase trips and <=1.5V PI outputs before
    // these jobs can publish. 20x18 maps to one Artix DSP, with identical Q16
    // rounding to mulq throughout the permitted domain.
    function automatic logic signed [31:0] mul_bounded(
        input logic signed [19:0] value,
        input logic signed [17:0] factor);
        logic signed [37:0] product;
        begin
            product=value*factor;
            mul_bounded=32'(product>>>16);
        end
    endfunction
    function automatic logic signed [31:0] addq(
        input logic signed [31:0] a, b);
        addq = sat32(64'(a) + 64'(b));
    endfunction
    function automatic logic signed [31:0] subq(
        input logic signed [31:0] a, b);
        subq = sat32(64'(a) - 64'(b));
    endfunction
    function automatic logic signed [31:0] clipq(
        input logic signed [31:0] x, limit);
        if (x > limit) clipq = limit;
        else if (x < -limit) clipq = -limit;
        else clipq = x;
    endfunction
endpackage
