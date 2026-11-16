`timescale 1ns/1ps
module tb_math;
    import focmini_math::*;
    logic clk=0; always #5 clk=~clk;
    logic reset=1,start=0; logic [15:0] phase=0;
    wire busy,done; wire signed [31:0] sine,cosine;
    focmini_sincos dut(.*);
    logic [23:0] adc_uv=0,zero_uv=1607143;
    wire signed [31:0] current_q16;
    focmini_current_scale scale(.*);
    real err,peak=0,theta;
    integer i,waited,seed=2805;
    logic signed [19:0] bounded_value;
    logic signed [17:0] bounded_factor;
    initial begin
        repeat(2) @(negedge clk); reset=0;
        for(i=0;i<65536;i=i+1) begin
            @(negedge clk); phase=16'(i);start=1;
            @(negedge clk);start=0;waited=0;
            while(!done && waited<20) begin @(negedge clk);waited=waited+1;end
            if(!done || busy || waited!=16) $fatal(1,"CORDIC latency/handshake");
            theta=i*6.283185307179586/65536.0;
            err=$itor(sine)/1073741824.0-$sin(theta); if(err<0) err=-err;
            if(err>peak) peak=err;
            if(err>0.000031) $fatal(1,"sine phase=%0d error=%g",i,err);
            err=$itor(cosine)/1073741824.0-$cos(theta);if(err<0)err=-err;
            if(err>peak)peak=err;
            if(err>0.000031) $fatal(1,"cosine phase=%0d error=%g",i,err);
        end
        // Full signed sensor range, at calibrated ADC-node voltages.
        for(i=-5000;i<=5000;i=i+1) begin
            adc_uv=24'(1607143+(i*1800)/7); #1;
            err=$itor(current_q16)/65536.0-$itor(i)/1000.0;
            if(err>0.000020 || err < -0.000020) $fatal(1,"ACS724 scale %0d %g",i,err);
        end
        for(i=0;i<65536;i=i+1)begin
            bounded_value=20'($random(seed));bounded_factor=18'($random(seed));
            if(mul_bounded(bounded_value,bounded_factor) !== mulq(32'(bounded_value),32'(bounded_factor)))
                $fatal(1,"bounded multiply changed fixed-point arithmetic");
        end
        @(negedge clk);start=1;phase=16'h4000;
        @(negedge clk);start=0;
        repeat(4)@(negedge clk);reset=1;
        @(negedge clk);reset=0;
        repeat(20)begin @(negedge clk);if(done || busy)$fatal(1,"reset cancellation");end
        $display("PASS math phases=65536 current_points=10001 bounded_products=65536 max_trig_error=%g",peak);
        $finish;
    end
endmodule
