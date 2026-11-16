`timescale 1ns/1ps
module tb_control;
    logic clk=0;always #5 clk=~clk;
    logic reset=1,run=0,sample_valid=0,angle_reverse=0;
    logic [11:0] angle_raw=0;
    logic [15:0] electrical_zero=0;
    logic signed [31:0] phase_a=0,phase_b=0,target_velocity=0;
    wire busy,overrun,duty_valid;
    wire [15:0] duty_a,duty_b,duty_c;
    wire signed [31:0] velocity,current_d,current_q,reference_q,voltage_d,voltage_q;
    focmini_control dut(.*);
    integer fd,rc,seq=0,rst,ang,z,rev,ed,eq,ev,er,evd,evq,da,db,dc,latency,max_latency=0,min_latency=999,cancel;
    string path;
    task automatic nearq(input integer actual,expected,tolerance);
        if(actual-expected>tolerance || expected-actual>tolerance)
            $fatal(1,"frame %0d got=%0d expected=%0d tol=%0d",seq,actual,expected,tolerance);
    endtask
    initial begin
        if(!$value$plusargs("vectors=%s",path))$fatal(1,"vectors missing");
        fd=$fopen(path,"r");if(!fd)$fatal(1,"vectors open");
        repeat(3)@(negedge clk);reset=0;run=1;
        while(!$feof(fd))begin
            rc=$fscanf(fd,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
                rst,ang,z,rev,phase_a,phase_b,target_velocity,ed,eq,ev,er,evd,evq,da,db,dc);
            if(rc!=16)$fatal(1,"vector parse %0d",rc);
            if(rst)begin run=0;repeat(2)@(negedge clk);run=1;end
            angle_raw=12'(ang);electrical_zero=16'(z);angle_reverse=1'(rev);
            sample_valid=1;@(negedge clk);sample_valid=0;latency=0;
            while(!duty_valid && latency<128)begin @(negedge clk);latency=latency+1;end
            if(!duty_valid || overrun)$fatal(1,"core protocol at frame %0d",seq);
            if(latency+1>max_latency)max_latency=latency+1;
            if(latency+1<min_latency)min_latency=latency+1;
            nearq(current_d,ed,30);nearq(current_q,eq,30);
            nearq(velocity,ev,40);nearq(reference_q,er,40);
            nearq(voltage_d,evd,200);nearq(voltage_q,evq,200);
            nearq(32'(duty_a),da,2);nearq(32'(duty_b),db,2);nearq(32'(duty_c),dc,2);
            if(voltage_d>98304 || voltage_d < -98304 || voltage_q>98304 || voltage_q < -98304)
                $fatal(1,"voltage limit");
            if(reference_q>32768 || reference_q < -32768)$fatal(1,"current target limit");
            seq=seq+1; repeat(70)@(negedge clk);
        end
        // An overlapped frame must be reported, then clear when stopped.
        sample_valid=1;@(negedge clk);@(negedge clk);sample_valid=0;
        if(!overrun)$fatal(1,"missing overrun");
        run=0;repeat(2)@(negedge clk);
        if(overrun || busy || voltage_d!=0 || voltage_q!=0 || reference_q!=0)
            $fatal(1,"state did not clear on stop");
        // Cancel at every possible pipeline offset and resume with a zero job.
        // No prior PI/filter/CORDIC/divider work may publish after the stop.
        for(cancel=0;cancel<max_latency+2;cancel=cancel+1)begin
            run=0;repeat(2)@(negedge clk);
            run=1;phase_a=12000;phase_b=-5000;target_velocity=65536;
            angle_raw=12'd513;electrical_zero=0;angle_reverse=0;
            sample_valid=1;@(negedge clk);sample_valid=0;
            repeat(cancel)@(negedge clk);run=0;repeat(2)@(negedge clk);
            if(duty_valid || busy || voltage_d!=0 || voltage_q!=0 || reference_q!=0)
                $fatal(1,"cancel offset %0d",cancel);
            run=1;phase_a=0;phase_b=0;target_velocity=0;angle_raw=0;
            sample_valid=1;@(negedge clk);sample_valid=0;latency=0;
            while(!duty_valid && latency<128)begin @(negedge clk);latency=latency+1;end
            if(!duty_valid || current_d!=0 || current_q!=0 || voltage_d!=0 || voltage_q!=0 ||
                reference_q!=0 || duty_a!=1250 || duty_b!=1250 || duty_c!=1250)
                $fatal(1,"stale work after offset %0d",cancel);
            @(negedge clk);
        end
        $display("PASS control float_reference_frames=%0d min_latency_clocks=%0d max_latency_clocks=%0d cancellation_offsets=%0d",seq,min_latency,max_latency,cancel);
        $finish;
    end
endmodule
