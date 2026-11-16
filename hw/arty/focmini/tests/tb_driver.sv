`timescale 1ns/1ps
module tb_driver;
    logic clk=0;always #5 clk=~clk;
    logic reset=1,fault_n=1,arm=0,stop=0,target_valid=0;
    logic signed [31:0] target_velocity=65536;
    logic calibration_valid=1,angle_reverse=0;
    logic [15:0] electrical_zero=0;
    logic signed [31:0] phase_trip=65536;
    logic sample_valid=0,sensor_ok=1;
    logic [11:0] angle_raw=0;
    logic signed [31:0] phase_a=0,phase_b=0;
    wire sample_request,pwm_a,pwm_b,pwm_c,enable,sleep_n,reset_n,ready;
    wire armed,fault_latched,command_rejected;
    wire [3:0] fault_code;
    wire signed [31:0] velocity,current_d,current_q,reference_q,voltage_d,voltage_q;
    focmini_driver #(.CLK_HZ(1000000),.WAKE_CYCLES(4),.COMMAND_TIMEOUT(1000))dut(.*);
    logic generate_frames=0, inject_frame=0;
    integer tests=0;
    always @(negedge clk) sample_valid=(generate_frames && sample_request) || inject_frame;
    task automatic tick(input integer n);repeat(n)@(negedge clk);endtask
    task automatic fresh;
        generate_frames=0;inject_frame=0;reset=1;fault_n=1;arm=0;stop=0;target_valid=0;
        target_velocity=65536;calibration_valid=1;electrical_zero=0;angle_reverse=0;
        phase_a=0;phase_b=0;sensor_ok=1;phase_trip=65536;
        tick(3);if(enable || sleep_n || pwm_a || pwm_b || pwm_c)$fatal(1,"reset outputs");
        reset=0;generate_frames=1;
        // Three distinct four-clock boot phases with EN always off.
        tick(1);if(!sleep_n || !reset_n)$fatal(1,"wake pins");
        tick(4);if(reset_n)$fatal(1,"reset pulse");
        tick(4);if(!reset_n)$fatal(1,"reset release");
        tick(4);if(!ready || enable)$fatal(1,"boot readiness");
        tick(100);
    endtask
    task automatic start_motion;
        arm=1;tick(1);arm=0;tick(1);
        if(!armed || enable)$fatal(1,"ARM energized or failed");
        target_valid=1;tick(1);target_valid=0;
        tick(260);
        if(!enable || fault_latched)$fatal(1,"failed motion start code=%0d",fault_code);
    endtask
    task automatic expect_fault(input logic [3:0] code);
        tick(8);
        if(!fault_latched || fault_code!=code || enable || sleep_n || pwm_a || pwm_b || pwm_c)
            $fatal(1,"fault expected=%0d got=%0d latch=%b",code,fault_code,fault_latched);
        arm=1;target_valid=1;tick(2);arm=0;target_valid=0;
        if(enable || armed || !fault_latched)$fatal(1,"fault rearmed");
        tests=tests+1;
    endtask
    integer count_a,count_b,count_c,i;
    logic [15:0] prior_a,prior_b,prior_c;
    initial begin
        fresh();
        target_valid=1;tick(1);if(!command_rejected || enable)$fatal(1,"unarmed target");target_valid=0;
        calibration_valid=0;arm=1;tick(1);if(!command_rejected || armed)$fatal(1,"uncalibrated ARM");
        arm=0;calibration_valid=1;start_motion();
        // PWM high-time agrees with the registered threshold on all phases.
        while(dut.carrier!=0)tick(1);
        prior_a=dut.active_a;prior_b=dut.active_b;prior_c=dut.active_c;
        count_a=0;count_b=0;count_c=0;
        for(i=0;i<50;i=i+1)begin
            count_a=count_a+32'(pwm_a);count_b=count_b+32'(pwm_b);count_c=count_c+32'(pwm_c);
            if(dut.active_a!=prior_a || dut.active_b!=prior_b || dut.active_c!=prior_c)
                $fatal(1,"duty changed within carrier period");
            tick(1);
        end
        if(count_a!=2*32'(prior_a) || count_b!=2*32'(prior_b) || count_c!=2*32'(prior_c))
            $fatal(1,"PWM duty/period mismatch");
        target_velocity=393216;target_valid=1;tick(1);
        if(!command_rejected || dut.target!=65536)$fatal(1,"out of range accepted");
        target_valid=0;target_velocity=65536;
        stop=1;#1;if(enable)$fatal(1,"STOP gating");tick(1);stop=0;tick(2);
        if(armed || enable)$fatal(1,"STOP disarm");
        start_motion();target_velocity=0;target_valid=1;tick(1);target_valid=0;tick(2);
        if(armed || enable)$fatal(1,"T0 disarm");tests=tests+1;

        fresh();start_motion();
        // A pulse shorter than one clock, entirely between rising edges.
        #1;fault_n=0;#1;if(enable || sleep_n || pwm_a || pwm_b || pwm_c)$fatal(1,"raw fault cutoff");
        #1;fault_n=1;expect_fault(1);

        fresh();start_motion();phase_a=39322;phase_b=39322;
        // A and B are individually below 1A; reconstructed C is -1.2A.
        tick(110);expect_fault(3);
        fresh();start_motion();sensor_ok=0;tick(110);expect_fault(2);
        fresh();start_motion();generate_frames=0;tick(310);expect_fault(4);
        fresh();start_motion();calibration_valid=0;expect_fault(7);
        fresh();start_motion();electrical_zero=16'h0100;expect_fault(7);
        fresh();start_motion();tick(1010);expect_fault(6);
        fresh();phase_trip=196608;arm=1;tick(1);arm=0;
        if(armed || !command_rejected)$fatal(1,"over-rated phase trip accepted");tests=tests+1;
        fresh();start_motion();
        while(!dut.core_busy)tick(1);
        inject_frame=1;tick(2);inject_frame=0;expect_fault(5);
        fresh();start_motion();
        while(!sample_valid)tick(1);
        tick(2);generate_frames=0;
        while(dut.frame_age<120)tick(1);
        inject_frame=1;tick(2);inject_frame=0;expect_fault(8);
        fresh();start_motion();
        // Valid repeated target transactions refresh the command watchdog.
        repeat(4)begin tick(500);target_valid=1;tick(1);target_valid=0;end
        if(fault_latched || !enable)$fatal(1,"heartbeat failed");tests=tests+1;
        // nFAULT is ignored during wake, but a line still low at READY trips.
        generate_frames=0;reset=1;fault_n=0;tick(3);reset=0;
        tick(8);if(fault_latched || enable)$fatal(1,"boot fault interpreted too early");
        tick(14);fault_n=1;expect_fault(1);
        fresh();fault_n=0;tick(2);fault_n=1;expect_fault(1);
        $display("PASS driver scenarios=%0d",tests);$finish;
    end
    initial begin #1000000;$fatal(1,"test timeout");end
endmodule
