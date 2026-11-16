// DRI0058/DRV8313 3PWM motor controller. All interface inputs are synchronous
// to clk EXCEPT fault_n. Sensor acquisition and board pin mapping are external.
module focmini_driver #(
    parameter integer CLK_HZ=100000000,
    parameter integer PWM_HZ=20000,
    parameter integer WAKE_CYCLES=CLK_HZ/100,  // 10ms per boot phase
    parameter integer SENSOR_TIMEOUT=CLK_HZ/10000*3,
    parameter integer FRAME_TOLERANCE=CLK_HZ/200000, // +/-5us sample jitter
    parameter integer COMMAND_TIMEOUT=CLK_HZ/4
) (
    input wire clk, reset, fault_n,
    input wire arm, stop, target_valid,
    input wire signed [31:0] target_velocity, // mechanical rad/s Q16.16
    input wire calibration_valid, angle_reverse,
    input wire [15:0] electrical_zero,       // electrical turns/65536
    input wire signed [31:0] phase_trip,     // explicit calibrated A Q16.16
    input wire sample_valid, sensor_ok,
    input wire [11:0] angle_raw,
    input wire signed [31:0] phase_a, phase_b,// mapped U,V amperes Q16.16
    output wire sample_request,
    output wire pwm_a, pwm_b, pwm_c, enable, sleep_n, reset_n,
    output wire ready,
    output logic armed, fault_latched, command_rejected,
    output logic [3:0] fault_code,
    output wire signed [31:0] velocity, current_d, current_q,
    output wire signed [31:0] reference_q, voltage_d, voltage_q
);
    localparam integer PERIOD=CLK_HZ/PWM_HZ, HALF=PERIOD/2;
    localparam integer SAMPLE_PERIOD=CLK_HZ/10000;
    localparam logic [1:0] WAKE=0, RESET_LOW=1, SETTLE=2, READY=3;
    logic [1:0] boot;
    logic [31:0] boot_age, frame_age, command_age, sample_clock, carrier;
    logic sensor_seen, pwm_primed, pending_valid;
    logic [15:0] pending_a,pending_b,pending_c,active_a,active_b,active_c;
    logic signed [31:0] target, trip_config;
    logic [15:0] zero_config;
    logic reverse_config;

    // Asynchronous SET catches sub-clock nFAULT pulses. It may clear only
    // during reset/startup. Immediate output gating does not wait for CDC.
    // Boot ignores nFAULT until the wake/reset/settle sequence has completed.
    wire monitor_fault=boot==READY;
    logic fault_capture;
    (* ASYNC_REG = "TRUE" *) logic fault_meta, fault_sync;
    always_ff @(posedge clk or negedge fault_n) begin
        if(!fault_n) fault_capture<=1;
        else if(reset || !monitor_fault) fault_capture<=0;
    end
    always_ff @(posedge clk) begin
        if(reset || !monitor_fault) begin fault_meta<=0; fault_sync<=0; end
        else begin fault_meta<=fault_capture; fault_sync<=fault_meta; end
    end
    wire raw_cut=monitor_fault && (!fault_n || fault_capture);
    assign ready=monitor_fault && !fault_latched && !raw_cut && !reset;
    assign sleep_n=!reset && !fault_latched && !raw_cut;
    assign reset_n=!reset && boot!=RESET_LOW;
    wire command_stop=stop || (target_valid && target_velocity==0);
    wire run=ready && armed && target!=0 && calibration_valid && !command_stop;
    assign enable=run && pwm_primed;
    wire [31:0] triangle=carrier<32'(HALF) ? carrier : 32'(PERIOD-1)-carrier;
    assign pwm_a=enable && triangle<32'(active_a);
    assign pwm_b=enable && triangle<32'(active_b);
    assign pwm_c=enable && triangle<32'(active_c);
    assign sample_request=!reset && sample_clock==32'(SAMPLE_PERIOD-1);

    wire config_ok=calibration_valid && phase_trip>0 && phase_trip<=32'sd163840;
    wire target_ok=target_velocity>=-32'sd327680 && target_velocity<=32'sd327680;
    wire signed [63:0] ia=64'(phase_a), ib=64'(phase_b), ic=-ia-ib;
    wire phase_over=ia>64'(trip_config) || ia < -64'(trip_config) ||
        ib>64'(trip_config) || ib < -64'(trip_config) ||
        ic>64'(trip_config) || ic < -64'(trip_config);
    wire core_busy, core_overrun, duty_valid;
    wire [15:0] duty_a,duty_b,duty_c;
    focmini_control #(.PWM_HALF(HALF)) controller(
        .clk(clk),.reset(reset),.run(run),.sample_valid(sample_valid && sensor_ok),
        .angle_raw(angle_raw),.phase_a(phase_a),.phase_b(phase_b),
        .target_velocity(target),.electrical_zero(zero_config),.angle_reverse(reverse_config),
        .busy(core_busy),.overrun(core_overrun),.duty_valid(duty_valid),
        .duty_a(duty_a),.duty_b(duty_b),.duty_c(duty_c),
        .velocity(velocity),.current_d(current_d),.current_q(current_q),
        .reference_q(reference_q),.voltage_d(voltage_d),.voltage_q(voltage_q));

    logic [3:0] trip_reason;
    always_comb begin
        trip_reason=0;
        if(monitor_fault) begin
            if(fault_sync) trip_reason=1;
            else if(sample_valid && !sensor_ok) trip_reason=2;
            else if(run && sample_valid && phase_over) trip_reason=3;
            else if(armed && frame_age>=32'(SENSOR_TIMEOUT)) trip_reason=4;
            else if(core_overrun || (run && sample_valid && core_busy)) trip_reason=5;
            else if(armed && target!=0 && command_age>=32'(COMMAND_TIMEOUT)) trip_reason=6;
            else if(armed && (!config_ok || electrical_zero!=zero_config ||
                    angle_reverse!=reverse_config || phase_trip!=trip_config)) trip_reason=7;
            else if(run && sample_valid && sensor_seen &&
                    (frame_age<32'(SAMPLE_PERIOD-1-FRAME_TOLERANCE) ||
                     frame_age>32'(SAMPLE_PERIOD-1+FRAME_TOLERANCE))) trip_reason=8;
        end
    end
    always_ff @(posedge clk) begin
        if(reset) begin
            boot<=WAKE; boot_age<=0; frame_age<=0; command_age<=0;
            sample_clock<=0; carrier<=0; sensor_seen<=0;
            armed<=0; target<=0; trip_config<=0; zero_config<=0; reverse_config<=0;
            fault_latched<=0; fault_code<=0; command_rejected<=0;
            pwm_primed<=0; pending_valid<=0;
            pending_a<=0; pending_b<=0; pending_c<=0;
            active_a<=0; active_b<=0; active_c<=0;
        end else begin
            command_rejected<=0;
            if(boot!=READY) begin
                if(boot_age==32'(WAKE_CYCLES-1)) begin
                    case(boot)
                        WAKE: boot<=RESET_LOW;
                        RESET_LOW: boot<=SETTLE;
                        default: boot<=READY;
                    endcase
                    boot_age<=0;
                end
                else boot_age<=boot_age+1'b1;
            end
            sample_clock<=sample_clock==32'(SAMPLE_PERIOD-1) ? 0 : sample_clock+1'b1;
            carrier<=carrier==32'(PERIOD-1) ? 0 : carrier+1'b1;
            if(sample_valid && sensor_ok) begin frame_age<=0; sensor_seen<=1; end
            else if(frame_age<32'(SENSOR_TIMEOUT)) frame_age<=frame_age+1'b1;
            if(command_age<32'(COMMAND_TIMEOUT)) command_age<=command_age+1'b1;

            if(trip_reason!=0) begin
                fault_latched<=1;
                if(!fault_latched) fault_code<=trip_reason;
                armed<=0; target<=0;
            end else if(fault_latched || raw_cut) begin armed<=0; target<=0; end
            else if(command_stop) begin armed<=0; target<=0; end
            else if(arm) begin
                // Explicit ARM always drops the old target and requires a
                // subsequent nonzero target transaction. No automatic restart.
                target<=0; armed<=0;
                if(ready && config_ok && sensor_seen && frame_age<32'(SENSOR_TIMEOUT)) begin
                    armed<=1; zero_config<=electrical_zero;
                    reverse_config<=angle_reverse; trip_config<=phase_trip;
                    command_age<=0;
                end else command_rejected<=1;
            end else if(target_valid) begin
                if(armed && ready && target_ok) begin target<=target_velocity; command_age<=0; end
                else command_rejected<=1;
            end

            if(!run || arm || trip_reason!=0) begin
                pwm_primed<=0; pending_valid<=0;
                active_a<=0; active_b<=0; active_c<=0;
            end else begin
                // Three duty registers commit together only at carrier wrap.
                if(carrier==32'(PERIOD-1) && pending_valid) begin
                    active_a<=pending_a; active_b<=pending_b; active_c<=pending_c;
                    pending_valid<=0; pwm_primed<=1;
                end
                if(duty_valid) begin
                    pending_a<=duty_a; pending_b<=duty_b; pending_c<=duty_c;
                    pending_valid<=1;
                end
            end
        end
    end
    initial begin
        if(CLK_HZ<1000000 || CLK_HZ%PWM_HZ!=0 || PERIOD%2!=0 ||
           HALF>65535 || CLK_HZ%10000!=0 || WAKE_CYCLES<1 ||
           SENSOR_TIMEOUT<SAMPLE_PERIOD || COMMAND_TIMEOUT<SAMPLE_PERIOD ||
           FRAME_TOLERANCE<0 || FRAME_TOLERANCE>=SAMPLE_PERIOD-1)
            $fatal(1,"Invalid FOCMini clock/timing parameters");
    end
endmodule
