// Classical FOC. 10kHz current frames, 1kHz speed PI; all physical values Q16.16.
// Arithmetic is registered in separate steps to meet the 100MHz clock target.
module focmini_control #(
    parameter integer POLE_PAIRS=7,
    parameter integer PWM_HALF=2500
) (
    input wire clk, reset, run, sample_valid,
    input wire [11:0] angle_raw,
    input wire signed [31:0] phase_a, phase_b, target_velocity,
    input wire [15:0] electrical_zero,
    input wire angle_reverse,
    output wire busy,
    output logic overrun, duty_valid,
    output logic [15:0] duty_a, duty_b, duty_c,
    output logic signed [31:0] velocity, current_d, current_q,
    output wire signed [31:0] reference_q, voltage_d, voltage_q
);
    import focmini_math::*;
    typedef enum logic [5:0] {
        IDLE, PREPARE, SCALE, VEL_ERROR, VEL_STEP, VEL_ROUND, VEL_COMMIT,
        TRIG_START, TRIG_WAIT, PARK_PRODUCTS, PARK_SUM,
        FILTER_ERROR, FILTER_PRODUCTS, FILTER_COMMIT,
        SPEED_START, SPEED_WAIT, CURRENT_START, CURRENT_WAIT,
        INVERSE_PRODUCTS, INVERSE_SUM, CLARKE_PRODUCT, PHASES,
        CENTER, DUTY_SCALE, DIV_INIT, DIV_STEP, EMIT
    } state_t;
    state_t state;
    assign busy=state!=IDLE;
    logic signed [19:0] alpha,beta,beta_sum,err_d,err_q,ubeta;
    logic signed [17:0] sn,cs;
    logic signed [31:0] target;
    logic signed [31:0] p0,p1,p2,p3,raw_d,raw_q,step_d,step_q;
    logic signed [31:0] ualpha,ubeta_scaled,ua,ub,uc,center_a,center_b,center_c;
    logic [15:0] theta;
    wire trig_busy,trig_done;
    wire signed [31:0] sine30,cosine30;
    focmini_sincos trig(.clk(clk),.reset(reset || !run),
        .start(state==TRIG_START && !trig_busy),.phase(theta),.busy(trig_busy),
        .done(trig_done),.sine(sine30),.cosine(cosine30));

    logic angle_seen,outer_due;
    logic [11:0] previous_angle;
    logic [3:0] outer_count;
    logic signed [15:0] accumulated_delta,delta_sum;
    logic signed [12:0] captured_delta,delta;
    logic signed [31:0] raw_velocity,velocity_error,velocity_step;
    logic signed [63:0] velocity_product;
    always_comb begin
        delta=$signed({1'b0,angle_raw})-$signed({1'b0,previous_angle});
        if(delta>13'sd2047) delta=delta-13'sd4096;
        else if(delta < -13'sd2048) delta=delta+13'sd4096;
        if(angle_reverse)delta=-delta;
    end
    wire [15:0] mechanical_turns={angle_raw,4'b0};
    wire [15:0] directed_turns=angle_reverse ? 16'b0-mechanical_turns : mechanical_turns;
    wire speed_done,d_done,q_done;
    focmini_pi #(.KP(32'sd32768),.KI_HALF_DT(32'sd328),.LIMIT(32'sd32768),.ERROR_BITS(32))
        velocity_pi(.clk(clk),.reset(reset),.clear(!run),
        .update(state==SPEED_START && outer_due),
        .error(subq(target,velocity)),.value(reference_q),.done(speed_done));
    focmini_pi d_pi(.clk(clk),.reset(reset),.clear(!run),
        .update(state==CURRENT_START),.error(22'(subq(32'sd0,current_d))),.value(voltage_d),.done(d_done));
    focmini_pi q_pi(.clk(clk),.reset(reset),.clear(!run),
        .update(state==CURRENT_START),.error(22'(subq(reference_q,current_q))),.value(voltage_q),.done(q_done));

    // Exact division: floor(N/(9*65536)) = floor((N>>16)/9).
    // Divide three 20-bit unsigned numerators by nine in 20 bounded clocks.
    logic [19:0] scaled_a,scaled_b,scaled_c;
    logic [19:0] div_a,div_b,div_c;
    logic [3:0] rem_a,rem_b,rem_c;
    logic [4:0] div_iteration;
    wire [4:0] trial_a={rem_a,div_a[19]},trial_b={rem_b,div_b[19]},trial_c={rem_c,div_c[19]};
    wire bit_a=trial_a>=9,bit_b=trial_b>=9,bit_c=trial_c>=9;
    function automatic logic [19:0] phase_clip(input logic signed [31:0] x);
        if(x<=0)phase_clip=0;
        else if(x>=589824)phase_clip=20'd589824;
        else phase_clip=x[19:0];
    endfunction
    always_ff @(posedge clk) begin
        if(reset || !run)begin
            state<=IDLE;overrun<=0;duty_valid<=0;theta<=0;target<=0;
            alpha<=0;beta<=0;beta_sum<=0;sn<=0;cs<=0;
            p0<=0;p1<=0;p2<=0;p3<=0;raw_d<=0;raw_q<=0;err_d<=0;err_q<=0;step_d<=0;step_q<=0;
            ualpha<=0;ubeta<=0;ubeta_scaled<=0;ua<=0;ub<=0;uc<=0;center_a<=0;center_b<=0;center_c<=0;
            duty_a<=16'(PWM_HALF/2);duty_b<=16'(PWM_HALF/2);duty_c<=16'(PWM_HALF/2);
            current_d<=0;current_q<=0;velocity<=0;
            angle_seen<=0;previous_angle<=0;outer_count<=0;accumulated_delta<=0;
            captured_delta<=0;delta_sum<=0;outer_due<=0;raw_velocity<=0;velocity_error<=0;velocity_step<=0;velocity_product<=0;
            scaled_a<=0;scaled_b<=0;scaled_c<=0;div_a<=0;div_b<=0;div_c<=0;
            rem_a<=0;rem_b<=0;rem_c<=0;div_iteration<=0;
        end else begin
            duty_valid<=0;
            if(sample_valid && busy)overrun<=1;
            case(state)
                IDLE: if(sample_valid)begin
                    target<=target_velocity;
                    theta<=16'(32'(directed_turns)*POLE_PAIRS)-electrical_zero;
                    alpha<=20'(phase_a);
                    beta_sum<=20'(sat32(64'(phase_a)+(64'(phase_b)<<<1)));
                    captured_delta<=delta;
                    previous_angle<=angle_raw;
                    outer_due<=!angle_seen || outer_count==9;
                    if(!angle_seen)begin angle_seen<=1;captured_delta<=0;end
                    else outer_count<=outer_count==9 ? 0 : outer_count+1'b1;
                    state<=PREPARE;
                end
                PREPARE:begin
                    beta<=20'(mul_bounded(beta_sum,18'(32'sd37837)));
                    delta_sum<=accumulated_delta+16'(captured_delta);
                    state<=SCALE;
                end
                SCALE:begin
                    // Sum of at most 10 wrapped encoder deltas is <=20480.
                    // Its product by 100531 fits signed 32 bits (2.059e9 max).
                    raw_velocity<=32'(delta_sum*$signed(18'd100531));
                    accumulated_delta<=outer_due ? 0 : delta_sum;
                    state<=VEL_ERROR;
                end
                VEL_ERROR:begin velocity_error<=subq(raw_velocity,velocity);state<=VEL_STEP;end
                VEL_STEP:begin velocity_product<=velocity_error*32'sd10923;state<=VEL_ROUND;end
                VEL_ROUND:begin velocity_step<=sat32(velocity_product>>>16);state<=VEL_COMMIT;end
                VEL_COMMIT:begin
                    if(outer_due)velocity<=addq(velocity,velocity_step);
                    state<=TRIG_START;
                end
                TRIG_START:state<=TRIG_WAIT;
                TRIG_WAIT:if(trig_done)begin sn<=18'(sine30>>>14);cs<=18'(cosine30>>>14);state<=PARK_PRODUCTS;end
                PARK_PRODUCTS:begin
                    p0<=mul_bounded(20'(alpha),18'(cs));p1<=mul_bounded(20'(beta),18'(sn));p2<=mul_bounded(20'(beta),18'(cs));p3<=mul_bounded(20'(alpha),18'(sn));state<=PARK_SUM;
                end
                PARK_SUM:begin raw_d<=addq(p0,p1);raw_q<=subq(p2,p3);state<=FILTER_ERROR;end
                FILTER_ERROR:begin err_d<=20'(subq(raw_d,current_d));err_q<=20'(subq(raw_q,current_q));state<=FILTER_PRODUCTS;end
                FILTER_PRODUCTS:begin step_d<=mul_bounded(20'(err_d),18'(32'sd10394));step_q<=mul_bounded(20'(err_q),18'(32'sd10394));state<=FILTER_COMMIT;end
                FILTER_COMMIT:begin current_d<=addq(current_d,step_d);current_q<=addq(current_q,step_q);state<=SPEED_START;end
                SPEED_START:state<=outer_due ? SPEED_WAIT : CURRENT_START;
                SPEED_WAIT:if(speed_done)state<=CURRENT_START;
                CURRENT_START:state<=CURRENT_WAIT;
                CURRENT_WAIT:if(d_done && q_done)state<=INVERSE_PRODUCTS;
                INVERSE_PRODUCTS:begin
                    p0<=mul_bounded(20'(voltage_d),18'(cs));p1<=mul_bounded(20'(voltage_q),18'(sn));p2<=mul_bounded(20'(voltage_d),18'(sn));p3<=mul_bounded(20'(voltage_q),18'(cs));state<=INVERSE_SUM;
                end
                INVERSE_SUM:begin ualpha<=subq(p0,p1);ubeta<=20'(addq(p2,p3));state<=CLARKE_PRODUCT;end
                CLARKE_PRODUCT:begin ubeta_scaled<=mul_bounded(20'(ubeta),18'(32'sd56756));state<=PHASES;end
                PHASES:begin ua<=ualpha;ub<=addq(-(ualpha>>>1),ubeta_scaled);uc<=subq(-(ualpha>>>1),ubeta_scaled);state<=CENTER;end
                CENTER:begin center_a<=addq(ua,32'sd294912);center_b<=addq(ub,32'sd294912);center_c<=addq(uc,32'sd294912);state<=DUTY_SCALE;end
                DUTY_SCALE:begin
                    scaled_a<=20'((36'(phase_clip(center_a))*36'(PWM_HALF))>>16);
                    scaled_b<=20'((36'(phase_clip(center_b))*36'(PWM_HALF))>>16);
                    scaled_c<=20'((36'(phase_clip(center_c))*36'(PWM_HALF))>>16);
                    state<=DIV_INIT;
                end
                DIV_INIT:begin
                    div_a<=scaled_a;div_b<=scaled_b;div_c<=scaled_c;
                    rem_a<=0;rem_b<=0;rem_c<=0;div_iteration<=0;state<=DIV_STEP;
                end
                DIV_STEP:begin
                    rem_a<=4'(bit_a ? trial_a-5'd9 : trial_a);
                    rem_b<=4'(bit_b ? trial_b-5'd9 : trial_b);
                    rem_c<=4'(bit_c ? trial_c-5'd9 : trial_c);
                    div_a<={div_a[18:0],bit_a};div_b<={div_b[18:0],bit_b};div_c<={div_c[18:0],bit_c};
                    if(div_iteration==19)begin
                        duty_a<={div_a[14:0],bit_a};duty_b<={div_b[14:0],bit_b};duty_c<={div_c[14:0],bit_c};state<=EMIT;
                    end else div_iteration<=div_iteration+1'b1;
                end
                EMIT:begin duty_valid<=1;state<=IDLE;end
                default:begin state<=IDLE;overrun<=1;end
            endcase
        end
    end
endmodule
