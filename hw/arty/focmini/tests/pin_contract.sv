// PORT-CONSTRAINT CHECK FIXTURE ONLY. No sensor acquisition or motor controller.
// Elaborated with synth_design -rtl to validate XDC names and package metadata.
// The validation script never implements this fixture or writes a bitstream.
module focmini_pin_contract (
    input wire clk, reset_button, fault_n,
    input wire vaux4_p,vaux4_n,vaux5_p,vaux5_n,uart_rx,
    inout wire as5600_scl,as5600_sda,
    output wire pwm_a,pwm_b,pwm_c,enable,reset_n,sleep_n,uart_tx,
    output wire [3:0] led
);
    assign {pwm_a,pwm_b,pwm_c,enable,reset_n,sleep_n}=6'b0;
    assign as5600_scl=1'bz;
    assign as5600_sda=1'bz;
    assign uart_tx=1'b1;
    assign led={clk,reset_button,fault_n,
                vaux4_p ^ vaux4_n ^ vaux5_p ^ vaux5_n ^ uart_rx};
endmodule
