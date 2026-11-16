set_param general.maxThreads 4
set src [file dirname [file normalize [info script]]]
set diag [file join [file dirname $src] sensor_diag]
if {[catch {
    foreach f {as5600_reader.sv uart_tx.sv xadc_a0.sv} {read_verilog -sv [file join $diag $f]}
    foreach f {uart_rx.sv command_rx.sv sweep_controller.sv open_loop_top.sv} {read_verilog -sv [file join $src $f]}
    read_xdc [file join $src open_loop.xdc]
    synth_design -top open_loop_top -part xc7a35ticsg324-1L
    foreach p [get_ports] {
        if {[get_property PACKAGE_PIN $p] eq ""} {error "Unassigned pin $p"}
    }
    foreach name {button_sync fault_telemetry scl_sync sda_sync rx_sync fault_level} {
        set re [format {.*%s_reg\[0\]/D} $name]
        if {[llength [get_pins -hier -regexp $re]] != 1} {error "Missing synchronizer $name"}
    }
    if {[llength [get_pins -hier -regexp {.*fault_capture_reg/PRE}]] != 1} {error "No asynchronous fault latch"}
    # Known physical net contract. No unspecified pins or DRC severity overrides.
    if {[get_property PACKAGE_PIN [get_ports uart_tx]] ne "D10" ||
        [get_property PACKAGE_PIN [get_ports uart_rx]] ne "A9"} {error "UART direction mismatch"}
    opt_design
    place_design
    phys_opt_design
    route_design
    report_timing_summary -report_unconstrained -file timing.rpt
    report_utilization -file utilization.rpt
    report_drc -file drc.rpt
    report_methodology -file methodology.rpt
    report_io -file io.rpt
    report_cdc -details -file cdc.rpt
    report_route_status -file route.rpt
    report_timing -from [get_ports fault_n] -to [get_ports {pwm_a pwm_b pwm_c enable reset_n sleep_n}] -max_paths 6 -file fault_kill_timing.rpt
    set setup [get_timing_paths -delay_type max -max_paths 1]
    set hold [get_timing_paths -delay_type min -max_paths 1]
    if {[llength $setup]==0 || [llength $hold]==0} {error "No timed paths"}
    set wns [get_property SLACK $setup]
    set whs [get_property SLACK $hold]
    set nerr [llength [get_drc_violations -filter {SEVERITY == Error}]]
    set summary [open summary.txt w]
    puts $summary "setup_slack_ns=$wns\nhold_slack_ns=$whs\ndrc_errors=$nerr"
    close $summary
    if {$wns<0 || $whs<0 || $nerr!=0} {error "Timing or DRC failed"}
    write_checkpoint routed.dcp
    write_bitstream open_loop.bit
    puts "OPEN_LOOP_BUILD_PASS"
} msg opts]} {
    puts stderr $msg
    puts stderr [dict get $opts -errorinfo]
    exit 1
}
exit 0
