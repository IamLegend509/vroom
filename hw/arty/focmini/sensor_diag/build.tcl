set_param general.maxThreads 4
set src [file dirname [file normalize [info script]]]
if {[catch {
    foreach f {as5600_reader.sv uart_tx.sv xadc_a0.sv sensor_diag_top.sv} {
        read_verilog -sv [file join $src $f]
    }
    read_xdc [file join $src sensor_diag.xdc]
    synth_design -top sensor_diag_top -part xc7a35ticsg324-1L
    # Fail if any digital top port is unconstrained.
    foreach p [get_ports] {
        if {[get_property PACKAGE_PIN $p] eq ""} { error "Unassigned pin $p" }
    }
    # Require all four asynchronous-input exception endpoints to exist.
    foreach name {button_sync fault_sync scl_sync sda_sync} {
        set re [format {.*%s_reg\[0\]/D} $name]
        if {[llength [get_pins -hier -regexp $re]] != 1} {error "Missing synchronizer $name"}
    }
    # Synthesis proof: every drive-control OBUF has a GND source, independent of reset.
    set proof [open motor_disabled.txt w]
    foreach name {pwm_a pwm_b pwm_c enable reset_n sleep_n} {
        set ob [get_cells -of_objects [get_nets -of_objects [get_ports $name]] -filter {REF_NAME == OBUF}]
        set ip [get_pins -of_objects $ob -filter {REF_PIN_NAME == I}]
        set drv [get_pins -of_objects [get_nets -of_objects $ip] -filter {DIRECTION == OUT}]
        set cells [get_cells -of_objects $drv]
        if {[llength $cells] != 1 || [get_property REF_NAME $cells] ne "GND"} {error "Motor pin not constant GND: $name"}
        puts $proof "$name = constant GND"
    }
    close $proof
    opt_design
    place_design
    phys_opt_design
    route_design
    report_timing_summary -report_unconstrained -file timing.rpt
    report_utilization -file utilization.rpt
    report_drc -file drc.rpt
    report_methodology -file methodology.rpt
    report_io -file io.rpt
    report_cdc -file cdc.rpt
    report_route_status -file route.rpt
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
    write_bitstream sensor_diag.bit
    puts "SENSOR_DIAG_BUILD_PASS"
} msg opts]} {
    puts stderr $msg
    puts stderr [dict get $opts -errorinfo]
    exit 1
}
exit 0
