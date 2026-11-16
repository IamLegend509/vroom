# Out-of-context diagnostic. No pin assignments, bitstream or hardware access.
set_param general.maxThreads 4
set stage [lindex $argv 0]
set root [file dirname [file dirname [file normalize [info script]]]]
set summary [open summary.txt w]
if {[catch {
    foreach name {focmini_math focmini_pi focmini_sincos focmini_current_scale focmini_control focmini_driver} {
        read_verilog -sv [file join $root rtl ${name}.sv]
    }
    read_verilog -sv fixture.sv
    synth_design -top focmini_impl_probe -part xc7a35ticsg324-1L -mode out_of_context
    set_property HD.CLK_SRC BUFGCTRL_X0Y0 [get_ports clk]
    create_clock -name core_clk -period 10.000 [get_ports clk]
    # Explicit diagnostic boundary assumptions; physical IO constraints are pending.
    set_input_delay -clock core_clk 0.000 [get_ports -filter {DIRECTION == IN && NAME != clk}]
    set_output_delay -clock core_clk 0.000 [all_outputs]
    report_utilization -file synth_utilization.rpt
    report_timing_summary -report_unconstrained -file synth_timing.rpt
    write_checkpoint synth.dcp
    if {$stage eq "route"} {
        opt_design
        place_design
        phys_opt_design
        route_design
        report_utilization -file routed_utilization.rpt
        report_timing_summary -report_unconstrained -file routed_timing.rpt
        report_timing -max_paths 20 -file critical_paths.rpt
        report_drc -file drc.rpt
        report_route_status -file route_status.rpt
        write_checkpoint routed.dcp
        # IO-to-fixture paths lack board pins/delays and are not sign-off paths.
        # Include all internal DUT and fixture-to-DUT / DUT-to-fixture paths.
        set registers [all_registers -clock core_clk]
        set setup [get_timing_paths -from $registers -to $registers -delay_type max -max_paths 1]
        set hold [get_timing_paths -from $registers -to $registers -delay_type min -max_paths 1]
        report_timing -from $registers -to $registers -max_paths 20 -file registered_paths.rpt
        if {[llength $setup] == 0 || [llength $hold] == 0} { error "Missing timing paths" }
        set wns [get_property SLACK $setup]
        set whs [get_property SLACK $hold]
        puts $summary "setup_slack_ns=$wns"
        puts $summary "hold_slack_ns=$whs"
        puts $summary "physical_io_status=unvalidated"
        puts $summary "drc_errors=[llength [get_drc_violations -filter {SEVERITY == Error}]]"
        flush $summary
        if {$wns < 0 || $whs < 0} { error "100MHz timing failed" }
        if {[llength [get_drc_violations -filter {SEVERITY == Error}]] != 0} { error "DRC errors" }
    }
    puts $summary "implementation_status=passed"
} message options]} {
    puts $summary "implementation_status=failed"
    close $summary
    puts stderr $message
    puts stderr [dict get $options -errorinfo]
    exit 1
}
close $summary
exit 0
