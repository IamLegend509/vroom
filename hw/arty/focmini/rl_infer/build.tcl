# Synthesis/implementation of the P1 parity image. Run from the run directory,
# which holds the generated *.mem files. Never programs a device.
set_param general.maxThreads 8
set src [file dirname [file normalize [info script]]]
set fm [file dirname $src]
if {[catch {
    foreach f [glob *.mem] {read_mem $f}
    read_verilog -sv [file join $fm position_loop uart_rx.sv]
    read_verilog -sv [file join $fm sensor_diag uart_tx.sv]
    read_verilog -sv [file join $src rl_dual_mlp.sv]
    read_verilog -sv [file join $src rl_parity_top.sv]
    read_xdc [file join $src rl_parity.xdc]
    synth_design -top rl_parity_top -part xc7a35ticsg324-1L
    foreach p [get_ports] {
        if {[get_property PACKAGE_PIN $p] eq ""} {error "Unassigned pin $p"}
    }
    report_utilization -file synth_utilization.rpt
    report_utilization -hierarchical -file synth_utilization_hier.rpt
    opt_design
    place_design
    phys_opt_design
    route_design
    phys_opt_design
    report_timing_summary -report_unconstrained -file timing.rpt
    report_utilization -file utilization.rpt
    report_utilization -hierarchical -file utilization_hier.rpt
    report_drc -file drc.rpt
    report_methodology -file methodology.rpt
    report_route_status -file route.rpt
    set setup [get_timing_paths -delay_type max -max_paths 1]
    set hold [get_timing_paths -delay_type min -max_paths 1]
    set wns [get_property SLACK $setup]
    set whs [get_property SLACK $hold]
    set nerr [llength [get_drc_violations -filter {SEVERITY == Error}]]
    set dsp [llength [get_cells -hier -filter {REF_NAME =~ DSP48*}]]
    set b36 [llength [get_cells -hier -filter {REF_NAME == RAMB36E1}]]
    set b18 [llength [get_cells -hier -filter {REF_NAME == RAMB18E1}]]
    set s [open summary.txt w]
    puts $s "setup_slack_ns=$wns\nhold_slack_ns=$whs\ndrc_errors=$nerr\ndsp48=$dsp\nramb36=$b36\nramb18=$b18"
    close $s
    if {$wns<0 || $whs<0 || $nerr!=0} {error "Timing or DRC failed"}
    write_checkpoint -force routed.dcp
    write_bitstream -force rl_parity.bit
    puts "RL_PARITY_BUILD_PASS"
} msg opts]} {
    puts stderr $msg
    puts stderr [dict get $opts -errorinfo]
    exit 1
}
exit 0
