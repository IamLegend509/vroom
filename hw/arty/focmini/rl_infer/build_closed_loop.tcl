set_param general.maxThreads 8
set src [file dirname [file normalize [info script]]]
set fm [file dirname $src]
if {[catch {
    foreach f [glob *.mem] {read_mem $f}
    foreach f {as5600_reader.sv uart_tx.sv xadc_a0.sv} {read_verilog -sv [file join $fm sensor_diag $f]}
    read_verilog -sv [file join $fm position_loop uart_rx.sv]
    read_verilog -sv [file join $fm interactive_lab command_rx.sv]
    foreach f {rl_dual_mlp.sv rl_ctrl.sv rl_closed_loop_top.sv} {read_verilog -sv [file join $src $f]}
    read_xdc [file join $src rl_closed_loop.xdc]
    synth_design -top rl_closed_loop_top -part xc7a35ticsg324-1L
    foreach p [get_ports] {
        if {[get_property PACKAGE_PIN $p] eq ""} {error "Unassigned pin $p"}
    }
    if {[get_property PACKAGE_PIN [get_ports uart_tx]] ne "D10" ||
        [get_property PACKAGE_PIN [get_ports uart_rx]] ne "A9"} {error "UART direction mismatch"}
    report_utilization -file synth_utilization.rpt
    report_utilization -hierarchical -file synth_utilization_hier.rpt
    opt_design
    place_design
    phys_opt_design
    route_design
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
    write_bitstream -force rl_closed_loop.bit
    puts "RL_CLOSED_LOOP_BUILD_PASS"
} msg opts]} {
    puts stderr $msg
    puts stderr [dict get $opts -errorinfo]
    exit 1
}
exit 0
