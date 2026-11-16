# Volatile FPGA configuration only: never accesses configuration flash.
# Usage: vivado ... -source program.tcl -tclargs path/to/verified.bit usb_serial
set bitfile [file normalize [lindex $argv 0]]
set serial [lindex $argv 1]
if {[llength $argv]!=2 || ![file exists $bitfile] || $serial eq ""} {error "bitfile and USB serial required"}
if {[catch {
    open_hw_manager
    connect_hw_server -allow_non_jtag
    refresh_hw_server
    set target [get_hw_targets -filter "NAME =~ *$serial*"]
    if {[llength $target]!=1} {error "Expected one matching USB target"}
    current_hw_target $target
    open_hw_target
    set ds [get_hw_devices]
    if {[llength $ds]!=1} {error "Expected one FPGA"}
    set d [lindex $ds 0]
    set part [get_property PART $d]
    puts "IDENTITY target=$target part=$part"
    if {$part ne "xc7a35t"} {error "Unexpected device: $part"}
    set_property PROGRAM.FILE $bitfile $d
    program_hw_devices $d
    puts "SENSOR_DIAG_PROGRAMMED_VOLATILE"
    close_hw_target
    disconnect_hw_server
    close_hw_manager
} msg opts]} {puts stderr $msg; puts stderr [dict get $opts -errorinfo];exit 1}
exit 0
