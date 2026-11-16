# Planned FOCMini board-top ports, Arty A7-35 Rev D/E (CSG324).
# Official pin source: Digilent/digilent-xdc/Arty-A7-35-Master.xdc
# Source SHA256: 3884f49d657924e2903e7cc275ee6d4acc6e5b17a55b818ed742fc59a0818492
# Apply to the future physical board top, NOT focmini_driver or the timing probe.
# Sensor acquisition/board top are not implemented by this file.
# No false paths, IO-delay assumptions or DRC-severity overrides are supplied.

set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports {clk}]
set_property -dict { PACKAGE_PIN D9 IOSTANDARD LVCMOS33 } [get_ports {reset_button}]
set_property -dict { PACKAGE_PIN G13 IOSTANDARD LVCMOS33 } [get_ports {pwm_a}]
set_property -dict { PACKAGE_PIN B11 IOSTANDARD LVCMOS33 } [get_ports {pwm_b}]
set_property -dict { PACKAGE_PIN A11 IOSTANDARD LVCMOS33 } [get_ports {pwm_c}]
set_property -dict { PACKAGE_PIN D12 IOSTANDARD LVCMOS33 } [get_ports {enable}]
set_property -dict { PACKAGE_PIN D13 IOSTANDARD LVCMOS33 } [get_ports {reset_n}]
set_property -dict { PACKAGE_PIN B18 IOSTANDARD LVCMOS33 } [get_ports {sleep_n}]
set_property -dict { PACKAGE_PIN A18 IOSTANDARD LVCMOS33 } [get_ports {fault_n}]
set_property -dict { PACKAGE_PIN E15 IOSTANDARD LVCMOS33 } [get_ports {as5600_scl}]
set_property -dict { PACKAGE_PIN E16 IOSTANDARD LVCMOS33 } [get_ports {as5600_sda}]
set_property -dict { PACKAGE_PIN C6 IOSTANDARD LVCMOS33 } [get_ports {vaux4_p}]
set_property -dict { PACKAGE_PIN C5 IOSTANDARD LVCMOS33 } [get_ports {vaux4_n}]
set_property -dict { PACKAGE_PIN A6 IOSTANDARD LVCMOS33 } [get_ports {vaux5_p}]
set_property -dict { PACKAGE_PIN A5 IOSTANDARD LVCMOS33 } [get_ports {vaux5_n}]
set_property -dict { PACKAGE_PIN A9 IOSTANDARD LVCMOS33 } [get_ports {uart_rx}]
set_property -dict { PACKAGE_PIN D10 IOSTANDARD LVCMOS33 } [get_ports {uart_tx}]
set_property -dict { PACKAGE_PIN H5 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN J5 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN T9 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
create_clock -name board_clk -period 10.000 [get_ports clk]
set_property SLEW SLOW [get_ports {pwm_a pwm_b pwm_c enable reset_n sleep_n uart_tx led[*]}]
set_property DRIVE 8 [get_ports {pwm_a pwm_b pwm_c enable reset_n sleep_n uart_tx led[*]}]
# DRI0058 includes a FAULT pull-up; this optional internal pull is supplementary.
set_property PULLUP TRUE [get_ports fault_n]
# I2C must be driven open-drain by the future IOBUF interface. External pull-ups
# to 3.3V are required; no push-pull high is permitted on either I2C line.
# A0/A1 use their board analog paths to VAUX4/5. Do not also constrain/use the
# separate A0/A1 digital nets (F5/D8) or connect the bare auxiliary FPGA pins.
