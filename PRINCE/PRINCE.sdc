
set sdc_version 2.0

create_clock -name clk -period 1000 [get_ports clk]

set_clock_uncertainty 80 [get_clocks clk]

set_clock_latency -source 120 [get_clocks clk]
set_clock_transition 100 [get_clocks clk]

set_input_delay -clock clk -max 150 [remove_from_collection [all_inputs] [get_ports clk]]
set_output_delay -clock clk -max 200 [all_outputs]

set_drive 0.001 [all_inputs]
set_load  0.05  [all_outputs]

set_max_transition 300 [current_design]
set_max_fanout     20  [all_inputs]


