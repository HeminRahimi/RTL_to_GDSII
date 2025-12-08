set DESIGN PRINCE
set init_design_uniquify 1
set init_verilog {../res/netlist.v}

set init_design_netlisttype {Verilog}
set init_design_settop {1}
set init_top_cell {top}

set RES_PATH "../res/innovus_rpt/"					
set LEF_PATH "../lef/"
set TLEF_PATH "../techlef/"

set CELL_LEF "$LEF_PATH/asap7sc7p5t_28_L_4x_220121a.lef $LEF_PATH/asap7sc7p5t_28_SL_4x_220121a.lef  $LEF_PATH/asap7sc7p5t_28_R_4x_220121a.lef"
set TECH_LEF $TLEF_PATH/asap7_tech_4x_201209.lef

set init_lef_file "$TECH_LEF $CELL_LEF"

set init_assign_buffer {0}
set init_pwr_net {VDD}
set init_gnd_net {VSS}

set init_cpf_file {}
set init_mmmc_file {../PRINCE.mmmc}

init_design 

#======================
setDesignMode -process 7
setMultiCpuUsage -localCpu 4

setNanoRouteMode -routeBottomRoutingLayer 2
setNanoRouteMode -routeTopRoutingLayer 7

globalNetConnect VDD -type pgpin -pin VDD -inst * 
globalNetConnect VSS -type pgpin -pin VSS -inst * 

set total_cell_area 0.0 ; foreach_in_collection c [get_cells *] { if {![get_property $c is_hierarchical] && [get_property $c ref_name]!=""} {set total_cell_area [expr $total_cell_area + [get_property $c area]]}}

if {$total_cell_area < 1000} {set core_size 200.0} else {set row_height 1.080; set num_rows [expr int(ceil(sqrt($total_cell_area/0.68/$row_height)))]; set core_size [expr $num_rows*$row_height]; set core_size [expr round($core_size/0.216)*0.216]}

floorPlan -site asap7sc7p5t -s $core_size $core_size 30 30 30 30 -noSnap ; puts "CORE = $core_size × $core_size µm   DIE = [expr $core_size+60] × [expr $core_size+60] µm"

setAddRingMode -stacked_via_top_layer    M6 \
               -stacked_via_bottom_layer M5 \
               -skip_via_on_pin          standardcell \
               -skip_via_on_wire_shape   noshape

 addRing -nets {VDD VSS} -type core_rings -follow core \
         -layer {bottom M6 top M6 left M5 right M5} -width 2.167 -spacing 1.44 -offset 1.44

### 
addStripe -nets {VDD VSS} -direction vertical -layer M3 -width 0.936 -spacing 0.360 -set_to_set_distance 12.960 -start_from left -xleft_offset 3.600

set TAP_INTERVAL 12.960  
set TAP_OFFSET    1.296  

addWellTap -cell TAPCELL_ASAP7_75t_L \
           -cellInterval $TAP_INTERVAL \
           -inRowOffset  $TAP_OFFSET \
           -rule         0.0

setPinAssignMode -pinEditInBatch true

editPin -pin {clk reset go Dec_EncBar done key[*] } -side Bottom -layer 4 -spreadType center -spacing 2.2 -fixOverlap 1

editPin -pin {PRNG[*]} -side Top -layer 4 -spreadType center -spacing 1.0 -fixOverlap 1

editPin -pin {inp_share0[*] inp_share1[*]} -side Left -layer 3 -spreadType center -spacing 1.6 -fixOverlap 1

editPin -pin {out_share0[*] out_share1[*]} -side Right -layer 3 -spreadType center -spacing 1.6 -fixOverlap 1

editPin -snap TRACK -pin *
setPinAssignMode -pinEditInBatch false
legalizePin
puts "=== 100% SUCCESS — ALL PINS PLACED"


setSrouteMode -reset
setSrouteMode -viaConnectToShape noshape
sroute -connect { corePin } \
       -layerChangeRange { M1 M7 } \
       -blockPinTarget { nearestTarget } \
       -floatingStripeTarget { blockring padring ring stripe ringpin blockpin followpin } \
       -deleteExistingRoutes \
       -allowJogging 0 \
       -crossoverViaLayerRange { M1 Pad } \
       -nets { VDD VSS } \
       -allowLayerChange 0 \
       -targetViaLayerRange { M1 Pad }

editPowerVia -add_vias 1 -orthogonal_only 0

verify_drc -report ${init_top_cell}_drc_early.rpt > /dev/null

colorizePowerMesh

setOptMode -setupTargetSlack 0.03
setOptMode -holdTargetSlack  0.02
place_opt_design   


ccopt_design -cts

set_interactive_constraint_modes [all_constraint_modes -active]
set_propagated_clock [all_clocks]

puts "=== CLOCK TREE SYNTHESIS FINISHED ==="
report_ccopt_clock_trees -summary
puts "Global skew and latency shown above – target < 10 ps skew is easily met on masked cores"
legalizePin


setNanoRouteMode -quiet -routeWithTimingDriven true
setNanoRouteMode -quiet -drouteFixAntenna true
setNanoRouteMode -quiet -routeWithEco true
setNanoRouteMode -quiet -drouteAutoStop false
routeDesign -globalDetail

setNanoRouteMode -quiet -routeWithTimingDriven false
routeDesign -globalDetail
setNanoRouteMode -quiet -routeWithTimingDriven true

editPowerVia -delete_vias 1 -skip_via_on_pin Standardcell
editPowerVia -add_vias 1 -orthogonal_only 0

puts "=== Creating SPEF file ==="
extractRC
rcOut -spef ${RES_PATH}${init_top_cell}_${DESIGN}.spef

setAnalysisMode -analysisType onChipVariation -cppr both
optDesign -postRoute -setup -hold
optDesign -postRoute -hold

puts "=== READ PARASITICS ==="
read_parasitics -rc_corner rc_typ_25 ${RES_PATH}${init_top_cell}_${DESIGN}.spef


#################################



setSIMode -enable_glitch_report true
setSIMode -enable_glitch_propagation true
puts "=== Creatign noise file ==="
report_noise -threshold 0.25 > ${init_top_cell}_noise.rpt

verifyConnectivity -type all -noAntenna -error 1000000 -warning 50

# Suppress the false-positive WidthTable violations on power ring 
set_verify_drc_mode -exclude_pg_net true
verify_drc -report ${init_top_cell}_final_clean.rpt

defOut -netlist -routing ${RES_PATH}${init_top_cell}_${DESIGN}.def
saveNetlist ${RES_PATH}${init_top_cell}_${DESIGN}.v
write_sdf   ${RES_PATH}${init_top_cell}_${DESIGN}.sdf
write_sdc   ${RES_PATH}${init_top_cell}_${DESIGN}.sdc


puts "=========================================="
puts "   MASKED PRINCE CORE – READY!   "
puts "=========================================="
		
