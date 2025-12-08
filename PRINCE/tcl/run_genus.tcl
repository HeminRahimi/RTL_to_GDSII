set RTL_PATH "../RTL/"
set LIB_PATH "../lib/"
set LEF_PATH "../lef/"
set TLEF_PATH "../techlef/"

sh mkdir -p ../res/genus_rpt

set DESIGN "top"

set LIB_LIST { asap7sc7p5t_AO_LVT_TT_nldm_211120.lib \
               asap7sc7p5t_INVBUF_LVT_TT_nldm_220122.lib \
               asap7sc7p5t_OA_LVT_TT_nldm_211120.lib \
               asap7sc7p5t_SEQ_LVT_TT_nldm_220123.lib \
               asap7sc7p5t_SIMPLE_LVT_TT_nldm_211120.lib \
               asap7sc7p5t_AO_SLVT_TT_nldm_211120.lib \
               asap7sc7p5t_INVBUF_SLVT_TT_nldm_220122.lib \
               asap7sc7p5t_OA_SLVT_TT_nldm_211120.lib \
               asap7sc7p5t_SEQ_SLVT_TT_nldm_220123.lib \
               asap7sc7p5t_SIMPLE_SLVT_TT_nldm_211120.lib }

set LEF_LIST { asap7_tech_4x_201209.lef \
               asap7sc7p5t_28_L_4x_220121a.lef \
               asap7sc7p5t_28_SL_4x_220121a.lef }

set_db init_lib_search_path "$LIB_PATH $LEF_PATH $TLEF_PATH"
set_db init_hdl_search_path $RTL_PATH
set_db library $LIB_LIST
set_db lef_library $LEF_LIST

read_hdl ../RTL/RTL.v
elaborate $DESIGN

# Load the clean SDC
read_sdc ../PRINCE.sdc

report_port -delay -driver -load [get_ports *] > ../res/genus_rpt/ports.rpt
report_timing -lint

# Synthesis
syn_generic
syn_map
syn_opt


#============ Final reports

write_hdl > ../res/netlist.v

report_timing -max_paths 1000 -nworst 1 \
              > ../res/genus_rpt/timing_setup.rpt

report_timing -max_paths 1000 -nworst 1 -path_type full_clock \
              > ../res/genus_rpt/timing_hold.rpt

report_area -depth 10 -min_count 1 \
            > ../res/genus_rpt/area.rpt

report_gates \
             > ../res/genus_rpt/cell_count.rpt

report_power \
             > ../res/genus_rpt/power.rpt

report_clocks \
              > ../res/genus_rpt/clocks.rpt

check_design \
             > ../res/genus_rpt/check_design.rpt

puts "=== GENUS SYNTHESIS FINISHED! ==="
puts "All reports in: ../res/genus_rpt/"
