# =============================================================================
# awg_multichannel_rfsoc.tcl  -  ONE script that builds the whole design
#
#   project  ->  AWG block design  ->  (ADC capture)  ->  wrapper  ->  bitstream  ->  copied to fpga/bitstreams/
#
# Needs: Vivado 2024.1 (the block design is a 2024.1 export), and the RFSoC4x2 board is NOT required
#        (the part is set directly).
#
# Usage
#   Vivado GUI, Tcl console:
#       source C:/Users/Shogun/Documents/GitCloned/AWG-MultiChannel-RFSoC/fpga/awg_multichannel_rfsoc.tcl
#     options: set the variable BEFORE sourcing, e.g.
#       set AWG_BUILD 0 ; source .../fpga/awg_multichannel_rfsoc.tcl      ;# stop before synthesis
#
#   Command line (batch):
#       vivado -mode batch -source fpga/awg_multichannel_rfsoc.tcl -tclargs [--no-adc] [--proj-dir DIR] [--no-build] [--jobs N]
#
# Options                    variable      default
#   --no-adc    (AWG only)   AWG_ADC 0     ADC capture ON
#   --no-build  (stop after the block design + wrapper are made)   AWG_BUILD 0   build ON
#   --jobs N                 AWG_JOBS N    4
#
# Result: fpga/awg_multichannel_rfsoc/   (git-ignored Vivado project)
#         fpga/bitstreams/awg_adc_v5.bit + .hwh   (2 ADC channels, ADC D + vin0_23, 4.9152 GSps raw, locked to AWG ch0; awg_only_new.* with --no-adc)
#         A full build takes roughly 50 minutes.
# =============================================================================

set awg_fpga_dir [file dirname [file normalize [info script]]]

# ------------------------------------------------------------ options
array set awg_opt {adc 1 build 1 jobs 4 projdir {}}
foreach {awg_var awg_key} {AWG_ADC adc AWG_BUILD build AWG_JOBS jobs AWG_PROJ_DIR projdir} {
    if {[info exists ::$awg_var]} { set awg_opt($awg_key) [set ::$awg_var] }
}
if {[info exists ::argv]} {
    set awg_args $::argv
    for {set awg_i 0} {$awg_i < [llength $awg_args]} {incr awg_i} {
        switch -exact -- [lindex $awg_args $awg_i] {
            --no-adc   { set awg_opt(adc) 0 }
            --adc      { set awg_opt(adc) 1 }
            --adc-mode { incr awg_i ; puts "AWG-BUILD: note: --adc-mode is gone (one ADC setup now), ignored" }
            --proj-dir { incr awg_i ; set awg_opt(projdir) [lindex $awg_args $awg_i] }
            --no-build { set awg_opt(build) 0 }
            --jobs     { incr awg_i ; set awg_opt(jobs) [lindex $awg_args $awg_i] }
            default    { }
        }
    }
}
proc awg_info {msg} { puts "AWG-BUILD: $msg" }
awg_info "adc=$awg_opt(adc) build=$awg_opt(build) jobs=$awg_opt(jobs)  (dir: $awg_fpga_dir)"

# ------------------------------------------------------------ checks (nothing is changed before these pass)
if {[info exists ::AWG_ADC_MODE]} { awg_info "note: AWG_ADC_MODE is gone (one ADC setup now), ignored" }
if {![string match 2024.1* [version -short]]} {
    error "This design needs Vivado 2024.1, found [version -short]. (The block design export is 2024.1; other versions change IP versions and the build fails.)"
}
set awg_adc_rtl [file join $awg_fpga_dir adc hdl adc_capture.v]
set awg_adc_tcl  [file join $awg_fpga_dir tcl add_adc.tcl]
set awg_required [list \
    [file join $awg_fpga_dir bd block_design.tcl] \
    [file join $awg_fpga_dir src awg_BRAM_dual.v] \
    [file join $awg_fpga_dir src control_or_gate.v] \
    [file join $awg_fpga_dir src pmod_test.v] \
    [file join $awg_fpga_dir src dataflower.v] \
    [file join $awg_fpga_dir constrs awg_top.xdc]]
if {$awg_opt(adc)} { lappend awg_required $awg_adc_rtl $awg_adc_tcl }
foreach awg_f $awg_required {
    if {![file exists $awg_f]} { error "Missing file: $awg_f" }
}

# ------------------------------------------------------------ project
set awg_proj     awg_multichannel_rfsoc
set awg_proj_dir [file join $awg_fpga_dir $awg_proj]
if {$awg_opt(projdir) ne ""} { set awg_proj_dir [file normalize $awg_opt(projdir)] }
if {[string length $awg_proj_dir] > 100} {
    puts "AWG-BUILD: WARNING project path is [string length $awg_proj_dir] characters. Vivado can fail on Windows paths over 260 characters once IP output folders are added.\n           If the build fails with 'path too long', use a short folder:  set AWG_PROJ_DIR C:/awg_build  (or --proj-dir C:/awg_build)"
}
set awg_part     xczu48dr-ffvg1517-2-e
set awg_bd       design_1

if {[llength [get_projects -quiet]]} { close_project }
awg_info "creating project in $awg_proj_dir (an existing one is overwritten)"
create_project -force $awg_proj $awg_proj_dir -part $awg_part

add_files [list \
    [file join $awg_fpga_dir src control_or_gate.v] \
    [file join $awg_fpga_dir src pmod_test.v] \
    [file join $awg_fpga_dir src awg_BRAM_dual.v] \
    [file join $awg_fpga_dir src dataflower.v]]

# constraints: top-level pins + async clock groups; the clocks only exist after the RFDC IP is loaded -> implementation only
add_files -fileset constrs_1 [file join $awg_fpga_dir constrs awg_top.xdc]
set_property USED_IN_SYNTHESIS false [get_files awg_top.xdc]

# run strategies FIRST (setting a strategy resets the step options set after it)
set_property strategy Flow_AlternateRoutability [get_runs synth_1]
set_property strategy Performance_NetDelay_high [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

# ------------------------------------------------------------ block design (AWG)
awg_info "building the AWG block design"
source [file join $awg_fpga_dir bd block_design.tcl]
if {[catch {current_bd_design $awg_bd}] || [llength [get_bd_cells -quiet usp_rf_data_converter_0]] == 0} {
    error "The AWG block design was not created (see the first ERROR above in the console, usually an IP version problem)."
}

# ------------------------------------------------------------ ADC capture
if {$awg_opt(adc)} {
    awg_info "adding the ADC capture path"
    set ::DO_BUILD 0           ;# this script does the build, after the wrapper exists
    source $awg_adc_tcl
    current_bd_design $awg_bd
}

# ------------------------------------------------------------ outputs + wrapper
awg_info "validating, generating output products, making the wrapper"
validate_bd_design
save_bd_design
set awg_bd_file [get_files ${awg_bd}.bd]
generate_target all $awg_bd_file
export_ip_user_files -of_objects $awg_bd_file -no_script -sync -force -quiet
set awg_wrapper [make_wrapper -files $awg_bd_file -top]
add_files -norecurse $awg_wrapper
set_property top ${awg_bd}_wrapper [get_filesets sources_1]
update_compile_order -fileset sources_1

if {!$awg_opt(build)} {
    awg_info "--no-build: stopping here. Start the build with:  launch_runs impl_1 -to_step write_bitstream -jobs $awg_opt(jobs)"
    return
}

# ------------------------------------------------------------ synthesis + implementation + bitstream
awg_info "synthesis + implementation + bitstream (about 50 min). Progress: Design Runs tab."
launch_runs impl_1 -to_step write_bitstream -jobs $awg_opt(jobs)
wait_on_run impl_1

set awg_run     [get_runs impl_1]
set awg_status  [get_property STATUS $awg_run]
set awg_run_dir [get_property DIRECTORY $awg_run]
awg_info "impl_1 status: $awg_status"
foreach {awg_lbl awg_prop} {WNS STATS.WNS WHS STATS.WHS TNS STATS.TNS} {
    awg_info "  $awg_lbl = [get_property $awg_prop $awg_run]"
}
if {![string match "*write_bitstream Complete*" $awg_status]} {
    error "Build did not reach the bitstream: $awg_status. Open the failed run's log in $awg_run_dir (runme.log)."
}
set awg_wns [get_property STATS.WNS $awg_run]
set awg_whs [get_property STATS.WHS $awg_run]
if {($awg_wns ne "" && $awg_wns < 0) || ($awg_whs ne "" && $awg_whs < 0)} {
    puts "AWG-BUILD: WARNING timing is NOT met (WNS=$awg_wns WHS=$awg_whs). Do not trust this bitstream; look at the routed timing summary."
}

# ------------------------------------------------------------ copy bit + hwh next to the others
set awg_name [expr {$awg_opt(adc) ? "awg_adc_v5" : "awg_only_new"}]
set awg_out  [file join $awg_fpga_dir bitstreams]
file mkdir $awg_out
set awg_bit [file join $awg_run_dir ${awg_bd}_wrapper.bit]
set awg_hwh [lindex [glob -nocomplain -directory $awg_proj_dir */sources_1/bd/$awg_bd/hw_handoff/$awg_bd.hwh] 0]
file copy -force $awg_bit [file join $awg_out $awg_name.bit]
if {$awg_hwh ne ""} {
    file copy -force $awg_hwh [file join $awg_out $awg_name.hwh]
} else {
    puts "AWG-BUILD: WARNING .hwh not found under $awg_proj_dir (look in *.gen/sources_1/bd/$awg_bd/hw_handoff/)"
}
awg_info "DONE. Copied $awg_name.bit/.hwh to $awg_out"
awg_info "On the board load it with  Overlay('$awg_name.bit')  and use python/adc_scope.py (BD cells: adc, axi_bram_ctrl_adc)."
