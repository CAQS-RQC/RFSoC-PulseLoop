# add_adc.tcl  (v5)  -  triggered 2-channel ADC capture for the AWG block design
#
# Replaces the v2 DMA capture (packet_generator / clock converter / FIFO / axi_dma) by:
#
#   RFDC m00_axis (slice 00 = vin0_01, ADC D) -> adc/s_axis   (ch0)
#   RFDC m02_axis (slice 02 = vin0_23)        -> adc/s1_axis  (ch1)   same tile 224, same clock
#        each: real, no decimation -> 4.9152 GSps, 12 samples/beat (192 bit)
#        | 409.6 MHz from clk_wiz_adc (MMCM fed by the DAC0 614.4 MHz clock -> ADC beats locked to AWG ch0)
#   adc      (adc/hdl/adc_capture.v : trigger engine + AXI-Lite registers)
#        | BRAM port A, ADC clock (never back-pressures the ADC, never loses a sample)
#   blk_mem_adc  (true dual-port BRAM, 16384 x 512 bit = 2 x 196608 samples = 40 us per channel)
#        | BRAM port B, PS clock
#   axi_bram_ctrl_adc  <- ps8_0_axi_periph/M08   (PS reads the samples)
#   adc/s_axi      <- ps8_0_axi_periph/M09   (PS reads/writes the registers)
#
# Trigger sources: immediate, ADC level rising/falling (sample accurate), AWG ch0/ch2 sequence
# start (new seq_tog0/seq_tog2 outputs of awg_BRAM_dual), control_trigger edges, manual.
#
# Normally you do NOT run this by hand: fpga/awg_multichannel_rfsoc.tcl sources it (with DO_BUILD 0).
# Manual use (Vivado 2024.1, AWG block design open):
#   source <repo>/fpga/tcl/add_adc.tcl
# It can be run again: it removes what v2/v3 added before building.
# Log: fpga/tcl/add_adc.log

if {![info exists DO_BUILD]} { set DO_BUILD 1 }   ;# 1 = after a clean validation, regenerate and launch synth+impl+bitstream (the master script sets 0)
set DEPTH_LOG2 14         ;# must match DEPTH_LOG2 / MEM_SIZE in adc_capture.v (64 << 14 = 1 MiB)
set MEM_W      512
set MEM_BYTES  [expr {($MEM_W / 8) << $DEPTH_LOG2}]

set script_dir [file dirname [file normalize [info script]]]
set repo_dir   [file normalize [file join $script_dir ..]]
set core_file  [file join $repo_dir adc hdl adc_capture.v]
set logf [open [file join $script_dir add_adc.log] w]
proc say {msg} { global logf; puts $msg; puts $logf $msg; flush $logf }
proc die {msg} { global logf; say "ERROR: $msg"; close $logf; error $msg }

proc pin {p} {
    set x [get_bd_pins -quiet $p]
    if {[llength $x] == 0} { die "pin $p not found" }
    return $x
}
proc unhook {p} {
    set n [get_bd_nets -quiet -of_objects [pin $p]]
    if {[llength $n]} { disconnect_bd_net $n [pin $p] }
}
# connect pin $src to all pins in $args (re-using the net already on $src)
proc hook {src args} {
    set sp [pin $src]
    foreach d $args { unhook $d }
    set n [get_bd_nets -quiet -of_objects $sp]
    if {[llength $n] == 0} {
        set dl {}
        foreach d $args { lappend dl [pin $d] }
        connect_bd_net $sp {*}$dl
    } else {
        foreach d $args { connect_bd_net -net $n [pin $d] }
    }
}
proc ihook {a b} {
    foreach p [list $a $b] {
        set ip [get_bd_intf_pins -quiet $p]
        if {[llength $ip] == 0} { die "interface pin $p not found" }
        set n [get_bd_intf_nets -quiet -of_objects $ip]
        if {[llength $n]} { delete_bd_objs $n }
    }
    connect_bd_intf_net [get_bd_intf_pins $a] [get_bd_intf_pins $b]
}

set rfdc_name usp_rf_data_converter_0
set S 00 ; set t 0          ;# ch0: ADC slice 00 = tile 224 block 0 (vin0_01, ADC D)
set S1 02                   ;# ch1: ADC slice 02 = tile 224 block 2 (vin0_23), same tile -> same clock

# ------------------------------------------------------------ STEP 0: checks
current_bd_design
set bd [current_bd_design]
say "INFO: block design '$bd'"
foreach c [list $rfdc_name zynq_ultra_ps_e_0 ps8_0_axi_periph rst_ps8_0_99M axi_smc awg_BRAM_dual_0 control_or_gate_0] {
    if {[llength [get_bd_cells -quiet $c]] == 0} { die "cell $c missing - open the AWG block design first" }
}
if {![file exists $core_file]} { die "missing $core_file" }

# ------------------------------------------------------------ STEP 1: RFDC ADC config (same as v2)

set rfdc [get_bd_cells $rfdc_name]
proc cfg_ok {rfdc wantv} {
    array set want $wantv
    set bad {}
    foreach k [lsort [array names want]] {
        set got [get_property $k $rfdc]
        if {[string is double -strict $got] && [string is double -strict $want($k)]} {
            if {abs($got - $want($k)) > 1e-9} { lappend bad "$k=$got (want $want($k))" }
        } elseif {[string tolower $got] ne [string tolower $want($k)]} { lappend bad "$k=$got (want $want($k))" }
    }
    return $bad
}
# 4.9152 GSps, real output, decimation 1 (off), 12 samples/beat -> 409.6 MHz AXI-Stream clock.
# 12 is the largest ADC stream width of the Gen3 RFDC; the RFDC's own clk_adc0 cannot make
# 409.6 MHz (it only offers 38.4/76.8/153.6/307.2), so the stream clock comes from clk_wiz_adc.
set common [list \
    CONFIG.ADC_Slice${S}_Enable  true \
    CONFIG.ADC_Slice${S1}_Enable true \
    CONFIG.ADC${t}_Sampling_Rate 4.9152 \
    CONFIG.ADC${t}_PLL_Enable    true \
    CONFIG.ADC${t}_Refclk_Freq   491.520]
set want_bytes 24
# order matters inside one set_property -dict: type and decimation first, then the values they unlock
set real1 {}
foreach sl [list $S $S1] {
    lappend real1 CONFIG.ADC_Data_Type${sl} 0 CONFIG.ADC_Decimation_Mode${sl} 1 CONFIG.ADC_Mixer_Type${sl} 1 \
                  CONFIG.ADC_Data_Width${sl} 12
}
set candidates [list \
    [concat $common $real1 [list CONFIG.ADC${t}_Outclk_Freq 307.200]] \
    [concat $common $real1]]

# On a rejected set: try each value alone and print what the IP accepts, so the log says exactly what to change.
proc diag_cfg {rfdc cand} {
    foreach {k v} $cand {
        set old [get_property $k $rfdc]
        if {[catch {set_property $k $v $rfdc} e]} {
            set valid ""
            catch {set valid [list_property_value $k $rfdc]}
            say "DIAG: $k = $v rejected (now $old). Valid now: $valid"
        } else {
            say "DIAG: $k = $v ok"
        }
    }
}

set applied 0
set last_err ""
foreach cand $candidates {
    if {[llength [cfg_ok $rfdc $cand]]} {
        # set_property -dict is all-or-nothing: if any value is rejected the previous config is restored
        if {[catch {set_property -dict $cand $rfdc} err]} {
            set last_err $err
            say "INFO: RFDC rejected candidate: $err"
            diag_cfg $rfdc $cand
            if {[llength [cfg_ok $rfdc $cand]]} { continue }
        }
    }
    set bad [cfg_ok $rfdc $cand]
    set nb  [get_property CONFIG.TDATA_NUM_BYTES [get_bd_intf_pins $rfdc_name/m${S}_axis]]
    set nb1 -1
    catch {set nb1 [get_property CONFIG.TDATA_NUM_BYTES [get_bd_intf_pins $rfdc_name/m${S1}_axis]]}
    if {[llength $bad] == 0 && $nb == $want_bytes && $nb1 == $want_bytes} { set applied 1 ; break }
    set last_err "readback: $bad  m${S}_axis bytes=$nb m${S1}_axis bytes=$nb1 (want $want_bytes)"
    say "INFO: candidate not usable: $last_err"
}
if {!$applied} { die "RFDC ADC config failed: $last_err (see the DIAG lines in fpga/tcl/add_adc.log)" }
foreach k [list CONFIG.ADC${t}_Fabric_Freq CONFIG.ADC_Mixer_Type${S} CONFIG.ADC_Mixer_Mode${S} CONFIG.ADC_Coarse_Mixer_Freq${S} CONFIG.ADC_Data_Width${S}] {
    if {![catch {get_property $k $rfdc} v] && $v ne ""} { say "INFO:   $k = $v" }
}
if {![catch {get_property CONFIG.ADC${t}_Fabric_Freq $rfdc} ff] && [string is double -strict $ff] && abs($ff - 409.6) > 0.01} {
    die "RFDC wants an ADC stream clock of $ff MHz, this design makes 409.6 MHz"
}
say "INFO: ADC config ok: slices $S and $S1 (real, no decimation -> 4.9152 GSps, 12 samples/beat each)"

# ------------------------------------------------------------ STEP 2: remove v2 / previous v3 cells
say "STEP 2: cleanup"
# (old pre-rename cell names are listed too, so a re-run cleans up older builds)
foreach c {packet_generator_adc axis_clock_converter_adc axis_fifo_adc axi_dma_adc proc_sys_reset_adc \
           adc_scope blk_mem_scope axi_bram_ctrl_scope adc blk_mem_adc axi_bram_ctrl_adc clk_wiz_adc} {
    set x [get_bd_cells -quiet $c]
    if {[llength $x]} { say "  delete $c"; delete_bd_objs $x }
}
# axi_smc back to its original 2 slaves (S02 was the v2 DMA)
set smc [get_bd_cells axi_smc]
if {[get_property CONFIG.NUM_SI $smc] > 2} {
    # whatever is still on S02 can only be a leftover of the v2 DMA path: show it, then remove it
    set s2 [get_bd_intf_pins -quiet axi_smc/S02_AXI]
    if {[llength $s2]} {
        set n2 [get_bd_intf_nets -quiet -of_objects $s2]
        if {[llength $n2]} {
            foreach n $n2 {
                set others {}
                foreach q [get_bd_intf_pins -quiet -of_objects $n] { lappend others $q }
                say "  S02_AXI leftover net $n connects: $others -> deleting"
            }
            delete_bd_objs $n2
        }
    }
    # clock / reset pins of the SI-2 side: disconnect so shrinking NUM_SI cannot fail
    foreach q [get_bd_pins -quiet axi_smc/aclk2] {
        set nn [get_bd_nets -quiet -of_objects $q]
        if {[llength $nn]} { disconnect_bd_net $nn $q }
    }
    set_property CONFIG.NUM_SI 2 $smc
    say "  axi_smc NUM_SI -> 2"
}
set ic [get_bd_cells ps8_0_axi_periph]
if {[get_property CONFIG.NUM_MI $ic] < 10} { set_property CONFIG.NUM_MI 10 $ic }

# ------------------------------------------------------------ STEP 3: RTL sources
say "STEP 3: sources"
if {[llength [get_files -quiet $core_file]] == 0} {
    add_files -norecurse -fileset sources_1 $core_file
}
update_compile_order -fileset sources_1
if {[llength [get_bd_pins -quiet awg_BRAM_dual_0/seq_tog0]] == 0} {
    say "  refreshing awg_BRAM_dual_0 module reference (new seq_tog0/seq_tog2 outputs)"
    set mref "${bd}_awg_BRAM_dual_0_0"
    if {[catch {update_module_reference $mref} err]} { say "  update_module_reference: $err" }
    current_bd_design $bd
    if {[llength [get_bd_pins -quiet awg_BRAM_dual_0/seq_tog0]] == 0} {
        die "awg_BRAM_dual_0 has no seq_tog0 pin: is src/awg_BRAM_dual.v the new version? In the BD, right-click awg_BRAM_dual_0 > Refresh Module, then run this script again."
    }
}

# ------------------------------------------------------------ STEP 4: external ADC ports
say "STEP 4: ADC ports"
if {[llength [get_bd_intf_ports -quiet adc${t}_clk_1]] == 0} {
    set p [create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 adc${t}_clk_1]
    set_property CONFIG.FREQ_HZ 491520000 $p
}
if {[llength [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins $rfdc_name/adc${t}_clk]]] == 0} {
    connect_bd_intf_net [get_bd_intf_ports adc${t}_clk_1] [get_bd_intf_pins $rfdc_name/adc${t}_clk]
}
if {[llength [get_bd_intf_ports -quiet vin0_01_1]] == 0} {
    create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_analog_io_rtl:1.0 vin0_01_1
}
if {[llength [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins $rfdc_name/vin0_01]]] == 0} {
    connect_bd_intf_net [get_bd_intf_ports vin0_01_1] [get_bd_intf_pins $rfdc_name/vin0_01]
}
# ch1 analog input (slice 02)
if {[llength [get_bd_intf_ports -quiet vin0_23_1]] == 0} {
    create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_analog_io_rtl:1.0 vin0_23_1
}
if {[llength [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins $rfdc_name/vin0_23]]] == 0} {
    connect_bd_intf_net [get_bd_intf_ports vin0_23_1] [get_bd_intf_pins $rfdc_name/vin0_23]
}

# ------------------------------------------------------------ STEP 5: cells
say "STEP 5: ADC capture cells"
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_adc
set adc_cell [create_bd_cell -type module -reference adc_capture adc]
set_property CONFIG.DEPTH_LOG2 $DEPTH_LOG2 $adc_cell
set bctl [create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 axi_bram_ctrl_adc]
set_property -dict [list CONFIG.DATA_WIDTH $MEM_W CONFIG.SINGLE_PORT_BRAM 1 CONFIG.ECC_TYPE 0] $bctl
# 614.4 MHz (DAC0 fabric clock, already on a BUFG) -> 409.6 MHz ADC stream clock + 204.8 MHz frame clock.
# Same MMCM, integer VCO 1228.8 MHz: both outputs keep a fixed phase to the AWG ch0 clock.
set cw [create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:6.0 clk_wiz_adc]
set cw_cfg [list CONFIG.PRIM_SOURCE No_buffer CONFIG.PRIM_IN_FREQ 614.400 \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ 409.600 CONFIG.CLKOUT2_USED true CONFIG.CLKOUT2_REQUESTED_OUT_FREQ 204.800 \
    CONFIG.USE_RESET true CONFIG.RESET_TYPE ACTIVE_HIGH CONFIG.USE_LOCKED true]
if {[catch {set_property -dict [concat $cw_cfg [list CONFIG.USE_PHASE_ALIGNMENT true]] $cw} err]} {
    say "  note: clk_wiz phase alignment not accepted ($err); using default feedback (still timed as related clocks)"
    set_property -dict $cw_cfg $cw
}
foreach {cw_out cw_mhz} {clk_out1 409.6 clk_out2 204.8} {
    set cw_hz [get_property CONFIG.FREQ_HZ [get_bd_pins clk_wiz_adc/$cw_out]]
    say "  clk_wiz_adc/$cw_out = $cw_hz Hz"
    if {![string is double -strict $cw_hz]} {
        say "  note: could not read the clk_wiz_adc/$cw_out frequency, check it in the block design"
    } elseif {abs($cw_hz / 1e6 - $cw_mhz) > 0.001} {
        die "clk_wiz_adc/$cw_out is $cw_hz Hz, need exactly $cw_mhz MHz"
    }
}
set bmg [create_bd_cell -type ip -vlnv xilinx.com:ip:blk_mem_gen:8.4 blk_mem_adc]
if {[catch {set_property -dict [list CONFIG.use_bram_block BRAM_Controller CONFIG.Memory_Type True_Dual_Port_RAM] $bmg} err]} {
    say "  note: blk_mem_gen dict: $err"
    set_property CONFIG.Memory_Type True_Dual_Port_RAM $bmg
}

# ------------------------------------------------------------ STEP 6: connections
say "STEP 6: connections"
ihook $rfdc_name/m${S}_axis             adc/s_axis
ihook $rfdc_name/m${S1}_axis            adc/s1_axis
ihook adc/BRAM_PORTA              blk_mem_adc/BRAM_PORTA
ihook axi_bram_ctrl_adc/BRAM_PORTA    blk_mem_adc/BRAM_PORTB
ihook ps8_0_axi_periph/M08_AXI          axi_bram_ctrl_adc/S_AXI
ihook ps8_0_axi_periph/M09_AXI          adc/s_axi

hook zynq_ultra_ps_e_0/pl_clk0  ps8_0_axi_periph/M08_ACLK ps8_0_axi_periph/M09_ACLK \
                                axi_bram_ctrl_adc/s_axi_aclk adc/s_axi_aclk
hook rst_ps8_0_99M/peripheral_aresetn  ps8_0_axi_periph/M08_ARESETN ps8_0_axi_periph/M09_ARESETN \
                                axi_bram_ctrl_adc/s_axi_aresetn adc/s_axi_aresetn
hook util_ds_buf_0/BUFG_O       clk_wiz_adc/clk_in1 adc/clk_awg
hook clk_wiz_adc/clk_out1       $rfdc_name/m${t}_axis_aclk adc/adc_clk proc_sys_reset_adc/slowest_sync_clk
hook clk_wiz_adc/clk_out2       adc/clk_frm
hook clk_wiz_adc/locked         proc_sys_reset_adc/dcm_locked adc/mmcm_locked
hook adc/mmcm_reset             clk_wiz_adc/reset
hook zynq_ultra_ps_e_0/pl_resetn0  proc_sys_reset_adc/ext_reset_in
hook proc_sys_reset_adc/peripheral_aresetn  $rfdc_name/m${t}_axis_aresetn adc/adc_aresetn
hook awg_BRAM_dual_0/seq_tog0   adc/awg_seq0_tog
hook awg_BRAM_dual_0/seq_tog2   adc/awg_seq2_tog
hook control_or_gate_0/control_trigger  adc/ctrl_trig

# ------------------------------------------------------------ STEP 7: addresses, validate, save
say "STEP 7: addresses / validate / save"
set space [get_bd_addr_spaces zynq_ultra_ps_e_0/Data]
set mseg  [get_bd_addr_segs axi_bram_ctrl_adc/S_AXI/Mem0]
if {[catch {assign_bd_address -target_address_space $space -range $MEM_BYTES $mseg} err]} {
    say "  note: assign with range: $err"
    assign_bd_address -target_address_space $space $mseg
    foreach s [get_bd_addr_segs -quiet -of_objects $space] {
        if {[string match "*axi_bram_ctrl_adc*" $s]} { set_property range $MEM_BYTES $s }
    }
}
assign_bd_address
foreach s [get_bd_addr_segs -quiet -of_objects $space] {
    if {[string match "*adc*" $s]} {
        say [format "  %-50s offset %s range %s" $s [get_property offset $s] [get_property range $s]]
    }
}
set rc [catch {validate_bd_design} verr]
save_bd_design
if {$rc} {
    die "VALIDATION FAILED: $verr (scroll up in the Tcl console to the first ERROR line)"
}
say "Block design validated and saved."

# post-route phys_opt: fixes the small hold violation seen in the DAC0 domain
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]

if {$DO_BUILD} {
    say "STEP 8: generate + synth + impl + bitstream (runs in the background, watch the Design Runs tab)"
    generate_target all [get_files ${bd}.bd]
    reset_run synth_1
    launch_runs impl_1 -to_step write_bitstream -jobs 4
}
if {$DO_BUILD} {
    say "Done. When the run finishes: check WNS >= 0 and WHS >= 0 in the routed timing summary,"
    say "then copy ${bd}_wrapper.bit + ${bd}.hwh to the board under one common name."
} else {
    say "ADC capture added and block design validated (DO_BUILD 0: no build started)."
}
close $logf
