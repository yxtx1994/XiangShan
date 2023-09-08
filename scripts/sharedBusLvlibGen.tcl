#!/usr/bin/tclsh
namespace import ::tcl::mathfunc::*

proc bit {x} {
    set num 0
    while { [expr $x / 2.0] > 0.5 } {
        incr num
        set x [expr $x / 2.0]
    }
    return $num
}


set file_name [lindex $argv 0]
set file_source [open $file_name r+]
set root_name [file rootname $file_name]
set file_sink_clu_name ${root_name}_cluster_memory.lvlib
set file_sink_log_name ${root_name}_logical_memory.lvlib
set file_sink_log_name_forward ""


array set arr_mem ""
set num_line 0
set err_flag false

set list_single "{RW0_clk clock Input} {RW0_addr Address Input} {RW0_wdata Data Input} {RW0_rdata Data Output} {RW0_en Select Input} {RW0_wmask GroupWriteEnable Input} {RW0_wmode WriteEnable Input} {BISTRE ReadEnable Input}"
set list_double "{W0_clk clock Input W} {R0_clk clock Input R} {R0_addr Address Input R} {W0_addr Address Input W} {W0_wdata Data Input W} {R0_rdata Data Output R} {W0_en WriteEnable Input W} {R0_en ReadEnable Input R} {W0_mask GroupWriteEnable Input W}"

while { [gets $file_source line] >= 0 } {
    if {$num_line == 1} {
        set list_temp [split $line ","]
        set length [llength $list_temp]
        set count 0
        while { $count < $length} {
            set arr_mem($num_line,$count) [file rootname [lindex $list_temp $count]]
            incr count
        }
    }
    if {$num_line > 2} {
        set list_temp [split $line ","]
        set root_name [file rootname [lindex $list_temp 1]]
        set file_sink_log_name mbist_${root_name}_logical_memory.lvlib
        set file_sink_log [open $file_sink_log_name w+]
        set length [llength $list_temp]
        set count 0
        while { $count < $length} {
            set arr_mem($num_line,$count) [file rootname [lindex $list_temp $count]]
            incr count
        }
        regexp {([0-9]*)p([0-9]*)x([0-9]*)m([0-9]*)(.*)} $arr_mem($num_line,1) sum port_num depth width mask multicycle
        set selectOH $arr_mem($num_line,5)
        set rambits [bit $depth]
        set mask_bits [expr $width / $mask]
        set width [expr $width / $selectOH]
        puts $file_sink_log "MemoryTemplate(mbist_$arr_mem($num_line,1)) \{"
        puts $file_sink_log "  Algorithm : SMarchChkbvcd;"
        if {[regexp {multicycle} $multicycle mtc]} {
           puts $file_sink_log "  OperationSet : SyncWRVcd_ReadCyclesPerOp2_Setup0_WriteCyclesPerOp2;"
           puts $file_sink_log "  ExtraOperationSets : SyncWRVcd_ReadCyclesPerOp2_Setup0_WriteCyclesPerOp2;"
        } else {
           puts $file_sink_log "  OperationSet : SyncWRvcd;"
        }
        if {$port_num == 2} {
            puts $file_sink_log "  LogicalPorts        : 1R1W;"
            puts $file_sink_log "  // \[start\] : Port functions \{\{\{"
            foreach line $list_double {
                if {[regexp {.*data.*} [lindex $line 0]]} {
                    puts $file_sink_log "  Port([lindex $line 0]\[[expr $width - 1]:0\]) \{"
                } elseif {[regexp {.*addr.*} [lindex $line 0]]} {
                    puts $file_sink_log "  Port([lindex $line 0]\[[expr $rambits - 1]:0\]) \{"
                } elseif {[regexp {.*mask.*} [lindex $line 0]] && $mask_bits != 1} {
                    if {$selectOH >=  $mask_bits} {
                       puts $file_sink_log "  Port([lindex $line 0]) \{"
                    } else {
                       set mask_bits [expr $mask_bits / $selectOH]
                       puts $file_sink_log "  Port([lindex $line 0]\[[expr $mask_bits - 1]:0\]) \{"
                    }
                } elseif {[regexp {.*mask.*} [lindex $line 0]] && $mask_bits == 1} {
                    continue
                } else {
                    puts $file_sink_log "  Port([lindex $line 0]) \{"
                }
                puts $file_sink_log "    Function    : [lindex $line 1];"
                puts $file_sink_log "    Direction   : [lindex $line 2];"
                puts $file_sink_log "    LogicalPort : [lindex $line 3];"
                puts $file_sink_log "  \}"
            }
        } else {
            puts $file_sink_log "  LogicalPorts        : 1RW;"
            puts $file_sink_log "  // \[start\] : Port functions \{\{\{"
            foreach line $list_single {
                if {[regexp {.*data.*} [lindex $line 0]]} {
                    puts $file_sink_log "  Port([lindex $line 0]\[[expr $width - 1]:0\]) \{"                
                } elseif {[regexp {.*addr.*} [lindex $line 0]]} {
                    puts $file_sink_log "  Port([lindex $line 0]\[[expr $rambits - 1]:0\]) \{"                
                } elseif {[regexp {.*mask.*} [lindex $line 0]] && $mask_bits != 1} {
                    if {$selectOH >=  $mask_bits} {
                       puts $file_sink_log "  Port([lindex $line 0]) \{"                
                    } else {
                       set mask_bits [expr $mask_bits / $selectOH]
                       puts $file_sink_log "  Port([lindex $line 0]\[[expr $mask_bits - 1]:0\]) \{"                
                    }
                } elseif {[regexp {.*mask.*} [lindex $line 0]] && $mask_bits == 1} {
                    continue
                } else {
                    puts $file_sink_log "  Port([lindex $line 0]) \{"                
                }
                puts $file_sink_log "    Function  : [lindex $line 1];"
                puts $file_sink_log "    Direction : [lindex $line 2];"
                puts $file_sink_log "  \}"
            }
        }
        puts $file_sink_log "  // \[end\] : Port functions \}\}\}"
        puts $file_sink_log "  AddressCounter \{"
        puts $file_sink_log "    Function(ColumnAddress) \{"
        puts $file_sink_log "      LogicalAddressMap \{"
        puts $file_sink_log "        ColumnAddress\[1:0\]:Address\[1:0\];"
        puts $file_sink_log "      \}"
        puts $file_sink_log "      CountRange \[0:3\];"
        puts $file_sink_log "    \}"
        puts $file_sink_log "    Function(RowAddress) \{"
        puts $file_sink_log "      LogicalAddressMap \{"
        puts $file_sink_log "        RowAddress\[[expr $rambits - 3]:0\]:Address\[[expr $rambits - 1]:2\];"
        puts $file_sink_log "      \}"
        puts $file_sink_log "      CountRange \[0:[expr [expr $depth / 4] - 1]\];"
        puts $file_sink_log "    \}"
        puts $file_sink_log "  \}"
        if {[regexp {([0-9]*)x([0-9]*)} $arr_mem($num_line,7)]} {
           regexp {([0-9]*)x([0-9]*)} $arr_mem($num_line,7) sum cell_depth cell_width
           set cell_rambits [bit $cell_depth] 
           set cell_sum [expr $cell_depth * $cell_width]
           set sum [expr $width * $depth]
           set addr_joint 1
           set data_joint 1
           set pvs_joint  1
           set decodebits 0
           if {$cell_width >= $width} {
              set addr_joint [expr ceil(double($sum)/$cell_sum)]
              set pvs_joint  [expr int(floor(double($cell_width)/$width))]
              set num_inst   [expr $addr_joint * $pvs_joint]
              if {$num_inst > 1} {
                 set decodebits [bit $num_inst]
                 if {$decodebits == 1} {
                    puts $file_sink_log "  MemoryGroupAddressDecoding(Address\[[expr $rambits - 1]\]) \{"
                 } else {
                    puts $file_sink_log "  MemoryGroupAddressDecoding(Address\[[expr $rambits - 1]:[expr $rambits - $decodebits]\]) \{"
                 }
                 set num_temp 0
                 while {$num_temp < $num_inst} {
                    set temp_code [format "%X" $num_temp]
                    puts $file_sink_log "    code($decodebits'h$temp_code) : u_sram_$num_temp;"
                    incr num_temp
                 }
                 puts $file_sink_log "  \}"
              }
           } else {
              set num_inst [expr ceil(double($sum)/$cell_sum)]
              set data_joint [expr int(ceil(double($width)/$cell_width))]
              set addr_joint [expr $num_inst / $data_joint]
              if {$addr_joint > 1} {
                 set decodebits [bit $addr_joint]
                 if {$decodebits == 1} {
                    puts $file_sink_log "  MemoryGroupAddressDecoding(Address\[[expr $rambits - 1]\]) \{"
                 } else {
                    puts $file_sink_log "  MemoryGroupAddressDecoding(Address\[[expr $rambits - 1]:[expr $rambits - $decodebits]\]) \{"
                 }
                 set num_temp 0
                 while {$num_temp < $addr_joint} {
                    set temp_code [format "%X" $num_temp]
                    set index [expr $num_temp * $data_joint]
                    set inst_col u_sram_$index
                    set temp 1
                    while {$temp < $data_joint} {
                       set index [expr $index + $temp]
                       set inst_col [concat $inst_col ",u_sram_$index"]
                       incr temp
                    }
                    puts $file_sink_log "    code($decodebits'h$temp_code) : $inst_col;"
                    incr num_temp
                 }
                 puts $file_sink_log "  \}"
              }
           }
           set num_temp 0
           while {$num_temp < $num_inst} {
               puts $file_sink_log "  PhysicalToLogicalMapping(u_sram_$num_temp) \{"
               puts $file_sink_log "  MemoryTemplate : $arr_mem($num_line,7);"
               puts $file_sink_log "  PinMappings \{"
               if {$pvs_joint > 1} {
                  set mod [expr $num_temp % $pvs_joint]
                  puts $file_sink_log "    PhysicalMemoryDataInput\[[expr [expr $width * [expr $mod + 1]] - 1]:[expr $width * $mod]\]  : LogicalMemoryDataInput\[[expr $width - 1]:0\];"
                  puts $file_sink_log "    PhysicalMemoryDataOutput\[[expr [expr $width * [expr $mod + 1]] - 1]:[expr $width * $mod]\] : LogicalMemoryDataOutput\[[expr $width - 1]:0\];"
               } else {
                  if {$data_joint > 1} {
                     set mod [expr $num_temp % $data_joint]
                     if {[expr $cell_width * [expr $mod + 1]] > $width} {
                        set valid_width [expr $width - $cell_width * $mod]
                        puts $file_sink_log "    PhysicalMemoryDataInput\[[expr $valid_width - 1]:0\]  : LogicalMemoryDataInput\[[expr $width - 1]:[expr $cell_width * $mod]\];"
                        puts $file_sink_log "    PhysicalMemoryDataOutput\[[expr $valid_width - 1]:0\] : LogicalMemoryDataOutput\[[expr $width - 1]:[expr $cell_width * $mod]\];"
                     } else {
                        puts $file_sink_log "    PhysicalMemoryDataInput\[[expr $cell_width - 1]:0\]  : LogicalMemoryDataInput\[[expr [expr $cell_width * [expr $mod + 1]] - 1]:[expr $cell_width * $mod]\];"
                        puts $file_sink_log "    PhysicalMemoryDataOutput\[[expr $cell_width - 1]:0\] : LogicalMemoryDataOutput\[[expr [expr $cell_width * [expr $mod + 1]] - 1]:[expr $cell_width * $mod]\];"
                     }
                  } else {
                     puts $file_sink_log "    PhysicalMemoryDataInput\[[expr $width - 1]:0\]  : LogicalMemoryDataInput\[[expr $width - 1]:0\];"
                     puts $file_sink_log "    PhysicalMemoryDataOutput\[[expr $width - 1]:0\] : LogicalMemoryDataOutput\[[expr $width - 1]:0\];"
                  }
               }
               if {$port_num == 2} {
                   puts $file_sink_log "    PhysicalMemoryWriteAddress\[[expr $cell_rambits - 1]:0\] : LogicalMemoryWriteAddress\[[expr $cell_rambits - 1]:0\];"
                   puts $file_sink_log "    PhysicalMemoryAddress\[[expr $cell_rambits - 1]:0\]      : LogicalMemoryAddress\[[expr $cell_rambits - 1]:0\];"
               } else {
                   puts $file_sink_log "    PhysicalMemoryAddress\[[expr $cell_rambits - 1]:0\]      : LogicalMemoryAddress\[[expr $cell_rambits - 1]:0\];"            
               }
               if { $arr_mem($num_line,4) == "true"} {
                  if {$pvs_joint > 1} {
                     set num_maskbits 0
                     set mod [expr $num_temp % $pvs_joint]
                     set num_group [expr $width * $mod]
                     while { $num_maskbits < $mask_bits } {
                        set num_mask 0
                        while {$num_mask < $mask} {
                           puts $file_sink_log "    PhysicalMemoryGroupWriteEnable\[$num_group\] : LogicalMemoryGroupWriteEnable\[$num_maskbits\];"
                           incr num_mask
                           incr num_group
                        }
                        incr num_maskbits
                     }
                  } else {
                     if {$data_joint < $mask_bits} {
                        set inst_group_form [expr [ceil [expr $cell_width / $mask_bits]]]
                        if {($inst_group_form != 1)} {
                           set cell_mask_bits [expr $cell_width / $mask]
                           set mod [expr $num_temp % $data_joint]
                           set num_maskbits [expr $cell_mask_bits * $mod]
                           set msb_maskbits [expr $cell_mask_bits * ($mod + 1)]
                           set num_group 0
                           while { $num_maskbits < $msb_maskbits } {
                              set num_mask 0
                              while {$num_mask < $mask} {
                                 puts $file_sink_log "    PhysicalMemoryGroupWriteEnable\[$num_group\] : LogicalMemoryGroupWriteEnable\[$num_maskbits\];"
                                 incr num_mask
                                 incr num_group
                              }
                              incr num_maskbits
                           }
                        } else {
                           puts $file_sink_log "    PhysicalMemoryGroupWriteEnable\[[expr $cell_width - 1]:0\] : LogicalMemoryGroupWriteEnable\[[expr $cell_width - 1]:0\];" 
                        }             
                     }
                  }             
               }
               puts $file_sink_log "    \}"
               puts $file_sink_log "  \}"
               incr num_temp
           }
           puts $file_sink_log "\}"
           close $file_sink_log
           if {$file_sink_log_name_forward == $file_sink_log_name} {
           } else {
              puts "$file_sink_log_name is generated successfully"
              set file_sink_log_name_forward $file_sink_log_name
           }
        } else {
           if {$file_sink_log_name_forward == $file_sink_log_name} {
           } else {
              puts "Warning!! $file_sink_log_name is generated partially"
              puts "Warning!! Last column \"SRAM Inst\" can be user defined to perform more detail for PhysicalToLogicalMapping"
              set err_flag true
              set file_sink_log_name_forward $file_sink_log_name
           }
        }
    }
    incr num_line
}

if {$err_flag == "true"} {
   puts ""
   puts "For Example:"
   puts "\"SRAM Name\",\"SRAM Type\",\"SRAM array\",\"pipeline depth\",\"bitWrite\",\"selectOH width\",\"foundry\",\"SRAM Inst\""
   puts "XSTop_XSTile_core_frontend_bpu_ftb_node0,sram_array_1p512x320m80.v,18,10,true,2,smic14,sacrls0s4STANDARD1p512x80m1b2w0c0p0d0t0s2z0rw00"
   puts "XSTop_XSTile_core_frontend_bpu_ftb_node1,sram_array_1p512x320m80.v,18,10,true,2,smic14,sacrls0s4STANDARD1p512x80m1b2w0c0p0d0t0s2z0rw00"
   puts ""
}

set code_num [bit $arr_mem([expr $num_line - 1],2)]
set file_sink_clu [open $file_sink_clu_name w+]
puts $file_sink_clu "MemoryClusterTemplate($arr_mem(1,0)) \{"
puts $file_sink_clu "  MemoryBistInterface($arr_mem(1,0)) \{"
set count 3
set list_rf_port "{mbist_addr WriteAddress Input} {mbist_addr_rd Address Input} {mbist_indata Data Input} {mbist_outdata Data Output} {mbist_readen ReadEnable Input} {mbist_writeen WriteEnable Input} {mbist_array MemoryGroupAddress Input} {mbist_be GroupWriteEnable Input} {mbist_all ConfigurationData Input}"
set list_sram_port "{mbist_addr Address Input} {mbist_indata Data Input} {mbist_outdata Data Output} {mbist_readen ReadEnable Input} {mbist_writeen WriteEnable Input} {mbist_array MemoryGroupAddress Input} {mbist_be GroupWriteEnable Input} {mbist_all ConfigurationData Input}"
if {[regexp {RF} $file_sink_clu_name]} {
    foreach line $list_rf_port {
        if {[regexp {.*data.*} [lindex $line 0]]} {
            puts $file_sink_clu "  Port([lindex $line 0]\[[expr $arr_mem(1,2) - 1]:0\]) \{"                
        } elseif {[regexp {.*addr.*} [lindex $line 0]]} {
            puts $file_sink_clu "  Port([lindex $line 0]\[[expr $arr_mem(1,1) - 1]:0\]) \{"                
        } elseif {[regexp {.*mbist_be.*} [lindex $line 0]]} {
            puts $file_sink_clu "  Port([lindex $line 0]\[[expr $arr_mem(1,4) - 1]:0\]) \{"                
        } elseif {[regexp {.*mbist_array.*} [lindex $line 0]]} {
            puts $file_sink_clu "  Port([lindex $line 0]\[[expr $arr_mem(1,3) - 1]:0\]) \{"
        } else {
            puts $file_sink_clu "  Port([lindex $line 0]) \{"
        }
        puts $file_sink_clu "      Function  : [lindex $line 1];"
        puts $file_sink_clu "      Direction : [lindex $line 2];"
        puts $file_sink_clu "    \}"
    }
} else {
    foreach line $list_rf_port {
        if {[regexp {.*data.*} [lindex $line 0]]} {
            puts $file_sink_clu "  Port([lindex $line 0]\[[expr $arr_mem(1,2) - 1]:0\]) \{"                
        } elseif {[regexp {.*addr.*} [lindex $line 0]]} {
            puts $file_sink_clu "  Port([lindex $line 0]\[[expr $arr_mem(1,1) - 1]:0\]) \{"                
        } elseif {[regexp {.*mbist_be.*} [lindex $line 0]]} {
            puts $file_sink_clu "  Port([lindex $line 0]\[[expr $arr_mem(1,4) - 1]:0\]) \{"                
        } elseif {[regexp {.*mbist_array.*} [lindex $line 0]]} {
            puts $file_sink_clu "  Port([lindex $line 0]\[[expr $arr_mem(1,3) - 1]:0\]) \{"
        } else {
            puts $file_sink_clu "  Port([lindex $line 0]) \{"
        }
        puts $file_sink_clu "      Function  : [lindex $line 1];"
        puts $file_sink_clu "      Direction : [lindex $line 2];"
        puts $file_sink_clu "    \}"
    }
}
puts $file_sink_clu "      // \[end\]   : Interface port functions }}}"
puts $file_sink_clu "    MemoryGroupAddressDecoding(GroupAddress) {"
set count 3
while {$count < $num_line} {
    set temp_code [format "%X" $arr_mem($count,2)]
    puts $file_sink_clu "        code($code_num'h$temp_code)   : $arr_mem($count,0);"
    incr count
}
puts $file_sink_clu "    }"
set count 3
set mtc_flag false
while {$count < $num_line} {
    regexp {([0-9]*)p([0-9]*)x([0-9]*)m([0-9]*)(.*)} $arr_mem($count,1) sum port_num depth width mask multicycle
    set selectOH $arr_mem($count,5)
    set rambits [bit $depth]
    set mask_bits [expr $width / $mask]
    set width [expr $width / $selectOH]
    if {[regexp {multicycle} $multicycle mtc]} {
       set mtc_flag true
    }
    puts $file_sink_clu "    LogicalMemoryToInterfaceMapping($arr_mem($count,0)) {"
    puts $file_sink_clu "      MemoryTemplate : mbist_$arr_mem($count,1);"
    puts $file_sink_clu "      PipelineDepth : $arr_mem($count,3);"
    puts $file_sink_clu "      PinMappings {"
    puts $file_sink_clu "        LogicalMemoryDataInput\[[expr $width - 1]:0\]        : InterfaceDataInput\[[expr $width - 1]:0\];"
    puts $file_sink_clu "        LogicalMemoryDataOutput\[[expr $width - 1]:0\]       : InterfaceDataOutput\[[expr $width - 1]:0\];"
    if {$port_num == 2} {
       puts $file_sink_clu "        LogicalMemoryWriteAddress\[[expr $rambits - 1]:0\]     : InterfaceWriteAddress\[[expr $rambits - 1]:0\];"
       puts $file_sink_clu "        LogicalMemoryAddress\[[expr $rambits - 1]:0\]          : InterfaceAddress\[[expr $rambits - 1]:0\];"
    } else {
       puts $file_sink_clu "        LogicalMemoryAddress\[[expr $rambits - 1]:0\]          : InterfaceAddress\[[expr $rambits - 1]:0\];"
    }
    if {$arr_mem($count,4) == "true"} {
       if {$selectOH >=  $mask_bits} {
          puts $file_sink_clu "        LogicalMemoryGroupWriteEnable\[0\]  : InterfaceGroupWriteEnable\[0\];"
       } else {
          set mask_bits [expr $mask_bits / $selectOH]
          puts $file_sink_clu "        LogicalMemoryGroupWriteEnable\[[expr $mask_bits - 1]:0\]  : InterfaceGroupWriteEnable\[[expr $mask_bits - 1]:0\];"
       }
    }
    puts $file_sink_clu "      }"
    puts $file_sink_clu "    }"
    incr count
}
puts $file_sink_clu "  \}"
puts $file_sink_clu "\}"
if {$mtc_flag == "true"} {
   puts $file_sink_clu ""
   puts $file_sink_clu "OperationSet(SyncWRVcd_ReadCyclesPerOp2_Setup0_WriteCyclesPerOp2) {"
   puts $file_sink_clu "  Operation(NoOperation) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(Write) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ShadowReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(Read) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ShadowReadEnable : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      ShadowReadEnable : On;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(ReadModifyWrite) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ShadowReadEnable : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ShadowReadAddress : On;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(ReadModifyWrite_WithSelectOff) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : Off;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : Off;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(WriteReadCompare) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ShadowReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ShadowReadAddress : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(WriteReadCompare_EvenGWE_ON) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ShadowReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : Off;"
   puts $file_sink_clu "      EvenGroupWriteEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : Off;"
   puts $file_sink_clu "      EvenGroupWriteEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : Off;"
   puts $file_sink_clu "      EvenGroupWriteEnable : On;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(WriteReadCompare_OddGWE_ON) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ShadowReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : On;"
   puts $file_sink_clu "      EvenGroupWriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : On;"
   puts $file_sink_clu "      EvenGroupWriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : On;"
   puts $file_sink_clu "      EvenGroupWriteEnable : Off;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(WriteReadCompare_AllGWE_OFF) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ShadowReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : Off;"
   puts $file_sink_clu "      EvenGroupWriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ShadowReadAddress : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : Off;"
   puts $file_sink_clu "      EvenGroupWriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "      OddGroupWriteEnable : Off;"
   puts $file_sink_clu "      EvenGroupWriteEnable : Off;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(Read_WithReadEnableOff) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(ReadModifyWrite_Column_ShadowWriteRead) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ConcurrentWriteColumnAddress : On;"
   puts $file_sink_clu "      ConcurrentWriteDataPolarity : Inverse;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ConcurrentReadEnable : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      ConcurrentWriteColumnAddress : Off;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ConcurrentReadEnable : On;"
   puts $file_sink_clu "      ConcurrentReadColumnAddress : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(ReadModifyWrite_Row_ShadowWriteRead) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ConcurrentWriteRowAddress : On;"
   puts $file_sink_clu "      ConcurrentWriteDataPolarity : Inverse;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ConcurrentWriteRowAddress : Off;"
   puts $file_sink_clu "      ConcurrentReadEnable : On;"
   puts $file_sink_clu "      ConcurrentReadRowAddress : On;"
   puts $file_sink_clu "      StrobeDataOut;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "  Operation(WriteRead_Column_ShadowReadWrite) {"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : On;"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "      ConcurrentReadEnable : On;"
   puts $file_sink_clu "      ConcurrentReadColumnAddress : On;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      Select : On;"
   puts $file_sink_clu "      WriteEnable : Off;"
   puts $file_sink_clu "      ReadEnable : On;"
   puts $file_sink_clu "      ConcurrentReadColumnAddress : Off;"
   puts $file_sink_clu "      ConcurrentWriteColumnAddress : On;"
   puts $file_sink_clu "      ConcurrentWriteDataPolarity : Inverse;"
   puts $file_sink_clu "      OutputEnable : On;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "    Tick {"
   puts $file_sink_clu "      ReadEnable : Off;"
   puts $file_sink_clu "    }"
   puts $file_sink_clu "  }"
   puts $file_sink_clu "}"
}


close $file_source
close $file_sink_clu
puts "$file_sink_clu_name is generated successfully"
