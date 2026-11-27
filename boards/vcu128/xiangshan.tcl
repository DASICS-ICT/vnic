
################################################################
# This is a generated script based on design: xiangshan
#
# Though there are limitations about the generated script,
# the main purpose of this utility is to make learning
# IP Integrator Tcl commands easier.
################################################################

namespace eval _tcl {
proc get_script_folder {} {
   set script_path [file normalize [info script]]
   set script_folder [file dirname $script_path]
   return $script_folder
}
}
variable script_folder
set script_folder [_tcl::get_script_folder]

################################################################
# Check if script is running in correct Vivado version.
################################################################
set scripts_vivado_version 2020.2
set current_vivado_version [version -short]

if { [string first $scripts_vivado_version $current_vivado_version] == -1 } {
   puts ""
   catch {common::send_gid_msg -ssname BD::TCL -id 2041 -severity "ERROR" "This script was generated using Vivado <$scripts_vivado_version> and is being run in <$current_vivado_version> of Vivado. Please run the script in Vivado <$scripts_vivado_version> then open the design in Vivado <$current_vivado_version>. Upgrade the design by running \"Tools => Report => Report IP Status...\", then run write_bd_tcl to create an updated script."}

   return 1
}

################################################################
# START
################################################################

# To test this script, run the following commands from Vivado Tcl console:
# source xiangshan_script.tcl


# The design that will be created by this Tcl script contains the following 
# module references:
# f2s_rising_intr_sync, role_top

# Please add the sources of those modules before sourcing this Tcl script.

# If there is no project opened, this script will create a
# project, but make sure you do not have an existing project
# <./myproj/project_1.xpr> in the current working folder.

set list_projs [get_projects -quiet]
if { $list_projs eq "" } {
   create_project project_1 myproj -part xcvu37p-fsvh2892-2L-e
   set_property BOARD_PART xilinx.com:vcu128:part0:1.0 [current_project]
}


# CHANGE DESIGN NAME HERE
variable design_name
set design_name xiangshan

# If you do not already have an existing IP Integrator design open,
# you can create a design using the following command:
#    create_bd_design $design_name

# Creating design if needed
set errMsg ""
set nRet 0

set cur_design [current_bd_design -quiet]
set list_cells [get_bd_cells -quiet]

if { ${design_name} eq "" } {
   # USE CASES:
   #    1) Design_name not set

   set errMsg "Please set the variable <design_name> to a non-empty value."
   set nRet 1

} elseif { ${cur_design} ne "" && ${list_cells} eq "" } {
   # USE CASES:
   #    2): Current design opened AND is empty AND names same.
   #    3): Current design opened AND is empty AND names diff; design_name NOT in project.
   #    4): Current design opened AND is empty AND names diff; design_name exists in project.

   if { $cur_design ne $design_name } {
      common::send_gid_msg -ssname BD::TCL -id 2001 -severity "INFO" "Changing value of <design_name> from <$design_name> to <$cur_design> since current design is empty."
      set design_name [get_property NAME $cur_design]
   }
   common::send_gid_msg -ssname BD::TCL -id 2002 -severity "INFO" "Constructing design in IPI design <$cur_design>..."

} elseif { ${cur_design} ne "" && $list_cells ne "" && $cur_design eq $design_name } {
   # USE CASES:
   #    5) Current design opened AND has components AND same names.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 1
} elseif { [get_files -quiet ${design_name}.bd] ne "" } {
   # USE CASES: 
   #    6) Current opened design, has components, but diff names, design_name exists in project.
   #    7) No opened design, design_name exists in project.

   set errMsg "Design <$design_name> already exists in your project, please set the variable <design_name> to another value."
   set nRet 2

} else {
   # USE CASES:
   #    8) No opened design, design_name not in project.
   #    9) Current opened design, has components, but diff names, design_name not in project.

   common::send_gid_msg -ssname BD::TCL -id 2003 -severity "INFO" "Currently there is no design <$design_name> in project, so creating one..."

   create_bd_design $design_name

   common::send_gid_msg -ssname BD::TCL -id 2004 -severity "INFO" "Making design <$design_name> as current_bd_design."
   current_bd_design $design_name

}

common::send_gid_msg -ssname BD::TCL -id 2005 -severity "INFO" "Currently the variable <design_name> is equal to \"$design_name\"."

if { $nRet != 0 } {
   catch {common::send_gid_msg -ssname BD::TCL -id 2006 -severity "ERROR" $errMsg}
   return $nRet
}

set bCheckIPsPassed 1
##################################################################
# CHECK IPs
##################################################################
set bCheckIPs 1
if { $bCheckIPs == 1 } {
   set list_check_ips "\ 
xilinx.com:ip:axi_gpio:2.0\
xilinx.com:ip:axi_register_slice:2.1\
xilinx.com:ip:blk_mem_gen:8.4\
xilinx.com:ip:axi_bram_ctrl:4.1\
xilinx.com:ip:xlconstant:1.1\
xilinx.com:ip:ddr4:2.2\
xilinx.com:ip:proc_sys_reset:5.0\
xilinx.com:ip:util_vector_logic:2.0\
xilinx.com:ip:axi_uartlite:2.0\
xilinx.com:ip:c_counter_binary:12.0\
xilinx.com:ip:xlslice:1.0\
xilinx.com:ip:util_ds_buf:2.1\
xilinx.com:ip:xlconcat:2.1\
xilinx.com:ip:xdma:4.1\
"

   set list_ips_missing ""
   common::send_gid_msg -ssname BD::TCL -id 2011 -severity "INFO" "Checking if the following IPs exist in the project's IP catalog: $list_check_ips ."

   foreach ip_vlnv $list_check_ips {
      set ip_obj [get_ipdefs -all $ip_vlnv]
      if { $ip_obj eq "" } {
         lappend list_ips_missing $ip_vlnv
      }
   }

   if { $list_ips_missing ne "" } {
      catch {common::send_gid_msg -ssname BD::TCL -id 2012 -severity "ERROR" "The following IPs are not found in the IP Catalog:\n  $list_ips_missing\n\nResolution: Please add the repository containing the IP(s) to the project." }
      set bCheckIPsPassed 0
   }

}

##################################################################
# CHECK Modules
##################################################################
set bCheckModules 1
if { $bCheckModules == 1 } {
   set list_check_mods "\ 
f2s_rising_intr_sync\
role_top\
"

   set list_mods_missing ""
   common::send_gid_msg -ssname BD::TCL -id 2020 -severity "INFO" "Checking if the following modules exist in the project's sources: $list_check_mods ."

   foreach mod_vlnv $list_check_mods {
      if { [can_resolve_reference $mod_vlnv] == 0 } {
         lappend list_mods_missing $mod_vlnv
      }
   }

   if { $list_mods_missing ne "" } {
      catch {common::send_gid_msg -ssname BD::TCL -id 2021 -severity "ERROR" "The following module(s) are not found in the project: $list_mods_missing" }
      common::send_gid_msg -ssname BD::TCL -id 2022 -severity "INFO" "Please add source files for the missing module(s) above."
      set bCheckIPsPassed 0
   }
}

if { $bCheckIPsPassed != 1 } {
  common::send_gid_msg -ssname BD::TCL -id 2023 -severity "WARNING" "Will not continue with creation of design due to the error(s) above."
  return 3
}

##################################################################
# DESIGN PROCs
##################################################################



# Procedure to create entire design; Provide argument to make
# procedure reusable. If parentCell is "", will use root.
proc create_root_design { parentCell } {

  variable script_folder
  variable design_name

  if { $parentCell eq "" } {
     set parentCell [get_bd_cells /]
  }

  # Get object for parentCell
  set parentObj [get_bd_cells $parentCell]
  if { $parentObj == "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2090 -severity "ERROR" "Unable to find parent cell <$parentCell>!"}
     return
  }

  # Make sure parentObj is hier blk
  set parentType [get_property TYPE $parentObj]
  if { $parentType ne "hier" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2091 -severity "ERROR" "Parent <$parentObj> has TYPE = <$parentType>. Expected to be <hier>."}
     return
  }

  # Save current instance; Restore later
  set oldCurInst [current_bd_instance .]

  # Set parent object as current
  current_bd_instance $parentObj


  # Create interface ports
  set c0_ddr4 [ create_bd_intf_port -mode Master -vlnv xilinx.com:interface:ddr4_rtl:1.0 c0_ddr4 ]

  set ddr4_mig_sys_clk [ create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 ddr4_mig_sys_clk ]
  set_property -dict [ list \
   CONFIG.FREQ_HZ {100000000} \
   ] $ddr4_mig_sys_clk

  set pcie_ep_gt_ref_clk [ create_bd_intf_port -mode Slave -vlnv xilinx.com:interface:diff_clock_rtl:1.0 pcie_ep_gt_ref_clk ]
  set_property -dict [ list \
   CONFIG.FREQ_HZ {100000000} \
   ] $pcie_ep_gt_ref_clk


  # Create ports
  set ddr4_mig_calib_done [ create_bd_port -dir O ddr4_mig_calib_done ]
  set pcie_ep_lnk_up [ create_bd_port -dir O pcie_ep_lnk_up ]
  set pcie_ep_perstn [ create_bd_port -dir I -type rst pcie_ep_perstn ]
  set pcie_ep_phy_ready [ create_bd_port -dir O -from 0 -to 0 pcie_ep_phy_ready ]
  set pcie_ep_rxn [ create_bd_port -dir I -from 15 -to 0 pcie_ep_rxn ]
  set pcie_ep_rxp [ create_bd_port -dir I -from 15 -to 0 pcie_ep_rxp ]
  set pcie_ep_txn [ create_bd_port -dir O -from 15 -to 0 pcie_ep_txn ]
  set pcie_ep_txp [ create_bd_port -dir O -from 15 -to 0 pcie_ep_txp ]

  # Create instance: axi_device_resetn_gpio, and set properties
  set axi_device_resetn_gpio [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 axi_device_resetn_gpio ]
  set_property -dict [ list \
   CONFIG.C_ALL_OUTPUTS {1} \
   CONFIG.C_DOUT_DEFAULT {0x00000000} \
   CONFIG.C_GPIO_WIDTH {1} \
 ] $axi_device_resetn_gpio

  # Create instance: axi_ic_bootrom, and set properties
  set axi_ic_bootrom [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_ic_bootrom ]
  set_property -dict [ list \
   CONFIG.NUM_MI {1} \
   CONFIG.NUM_SI {2} \
 ] $axi_ic_bootrom

  # Create instance: axi_ic_ddr_mem, and set properties
  set axi_ic_ddr_mem [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_ic_ddr_mem ]
  set_property -dict [ list \
   CONFIG.NUM_MI {1} \
   CONFIG.NUM_SI {2} \
 ] $axi_ic_ddr_mem

  # Create instance: axi_ic_dma, and set properties
  set axi_ic_dma [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_ic_dma ]
  set_property -dict [ list \
   CONFIG.NUM_MI {1} \
 ] $axi_ic_dma

  # Create instance: axi_ic_ep_bar_axi_lite, and set properties
  set axi_ic_ep_bar_axi_lite [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_ic_ep_bar_axi_lite ]
  set_property -dict [ list \
   CONFIG.NUM_MI {4} \
   CONFIG.NUM_SI {1} \
 ] $axi_ic_ep_bar_axi_lite

  # Create instance: axi_ic_role_io, and set properties
  set axi_ic_role_io [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_interconnect:2.1 axi_ic_role_io ]
  set_property -dict [ list \
   CONFIG.NUM_MI {2} \
   CONFIG.NUM_SI {1} \
 ] $axi_ic_role_io

  # Create instance: axi_register_slice_ctrl, and set properties
  set axi_register_slice_ctrl [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_register_slice:2.1 axi_register_slice_ctrl ]

  # Create instance: axi_register_slice_dma, and set properties
  set axi_register_slice_dma [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_register_slice:2.1 axi_register_slice_dma ]

  # Create instance: axi_register_slice_io, and set properties
  set axi_register_slice_io [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_register_slice:2.1 axi_register_slice_io ]

  # Create instance: axi_register_slice_mem, and set properties
  set axi_register_slice_mem [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_register_slice:2.1 axi_register_slice_mem ]

  # Create instance: bootrom_bram, and set properties
  set bootrom_bram [ create_bd_cell -type ip -vlnv xilinx.com:ip:blk_mem_gen:8.4 bootrom_bram ]

  # Create instance: bootrom_bram_ctrl, and set properties
  set bootrom_bram_ctrl [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_bram_ctrl:4.1 bootrom_bram_ctrl ]
  set_property -dict [ list \
   CONFIG.PROTOCOL {AXI4LITE} \
   CONFIG.SINGLE_PORT_BRAM {1} \
 ] $bootrom_bram_ctrl

  # Create instance: const_gnd, and set properties
  set const_gnd [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_gnd ]
  set_property -dict [ list \
   CONFIG.CONST_VAL {0x0} \
   CONFIG.CONST_WIDTH {1} \
 ] $const_gnd

  # Create instance: const_vcc, and set properties
  set const_vcc [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant:1.1 const_vcc ]
  set_property -dict [ list \
   CONFIG.CONST_VAL {0x1} \
   CONFIG.CONST_WIDTH {1} \
 ] $const_vcc

  # Create instance: ddr4_mig, and set properties
  set ddr4_mig [ create_bd_cell -type ip -vlnv xilinx.com:ip:ddr4:2.2 ddr4_mig ]
  set_property -dict [ list \
   CONFIG.ADDN_UI_CLKOUT1_FREQ_HZ {50} \
   CONFIG.C0.BANK_GROUP_WIDTH {1} \
   CONFIG.C0.CS_WIDTH {2} \
   CONFIG.C0.DDR4_AxiAddressWidth {32} \
   CONFIG.C0.DDR4_AxiDataWidth {512} \
   CONFIG.C0.DDR4_Clamshell {true} \
   CONFIG.C0.DDR4_DataMask {DM_NO_DBI} \
   CONFIG.C0.DDR4_DataWidth {64} \
   CONFIG.C0.DDR4_Ecc {false} \
   CONFIG.C0.DDR4_InputClockPeriod {10000} \
   CONFIG.C0.DDR4_MemoryPart {MT40A512M16HA-075E} \
   CONFIG.C0.DDR4_TimePeriod {750} \
   CONFIG.System_Clock {Differential} \
 ] $ddr4_mig

  # Create instance: ddr4_mig_sync_reset, and set properties
  set ddr4_mig_sync_reset [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 ddr4_mig_sync_reset ]

  # Create instance: ep_perst_gen, and set properties
  set ep_perst_gen [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic:2.0 ep_perst_gen ]
  set_property -dict [ list \
   CONFIG.C_OPERATION {not} \
   CONFIG.C_SIZE {1} \
 ] $ep_perst_gen

  # Create instance: f2s_rising_intr_sync_0, and set properties
  set block_name f2s_rising_intr_sync
  set block_cell_name f2s_rising_intr_sync_0
  if { [catch {set f2s_rising_intr_sync_0 [create_bd_cell -type module -reference $block_name $block_cell_name] } errmsg] } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2095 -severity "ERROR" "Unable to add referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   } elseif { $f2s_rising_intr_sync_0 eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2096 -severity "ERROR" "Unable to referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   }
    set_property -dict [ list \
   CONFIG.INTR_WIDTH {16} \
 ] $f2s_rising_intr_sync_0

  # Create instance: host_uart, and set properties
  set host_uart [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_uartlite:2.0 host_uart ]
  set_property -dict [ list \
   CONFIG.C_BAUDRATE {115200} \
 ] $host_uart

  # Create instance: led_counter, and set properties
  set led_counter [ create_bd_cell -type ip -vlnv xilinx.com:ip:c_counter_binary:12.0 led_counter ]
  set_property -dict [ list \
   CONFIG.Output_Width {28} \
 ] $led_counter

  # Create instance: led_counter_slice, and set properties
  set led_counter_slice [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice:1.0 led_counter_slice ]
  set_property -dict [ list \
   CONFIG.DIN_FROM {27} \
   CONFIG.DIN_TO {27} \
   CONFIG.DIN_WIDTH {28} \
 ] $led_counter_slice

  # Create instance: pcie_ep_ref_clk_buf, and set properties
  set pcie_ep_ref_clk_buf [ create_bd_cell -type ip -vlnv xilinx.com:ip:util_ds_buf:2.1 pcie_ep_ref_clk_buf ]
  set_property -dict [ list \
   CONFIG.C_BUF_TYPE {IBUFDSGTE} \
 ] $pcie_ep_ref_clk_buf

  # Create instance: proc_sys_reset_role, and set properties
  set proc_sys_reset_role [ create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset:5.0 proc_sys_reset_role ]

  # Create instance: role_intr_concat, and set properties
  set role_intr_concat [ create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat:2.1 role_intr_concat ]
  set_property -dict [ list \
   CONFIG.NUM_PORTS {16} \
 ] $role_intr_concat

  # Create instance: role_uart, and set properties
  set role_uart [ create_bd_cell -type ip -vlnv xilinx.com:ip:axi_uartlite:2.0 role_uart ]
  set_property -dict [ list \
   CONFIG.C_BAUDRATE {115200} \
 ] $role_uart

  # Create instance: u_role, and set properties
  set block_name role_top
  set block_cell_name u_role
  if { [catch {set u_role [create_bd_cell -type module -reference $block_name $block_cell_name] } errmsg] } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2095 -severity "ERROR" "Unable to add referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   } elseif { $u_role eq "" } {
     catch {common::send_gid_msg -ssname BD::TCL -id 2096 -severity "ERROR" "Unable to referenced block <$block_name>. Please add the files for ${block_name}'s definition into the project."}
     return 1
   }
  
  # Create instance: xdma_ep, and set properties
  set xdma_ep [ create_bd_cell -type ip -vlnv xilinx.com:ip:xdma:4.1 xdma_ep ]
  set_property -dict [ list \
   CONFIG.axilite_master_en {true} \
   CONFIG.axilite_master_scale {Megabytes} \
   CONFIG.axilite_master_size {32} \
   CONFIG.axist_bypass_en {true} \
   CONFIG.axist_bypass_scale {Megabytes} \
   CONFIG.axist_bypass_size {256} \
   CONFIG.cfg_mgmt_if {false} \
   CONFIG.en_gt_selection {true} \
   CONFIG.functional_mode {DMA} \
   CONFIG.mode_selection {Advanced} \
   CONFIG.pcie_blk_locn {PCIE4C_X1Y0} \
   CONFIG.pciebar2axibar_axil_master {0x10000000} \
   CONFIG.pciebar2axibar_axist_bypass {0x00000000f0000000} \
   CONFIG.pf0_base_class_menu {Processing_accelerators} \
   CONFIG.pf0_sub_class_interface_menu {Unknown} \
   CONFIG.pl_link_cap_max_link_speed {8.0_GT/s} \
   CONFIG.pl_link_cap_max_link_width {X16} \
   CONFIG.select_quad {GTY_Quad_227} \
   CONFIG.xdma_axi_intf_mm {AXI_Memory_Mapped} \
 ] $xdma_ep

  # Create interface connections
  connect_bd_intf_net -intf_net S00_AXI_1 [get_bd_intf_pins axi_ic_bootrom/S00_AXI] [get_bd_intf_pins axi_ic_role_io/M01_AXI]
  connect_bd_intf_net -intf_net S00_AXI_2 [get_bd_intf_pins axi_ic_dma/S00_AXI] [get_bd_intf_pins xdma_ep/M_AXI_BYPASS]
  connect_bd_intf_net -intf_net S00_AXI_4 [get_bd_intf_pins axi_ic_ddr_mem/S00_AXI] [get_bd_intf_pins xdma_ep/M_AXI]
  connect_bd_intf_net -intf_net S00_AXI_5 [get_bd_intf_pins axi_ic_role_io/S00_AXI] [get_bd_intf_pins axi_register_slice_io/M_AXI]
  connect_bd_intf_net -intf_net S01_AXI_1 [get_bd_intf_pins axi_ic_bootrom/S01_AXI] [get_bd_intf_pins axi_ic_ep_bar_axi_lite/M02_AXI]
  connect_bd_intf_net -intf_net axi_ic_bootrom_M00_AXI [get_bd_intf_pins axi_ic_bootrom/M00_AXI] [get_bd_intf_pins bootrom_bram_ctrl/S_AXI]
  connect_bd_intf_net -intf_net axi_ic_ddr_mem_M00_AXI [get_bd_intf_pins axi_ic_ddr_mem/M00_AXI] [get_bd_intf_pins ddr4_mig/C0_DDR4_S_AXI]
  connect_bd_intf_net -intf_net axi_ic_dma_M00_AXI [get_bd_intf_pins axi_ic_dma/M00_AXI] [get_bd_intf_pins axi_register_slice_dma/S_AXI]
  connect_bd_intf_net -intf_net axi_ic_ep_bar_axi_lite_M00_AXI [get_bd_intf_pins axi_ic_ep_bar_axi_lite/M00_AXI] [get_bd_intf_pins host_uart/S_AXI]
  connect_bd_intf_net -intf_net axi_ic_ep_bar_axi_lite_M01_AXI [get_bd_intf_pins axi_ic_ep_bar_axi_lite/M01_AXI] [get_bd_intf_pins axi_register_slice_ctrl/S_AXI]
  connect_bd_intf_net -intf_net axi_ic_ep_bar_axi_lite_M03_AXI [get_bd_intf_pins axi_device_resetn_gpio/S_AXI] [get_bd_intf_pins axi_ic_ep_bar_axi_lite/M03_AXI]
  connect_bd_intf_net -intf_net axi_ic_role_io_M00_AXI [get_bd_intf_pins axi_ic_role_io/M00_AXI] [get_bd_intf_pins role_uart/S_AXI]
  connect_bd_intf_net -intf_net axi_register_slice_ctrl_M_AXI [get_bd_intf_pins axi_register_slice_ctrl/M_AXI] [get_bd_intf_pins u_role/s_axi_ctrl]
  connect_bd_intf_net -intf_net axi_register_slice_dma_M_AXI [get_bd_intf_pins axi_register_slice_dma/M_AXI] [get_bd_intf_pins u_role/s_axi_dma]
  connect_bd_intf_net -intf_net axi_register_slice_mem_M_AXI [get_bd_intf_pins axi_ic_ddr_mem/S01_AXI] [get_bd_intf_pins axi_register_slice_mem/M_AXI]
  connect_bd_intf_net -intf_net bootrom_bram_ctrl_BRAM_PORTA [get_bd_intf_pins bootrom_bram/BRAM_PORTA] [get_bd_intf_pins bootrom_bram_ctrl/BRAM_PORTA]
  connect_bd_intf_net -intf_net ddr4_mig_C0_DDR4 [get_bd_intf_ports c0_ddr4] [get_bd_intf_pins ddr4_mig/C0_DDR4]
  connect_bd_intf_net -intf_net ddr4_mig_sys_clk_in [get_bd_intf_ports ddr4_mig_sys_clk] [get_bd_intf_pins ddr4_mig/C0_SYS_CLK]
  connect_bd_intf_net -intf_net pcie_ep_gt_ref_clk [get_bd_intf_ports pcie_ep_gt_ref_clk] [get_bd_intf_pins pcie_ep_ref_clk_buf/CLK_IN_D]
  connect_bd_intf_net -intf_net u_role_m_axi_io [get_bd_intf_pins axi_register_slice_io/S_AXI] [get_bd_intf_pins u_role/m_axi_io]
  connect_bd_intf_net -intf_net u_role_m_axi_mem [get_bd_intf_pins axi_register_slice_mem/S_AXI] [get_bd_intf_pins u_role/m_axi_mem]
  connect_bd_intf_net -intf_net xdma_ep_M_AXI_LITE [get_bd_intf_pins axi_ic_ep_bar_axi_lite/S00_AXI] [get_bd_intf_pins xdma_ep/M_AXI_LITE]

  # Create port connections
  connect_bd_net -net ARESETN_1 [get_bd_pins axi_ic_bootrom/ARESETN] [get_bd_pins axi_ic_bootrom/M00_ARESETN] [get_bd_pins axi_ic_bootrom/S00_ARESETN] [get_bd_pins axi_ic_bootrom/S01_ARESETN] [get_bd_pins axi_ic_ddr_mem/ARESETN] [get_bd_pins axi_ic_dma/ARESETN] [get_bd_pins axi_ic_ep_bar_axi_lite/ARESETN] [get_bd_pins axi_ic_role_io/ARESETN] [get_bd_pins bootrom_bram_ctrl/s_axi_aresetn] [get_bd_pins proc_sys_reset_role/interconnect_aresetn]
  connect_bd_net -net const_gnd_dout [get_bd_pins const_gnd/dout] [get_bd_pins role_intr_concat/In1] [get_bd_pins role_intr_concat/In2] [get_bd_pins role_intr_concat/In3] [get_bd_pins role_intr_concat/In4] [get_bd_pins role_intr_concat/In5] [get_bd_pins role_intr_concat/In6] [get_bd_pins role_intr_concat/In7] [get_bd_pins role_intr_concat/In8] [get_bd_pins role_intr_concat/In9] [get_bd_pins role_intr_concat/In10] [get_bd_pins role_intr_concat/In11] [get_bd_pins role_intr_concat/In12] [get_bd_pins role_intr_concat/In13] [get_bd_pins role_intr_concat/In14] [get_bd_pins role_intr_concat/In15]
  connect_bd_net -net const_vcc_dout [get_bd_pins const_vcc/dout] [get_bd_pins ddr4_mig_sync_reset/dcm_locked] [get_bd_pins proc_sys_reset_role/dcm_locked]
  connect_bd_net -net ddr4_mig_c0_ddr4_ui_clk [get_bd_pins axi_ic_ddr_mem/M00_ACLK] [get_bd_pins ddr4_mig/c0_ddr4_ui_clk] [get_bd_pins ddr4_mig_sync_reset/slowest_sync_clk]
  connect_bd_net -net ddr4_mig_dut_clk_out [get_bd_pins axi_device_resetn_gpio/s_axi_aclk] [get_bd_pins axi_ic_bootrom/ACLK] [get_bd_pins axi_ic_bootrom/M00_ACLK] [get_bd_pins axi_ic_bootrom/S00_ACLK] [get_bd_pins axi_ic_bootrom/S01_ACLK] [get_bd_pins axi_ic_ddr_mem/ACLK] [get_bd_pins axi_ic_ddr_mem/S01_ACLK] [get_bd_pins axi_ic_dma/ACLK] [get_bd_pins axi_ic_dma/M00_ACLK] [get_bd_pins axi_ic_ep_bar_axi_lite/ACLK] [get_bd_pins axi_ic_ep_bar_axi_lite/M00_ACLK] [get_bd_pins axi_ic_ep_bar_axi_lite/M01_ACLK] [get_bd_pins axi_ic_ep_bar_axi_lite/M02_ACLK] [get_bd_pins axi_ic_ep_bar_axi_lite/M03_ACLK] [get_bd_pins axi_ic_role_io/ACLK] [get_bd_pins axi_ic_role_io/M00_ACLK] [get_bd_pins axi_ic_role_io/M01_ACLK] [get_bd_pins axi_ic_role_io/S00_ACLK] [get_bd_pins axi_register_slice_ctrl/aclk] [get_bd_pins axi_register_slice_dma/aclk] [get_bd_pins axi_register_slice_io/aclk] [get_bd_pins axi_register_slice_mem/aclk] [get_bd_pins bootrom_bram_ctrl/s_axi_aclk] [get_bd_pins ddr4_mig/addn_ui_clkout1] [get_bd_pins f2s_rising_intr_sync_0/slow_clk] [get_bd_pins host_uart/s_axi_aclk] [get_bd_pins led_counter/CLK] [get_bd_pins proc_sys_reset_role/slowest_sync_clk] [get_bd_pins role_uart/s_axi_aclk] [get_bd_pins u_role/aclk]
  connect_bd_net -net ddr4_mig_sync_reset_peripheral_aresetn [get_bd_pins axi_ic_ddr_mem/M00_ARESETN] [get_bd_pins ddr4_mig/c0_ddr4_aresetn] [get_bd_pins ddr4_mig_sync_reset/peripheral_aresetn]
  connect_bd_net -net f2s_rising_intr_sync_0_slow_intr [get_bd_pins f2s_rising_intr_sync_0/slow_intr] [get_bd_pins u_role/s2r_intr]
  connect_bd_net -net host_uart_interrupt [get_bd_pins host_uart/interrupt] [get_bd_pins xdma_ep/usr_irq_req]
  connect_bd_net -net host_uart_tx [get_bd_pins host_uart/tx] [get_bd_pins role_uart/rx]
  connect_bd_net -net led_counter_Q [get_bd_pins led_counter/Q] [get_bd_pins led_counter_slice/Din]
  connect_bd_net -net led_counter_slice_Dout [get_bd_ports pcie_ep_phy_ready] [get_bd_pins led_counter_slice/Dout]
  connect_bd_net -net mig_calib_done [get_bd_ports ddr4_mig_calib_done] [get_bd_pins ddr4_mig/c0_init_calib_complete]
  connect_bd_net -net pcie_ep_perst [get_bd_pins ddr4_mig/sys_rst] [get_bd_pins ep_perst_gen/Res]
  connect_bd_net -net pcie_ep_perstn [get_bd_ports pcie_ep_perstn] [get_bd_pins ddr4_mig_sync_reset/ext_reset_in] [get_bd_pins ep_perst_gen/Op1] [get_bd_pins proc_sys_reset_role/ext_reset_in] [get_bd_pins xdma_ep/sys_rst_n]
  connect_bd_net -net pcie_ep_rxn_1 [get_bd_ports pcie_ep_rxn] [get_bd_pins xdma_ep/pci_exp_rxn]
  connect_bd_net -net pcie_ep_rxp_1 [get_bd_ports pcie_ep_rxp] [get_bd_pins xdma_ep/pci_exp_rxp]
  connect_bd_net -net pcie_ep_sys_clk [get_bd_pins pcie_ep_ref_clk_buf/IBUF_DS_ODIV2] [get_bd_pins xdma_ep/sys_clk]
  connect_bd_net -net pcie_ep_sys_clk_gt [get_bd_pins pcie_ep_ref_clk_buf/IBUF_OUT] [get_bd_pins xdma_ep/sys_clk_gt]
  connect_bd_net -net proc_sys_reset_role_peripheral_aresetn [get_bd_pins axi_device_resetn_gpio/s_axi_aresetn] [get_bd_pins axi_ic_ddr_mem/S01_ARESETN] [get_bd_pins axi_ic_dma/M00_ARESETN] [get_bd_pins axi_ic_ep_bar_axi_lite/M00_ARESETN] [get_bd_pins axi_ic_ep_bar_axi_lite/M01_ARESETN] [get_bd_pins axi_ic_ep_bar_axi_lite/M02_ARESETN] [get_bd_pins axi_ic_ep_bar_axi_lite/M03_ARESETN] [get_bd_pins axi_ic_role_io/M00_ARESETN] [get_bd_pins axi_ic_role_io/M01_ARESETN] [get_bd_pins axi_ic_role_io/S00_ARESETN] [get_bd_pins axi_register_slice_ctrl/aresetn] [get_bd_pins axi_register_slice_dma/aresetn] [get_bd_pins axi_register_slice_io/aresetn] [get_bd_pins axi_register_slice_mem/aresetn] [get_bd_pins host_uart/s_axi_aresetn] [get_bd_pins proc_sys_reset_role/peripheral_aresetn] [get_bd_pins role_uart/s_axi_aresetn] [get_bd_pins u_role/aresetn]
  connect_bd_net -net role_intr_concat_dout [get_bd_pins f2s_rising_intr_sync_0/fast_intr] [get_bd_pins role_intr_concat/dout]
  connect_bd_net -net role_uart_interrupt [get_bd_pins role_intr_concat/In0] [get_bd_pins role_uart/interrupt]
  connect_bd_net -net role_uart_tx [get_bd_pins host_uart/rx] [get_bd_pins role_uart/tx]
  connect_bd_net -net xdma_ep_axi_aclk [get_bd_pins axi_ic_ddr_mem/S00_ACLK] [get_bd_pins axi_ic_dma/S00_ACLK] [get_bd_pins axi_ic_ep_bar_axi_lite/S00_ACLK] [get_bd_pins f2s_rising_intr_sync_0/fast_clk] [get_bd_pins xdma_ep/axi_aclk]
  connect_bd_net -net xdma_ep_axi_aresetn [get_bd_pins axi_ic_ddr_mem/S00_ARESETN] [get_bd_pins axi_ic_dma/S00_ARESETN] [get_bd_pins axi_ic_ep_bar_axi_lite/S00_ARESETN] [get_bd_pins xdma_ep/axi_aresetn]
  connect_bd_net -net xdma_ep_pci_exp_txn [get_bd_ports pcie_ep_txn] [get_bd_pins xdma_ep/pci_exp_txn]
  connect_bd_net -net xdma_ep_pci_exp_txp [get_bd_ports pcie_ep_txp] [get_bd_pins xdma_ep/pci_exp_txp]
  connect_bd_net -net xdma_ep_user_lnk_up [get_bd_ports pcie_ep_lnk_up] [get_bd_pins xdma_ep/user_lnk_up]

  # Create address segments
  assign_bd_address -offset 0x10000000 -range 0x00010000 -target_address_space [get_bd_addr_spaces u_role/m_axi_io] [get_bd_addr_segs bootrom_bram_ctrl/S_AXI/Mem0] -force
  assign_bd_address -offset 0x00000000 -range 0x000100000000 -target_address_space [get_bd_addr_spaces u_role/m_axi_mem] [get_bd_addr_segs ddr4_mig/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK] -force
  assign_bd_address -offset 0x30000000 -range 0x00010000 -target_address_space [get_bd_addr_spaces u_role/m_axi_io] [get_bd_addr_segs role_uart/S_AXI/Reg] -force
  assign_bd_address -offset 0x10200000 -range 0x00100000 -target_address_space [get_bd_addr_spaces xdma_ep/M_AXI_LITE] [get_bd_addr_segs axi_device_resetn_gpio/S_AXI/Reg] -force
  assign_bd_address -offset 0x10000000 -range 0x00010000 -target_address_space [get_bd_addr_spaces xdma_ep/M_AXI_LITE] [get_bd_addr_segs bootrom_bram_ctrl/S_AXI/Mem0] -force
  assign_bd_address -offset 0x00000000 -range 0x000100000000 -target_address_space [get_bd_addr_spaces xdma_ep/M_AXI] [get_bd_addr_segs ddr4_mig/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK] -force
  assign_bd_address -offset 0x10011000 -range 0x00001000 -target_address_space [get_bd_addr_spaces xdma_ep/M_AXI_LITE] [get_bd_addr_segs host_uart/S_AXI/Reg] -force
  assign_bd_address -offset 0x10100000 -range 0x00100000 -target_address_space [get_bd_addr_spaces xdma_ep/M_AXI_LITE] [get_bd_addr_segs u_role/s_axi_ctrl/reg0] -force
  assign_bd_address -offset 0x00000000 -range 0x000100000000 -target_address_space [get_bd_addr_spaces xdma_ep/M_AXI_BYPASS] [get_bd_addr_segs u_role/s_axi_dma/reg0] -force


  # Restore current instance
  current_bd_instance $oldCurInst

  validate_bd_design
  save_bd_design
}
# End of create_root_design()


##################################################################
# MAIN FLOW
##################################################################

create_root_design ""


