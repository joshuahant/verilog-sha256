#------------------------------------------------------------
#
# Basic Synthesis Script (TCL format)
#                                  
# Revision History                
#   1/15/03  : Author Shane T. Gehring - from class example
#   2/09/07  : Author Zhengtao Yu      - from class example
#   12/14/07 : Author Ravi Jenkal      - updated to 180 nm & tcl
#
#------------------------------------------------------------

#---------------------------------------------------------
# Read in Verilog file and map (synthesize) onto a generic
# library.
# MAKE SURE THAT YOU CORRECT ALL WARNINGS THAT APPEAR
# during the execution of the read command are fixed 
# or understood to have no impact.
# ALSO CHECK your latch/flip-flop list for unintended 
# latches                                            
#---------------------------------------------------------

#read_sverilog  $RTL_DIR/big_control.v
#read_sverilog  $RTL_DIR/compression_control.v
#read_sverilog  $RTL_DIR/compression_datapath.v
#read_sverilog  $RTL_DIR/m_control.v
#read_sverilog  $RTL_DIR/m_datapath.v
#read_sverilog  $RTL_DIR/w_control.v
#read_sverilog  $RTL_DIR/w_datapath.v
#read_sverilog  $RTL_DIR/sram_push.v
#read_sverilog  $RTL_DIR/sram_pull.v
read_sverilog  $RTL_DIR/MyDesign.v
