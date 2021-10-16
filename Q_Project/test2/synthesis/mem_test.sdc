#**************************************************************
# This .sdc file is created by Terasic Tool.
# Users are recommended to modify this file to match users logic.
#**************************************************************

#**************************************************************
# Create Clock
#**************************************************************
create_clock -period "13.468" [get_ports clk74_25M]




# SDRAM CLK
create_generated_clock -source [get_pins {sdram_controller_inst|pll166_inst|altpll_component|auto_generated|pll1|clk[0]}] \
                      -name fpga_clk [get_ports {ram_clk}]


#**************************************************************
# Create Generated Clock
#**************************************************************
derive_pll_clocks



#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************
derive_clock_uncertainty


