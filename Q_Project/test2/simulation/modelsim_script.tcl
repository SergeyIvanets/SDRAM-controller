
# create modelsim working library
vlib work

# compile all the Verilog sources
vlog ../testbench.v ../../*.v 

# open the testbench module for simulation
vsim -novopt work.testbench

# add all testbench signals to time diagram
#add wave sim:/testbench/*

  add wave -radix bin sim:/testbench/fpga_clk       
  add wave -radix bin sim:/testbench/fpga_reset     
  add wave -radix hex sim:/testbench/fpga_addr      
  add wave -radix bin sim:/testbench/fpga_wr_en     
  add wave -radix hex sim:/testbench/fpga_wr_data   
  add wave -radix hex sim:/testbench/fpga_rd_data   
  add wave -radix bin sim:/testbench/fpga_rd_en     
  add wave -radix bin sim:/testbench/fpga_req       
  add wave -radix bin sim:/testbench/fpga_ack       
  add wave -radix hex sim:/testbench/ram_addr       
  add wave -radix bin sim:/testbench/ram_bank_addr  
  add wave -radix bin sim:/testbench/ram_dqm        
  add wave -radix hex sim:/testbench/ram_data       
  add wave -radix bin sim:/testbench/ram_ras_n      
  add wave -radix bin sim:/testbench/ram_cas_n      
  add wave -radix bin sim:/testbench/ram_clk        
  add wave -radix bin sim:/testbench/ram_clk_en     
  add wave -radix bin sim:/testbench/ram_cs_n       
  add wave -radix bin sim:/testbench/ram_we_n       
  add wave -radix bin sim:/testbench/sdram_controller_inst/write_state_flag       
  add wave -radix bin sim:/testbench/sdram_controller_inst/oe       
  add wave -radix hex sim:/testbench/sdram_controller_inst/data_reg

# run the simulation
run -all

# expand the signals time diagram
wave zoom full
