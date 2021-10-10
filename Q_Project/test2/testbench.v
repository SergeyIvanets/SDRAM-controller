`timescale 1ns/100ps

module testbench;

  parameter  
                T_POWER_UP      = 100,
                FPGA_ADDR_WIDTH   = 23, //! System address width
                FPGA_DATA_WIDTH   = 32,   //! System data width
                CLK_FREQUENCY_SYS = 166,  //! MHz System clock
                SDRAM_ADDR_WIDTH  = 12,   //! SDRAM address width
                SDRAM_DATA_WIDTH  = 32,   //! SDRAM data width
                SDRAM_COL_WIDTH   = 9,    //! SDRAM colomn width
                SDRAM_ROW_WIDTH   = 12,   //! SDRAM row width
                SDRAM_BANK_WIDTH  = 2,    //! SDRAM bank width
                SDRAM_BYTES_WIDTH = $clog2 (SDRAM_DATA_WIDTH / 8),
                SDRAM_CLK_MAX     = 166,  //! MHz From SDRAM datasheet
                T_RP              = 18,
                T_RC              = 60,    
                T_MRD             = 18,
                T_RCD             = 18,
                T_REF             = 640,
                T_RAS             = 42,
                T_WR              = 12,
                WRITE_BURST_MODE  = 1'b1,
                BURST_TYPE        = 1'b0,
                BURST_LENGTH      = 3'b000,
                CAS_LATENCY       = 3'b010,

                // simulation options
                clock_period      = 10;

  localparam               CLK_PERIOD = 1.0/SDRAM_CLK_MAX * 1_000.0;
  localparam integer REFRESH_INTERVAL =T_REF/CLK_PERIOD - 10;

  reg                              fpga_clk;
  reg                              fpga_reset;
  //! FPGA port
  reg  [FPGA_ADDR_WIDTH   - 1 : 0] tb_fpga_addr;
  reg [FPGA_DATA_WIDTH   - 1 : 0] tb_rd_data;

  reg  [FPGA_ADDR_WIDTH   - 1 : 0] fpga_addr;
  reg                              fpga_wr_en;
  reg  [FPGA_DATA_WIDTH   - 1 : 0] fpga_wr_data;
  wire [FPGA_DATA_WIDTH   - 1 : 0] fpga_rd_data;
  reg                              fpga_rd_en;
  reg                              fpga_req;
  wire                             fpga_ack;

  //! SDRAM ports
  wire [SDRAM_ADDR_WIDTH  - 1 : 0] ram_addr;
  wire [SDRAM_BANK_WIDTH  - 1 : 0] ram_bank_addr;
  wire [SDRAM_BYTES_WIDTH - 1 : 0] ram_dqm;
  wire [SDRAM_DATA_WIDTH  - 1 : 0] ram_data;

  wire                             ram_ras_n;
  wire                             ram_cas_n;
  wire                             ram_clk;
  wire                             ram_clk_en;
  wire                             ram_cs_n;
  wire                             ram_we_n;

  reg    [SDRAM_COL_WIDTH - 1 : 0] addr_col;
  reg    [SDRAM_ROW_WIDTH - 1 : 0] addr_row;
  reg   [SDRAM_BANK_WIDTH - 1 : 0] addr_bank; 
  reg   [FPGA_DATA_WIDTH  - 1 : 0] ram_in_data;
  reg   [FPGA_DATA_WIDTH  - 1 : 0] ram_out_data;

sdram_controller 
  #(
  FPGA_ADDR_WIDTH   ,
  FPGA_DATA_WIDTH   ,
  CLK_FREQUENCY_SYS ,
  SDRAM_ADDR_WIDTH  ,
  SDRAM_DATA_WIDTH  ,
  SDRAM_COL_WIDTH   ,
  SDRAM_ROW_WIDTH   ,
  SDRAM_BANK_WIDTH  ,
  SDRAM_BYTES_WIDTH ,
  SDRAM_CLK_MAX     ,
  T_POWER_UP        ,
  T_RP              ,
  T_RC              ,
  T_MRD             ,
  T_RCD             ,
  T_REF             ,
  T_RAS             ,
  T_WR              ,
  WRITE_BURST_MODE  ,
  BURST_TYPE        ,
  BURST_LENGTH      ,
  CAS_LATENCY
)  

sdram_controller_inst
(
	.fpga_clk        (fpga_clk            ) ,	// input  clk
	.fpga_reset      (fpga_reset          ) ,	// input  reset
	.fpga_addr       (fpga_addr           ) ,	// input [FPGA_ADDR_WIDTH-1:0] addr
	.fpga_wr_en      (fpga_wr_en          ) ,	// input  wr_en
	.fpga_wr_data    (fpga_wr_data        ) ,	// input [FPGA_DATA_WIDTH-1:0] wr_data
	.fpga_rd_data    (fpga_rd_data        ) ,	// output [FPGA_DATA_WIDTH-1:0] rd_data
	.fpga_rd_en      (fpga_rd_en          ) ,	// input  rd_ready
	.fpga_req        (fpga_req            ) ,	// input  req
	.fpga_ack        (fpga_ack            ) ,	// output  ack
	.ram_addr        (ram_addr            ) ,	// output [SDRAM_ADDR_WIDTH-1:0] ram_addr
	.ram_bank_addr   (ram_bank_addr       ) ,	// output [SDRAM_BANK_WIDTH-1:0] ram_bank_addr
	.ram_dqm         (ram_dqm             ) ,	// output [SDRAM_BYTES_WIDTH-1:0] ram_dqm
	.ram_data        (ram_data            ) ,	// inout [SDRAM_DATA_WIDTH-1:0] ram_data
	.ram_ras_n       (ram_ras_n           ) ,	// output  ram_ras_n
	.ram_cas_n       (ram_cas_n           ) ,	// output  ram_cas_n
	.ram_clk         (ram_clk             ) ,	// output  ram_clk
	.ram_clk_en      (ram_clk_en          ) ,	// output  ram_clk_en
	.ram_cs_n        (ram_cs_n            ) ,	// output  ram_cs_n
	.ram_we_n        (ram_we_n            ) 	// output  ram_we_n
);

//-----------------------------------------------------//
// ------------- Clock, reset, tasks ------------------//
  
initial
  begin

    fpga_clk = 1'b0;
    forever # (clock_period / 2) fpga_clk = ~ fpga_clk;
  end 

task reset_task (); begin
    fpga_reset      = 1'b1;
    fpga_req        = 1'b0;
    fpga_wr_en      = 1'b0;
    fpga_rd_en      = 1'b0;
    fpga_addr <= {FPGA_ADDR_WIDTH{1'b0}};
    repeat (10)  @ (posedge fpga_clk);    
    fpga_reset = 1'b0;
  end
endtask


// ----------------- write_task -----------------------//

task write_task (); begin

  fpga_reset = 0;
  fpga_rd_en = 0;

  fpga_wr_en = 1;
  fpga_req = 1;
  //wait acknowledge
  while(~fpga_ack) @(posedge fpga_clk);

  //do idle
  fpga_wr_en = 0;
  fpga_req = 0;
end
endtask

// ----------------- read_task -----------------------//

task read_task (); begin

  fpga_reset = 0;
  fpga_rd_en = 0;

  fpga_rd_en = 1;
  fpga_req = 1;
  //wait acknowledge
  while(~fpga_ack) @(posedge fpga_clk);
  
  //do idle
  fpga_rd_en = 0;
  fpga_req = 0;
end
endtask


//-----------------------------------------------------//
// ----------------- Simulation -----------------------//

//! FPGA address and data register
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) begin
    fpga_addr <= {FPGA_ADDR_WIDTH{1'b0}};
  end
  else begin
    fpga_addr <= tb_fpga_addr;
  end
end 

initial
begin

  $dumpfile("test.vcd");
  $dumpvars(0, testbench);
  /*
  $monitor ("T = %4d, write = %h, read = %h, write_data = %h, read_data = %h, queue_FIFO [0] = %h, empty = %h, full = %h, almost_empty = %h, almost_full = %h, i_fifo.rd_ptr = %h, i_fifo.wr_ptr = %h", 
            $stime, write, read, write_data, read_data, queue_FIFO [0], empty, full, almost_empty, almost_full, i_fifo.rd_ptr, i_fifo.wr_ptr);
*/
//-----------------------------------------------------//
// ------------- Full amd empty test ------------------//

$display ("\n    ------------- Full amd empty test ----------------\n");
  reset_task ();

  # (clock_period * REFRESH_INTERVAL * 2);

  addr_col = 9'h001;
  addr_row = 12'hfff;
  addr_bank = 2'h0;
  ram_in_data = 32'hff01; 
 
  tb_fpga_addr = {addr_bank, addr_row, addr_col};
  fpga_wr_data = ram_in_data;

  write_task (/*addr_col, addr_row, addr_bank, 
    tb_fpga_addr, ram_in_data*/);

  # (clock_period * REFRESH_INTERVAL/4);
  addr_col = 9'h0f2;
  addr_row = 12'h5f2;
  addr_bank = 2'h1;
  ram_in_data = 32'hff02; 
 
  tb_fpga_addr = {addr_bank, addr_row, addr_col};
  fpga_wr_data = ram_in_data;

  write_task (/*addr_col, addr_row, addr_bank, 
    tb_fpga_addr, ram_in_data*/);

  # (clock_period * REFRESH_INTERVAL/4);

  addr_col = 9'h001;
  addr_row = 12'hfff;
  addr_bank = 2'h0;
 
  tb_fpga_addr = {addr_bank, addr_row, addr_col};
 
  read_task ();

  # (clock_period * REFRESH_INTERVAL/4);
  addr_col = 9'h0f2;
  addr_row = 12'h5f2;
  addr_bank = 2'h1;
  ram_in_data = 32'hff02; 
 
  tb_fpga_addr = {addr_bank, addr_row, addr_col};
 
  read_task ();

  # (clock_period * 30);

  $finish;
end

endmodule