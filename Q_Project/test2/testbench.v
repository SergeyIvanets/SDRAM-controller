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
                T_REF             = 64,
                T_RAS             = 42,
                T_WR              = 12,
                WRITE_BURST_MODE  = 1'b1,
                BURST_TYPE        = 1'b0,
                BURST_LENGTH      = 3'b011,
                CAS_LATENCY       = 3'b010,

                // simulation options
                clock_period      = 10;

  reg                              clk;
  reg                              reset;
  //! FPGA port
  reg  [FPGA_ADDR_WIDTH   - 1 : 0] addr;
  reg                              wr_en;
  reg  [FPGA_DATA_WIDTH   - 1 : 0] wr_data;
  wire [FPGA_DATA_WIDTH   - 1 : 0] rd_data;
  reg                              rd_ready;
  reg                              req;
  wire                             ack;

  //! SDRAM ports
  wire [SDRAM_ADDR_WIDTH  - 1 : 0] ram_addr;
  wire [SDRAM_BANK_WIDTH  - 1 : 0] ram_bank_addr;
  wire [SDRAM_BYTES_WIDTH - 1 : 0] ram_dqm;
  wire [SDRAM_DATA_WIDTH  - 1 : 0] ram_data;


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
	.clk            (clk            ) ,	// input  clk
	.reset          (reset          ) ,	// input  reset
	.addr           (addr           ) ,	// input [FPGA_ADDR_WIDTH-1:0] addr
	.wr_en          (wr_en          ) ,	// input  wr_en
	.wr_data        (wr_data        ) ,	// input [FPGA_DATA_WIDTH-1:0] wr_data
	.rd_data        (rd_data        ) ,	// output [FPGA_DATA_WIDTH-1:0] rd_data
	.rd_ready       (rd_ready       ) ,	// input  rd_ready
	.req            (req            ) ,	// input  req
	.ack            (ack            ) ,	// output  ack
	.ram_addr       (ram_addr       ) ,	// output [SDRAM_ADDR_WIDTH-1:0] ram_addr
	.ram_bank_addr  (ram_bank_addr  ) ,	// output [SDRAM_BANK_WIDTH-1:0] ram_bank_addr
	.ram_dqm        (ram_dqm        ) ,	// output [SDRAM_BYTES_WIDTH-1:0] ram_dqm
	.ram_data       (ram_data       ) ,	// inout [SDRAM_DATA_WIDTH-1:0] ram_data
	.ram_ras_n      (ram_ras_n      ) ,	// output  ram_ras_n
	.ram_cas_n      (ram_cas_n      ) ,	// output  ram_cas_n
	.ram_clk        (ram_clk        ) ,	// output  ram_clk
	.ram_clk_en     (ram_clk_en     ) ,	// output  ram_clk_en
	.ram_cs_n       (ram_cs_n       ) ,	// output  ram_cs_n
	.ram_we_n       (ram_we_n       ) 	// output  ram_we_n
);

//-----------------------------------------------------//
// ------------- Clock, reset, tasks ------------------//
  
initial
  begin
    clk = 1'b0;
    forever # (clock_period / 2) clk = ~ clk;
  end 

task reset_task ();
  begin
    reset      = 1'b1;
    req        = 1'b0;
    repeat (10)  @ (posedge clk);    
    reset = 1'b0;
  end
endtask


//-----------------------------------------------------//
// ----------------- Simulation -----------------------//
  
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
  # (clock_period * 1.5);
    
  
  # (clock_period * 10000);
  $finish;
end

endmodule