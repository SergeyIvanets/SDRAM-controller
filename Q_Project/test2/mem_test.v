module mem_test
#(
  parameter  
                //! FPGA systen interface
                //! addr = {ram_bank_addr, row, col}
                FPGA_ADDR_WIDTH   = 23, //! System address width
                FPGA_DATA_WIDTH   = 32,   //! System data width
                CLK_FREQUENCY_SYS = 166,  //! MHz System clock

                //! SDRAM interface
                SDRAM_ADDR_WIDTH  = 12,   //! SDRAM address width
                SDRAM_DATA_WIDTH  = 32,   //! SDRAM data width
                SDRAM_COL_WIDTH   = 9,    //! SDRAM colomn width
                SDRAM_ROW_WIDTH   = 12,   //! SDRAM row width
                SDRAM_BANK_WIDTH  = 2,    //! SDRAM bank width
                SDRAM_BYTES_WIDTH = 4,
                SDRAM_CLK_MAX     = 166,  //! MHz From SDRAM datasheet
                //! Timig prameters
                //! Power-up delay 100 us min. Set 200 ms
                T_POWER_UP = 200_000,
                //! Command period (PRE to ACT) delay, ns min
                T_RP       = 18,
                //! Command period (REF to REF/ACT to ACT) delay, ns min
                T_RC       = 60,    
                //! Mode register Program Time delay, ns min
                T_MRD      = 18,
                //! Active to Read/Write delay, ns min
                T_RCD      = 18,
                //! Refresh cycle time, ns max
                T_REF      = 64_000_000,
                //! RAS time, ns min
                T_RAS       = 42,
                //! The number of 16-bit words to be bursted during a read/write
                // parameter BURST_LENGTH = 2;
                //! Write recovery Time or InputData to Precharge Command Delay Time
                T_WR        = 12,
                //! Mode Register Definition
                //! M9: Write Burst Mode: 0 - programmed Burst Lengtn, 1 - Single Location
                WRITE_BURST_MODE = 1'b1,
                //! M3:  Burst Type: 0 - Sequention, 1 - Interleaved
                BURST_TYPE       = 1'b0,
                //! M2: - M0  Burst Length in  000 - 1; 001 - 2; 010 - 4; 011 - 8
                BURST_LENGTH     = 3'b000,
                //!	Controller Parameter
                //! 166 MHz
                //! CAS Latence 2 cycle
                CAS_LATENCY = 3'b010
)

   (
     //! 
     input                                  clk74_25M,
     input                                  fpga_reset,
	  output                         [1 : 0] user_io,

     //! SDRAM ports
     output     [SDRAM_ADDR_WIDTH  - 1 : 0] ram_addr,
     output     [SDRAM_BANK_WIDTH  - 1 : 0] ram_bank_addr,
     output     [SDRAM_BYTES_WIDTH - 1 : 0] ram_dqm,
     inout      [SDRAM_DATA_WIDTH  - 1 : 0] ram_data,

     output                                 ram_ras_n,
     output                                 ram_cas_n,
     output                                 ram_clk,
     output                                 ram_clk_en,
     output                                 ram_cs_n,
     output                                 ram_we_n
   );

  localparam               CLK_PERIOD = 1.0/SDRAM_CLK_MAX * 1_000.0;
  localparam integer REFRESH_INTERVAL = T_REF/CLK_PERIOD - 10;

  //! FPGA port
  wire [FPGA_ADDR_WIDTH   - 1 : 0] fpga_addr;
  reg                              fpga_wr_en;
  reg  [FPGA_DATA_WIDTH   - 1 : 0] fpga_wr_data;
  wire [FPGA_DATA_WIDTH   - 1 : 0] fpga_rd_data;
  reg                              fpga_rd_en;
  reg                              fpga_req;
  wire                             fpga_ack;
  wire                             fpga_clk;

  reg    [SDRAM_COL_WIDTH - 1 : 0] addr_col;
  reg    [SDRAM_ROW_WIDTH - 1 : 0] addr_row;
  reg   [SDRAM_BANK_WIDTH - 1 : 0] addr_bank; 
  reg   [FPGA_DATA_WIDTH  - 1 : 0] ram_in_data;
  reg   [FPGA_DATA_WIDTH  - 1 : 0] ram_out_data;
  reg  [FPGA_DATA_WIDTH   - 1 : 0] control_rd_data;
  reg                     [14 : 0] wait_counter;  
  wire                    [14 : 0] wait_power;  

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

pll166	
pll166_inst (
	.areset          ( fpga_reset         ), // system reset 
	.inclk0          ( clk74_25M          ), // input clock 74.25 MHz
	.c0              ( fpga_clk           )  // output clock 166 MHz
	);

// State machine for control
  localparam           INIT              = 4'b0000;
  localparam           WR0               = 4'b0001;
  localparam           WR1               = 4'b0010;
  localparam           RD0               = 4'b0100;
  localparam           RD1               = 4'b1000;
  localparam           FINISH            = 4'b1111;

  reg [3 : 0] state, next_state;
  reg [1 : 0] comp;   

//! Next state logic
always @ * begin: fsm_next_state
  next_state <= state;

  case (state)
  INIT:
   if (wait_power == 0) 
        next_state <= INIT;
      else if (wait_counter == INIT) 
        next_state <= WR0;
  WR0:
      next_state <= WR1;
  WR1:
      next_state <= RD0;
  RD0:
      next_state <= RD1;
  RD1:
      next_state <= FINISH;
  FINISH:
      next_state <= FINISH;
  endcase
end

//! Next state register
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) 
    state <= INIT;
  else 
    state <= next_state;
end 

//! The wait counter is used to make power-up delay
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) 
    wait_counter <= 0;
  else
    wait_counter <= wait_counter + 1;
end

assign wait_power = (wait_counter == ({15{1'b1}})) ? 1'b1 : 1'b0;

//! Set memory controller signal
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) begin
    addr_col      <= 9'h000;
    addr_row      <= 12'h000;
    addr_bank     <= 2'h0;
    fpga_wr_data  <= 32'h0000; 
    fpga_wr_en    <= 1'b0;
    fpga_rd_en    <= 1'b0;
    fpga_req      <= 1'b0;
  end
  else
  case (state)
    INIT   : begin
      addr_col      <= 9'h000;
      addr_row      <= 12'h000;
      addr_bank     <= 2'h0;
      fpga_wr_data  <= 32'h0000; 
      fpga_wr_en    <= 1'b0;
      fpga_rd_en    <= 1'b0;
      fpga_req      <= 1'b0;
    end
    WR0    : begin
      addr_col      <= 9'h001;
      addr_row      <= 12'hfff;
      addr_bank     <= 2'h0;
      fpga_wr_data  <= 32'hff01; 
      fpga_wr_en    <= 1'b1;
      fpga_rd_en    <= 1'b0;
      fpga_req      <= 1'b1;
    end
    WR1    : begin
      addr_col      <= 9'h0f2;
      addr_row      <= 12'h5f2;
      addr_bank     <= 2'h1;
      fpga_wr_data  <= 32'hff02; 
      fpga_wr_en    <= 1'b1;
      fpga_rd_en    <= 1'b0;
      fpga_req      <= 1'b1;
    end
    RD0    : begin
      addr_col      <= 9'h001;
      addr_row      <= 12'hfff;
      addr_bank     <= 2'h0;
      fpga_wr_data  <= 32'h0000; 
      fpga_wr_en    <= 1'b0;
      fpga_rd_en    <= 1'b1;
      fpga_req      <= 1'b1;
    end
    RD1    : begin
      addr_col      <= 9'h0f2;
      addr_row      <= 12'h5f2;
      addr_bank     <= 2'h1;
      fpga_wr_data  <= 32'h0000; 
      fpga_wr_en    <= 1'b0;
      fpga_rd_en    <= 1'b1;
      fpga_req      <= 1'b1;
    end
    FINISH : begin
      addr_col      <= 9'h000;
      addr_row      <= 12'h000;
      addr_bank     <= 2'h0;
      fpga_wr_data  <= 32'h0000; 
      fpga_wr_en    <= 1'b0;
      fpga_rd_en    <= 1'b0;
      fpga_req      <= 1'b0;
    end

  endcase
end
 
  assign fpga_addr = {addr_bank, addr_row, addr_col};

// read data reg
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) 
    control_rd_data <= 0;
  else
    control_rd_data <= fpga_rd_data;
end

always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) 
    comp <= 0;
  else
    case (state)
      INIT:
        comp <= 0;
      
      WR0:
        comp <= 0;

      WR1:
        comp <= 0;

      RD0: begin
        if (control_rd_data == 32'hff01)
          comp [0] <= 1;
        else
          comp [0] <= 0;
      
        comp [1] <= comp [1];
      end

      RD1: begin
        if (control_rd_data == 32'hff02)
          comp [1] <= 1;
        else
          comp [1] <= 0;
      
        comp [0] <= comp [0];
      end
    endcase
end

assign user_io = comp;
endmodule