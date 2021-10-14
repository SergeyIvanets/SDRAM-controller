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
                T_POWER_UP = 200_000_000,
                //! Command period (PRE to ACT) delay, ns min
                T_RP       = 18,
                //! Command period (REF to REF/ACT to ACT) delay, ns min
                T_RC       = 60,    
                //! Mode register Program Time delay, ns min
                T_MRD      = 18,
                //! Active to Read/Write delay, ns min
                T_RCD      = 18,
                //! Refresh cycle time 64ms max, value in ns 
                T_REF      = 63_999_000,
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
                CAS_LATENCY = 3'b010,

                //! waiting time before write
                WAIT_BEFORE = 3'h7
)

   (
    //! 
    input                                   clk74_25M,
    input                                   reset_n,
	  output                         [11 : 2] user_io,
    input                           [1 : 0] key_io,
     

    //! SDRAM ports
    output      [SDRAM_ADDR_WIDTH  - 1 : 0] ram_addr,
    output      [SDRAM_BANK_WIDTH  - 1 : 0] ram_bank_addr,
    output      [SDRAM_BYTES_WIDTH - 1 : 0] ram_dqm,
    inout       [SDRAM_DATA_WIDTH  - 1 : 0] ram_data,

    output                                  ram_ras_n,
    output                                  ram_cas_n,
    output                                  ram_clk,
    output                                  ram_clk_en,
    output                                  ram_cs_n,
    output                                  ram_we_n
   );

  localparam               CLK_PERIOD = 1.0/SDRAM_CLK_MAX * 1_000.0;
  localparam integer REFRESH_INTERVAL = T_REF/CLK_PERIOD;
  // the number of clock cycles to wait while a REFRESH command is being
  // executed
  // 10 
  localparam integer     REFRESH_WAIT = T_RC/CLK_PERIOD;
  // Interval between write or read operation
  // Refresh + T_RCD + CAS lat (2) + 5 
  localparam                TEST_WAIT = REFRESH_WAIT + T_RCD + 2 + 5;

  //! FPGA port
  wire [FPGA_ADDR_WIDTH   - 1 : 0] fpga_addr;
  reg                              fpga_wr_en;
  reg  [FPGA_DATA_WIDTH   - 1 : 0] fpga_wr_data;
  wire [FPGA_DATA_WIDTH   - 1 : 0] fpga_rd_data;
  reg                              fpga_rd_en;
  reg                              fpga_req;
  wire                             fpga_ack;
  wire                             fpga_clk;
  wire                             fpga_reset;  

  reg    [SDRAM_COL_WIDTH - 1 : 0] addr_col;
  reg    [SDRAM_ROW_WIDTH - 1 : 0] addr_row;
  reg   [SDRAM_BANK_WIDTH - 1 : 0] addr_bank; 
  reg   [FPGA_DATA_WIDTH  - 1 : 0] ram_in_data;
  reg   [FPGA_DATA_WIDTH  - 1 : 0] ram_out_data;
  reg  [FPGA_DATA_WIDTH   - 1 : 0] control_rd_data;
  reg                     [14 : 0] wait_counter;  
  wire                    [14 : 0] wait_power;  
  reg                     [10 : 0] io_fpga_counter;  
  reg                     [10 : 0] io_hdmi_counter;  
  reg                      [1 : 0] reg_key_io;                   


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

assign fpga_reset = ~reset_n;

// State machine for control
  localparam           INIT1             = 4'b0000;
  localparam           WR0               = 4'b0001;
  localparam           WR0_NOP           = 4'b0011;
  localparam           WR1               = 4'b0010;
  localparam           WR1_NOP           = 4'b0111;
  localparam           RD0               = 4'b0100;
  localparam           RD0_NOP           = 4'b1001;
  localparam           RD1               = 4'b1000;
  localparam           RD1_NOP           = 4'b1011;
  localparam           FINISH            = 4'b1111;

  reg [3 : 0] state, next_state;
  reg [1 : 0] comp;   

//! Next state logic
always @ * begin: fsm_next_state
  next_state <= state;

  case (state)
  INIT1:
   if (key_io [0] == 1'b0) 
        next_state <= WR0;
  WR0:
    next_state <= WR0_NOP;
  WR0_NOP:
   if (wait_counter == (T_RCD - 4))
    if (fpga_ack == 1'b0) next_state <= WR1;
  WR1:
    next_state <= WR1_NOP;
  WR1_NOP:
   if (wait_counter == (T_RCD - 4))
    if (fpga_ack == 1'b0) next_state <= RD0;
  RD0:
    next_state <= RD0_NOP;
  RD0_NOP:
   if (wait_counter == (T_RCD - 4))
    if (fpga_ack == 1'b0) next_state <= RD1;
  RD1:
    next_state <= RD1_NOP;
  RD1_NOP:
   if (wait_counter == (T_RCD - 4))
    if (fpga_ack == 1'b0) next_state <= FINISH;
  FINISH:
      next_state <= FINISH;
  endcase
end

//! Next state register
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) 
    state <= INIT1;
  else 
    state <= next_state;
end 

//! The wait counter is used to hold the current state 
//! for a number of clock cycles
always @ (posedge fpga_clk, posedge fpga_reset) begin: wait_counter_always
  if (fpga_reset) 
    wait_counter <= #1 0;
  // state changing
  else if (state != next_state) 
    wait_counter <= #1 0;
  else
    wait_counter <= #1 wait_counter + 1;
end

//assign wait_power = (wait_counter == ({5{1'b1}})) ? 1'b1 : 1'b0;

//! Set memory controller signal
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) begin
    addr_col      <= 9'h000;
    addr_row      <= 12'h000;
    addr_bank     <= 2'h0;
    fpga_wr_data  <= 32'h0000_0000; 
    fpga_wr_en    <= 1'b0;
    fpga_rd_en    <= 1'b0;
    fpga_req      <= 1'b0;
  end
  else
  case (state)
    INIT1   : begin
      addr_col      <= 9'h000;
      addr_row      <= 12'h000;
      addr_bank     <= 2'h0;
      fpga_wr_data  <= 32'h0000_0000; 
      fpga_wr_en    <= 1'b0;
      fpga_rd_en    <= 1'b0;
      fpga_req      <= 1'b0;
    end
    WR0    : begin
      addr_col      <= 9'h001;
      addr_row      <= 12'hfff;
      addr_bank     <= 2'h0;
      fpga_wr_data  <= 32'h0000_ff01; 
      fpga_wr_en    <= 1'b1;
      fpga_rd_en    <= 1'b0;
      fpga_req      <= 1'b1;
    end
    WR1    : begin
      addr_col      <= 9'h0f2;
      addr_row      <= 12'h5f2;
      addr_bank     <= 2'h1;
      fpga_wr_data  <= 32'hf0f0_ff02; 
      fpga_wr_en    <= 1'b1;
      fpga_rd_en    <= 1'b0;
      fpga_req      <= 1'b1;
    end
    RD0    : begin
      addr_col      <= 9'h001;
      addr_row      <= 12'hfff;
      addr_bank     <= 2'h0;
      fpga_wr_data  <= 32'h0000_0000; 
      fpga_wr_en    <= 1'b0;
      fpga_rd_en    <= 1'b1;
      fpga_req      <= 1'b1;
    end
    RD1    : begin
      addr_col      <= 9'h0f2;
      addr_row      <= 12'h5f2;
      addr_bank     <= 2'h1;
      fpga_wr_data  <= 32'h0000_0000; 
      fpga_wr_en    <= 1'b0;
      fpga_rd_en    <= 1'b1;
      fpga_req      <= 1'b1;
    end
    default : begin
      addr_col      <= 9'h000;
      addr_row      <= 12'h000;
      addr_bank     <= 2'h0;
      fpga_wr_data  <= 32'h0000_0000; 
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
/*
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) 
    comp <= 0;
  else
    case (state)
      INIT1:
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
*/
/*
always @ (posedge fpga_clk, posedge fpga_reset) begin
  if (fpga_reset) 
    io_fpga_counter <= 0;
  else
    io_fpga_counter <= io_fpga_counter + 1;
end

always @ (posedge clk74_25M, posedge fpga_reset) begin
  if (fpga_reset) 
    io_hdmi_counter <= 0;
  else
    io_hdmi_counter <= io_hdmi_counter + 1;
end
*/
always @ (posedge fpga_clk) begin
  if (fpga_reset) 
    reg_key_io <= key_io;
end



assign user_io [10] = reg_key_io [0];

endmodule