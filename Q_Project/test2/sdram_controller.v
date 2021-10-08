//! Module description

//! | Mode | ram_clk_en | ram_cs_n | ras_n | cas_n | ram_we_n | ram_addr[8:0] | ram_addr[9], ram_addr[11] | ram_addr[10] | ram_bank_addr[1:0] |
//! | ----------------------- | ------------ | ---------- | ------- | ------- | ---------- | --------------- | --------------------------- | -------------- | -------------------- |
//! | Activate | 1 | 0 | 0 | 1 | 1 | Row Address | Row Address | Row Address | Bank Address |
//! | Read Auto Precharge | 1 | 0 | 1 | 0 | 1 | Column | XX | 1 | Bank Address |
//! | Read No Precharge | 1 | 0 | 1 | 0 | 1 | Column | XX | 0 | Bank Address |
//! | Write Auto Precharge | 1 | 0 | 1 | 0 | 0 | Column | XX | 1 | Bank Address |
//! | Write No Precharge | 1 | 0 | 1 | 0 | 0 | Column | XX | 0 | Bank Address |
//! | Precharge All Banks | 1 | 0 | 0 | 1 | 0 | XX | XX | 1 | Bank Address |
//! | Precharge Bank Select | 1 | 0 | 0 | 1 | 0 | XX | XX | 0 | Bank Address |


//!{addr = [
//!    {name: 'bank',   bits: 2,},
//!    {name: 'row',    bits: 12,},
//!    {name: 'col',    bits: 9}
//! ]}
//`include "sdram_params.svh"

module sdram_controller

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
                SDRAM_BYTES_WIDTH = $clog2 (SDRAM_DATA_WIDTH / 8),
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
                //! Refresh cycle time, ms max
                T_REF      = 64,
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
                //! M2: - M0  Burst Length 000 - 1; 001 - 2; 010 - 4; 011 - 8
                BURST_LENGTH     = 3'b011,
                //!	Controller Parameter
                //! 166 MHz
                //! CAS Latence 2 cycle
                CAS_LATENCY = 3'b010


//! 100 MHz	
/*
//! CAS Latence 3 cycle
parameter CAS_LATENCY = 3'b011
*/
   )
  
   (
     //! 
     input                            clk,
     input                            reset,

     //! FPGA port
     input  [FPGA_ADDR_WIDTH   - 1 : 0] addr,
     input                              wr_en,
     input  [FPGA_DATA_WIDTH   - 1 : 0] wr_data,
     output [FPGA_DATA_WIDTH   - 1 : 0] rd_data,
     input                              rd_ready,
     input                              req,
     output                             ack,

     //! SDRAM ports
     output reg [SDRAM_ADDR_WIDTH  - 1 : 0] ram_addr,
     output reg [SDRAM_BANK_WIDTH  - 1 : 0] ram_bank_addr,
     output [SDRAM_BYTES_WIDTH - 1 : 0] ram_dqm,
     inout  [SDRAM_DATA_WIDTH  - 1 : 0] ram_data,

     output                             ram_ras_n,
     output                             ram_cas_n,
     output                             ram_clk,
     output                             ram_clk_en,
     output                             ram_cs_n,
     output                             ram_we_n
   );

//! ------------------ Local Parameters --------------------
//! SDRAM command code acording to datasheet

//! | Command | Code | ram_clk_en | ram_cs_n | ras_n | cas_n | ram_we_n | ram_bank_addr[1] | ram_bank_addr[0] | ram_addr[10] | 											
//! | ---------------------- | ------ | ------------ | ---------- | ------- | ------- | ---------- | ------------------ | ------------------ | -------------- | 											
//! | Device deselect  | DESLECT | X | 1 | X | X | X | X | X | X | 											
//! | No operation  | NOP | X | 0 | 1 | 1 | 1 | X | X | X | 											
//! | Burst stop  | BST | X | 0 | 1 | 1 | 0 | X | X | X | 											
//! | Read | READ | X | 0 | 1 | 0 | 1 | X | X | 0 | 											
//! | Read with auto precharge | READ_PRE | X | 0 | 1 | 0 | 1 | X | X | 1 | 											
//! | Write | WRITE | X | 0 | 1 | 0 | 0 | X | X | 0 | 											
//! | Write with auto precharge | WRITE_PRE | X | 0 | 1 | 0 | 0 | X | X | 1 | 											
//! | Bank activate  | ACTIVE | X | 0 | 0 | 1 | 1 | X | X | X | 											
//! | Precharge select bank  | PRECHARGE | X | 0 | 0 | 1 | 0 | X | X | 0 | 											
//! | Precharge all banks  | PRECHARGE_ALL | X | 0 | 0 | 1 | 0 | X | X | 1 | 											
//! | CBR Auto-Refresh  | REFRESH | 1 | 0 | 0 | 0 | 1 | X | X | X | 											
//! | Self-Refresh  | SELF_REFRESH | 0 | 0 | 0 | 0 | 1 | X | X | X | 											
//! | Mode register set  | LOAD_MODE | X | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 											

  localparam [7:0]
             CMD_DESLECT       = 8'b11111111,
             CMD_NOP           = 8'b10111XXX,
             CMD_BST           = 8'b10110XXX,
             CMD_READ          = 8'b10101XX0,
             CMD_READ_PRE      = 8'b10101XX1,
             CMD_WRITE         = 8'b10100XX0,
             CMD_WRITE_PRE     = 8'b10100XX1,
             CMD_ACTIVE        = 8'b10011XXX,
             CMD_PRECHARGE     = 8'b10010XX0,
             CMD_PRECHARGE_ALL = 8'b10010XX1,
             CMD_REFRESH       = 8'b10001XXX,
             CMD_SELF_REFRESH  = 8'b00001XXX,
             CMD_LOAD_MODE     = 8'b10000000;
  reg [7:0] cmd, next_cmd;

  //! The value written to the mode register to configure the memory
  localparam MODE_REG = {2'b00, WRITE_BURST_MODE, 2'b00, CAS_LATENCY, BURST_TYPE, BURST_LENGTH};

  //! Calculate the clock period (in nanoseconds)
  // 1/166*1000 = 6.02
  localparam CLK_PERIOD = 1.0/SDRAM_CLK_MAX * 1_000.0;

  //! The number of clock cycles from power-up to inicialaze SDRAM
  localparam integer INIT_WAIT = T_POWER_UP/CLK_PERIOD;

  // the number of clock cycles to wait while a LOAD MODE command is being
  // executed
  localparam integer LOAD_MODE_WAIT = T_MRD/CLK_PERIOD;

  // the number of clock cycles to wait while an ACTIVE command is being
  // executed
  localparam integer ACTIVE_WAIT = T_RCD/CLK_PERIOD;

  // the number of clock cycles to wait while a REFRESH command is being
  // executed
  // 10 
  localparam integer REFRESH_WAIT = T_RC/CLK_PERIOD;

  // the number of clock cycles to wait while a PRECHARGE command is being
  // executed
  //3
  localparam integer PRECHARGE_WAIT = T_RP/CLK_PERIOD;

  // the number of clock cycles to wait while a READ command is being executed
  localparam integer READ_WAIT = CAS_LATENCY+BURST_LENGTH;

  // the number of clock cycles to wait while a WRITE command is being executed
  localparam integer WRITE_WAIT = BURST_LENGTH + (T_WR+T_RP)/CLK_PERIOD;

  // the number of clock cycles before the memory controller needs to refresh
  // the SDRAM
  localparam integer REFRESH_INTERVAL =T_REF/CLK_PERIOD - 10;

  //! control signals
  wire start;
  wire load_mode_done;
  wire active_done;
  wire refresh_done;
  //wire first_word;
  wire read_done;
  wire write_done;
  wire should_refresh;

  //! Wait counter [$clog2 (INIT_WAIT) - 1 : 0]
  reg [14 : 0] wait_counter;  
  //! Number of row per refresh time [$clog2 (ROW_WIDTH) - 1 : 0]
  reg [4 - 1 : 0] refresh_counter;

  //! Registers
  reg [SDRAM_COL_WIDTH+SDRAM_ROW_WIDTH+SDRAM_BANK_WIDTH-1 : 0] addr_reg;
  reg [FPGA_DATA_WIDTH-1                                  : 0] data_reg;
  reg                                                          we_reg;
  reg [FPGA_DATA_WIDTH-1                                  : 0] q_reg;

  //! Wires for column, row and banks
  wire [SDRAM_COL_WIDTH  - 1 : 0] col;
  wire [SDRAM_ROW_WIDTH  - 1 : 0] row;
  wire [SDRAM_BANK_WIDTH - 1 : 0] bank;
  
  //! Split FPGA address
  assign col = addr_reg [FPGA_ADDR_WIDTH - 1 : 
                FPGA_ADDR_WIDTH - SDRAM_BANK_WIDTH];
  assign row = addr_reg [SDRAM_COL_WIDTH + SDRAM_ROW_WIDTH - 1 : 
                SDRAM_COL_WIDTH];
  assign bank = addr_reg [SDRAM_COL_WIDTH - 1 : 0];

  assign ram_clk = clk;

  //! SDRAM control FSM
  //! States ;
  localparam [3:0]
             INIT      = 4'h0,
             MODE      = 4'hf,
             IDLE      = 4'h3,
             REFRESH   = 4'h6,
             ACTIVATE  = 4'h2,
             NOP       = 4'h1,
//             READ      = 4'b0100,
             READ_A    = 4'h5,
//             WRITE     = 4'b1000,
             WRITE_A   = 4'h9,
             PRECHARGE = 4'hc;
  reg [3:0] state, next_state;

//! Next state logic
always @ * begin: fsm_next_state
  next_state <= state;

  // default to a NOP command
  next_cmd <= CMD_NOP;

  case (state)
    // Execute power-on sequence
    INIT:
   if (wait_counter == 0) begin
        next_cmd   <= CMD_DESLECT;
        next_state <= INIT;
      end
      else if (wait_counter == INIT_WAIT) begin
        next_cmd   <= CMD_PRECHARGE;
        next_state <= INIT;
      end
      else if (wait_counter == INIT_WAIT+PRECHARGE_WAIT) begin
        next_cmd   <= CMD_REFRESH;
        next_state <= INIT;
      end
      else if (wait_counter == INIT_WAIT+PRECHARGE_WAIT+REFRESH_WAIT) begin
        next_cmd   <= CMD_REFRESH;
        next_state <= INIT;
      end
      else if (wait_counter == INIT_WAIT+PRECHARGE_WAIT+REFRESH_WAIT+REFRESH_WAIT)
      begin
        next_state <= MODE;
        next_cmd   <= CMD_LOAD_MODE;
      end
 

    // Set control register
    MODE:
      if (load_mode_done)
        next_state <= IDLE;

    // Wait for a read/write request
    IDLE:
      if (should_refresh) begin
        next_state <= REFRESH;
        next_cmd   <= CMD_REFRESH;
      end
      else if (req) begin
        next_state <= ACTIVATE;
        next_cmd   <= CMD_ACTIVE;
      end

    // Activate the row
    ACTIVATE:
      if (active_done)
        if (we_reg) begin
          next_state <= WRITE_A;
          next_cmd   <= CMD_WRITE_PRE;
        end
        else begin
          next_state <= READ_A;
          next_cmd   <= CMD_READ_PRE;
        end

    // Execute a read and autoprecharge command
    READ_A:
      if (read_done)
        if (should_refresh) begin
          next_state <= REFRESH;
          next_cmd   <= CMD_REFRESH;
        end
        else if (req) begin
          next_state <= ACTIVATE;
          next_cmd   <= CMD_ACTIVE;
        end
        else
          next_state <= IDLE;

    // Execute a write and autoprecharge command
    WRITE_A:
      if (write_done)
        if (should_refresh) begin
          next_state <= REFRESH;
          next_cmd   <= CMD_REFRESH;
        end
        else if (req) begin
          next_state <= ACTIVATE;
          next_cmd   <= CMD_ACTIVE;
        end
        else
          next_state <= IDLE;

    // Execute an auto refresh
    REFRESH:
      if (refresh_done)
        if (req) begin
          next_state <= ACTIVATE;
          next_cmd   <= CMD_ACTIVE;
        end
        else
          next_state <= IDLE;
  endcase
end

//! Next state register
always @ (posedge clk, posedge reset) begin: fsm_next_state_register
  if (reset) begin
    state <= INIT;
    cmd   <= CMD_NOP;
  end
  else begin
    state <= next_state;
    cmd   <= next_cmd;
  end
end 

//! The wait counter is used to hold the current state 
//! for a number of clock cycles
always @ (posedge clk, posedge reset) begin: wait_counter_always
  if (reset) 
    wait_counter <= 0;
  // state changing
  else if (state != next_state) 
    wait_counter <= 0;
  else
    wait_counter <= wait_counter + 1;
end

//! The refresh counter is used to periodically trigger a refresh operation
always @ (posedge clk, posedge reset) begin: update_refresh_counter 
  if (reset) 
    refresh_counter <= 0;
  else if ((state == REFRESH) & (wait_counter == 0))
      refresh_counter <= 0;
  else
    refresh_counter <= refresh_counter + 1;
end 

//! Register for FPGA side signals
always @ (posedge clk) begin: fpga_side_reg
  if (start) begin
    addr_reg <= addr;
    data_reg <= wr_data;
    we_reg   <= wr_en;
  end
end

//  SDRAM data output register
always @ (posedge clk) begin: sdram_data_reg
   if (state == READ_A )
      q_reg <= ram_dqm;
end 

// Assign output FPGA data port 
assign rd_data = q_reg;

// set wait signals
assign load_mode_done = (wait_counter == (LOAD_MODE_WAIT-1)) ? 1'b1 : 1'b0;
assign active_done    = (wait_counter == (ACTIVE_WAIT-1   )) ? 1'b1 : 1'b0;
assign refresh_done   = (wait_counter == (REFRESH_WAIT-1  )) ? 1'b1 : 1'b0;
//assign first_word     = (wait_counter == CAS_LATENCY     ) ? 1'b1 : 1'b0;
assign read_done      = (wait_counter == (READ_WAIT-1     )) ? 1'b1 : 1'b0;
assign write_done     = (wait_counter == (WRITE_WAIT-1    )) ? 1'b1 : 1'b0;

// the SDRAM should be refreshed when the refresh interval has elapsed
assign should_refresh  = (refresh_counter >= REFRESH_INTERVAL-1) ? 1'b1 : 1'b0;

// a new request is only allowed at the end of the 
// IDLE, READ, WRITE, and REFRESH states

		  
assign start = ((state == IDLE) |
        ((state == READ_A) & (read_done == 1'b1)) |
        ((state == WRITE_A) & (write_done == 1'b1)) |
        ((state == REFRESH) & (refresh_done == 1'b1)) )
        ? 1'b1 : 1'b0;

// assert the acknowledge signal at the beginning of the ACTIVE state
assign ack = ((state == ACTIVATE) & (wait_counter == 0)) ? 1'b1 : 1'b0;

// deassert the clock enable at the beginning of the INIT state
assign ram_clk_en = ((state == INIT) & (wait_counter == 0)) ? 1'b0 : 1'b1;

//! Set SDRAM control signals
//! ram_cs_n, ras_n, cas_n, ram_we_n from SDRAM command
assign {ram_cs_n, ram_ras_n, ram_cas_n, ram_we_n} = cmd [6:3];

//! Set SDRAM bank
always @ (state)
  case (state)
    ACTIVATE: ram_bank_addr = bank;
    READ_A  : ram_bank_addr = bank;
    WRITE_A : ram_bank_addr = bank;
    default : ram_bank_addr = 0;
  endcase

// set SDRAM address
always @ (state)
  case (state)
    INIT     : ram_addr = "0100_0000_0000";
    MODE     : ram_addr = MODE_REG;
    ACTIVATE : ram_addr = row;
    READ_A   : ram_addr = {3'b010, col};
    WRITE_A  : ram_addr = {3'b010, col};
    default  : ram_addr = 12'h000;
  endcase
 
  // set SDRAM data signals
  assign  ram_data = (state == WRITE_A) ? data_reg : 32'hzzzz;

  // set SDRAM data mask DQM
  assign ram_dqm = 0;


endmodule 