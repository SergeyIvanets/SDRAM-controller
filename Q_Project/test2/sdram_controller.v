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


//!{fpga_addr = [
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
/*
//! 100 MHz	
//! CAS Latence 3 cycle
parameter CAS_LATENCY = 3'b011
*/
   )
  
   (
     //! 
     input                                  fpga_clk,
     input                                  fpga_reset,

     //! FPGA port
     input      [FPGA_ADDR_WIDTH   - 1 : 0] fpga_addr,
     input                                  fpga_wr_en,
     input      [FPGA_DATA_WIDTH   - 1 : 0] fpga_wr_data,
     output     [FPGA_DATA_WIDTH   - 1 : 0] fpga_rd_data,
     input                                  fpga_rd_en,
     input                                  fpga_req,
     output                                 fpga_ack,

     //! SDRAM ports
     output reg [SDRAM_ADDR_WIDTH  - 1 : 0] ram_addr,
     output reg [SDRAM_BANK_WIDTH  - 1 : 0] ram_bank_addr,
     output     [SDRAM_BYTES_WIDTH - 1 : 0] ram_dqm,
     inout      [SDRAM_DATA_WIDTH  - 1 : 0] ram_data,

     output                                 ram_ras_n,
     output                                 ram_cas_n,
     output                                 ram_clk,
     output                                 ram_clk_en,
     output                                 ram_cs_n,
     output                                 ram_we_n
   );

//! ------------------ Local Parameters --------------------

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
  // 12 = max burst (12) + cas delay + 2 cycles
  localparam integer REFRESH_INTERVAL =T_REF/CLK_PERIOD - 12;

//! SDRAM command code acording to datasheet
//! | Command | Code | ram_cs_n | ras_n | cas_n | ram_we_n |  											
//! | ---------------------- | ------------ | ---------- | ------- | ------- | ---------- | 											
//! | Device deselect  | DESLECT | 1 | X | X | X | 											
//! | No operation  | NOP | 0 | 1 | 1 | 1 |
//! | Burst stop  | BST | 0 | 1 | 1 | 0 |
//! | Read | READ | 0 | 1 | 0 | 1 |
//! | Read with auto precharge | READ_PRE | 0 | 1 | 0 | 1 |
//! | Write | WRITE | 0 | 1 | 0 | 0 |
//! | Write with auto precharge | WRITE_PRE | 0 | 1 | 0 | 0 |
//! | Bank activate  | ACTIVE | 0 | 0 | 1 | 1 |
//! | Precharge select bank  | PRECHARGE | 0 | 0 | 1 | 0 | 
//! | Precharge all banks  | PRECHARGE_ALL | 0 | 0 | 1 | 0 | 
//! | CBR Auto-Refresh  | REFRESH | 0 | 0 | 0 | 1 |
//! | Self-Refresh  | SELF_REFRESH | 0 | 0 | 0 | 1 |
//! | Mode register set  | LOAD_MODE | 0 | 0 | 0 | 0 |

  localparam           CMD_DESLECT       = 4'b1111;
  localparam           CMD_NOP           = 4'b0111;
  localparam           CMD_BST           = 4'b0110;
  localparam           CMD_READ_PRE      = 4'b0101;
  localparam           CMD_WRITE_PRE     = 4'b0100;
  localparam           CMD_ACTIVE        = 4'b0011;
  localparam           CMD_PRECHARGE     = 4'b0010;
  localparam           CMD_REFRESH       = 4'b0001;
  localparam           CMD_LOAD_MODE     = 4'b0000;

  //! SDRAM control FSM States
  localparam           INIT              = 8'b0000_0000;
  localparam           MODE              = 8'b0000_0001;
  localparam           IDLE              = 8'b0000_0010;
  localparam           REFRESH           = 8'b0000_0100;
  localparam           ACTIVATE          = 8'b0000_1000;
  localparam           NOP               = 8'b0001_0000;
  localparam           READ_A            = 8'b0010_0000;
  localparam           WRITE_A           = 8'b0100_0000;
  localparam           PRECHARGE         = 8'b1000_0000;

  reg                                                     [7:0] state, next_state;
  reg                                                     [3:0] cmd, next_cmd;

  //! control signals
  wire                                                           start;
  wire                                                           load_mode_done;
  wire                                                           active_done;
  wire                                                           refresh_done;
  wire                                                           read_done;
  wire                                                           write_done;
  wire                                                           should_refresh;
  reg                                                            write_state_flag;
  reg                                                            oe;                                

  //! Wait counter [$clog2 (INIT_WAIT) - 1 : 0]
  reg                                                   [24 : 0] wait_counter;  
  //! Number of row per refresh time [$clog2 (REFRESH_INTERVAL) - 1 : 0]
  reg                                                   [23 : 0] refresh_counter;

  //! Registers
  reg [SDRAM_COL_WIDTH+SDRAM_ROW_WIDTH+SDRAM_BANK_WIDTH - 1 : 0] addr_reg;
  reg                                  [FPGA_DATA_WIDTH - 1 : 0] data_reg;
  reg                                                            we_reg;
  reg                                                            rd_reg;
  reg                                   [FPGA_DATA_WIDTH - 1: 0] q_reg;

  //! Wires for column, row and banks
  wire                                 [SDRAM_COL_WIDTH - 1 : 0] col;
  wire                                 [SDRAM_ROW_WIDTH - 1 : 0] row;
  wire                                [SDRAM_BANK_WIDTH - 1 : 0] bank;
  
  //! Split FPGA address {bank, row, column}
  assign bank  = addr_reg [FPGA_ADDR_WIDTH - 1 : 
                          FPGA_ADDR_WIDTH - SDRAM_BANK_WIDTH];
  assign row   = addr_reg [SDRAM_COL_WIDTH + SDRAM_ROW_WIDTH - 1 : 
                          SDRAM_COL_WIDTH];
  assign col  = addr_reg [SDRAM_COL_WIDTH - 1 : 0];

  assign ram_clk = fpga_clk;

//! Next state logic
always @ * begin: fsm_next_state
  next_state <= state;
  write_state_flag <= 1'b0;
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
      if (load_mode_done) begin
        next_state <= IDLE;
        next_cmd <= CMD_NOP;
      end

    // Wait for a read/write request
    IDLE:
      if (should_refresh) begin
        next_state <= REFRESH;
        next_cmd   <= CMD_REFRESH;
      end
      else if (fpga_req) begin
        next_state <= ACTIVATE;
        next_cmd   <= CMD_ACTIVE;
      end  
        else begin
          next_state <= IDLE;
          next_cmd <= CMD_NOP;
        end

    // Activate the row
    ACTIVATE:
      if (active_done)
        if (we_reg) begin
          next_state <= WRITE_A;
          next_cmd   <= CMD_WRITE_PRE;
        end
        else if (rd_reg) begin
          next_state <= READ_A;
          next_cmd   <= CMD_READ_PRE;
        end
          else begin
            next_state <= IDLE;
            next_cmd <= CMD_NOP;
          end
          

    // Execute a read and autoprecharge command
    READ_A:
      if (read_done)
        if (should_refresh) begin
          next_state <= REFRESH;
          next_cmd   <= CMD_REFRESH;
        end
        else if (fpga_req) begin
          next_state <= ACTIVATE;
          next_cmd   <= CMD_ACTIVE;
        end
          else begin
            next_state <= IDLE;
            next_cmd <= CMD_NOP;
          end

    // Execute a write and autoprecharge command
    WRITE_A: begin
    write_state_flag <= 1'b1;
      if (write_done)
        if (should_refresh) begin
          next_state <= REFRESH;
          next_cmd   <= CMD_REFRESH;
        end
        else if (fpga_req) begin
          next_state <= ACTIVATE;
          next_cmd   <= CMD_ACTIVE;
        end
          else begin
            next_state <= IDLE;
            next_cmd <= CMD_NOP;
          end
    end

    // Execute an auto refresh
    REFRESH:
      if (refresh_done)
        if (fpga_req) begin
          next_state <= ACTIVATE;
          next_cmd   <= CMD_ACTIVE;
        end
          else begin
            next_state <= IDLE;
            next_cmd <= CMD_NOP;
          end
    
    default: begin
      next_state <= IDLE;
      next_cmd <= CMD_NOP;
	end
  endcase
end

//! Next state register
always @ (posedge fpga_clk, posedge fpga_reset) begin: fsm_next_state_register
  if (fpga_reset) begin
    state <= #1 INIT;
    cmd   <= #1 CMD_NOP;
  end
  else begin
    state <= #1 next_state;
    cmd   <= #1 next_cmd;
  end
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

//! The refresh counter is used to periodically trigger a refresh operation
always @ (posedge fpga_clk, posedge fpga_reset) begin: update_refresh_counter 
  if (fpga_reset) 
    refresh_counter <= #1 0;
  else if ((state == REFRESH) & (wait_counter == 0))
      refresh_counter <= #1 0;
  else
    refresh_counter <= #1 refresh_counter + 1;
end 
// the SDRAM should be refreshed when the refresh interval has elapsed
assign should_refresh  = (refresh_counter >= REFRESH_INTERVAL-1) ? 1'b1 : 1'b0;

//! Register for FPGA side signals
always @ (posedge fpga_clk) begin: fpga_side_reg
  if (start) begin
    addr_reg <= #1 fpga_addr;
    data_reg <= #1 fpga_wr_data;
    we_reg   <= #1 fpga_wr_en;
    rd_reg   <= #1 fpga_rd_en;
  end
end

// set wait signals
assign load_mode_done = (wait_counter == (LOAD_MODE_WAIT-1)) ? 1'b1 : 1'b0;
assign active_done    = (wait_counter == (ACTIVE_WAIT-1   )) ? 1'b1 : 1'b0;
assign refresh_done   = (wait_counter == (REFRESH_WAIT-1  )) ? 1'b1 : 1'b0;
assign read_done      = (wait_counter == (READ_WAIT-1     )) ? 1'b1 : 1'b0;
assign write_done     = (wait_counter == (WRITE_WAIT-1    )) ? 1'b1 : 1'b0;

// A new request is only allowed at the 
// IDLE state and end of the 
// READ_A, WRITE_A and REFRESH states
assign start = ((state == IDLE) |
        ((state == READ_A) & (read_done == 1'b1)) |
        ((state == WRITE_A) & (write_done == 1'b1)) |
        ((state == REFRESH) & (refresh_done == 1'b1)) )
        ? 1'b1 : 1'b0;

// assert the acknowledge signal at the beginning of the ACTIVE state
  assign fpga_ack = (((state == READ_A)|(state == WRITE_A)) & (wait_counter == 0)) ? 1'b1 : 1'b0;

// deassert the clock enable at the beginning of the INIT state
assign ram_clk_en = ((state == INIT) & (wait_counter == 0)) ? 1'b0 : 1'b1;

//! Set SDRAM control signals
//! ram_cs_n, ras_n, cas_n, ram_we_n from SDRAM command
assign {ram_cs_n, ram_ras_n, ram_cas_n, ram_we_n} = cmd;

//! Set SDRAM bank
always @ (state)
  case (state)
    ACTIVATE : ram_bank_addr = bank;
    READ_A   : ram_bank_addr = bank;
    WRITE_A  : ram_bank_addr = bank;
    default  : ram_bank_addr = {SDRAM_BANK_WIDTH{1'b0}};
  endcase

// set SDRAM address
always @ (state)
  case (state)
    INIT     : ram_addr = 12'b0100_0000_0000;
    MODE     : ram_addr = MODE_REG;
    ACTIVATE : ram_addr = row;
    READ_A   : ram_addr = {3'b010, col};
    WRITE_A  : ram_addr = {3'b010, col};
    default  : ram_addr = {SDRAM_ADDR_WIDTH{1'b0}};
  endcase

  //  SDRAM data output register
  always @ (posedge fpga_clk) begin: sdram_rd_data_reg
    if (state == READ_A)
      q_reg <= #1 ram_data;
    else 
      q_reg <= #1 {SDRAM_DATA_WIDTH{1'b0}};
  end 

  // Assign output FPGA data port 
  assign fpga_rd_data = q_reg;

  always @ (posedge fpga_clk, posedge fpga_reset) begin: control_data_path 
    if (fpga_reset) 
      oe <= #1 1'b0;
    else if (write_state_flag)
      oe <= #1 1'b1;
    else
      oe <= #1 1'b0;
  end

  // set SDRAM data signals
  assign  ram_data = (oe) ? data_reg : {SDRAM_DATA_WIDTH{1'bz}};

  // set SDRAM data mask DQM
  assign ram_dqm = 0;


endmodule 