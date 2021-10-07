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
`include "sdram_params.h"

module sdram_controller

  #(
     parameter  
                //! FPGA systen interface
                //! addr = {ram_bank_addr, row, col}
                FPGA_ADDR_WIDTH   = SDRAM_BANK_WIDTH +
                                    SDRAM_ROW_WIDTH +
                                    SDRAM_COL_WIDTH, //! System address width
                FPGA_DATA_WIDTH   = 32,   //! System data width
                CLK_FREQUENCY_SYS = 166,  //! MHz System clock

                //! SDRAM interface
                SDRAM_ADDR_WIDTH  = 12,   //! SDRAM address width
                SDRAM_DATA_WIDTH  = 32,   //! SDRAM data width
                SDRAM_COL_WIDTH   = 9,    //! SDRAM colomn width
                SDRAM_ROW_WIDTH   = 12,   //! SDRAM row width
                SDRAM_BANK_WIDTH  = 2,    //! SDRAM bank width
                SDRAM_BYTES_WIDTH = $clog2 (SDRAM_DATA_WIDTH / 8);
                SDRAM_CLK_MAX     = 166;  //! MHz From SDRAM datasheet
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

     //! SDRAM ports
     output [SDRAM_ADDR_WIDTH  - 1 : 0] ram_addr,
     output [SDRAM_BANK_WIDTH  - 1 : 0] ram_bank_addr,
     output [SDRAM_BYTES_WIDTH - 1 : 0] ram_dqm,
     inout  [SDRAM_DATA_WIDTH  - 1 : 0] ram_data,

     output                             ram_ras_n,
     output                             ram_cas_n,
     output                             ram_clk,
     output                             ram_clk_en,
     output                             ram_cs_n
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
  localparam MODE_REG : {2'b00, WRITE_BURST_MODE, 2'b00, CAS_LATENCY, BURST_TYPE, BURST_LENGTH};

  //! Calculate the clock period (in nanoseconds)
  localparam CLK_PERIOD = 1.0/SDRAM_CLK_MAX * 1_000.0;

  //! The number of clock cycles from power-up to inicialaze SDRAM
  localparam INIT_WAIT = T_POWER_UP/CLK_PERIOD;

  // the number of clock cycles to wait while a LOAD MODE command is being
  // executed
  localparam LOAD_MODE_WAIT = T_MRD/CLK_PERIOD;

  // the number of clock cycles to wait while an ACTIVE command is being
  // executed
  localparam ACTIVE_WAIT = T_RCD/CLK_PERIOD;

  // the number of clock cycles to wait while a REFRESH command is being
  // executed
  localparam REFRESH_WAIT = T_RC/CLK_PERIOD;

  // the number of clock cycles to wait while a PRECHARGE command is being
  // executed
  localparam PRECHARGE_WAIT = T_RP/CLK_PERIOD;

  // the number of clock cycles to wait while a READ command is being executed
  localparam READ_WAIT = CAS_LATENCY+BURST_LENGTH;

  // the number of clock cycles to wait while a WRITE command is being executed
  localparam WRITE_WAIT = BURST_LENGTH + (T_WR+T_RP)/CLK_PERIOD;

  // the number of clock cycles before the memory controller needs to refresh
  // the SDRAM
  localparam REFRESH_INTERVAL =T_REF/CLK_PERIOD-10;

  //! control signals
  wire start;
  wire load_mode_done;
  wire active_done;
  wire refresh_done;
  //wire first_word;
  wire read_done;
  wire write_done;
  wire should_refresh;

  //! Wait counter
  reg [$clog2 (INIT_WAIT) - 1 : 0] wait_counter;  
  //! Number of row per refresh time
  reg [$clog2 (ROW_WIDTH) - 1 : 0] refresh_counter;

  //! Registers
  reg [SDRAM_COL_WIDTH+SDRAM_ROW_WIDTH+SDRAM_BANK_WIDTH-1 : 0] addr_reg;
  reg [FPGA_DATA_WIDTH-1                                  : 0] data_reg;
  reg                                                          wr_en;
  reg [FPGA_DATA_WIDTH-1                                  : 0] q_reg;

  //! Wires for column, row and banks
  wire [SDRAM_COL_WIDTH  - 1 : 0] col;
  wire [SDRAM_ROW_WIDTH  - 1 : 0] row;
  wire [SDRAM_BANK_WIDTH - 1 : 0] bank;
  
  //! Split FPGA address
  assign col = addr [FPGA_ADDR_WIDTH - 1 : 
                FPGA_ADDR_WIDTH - SDRAM_BANK_WIDTH];
  assign row = addr [SDRAM_COL_WIDTH + SDRAM_ROW_WIDTH - 1 : 
                SDRAM_COL_WIDTH];
  assign bank = addr [SDRAM_COL_WIDTH - 1 : 0];

  //! SDRAM control FSM
  //! States ;
  localparam [3:0]
             INIT      = 4'b0000,
             MODE      = 4'b1111,
             IDLE      = 4'b0011,
             REFRESH   = 4'b0110,
             ACTIVATE  = 4'b0010,
             NOP       = 4'b0001,
//             READ      = 4'b0100,
             READ_A    = 4'b0101,
//             WRITE     = 4'b1000,
             WRITE_A   = 4'b1001,
             PRECHARGE = 4'b1100;
  reg [3:0] state, next_state;

//! Next state logic
fsm : always @ * begin: fsm_next_state
  next_state <= state;

  // default to a NOP command
  next_cmd <= CMD_NOP;

  case (state)
    // Execute power-on sequence
    INIT:
      if (wait_counter == 0)
        next_cmd   <= CMD_DESLECT;
      else if (wait_counter == INIT_WAIT-1)
        next_cmd   <= CMD_PRECHARGE;
      else if (wait_counter == INIT_WAIT+PRECHARGE_WAIT-1)
        next_cmd   <= CMD_REFRESH;
      else if (wait_counter == INIT_WAIT+PRECHARGE_WAIT+REFRESH_WAIT-1)
        next_cmd   <= CMD_REFRESH;
      else if (wait_counter == INIT_WAIT+PRECHARGE_WAIT+REFRESH_WAIT+REFRESH_WAIT-1)
      begin
        next_state <= MODE;
        next_cmd   <= CMD_LOAD_MODE;
      end

    // Set control register
    MODE:
      if load_mode_done
        next_state <= IDLE;

    // Wait for a read/write request
    IDLE:
      if should_refresh begin
        next_state <= REFRESH;
        next_cmd   <= CMD_REFRESH;
      end
      else if req begin
        next_state <= ACTIVE;
        next_cmd   <= CMD_ACTIVE;
      end

    // Activate the row
    ACTIVE:
      if active_done
        if we_reg begin
          next_state <= WRITE_A;
          next_cmd   <= CMD_WRITE_PRE;
        end
        else begin
          next_state <= READ_A;
          next_cmd   <= CMD_READ_PRE;
        end

    // Execute a read and autoprecharge command
    READ_A:
      if read_done
        if (should_refresh) begin
          next_state <= REFRESH;
          next_cmd   <= CMD_AUTO_REFRESH;
        end
        else if req begin
          next_state <= ACTIVE;
          next_cmd   <= CMD_ACTIVE;
        end
        else
          next_state <= IDLE;

    // Execute a write and autoprecharge command
    WRITE_A:
      if write_done
        if should_refresh begin
          next_state <= REFRESH;
          next_cmd   <= CMD_AUTO_REFRESH;
        end
        else if req begin
          next_state <= ACTIVE;
          next_cmd   <= CMD_ACTIVE;
        end
        else
          next_state <= IDLE;

    // Execute an auto refresh
    REFRESH:
      if refresh_done
        if req begin
          next_state <= ACTIVE;
          next_cmd   <= CMD_ACTIVE;
        end
        else
          next_state <= IDLE;
  endcase
end

//! Next state register
always @ (posedge clk, posedge reset) begin: fsm_next_state_register
  if reset begin
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
always @ (posedge clk, posedge reset) begin: wait_counter
  if reset 
    wait_counter <= 0;
  // state changing
  else if state != next_state 
    wait_counter <= 0;
  else
    wait_counter <= wait_counter + 1;
end

//! The refresh counter is used to periodically trigger a refresh operation
always @ (posedge clk, posedge reset) begin: update_refresh_counter 
  if reset 
    refresh_counter <= 0;
  else ((state == REFRESH) and (wait_counter == 0))
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
end process;

//  SDRAM data output register
always @ (posedge clk) begin: sdram_data_reg
   if state == READ then
      q_reg <= sdram_dq;
    end 
  end 
end 

// Assign output FPGA data port 
assign rd_data = q_reg;

// set wait signals
assign load_mode_done = (wait_counter == LOAD_MODE_WAIT-1) ? 1'b1 : 1'b0;
assign active_done    = (wait_counter == ACTIVE_WAIT-1   ) ? 1'b1 : 1'b0;
assign refresh_done   = (wait_counter == REFRESH_WAIT-1  ) ? 1'b1 : 1'b0;
//assign first_word     = (wait_counter == CAS_LATENCY     ) ? 1'b1 : 1'b0;
assign read_done      = (wait_counter == READ_WAIT-1     ) ? 1'b1 : 1'b0;
assign write_done     = (wait_counter == WRITE_WAIT-1    ) ? 1'b1 : 1'b0;

// the SDRAM should be refreshed when the refresh interval has elapsed
assign should_refresh  = (refresh_counter >= REFRESH_INTERVAL-1) ? 1'b1 : 1'b0;

// a new request is only allowed at the end of the 
// IDLE, READ, WRITE, and REFRESH states
assign start = (state = IDLE) |
        (state == READ & read_done == 1'b1) |
        (state == WRITE & write_done == 1'b1) |
        (state == REFRESH & refresh_done == 1'b1) 
        ? 1'b1 : 1'b0;

// assert the acknowledge signal at the beginning of the ACTIVE state
assign ack = ((state == ACTIVE) & (wait_counter == 0)) ? 1'b1 : 1'b0;;

// deassert the clock enable at the beginning of the INIT state
assign ram_clk_en = ((state == INIT) and (wait_counter == 0)) ) ? 1'b0 : 1'b1;;

//! Set SDRAM control signals
//! ram_cs_n, ras_n, cas_n, ram_we_n from SDRAM command
assign {ram_cs_n, ram_ras_n, ram_cas_n, ram_we_n} = cmd [6:3];

//! Set SDRAM bank
always @ (state)
  case (state)
    ACTIVE : ram_bank_addr = bank;
    READ_A : ram_bank_addr = bank;
    WRITE_A: ram_bank_addr = bank;
    default: ram_bank_addr = 0;
  endcase

// set SDRAM address
always @ (state)
  case (state)
    INIT   : ram_addr = "0100_0000_0000";
    MODE   : ram_addr = MODE_REG;
    ACTIVE : ram_addr = row;
    READ_A : ram_addr = {3'b010, col};
    WRITE_A: ram_addr = {3'b010, col};
    default: ram_addr = SDRAM_ADDR_WIDTH'h000;
  endcase
 
  // set SDRAM data signals
  assign  ram_data = (state == WRITE_A) ? data_reg : FPGA_DATA_WIDTH'hzzzz;

  // set SDRAM data mask DQM
  assign ram_dqm = 0;


endmodule 