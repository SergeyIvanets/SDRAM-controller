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

`include "sdram_params.h"

module sdram_controller

  #(
     parameter  ROW_WIDTH         = 12,
                COL_WIDTH         = 9,
                BANK_WIDTH        = 2,
                DATA_WIDTH        = 32,
                CLK_FREQUENCY_SYS = 166,  //! MHz System clock
                CLK_FREQUENCY_MAX = 166,  //! MHz From SDRAM datasheet
                REFRESH_TIME      = 64    //! ms Refresh Cycle Time


   )
  
   (
     //! 
     input                            clk,
     input                            reset,
     input                            busy,

     //! Write port
     input [FPGA_ADDR_WIDTH - 1 : 0]  wr_addr,
     input [     DATA_WIDTH - 1 : 0]  wr_data,
     input                            wr_en,
    
    //! Read port
     input  [FPGA_ADDR_WIDTH - 1 : 0] rd_addr,
     output [     DATA_WIDTH - 1 : 0] rd_data,
     input                            rd_en,
     input                            rd_ready,

     //! SDRAM ports
     output [ RAM_ADDR_WIDTH - 1 : 0] ram_addr,
     output [     BANK_WIDTH - 1 : 0] ram_bank_addr,
     output [RAM_BYTES_WIDTH - 1 : 0] ram_dqm,
     inout  [     DATA_WIDTH - 1 : 0] ram_data,

     output                           ram_ras_n,
     output                           ram_cas_n,
     output                           ram_clk,
     output                           ram_clk_en,
     output                           ram_cs_n
   );

  // ------------------ Local Parameters --------------------

  //! | Command | Code | ram_clk_en | ram_cs_n | ras_n | cas_n | ram_we_n | ram_bank_addr[1] | ram_bank_addr[0] | ram_addr[10] | 
  //! | ---------------------- | ------ | ------------ | ---------- | ------- | ------- | ---------- | ------------------ | ------------------ | -------------- | 
  //! | Device deselect  | DESL | X | 1 | X | X | X | X | X | X | 
  //! | No operation  | NOP | X | 0 | 1 | 1 | 1 | X | X | X | 
  //! | Burst stop  | BST | X | 0 | 1 | 1 | 0 | X | X | X | 
  //! | Read | RD | X | 0 | 1 | 0 | 1 | X | X | 0 | 
  //! | Read with auto precharge | RDAR | X | 0 | 1 | 0 | 1 | X | X | 1 | 
  //! | Write | WR | X | 0 | 1 | 0 | 0 | X | X | 0 | 
  //! | Write with auto precharge | WRAR | X | 0 | 1 | 0 | 0 | X | X | 1 | 
  //! | Bank activate  | ACT | X | 0 | 0 | 1 | 1 | X | X | X | 
  //! | Precharge select bank  | PRE | X | 0 | 0 | 1 | 0 | X | X | 0 | 
  //! | Precharge all banks  | PALL | X | 0 | 0 | 1 | 0 | X | X | 1 | 

  localparam CMD_DESL = 8'bX1XXXXXX;
  localparam CMD_NOP  = 8'bX0111XXX;
  localparam CMD_BST  = 8'bX0110XXX;
  localparam CMD_RD   = 8'bX0101XX0;
  localparam CMD_RDAR = 8'bX0101XX1;
  localparam CMD_WR   = 8'bX0100XX0;
  localparam CMD_WRAR = 8'bX0100XX1;
  localparam CMD_ACT  = 8'bX0011XXX;
  localparam CMD_PRE  = 8'bX0010XX0;
  localparam CMD_PALL = 8'bX0010XX1;
  localparam CMD_REF  = 8'b10001XXX;
  localparam CMD_SELF = 8'b00001XXX;
  localparam CMD_MRS  = 8'bX0000000;

  //! FPGA system address width
  localparam FPGA_ADDR_WIDTH        = BANK_WIDTH + ROW_WIDTH + COL_WIDTH; 
  
  //! SDRAM address width ??
  localparam RAM_ADDR_WIDTH         = ROW_WIDTH > COL_WIDTH ?     

  //! System clock cycles required per refresh time
  localparam CYCLES_BETWEEN_REFRESH = CLK_FREQUENCY_SYS * REFRESH_TIME * 1000; 
/* 
  // clk / refresh =  clk / sec
  //                , sec / refbatch
  //                , ref / refbatch
 localparam CYCLES_BETWEEN_REFRESH = ( CLK_FREQUENCY
                                        * 1_000
                                        * REFRESH_TIME
                                      ) / REFRESH_COUNT;*/

  //! Number of bit needed for data byte encoding
  localparam RAM_BYTES_WIDTH        = $clog2 (DATA_WIDTH / 8);

  //! Number of row per refresh time
  localparam REFRESH_COUNT          = $clog2 (ROW_WIDTH); 

  //! SDRAM control FSM
  //! States 
  localparam init      = 4'b0000,
             mode      = 4'b1111,
             idle      = 4'b0011,
             refresh   = 4'b0110,
             activate  = 4'b0010,
             nop       = 4'b0001,
             read      = 4'b0100,
             read_a    = 4'b0101,
             write     = 4'b1000,
             write_a   = 4'b1001,
             precharge = 4'b1100;

always @ (posedge clk or posedge reset)
begin
  
end

endmodule 