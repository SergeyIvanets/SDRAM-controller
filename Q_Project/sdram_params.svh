//! sdram_params.vh

//! Timig prameters
//! Power-up delay 100 us min. Set 200 ms
parameter  T_POWER_UP = 200_000;
//! Command period (PRE to ACT) delay, ns min
parameter  T_RP       = 18;    
//! Command period (REF to REF/ACT to ACT) delay, ns min
parameter  T_RC       = 60;    
//! Mode register Program Time delay, ns min
parameter  T_MRD      = 18;
//! Active to Read/Write delay, ns min
parameter  T_RCD      = 18;
//! Refresh cycle time, ms max
parameter  T_REF      = 64;
//! RAS time, ns min
parameter T_RAS       = 42;
//! The number of 16-bit words to be bursted during a read/write
// parameter BURST_LENGTH = 2;
//! Write recovery Time or InputData to Precharge Command Delay Time
parameter  T_WR        = 12;


//! Mode Register Definition
//! M9: Write Burst Mode: 0 - programmed Burst Lengtn, 1 - Single Location
parameter WRITE_BURST_MODE = 1'b1;
//! M3:  Burst Type: 0 - Sequention, 1 - Interleaved
parameter BURST_TYPE       = 1'b0;
//! M2: - M0  Burst Length 000 - 1; 001 - 2; 010 - 4; 011 - 8
parameter BURST_LENGTH     = 3'b011;

//!	Controller Parameter
//! 166 MHz
//! CAS Latence 2 cycle
parameter CAS_LATENCY = 3'b010;


//! 100 MHz	
/*
//! CAS Latence 3 cycle
parameter CAS_LATENCY = 3'b011;

*/