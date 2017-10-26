#ifndef __PARAMS_H__
#define __PARAMS_H__

/********************/
/* Processor params */
/********************/
// number of cores in mulicore 
int NUMCORES;

// number of HMCs
 int NUM_HMCS;
 
// number of DIMMs
 int NUM_DIMMS;

// processor clock frequency multiplier : multiplying the
// DRAM_CLK_FREQUENCY by the following parameter gives the processor
// clock frequency 
 int PROCESSOR_CLK_MULTIPLIER;
 
 int DIMM_CLK_MULTIPLIER;
 
 int HMC_CLK_MULTIPLIER;
 
 int SERDES_CLK_MULTIPLIER;

 int MEMORY_CLK_MULTIPLIER[4];
 
//size of ROB
 int ROBSIZE ;// 128;		

// maximum commit width
 int MAX_RETIRE ;// 2;

// maximum instruction fetch width
 int MAX_FETCH ;// 4;	

// depth of pipeline
 int PIPELINEDEPTH ;// 5;


/*****************************/
/* DRAM System Configuration */
/*****************************/
// total number of channels in the system
 int NUM_CHANNELS ;// 1;

// total number of vaults per channel
 int NUM_VAULTS[4] ;// 1;

// number of ranks per channel
 int NUM_RANKS[4] ;// 2;

// number of banks per rank
 int NUM_BANKS[4] ;// 8;

// number of rows per bank
 int NUM_ROWS[4] ;// 32768;

// number of columns per rank
 int NUM_COLUMNS[4] ;// 128;

// cache-line size (bytes)
 int CACHE_LINE_SIZE[4] ;// 64;

// total number of address bits (i.e. indicates size of memory)
 int ADDRESS_BITS ;// 32;

/****************************/
/* DRAM Chip Specifications */
/****************************/

// All the following timing parameters should be 
// entered in the config file in terms of memory 
// clock cycles.

// RAS to CAS delay
 int T_RCD[4] ;// 44;

// PRE to RAS
 int T_RP[4] ;// 44;

// ColumnRD to Data burst
 int T_CAS[4] ;// 44;

// RAS to PRE delay
 int T_RAS[4] ;// 112;

// Row Cycle time
 int T_RC[4] ;// 156;

// ColumnWR to Data burst
 int T_CWD[4] ;// 20;

// write recovery time (COL_WR to PRE)
 int T_WR[4] ;// 48;

// write to read turnaround
 int T_WTR[4] ;// 24;

// rank to rank switching time
 int T_RTRS[4] ;// 8;

// Data transfer
 int T_DATA_TRANS[4] ;// 16;

// Read to PRE
 int T_RTP[4] ;// 24;

// CAS to CAS
 int T_CCD[4] ;// 16;

// Power UP time fast
 int T_XP[4] ;// 20;

// Power UP time slow
 int T_XP_DLL[4] ;// 40;

// Power down entry
 int T_CKE[4] ;// 16;

// Minimum power down duration
 int T_PD_MIN[4] ;// 16;

// rank to rank delay (ACTs to same rank)
 int T_RRD[4] ;// 20;

// four bank activation window
 int T_FAW[4] ;// 128;

// refresh interval
 int T_REFI[4];

 // refresh cycle time
 int T_RFC[4];

/****************************/
/* VOLTAGE & CURRENT VALUES */
/****************************/

float VDD[4];

float IDD0[4];

float IDD1[4];

float IDD2P0[4];

float IDD2P1[4];

float IDD2N[4];

float IDD3P[4];

float IDD3N[4];

float IDD4R[4];

float IDD4W[4];

float IDD5[4];

/******************************/
/* MEMORY CONTROLLER Settings */
/******************************/

// maximum capacity of write queue (per channel)
 int WQ_CAPACITY[4] ;// 64;

//  int ADDRESS_MAPPING mode
// 1 is consecutive cache-lines to same row
// 2 is consecutive cache-lines striped across different banks 
 int ADDRESS_MAPPING ;// 1;

 //RQ associative lookup
 int RQ_LOOKUP_LATENCY[4];

 // WQ associative lookup 
 int WQ_LOOKUP_LATENCY[4];

 // RQ link latency 
 int RQ_LINK_LATENCY;
 
  // WQ link latency 
 int WQ_LINK_LATENCY;

#endif // __PARAMS_H__

