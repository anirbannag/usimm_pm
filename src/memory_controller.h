#ifndef __MEMORY_CONTROLLER_H__
#define __MEMORY_CONTROLLER_H__

#define MAX_NUM_CHANNELS 4
#define MAX_NUM_VAULTS 16
#define MAX_NUM_RANKS 16
#define MAX_NUM_BANKS 8
#define MAX_NUM_HMCS 2
#define MAX_NUM_CORES 4

// Moved here from main.c 
long long int *committed; // total committed instructions in each core
long long int *fetched;   // total fetched instructions in each core


//////////////////////////////////////////////////
//	Memory Controller Data Structures	//
//////////////////////////////////////////////////

// DRAM Address Structure
typedef struct draddr
{
  long long int actual_address; // physical_address being accessed
  int channel;	// channel id
  int vault;	//vault id
  int rank;	// rank id
  int bank;	// bank id
  long long int row;	// row/page id
  int column;	// column id
} dram_address_t;

// DRAM Commands 
typedef enum {ACT_CMD, COL_READ_CMD, PRE_CMD, COL_WRITE_CMD, PWR_DN_SLOW_CMD, PWR_DN_FAST_CMD, PWR_UP_CMD, REF_CMD, NOP} command_t; 

// Request Types
typedef enum {READ, WRITE} optype_t;

// Single request structure self-explanatory
typedef struct req
{
  unsigned long long int physical_address;
  dram_address_t dram_addr;
  long long int arrival_time;     
  long long int dispatch_time; // when COL_RD or COL_WR is issued for this request
  long long int completion_time; //final completion time
  long long int latency; // dispatch_time-arrival_time
  int thread_id; // core that issued this request
  command_t next_command; // what command needs to be issued to make forward progress with this request
  int command_issuable; // can this request be issued in the current cycle
  optype_t operation_type; // Read/Write
  int request_served; // if request has it's final command issued or not
  int instruction_id; // 0 to ROBSIZE-1
  long long int instruction_pc; // phy address of instruction that generated this request (valid only for reads)
  void * user_ptr; // user_specified data
  struct req * next;
} request_t;

// Bankstates
typedef enum 
{
  IDLE, PRECHARGING, REFRESHING, ROW_ACTIVE, PRECHARGE_POWER_DOWN_FAST, PRECHARGE_POWER_DOWN_SLOW, ACTIVE_POWER_DOWN
} bankstate_t;

// Structure to hold the state of a bank
typedef struct bnk
{
  bankstate_t state;
  long long int active_row;
  long long int next_pre;
  long long int next_act;
  long long int next_read;
  long long int next_write;
  long long int next_powerdown;
  long long int next_powerup;
  long long int next_refresh;
}bank_t;

// contains the states of all banks in the system 
bank_t dram_state[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];

// command issued this cycle to this channel
int command_issued_current_cycle[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

// cas command issued this cycle to this channel
int cas_issued_current_cycle[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS]; // 1/2 for COL_READ/COL_WRITE

// Per channel read queue
request_t * read_queue_head[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

// Per channel write queue
request_t * write_queue_head[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

// SerDes queue at host
request_t * read_queue_per_core_head[MAX_NUM_CORES][MAX_NUM_HMCS];

// SerDes queue at host
request_t * write_queue_per_core_head[MAX_NUM_CORES][MAX_NUM_HMCS];

request_t * read_return_queue_head[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

// issuables_for_different commands
int cmd_precharge_issuable[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];
int cmd_all_bank_precharge_issuable[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
int cmd_powerdown_fast_issuable[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
int cmd_powerdown_slow_issuable[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
int cmd_powerup_issuable[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
int cmd_refresh_issuable[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];


// refresh variables
long long int next_refresh_completion_deadline[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int last_refresh_completion_deadline[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
int forced_refresh_mode_on[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
int refresh_issue_deadline[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
int issued_forced_refresh_commands[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
int num_issued_refreshes[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];

long long int read_queue_length[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
long long int write_queue_length[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

long long int write_queue_length_for_core[MAX_NUM_CORES][MAX_NUM_HMCS];
long long int read_queue_length_for_core[MAX_NUM_CORES][MAX_NUM_HMCS];
long long int read_return_queue_length[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

// Stats
long long int num_read_merge ;
long long int num_write_merge ;
long long int stats_reads_merged_per_vault[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
long long int stats_writes_merged_per_vault[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
long long int stats_reads_seen[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
long long int stats_writes_seen[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
long long int stats_reads_completed[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
long long int stats_writes_completed[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

double stats_average_read_latency[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
double stats_average_read_queue_latency[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
double stats_average_write_latency[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
double stats_average_write_queue_latency[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

long long int stats_page_hits[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
double stats_read_row_hit_rate[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];

// Time spent in various states
long long int stats_time_spent_in_active_standby[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_time_spent_in_active_power_down[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_time_spent_in_precharge_power_down_fast[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_time_spent_in_precharge_power_down_slow[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_time_spent_in_power_up[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int last_activate[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int last_refresh[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
double average_gap_between_activates[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
double average_gap_between_refreshes[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_time_spent_terminating_reads_from_other_ranks[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_time_spent_terminating_writes_to_other_ranks[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];

// Command Counters
long long int stats_num_activate_read[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];
long long int stats_num_activate_write[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];
long long int stats_num_activate_spec[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];
long long int stats_num_activate[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_num_precharge[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];
long long int stats_num_read[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];
long long int stats_num_write[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];
long long int stats_num_powerdown_slow[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_num_powerdown_fast[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];
long long int stats_num_powerup[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS];


long long int next_request_schedule_time[MAX_NUM_HMCS];
long long int next_respond_schedule_time[MAX_NUM_HMCS];

int drain_writes[MAX_NUM_CHANNELS][MAX_NUM_VAULTS];
int drain_write_for_core[MAX_NUM_CORES][MAX_NUM_HMCS];


// scheduler params


// functions

// to get log with base 2
unsigned int log_base2(unsigned int new_value);

// initialize memory_controller variables
void init_memory_controller_vars();

// called every cycle to update the read/write queues
void update_memory(int channel);

// activate to bank allowed or not
int is_activate_allowed(int channel, int vault, int rank, int bank);

// precharge to bank allowed or not
int is_precharge_allowed(int channel, int vault, int rank, int bank);

// all bank precharge allowed or not
int is_all_bank_precharge_allowed(int channel, int vault, int rank);

// autoprecharge allowed or not
int is_autoprecharge_allowed(int channel, int vault, int rank, int bank);

// power_down fast allowed or not
int is_powerdown_fast_allowed(int channel, int vault, int rank);

// power_down slow allowed or not
int is_powerdown_slow_allowed(int channel, int vault, int rank);

// powerup allowed or not
int is_powerup_allowed(int channel,int vault, int rank);

// refresh allowed or not
int is_refresh_allowed(int channel, int vault, int rank);

// issues command to make progress on a request
int issue_request_command(request_t * req);

// to check if all write instructions are committed
int are_all_writes_completed();

// power_down command
int issue_powerdown_command(int channel, int vault, int rank, command_t cmd);

// powerup command
int issue_powerup_command(int channel, int vault, int rank);

// precharge a bank
int issue_activate_command(int channel, int vault, int rank, int bank, long long int row);

// precharge a bank
int issue_precharge_command(int channel, int vault, int rank, int bank);

// precharge all banks in a rank
int issue_all_bank_precharge_command(int channel, int vault, int rank);

// refresh all banks
int issue_refresh_command(int channel, int vault, int rank);

// autoprecharge all banks
int issue_autoprecharge(int channel, int vault, int rank, int bank);

// find if there is a matching write request
int read_matches_write_or_read_queue(long long int physical_address, int thread_id);

// find if there is a matching request in the write queue
int write_exists_in_write_queue(long long int physical_address, int thread_id);

// to transfer eligible request from READ/WRITE request queues from processor to HMC
void transfer_request_to_HMCs(int channel);

// to transfer eligible resquest response from READ return queue from HMC to processor 
void transfer_response_to_PROCESSOR(int channel);

// enqueue a read into the corresponding read queue (returns ptr to new node)
request_t* insert_read(long long int physical_address, long long int arrival_cycle, int thread_id, int instruction_id, long long int instruction_pc);

// enqueue a write into the corresponding write queue (returns ptr to new_node)
request_t* insert_write(long long int physical_address, long long int arrival_time, int thread_id, int instruction_id);

// update stats counters
void gather_stats(int channel, int vault);

// print statistics
void print_stats();

// calculate power for each channel
float calculate_power(int channel, int vault, int rank, int print_stats_type, int chips_per_rank);

// Calculate DRAM address
dram_address_t * calc_dram_addr(long long int physical_address);

int is_writeq_full(int thread_id);

#endif // __MEM_CONTROLLER_HH__
