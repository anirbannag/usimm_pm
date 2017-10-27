#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "utlist.h"

#include "utils.h"

#include "params.h"
#include "memory_controller.h"
#include "scheduler.h"
#include "processor.h"

// ROB Structure, used to release stall on instructions 
// when the read request completes
extern struct robstructure * ROB;

// Current Processor Cycle
extern long long int CYCLE_VAL;

#define max(a,b) (((a)>(b))?(a):(b))

#define BIG_ACTIVATION_WINDOW 1000000

// moving window that captures each activate issued in the past 
int activation_record[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][BIG_ACTIVATION_WINDOW];

int is_writeq_full(int thread_id)
{
	for(int channel=0; channel<NUM_CHANNELS; channel++){
		if(channel < NUM_HMCS) {
			if(write_queue_length_for_core[thread_id][channel] == WQ_CAPACITY[channel])
				return 1;
		}
		else {
			for(int vault=0; vault<NUM_VAULTS[channel]; vault++) {				
				if(write_queue_length[channel][vault] == WQ_CAPACITY[channel])
					return 1;
			}
		}
    }
    return 0;
}

// record an activate in the activation record
void record_activate(int channel, int vault, int rank, long long int cycle)
{
	assert(activation_record[channel][vault][rank][(cycle%BIG_ACTIVATION_WINDOW)] == 0); //can't have two commands issued the same cycle - hence no two activations in the same cycle
	activation_record[channel][vault][rank][(cycle%BIG_ACTIVATION_WINDOW)]=1;
	
	return;
}

// Have there been 3 or less activates in the last T_FAW period 
int is_T_FAW_met(int channel, int vault, int rank, int cycle)
{
	int start = cycle;

	int number_of_activates =0 ;

	if(start >= T_FAW[channel])
	{
	  for(int i=1; i<= T_FAW[channel]; i++)
	  {
	    if(activation_record[channel][vault][rank][(start-i)%BIG_ACTIVATION_WINDOW] == 1)
	      number_of_activates ++;
	  }
	}
	else
	{
	  for(int i=1 ; i<=start ; i++)
	  {
	    if(activation_record[channel][vault][rank][start-i]%BIG_ACTIVATION_WINDOW == 1)
	      number_of_activates ++;
	  }
	}
	if(number_of_activates < 4)
		return 1;
	else
		return 0;
}

// shift the moving window, clear out the past
void flush_activate_record(int channel, int vault, int rank, long long int cycle)
{
	if(cycle >= T_FAW[channel] + MEMORY_CLK_MULTIPLIER[channel])
	{
	  for(int i=1; i<=MEMORY_CLK_MULTIPLIER[channel] ;i++) 
		activation_record[channel][vault][rank][(cycle-T_FAW[channel]-i)%BIG_ACTIVATION_WINDOW] = 0; // make sure cycle >tFAW
	}

}

// initialize dram variables and statistics
void init_memory_controller_vars()
{
    num_read_merge =0;
	num_write_merge =0;
	for(int i=0; i<NUM_CHANNELS; i++)
	{
		for(int v=0; v<NUM_VAULTS[i]; v++)
		{
			for(int j=0; j<NUM_RANKS[i]; j++)
			{
				for(int w=0;w<BIG_ACTIVATION_WINDOW;w++)
					activation_record[i][v][j][w] = 0;

				for (int k=0; k<NUM_BANKS[i]; k++)
				{
					dram_state[i][v][j][k].state = IDLE;
					dram_state[i][v][j][k].active_row = -1;
					dram_state[i][v][j][k].next_pre = -1;
					dram_state[i][v][j][k].next_pre = -1;
					dram_state[i][v][j][k].next_pre = -1;
					dram_state[i][v][j][k].next_pre = -1;

					cmd_precharge_issuable[i][v][j][k] = 0;

					stats_num_activate_read[i][v][j][k]=0;
					stats_num_activate_write[i][v][j][k]=0;
					stats_num_activate_spec[i][v][j][k]=0;
					stats_num_precharge[i][v][j][k]=0;
					stats_num_read[i][v][j][k]=0;
					stats_num_write[i][v][j][k]=0;
					cas_issued_current_cycle[i][v][j][k]=0;
					
				}

				cmd_all_bank_precharge_issuable[i][v][j] =0;
				cmd_powerdown_fast_issuable[i][v][j]=0;
				cmd_powerdown_slow_issuable[i][v][j]=0;
				cmd_powerup_issuable[i][v][j]=0;
				cmd_refresh_issuable[i][v][j]=0;

				next_refresh_completion_deadline[i][v][j] = 8*T_REFI[i];
				last_refresh_completion_deadline[i][v][j] = 0;
				forced_refresh_mode_on[i][v][j]=0;
				refresh_issue_deadline[i][v][j] = next_refresh_completion_deadline[i][v][j] - T_RP[i] - 8*T_RFC[i];
				num_issued_refreshes[i][v][j] = 0;
			
				stats_time_spent_in_active_power_down[i][v][j]=0;
				stats_time_spent_in_precharge_power_down_slow[i][v][j]=0;
				stats_time_spent_in_precharge_power_down_fast[i][v][j]=0;
				last_activate[i][v][j]=0;
				//If average_gap_between_activates is 0 then we know that there have been no activates to [i][j]
				average_gap_between_activates[i][v][j]=0;

				stats_num_powerdown_slow[i][v][j]=0;
				stats_num_powerdown_fast[i][v][j]=0;
				stats_num_powerup[i][v][j]=0;

				stats_num_activate[i][v][j]=0;

				command_issued_current_cycle[i][v]=0;
			}

			read_queue_head[i][v]=NULL;
			write_queue_head[i][v]=NULL;

			read_queue_length[i][v]=0;
			write_queue_length[i][v]=0;

			read_return_queue_head[i][v]=NULL;
			read_return_queue_length[i][v] = 0;

			command_issued_current_cycle[i][v]=0;

			// Stats
			stats_reads_merged_per_vault[i][v]=0;
			stats_writes_merged_per_vault[i][v]=0;

			stats_reads_seen[i][v]=0;
			stats_writes_seen[i][v]=0;
			stats_reads_completed[i][v]=0;
			stats_writes_completed[i][v]=0;
			stats_average_read_latency[i][v]=0;
			stats_average_read_queue_latency[i][v]=0;
			stats_average_write_latency[i][v]=0;
			stats_average_write_queue_latency[i][v]=0;
			stats_page_hits[i][v]=0;
			stats_read_row_hit_rate[i][v]=0;
			
			drain_writes[i][v] = 0;
		}
	}
	
	for(int cores = 0; cores < NUMCORES; cores++)
	{
		for(int channel=0; channel < NUM_HMCS; channel++)
		{
			read_queue_per_core_head[cores][channel] = NULL;
			write_queue_per_core_head[cores][channel] =  NULL;

			write_queue_length_for_core[cores][channel] = 0;
			read_queue_length_for_core[cores][channel] = 0;
			
			drain_write_for_core[cores][channel] = 0;
			
			next_request_schedule_time[channel] = 0;
			next_respond_schedule_time[channel] = 0;
		}
	}
}

/********************************************************/
/*	Utility Functions				*/
/********************************************************/


unsigned int log_base2(unsigned int new_value)
{
	int i;
	for (i = 0; i < 32; i++) {
		new_value >>= 1;
		if (new_value == 0)
			break;
	}
	return i;
}

// Function to decompose the incoming DRAM address into the
// constituent channel, rank, bank, row and column ids. 
// Note : To prevent memory leaks, call free() on the pointer returned
// by this function after you have used the return value.
dram_address_t * calc_dram_addr(long long int physical_address)
{


	long long int input_a, temp_b, temp_a;
	
	//right now, first 64GB reserved for HMC, next 64GB for DIMMs
	int channel_num = 0;
	int is_dimm_address = 0;
	if(NUM_HMCS != 0) {
		is_dimm_address = physical_address >> 36;
		if(!is_dimm_address)
			channel_num = 0;
		else if(NUM_CHANNELS != NUM_HMCS)
			channel_num = NUM_HMCS; 
	}
	else {
		is_dimm_address = 1;
		channel_num = 0;
	}
	
	int channelBitWidth = (is_dimm_address)?log_base2(NUM_DIMMS):log_base2(NUM_HMCS);
	int vaultBitWidth = log_base2(NUM_VAULTS[channel_num]);
	int rankBitWidth = log_base2(NUM_RANKS[channel_num]);
	int bankBitWidth = log_base2(NUM_BANKS[channel_num]);
	int rowBitWidth = log_base2(NUM_ROWS[channel_num]);
	int colBitWidth = log_base2(NUM_COLUMNS[channel_num]);
	int byteOffsetWidth = log_base2(CACHE_LINE_SIZE[channel_num]);



	dram_address_t * this_a = (dram_address_t*)malloc(sizeof(dram_address_t));

	this_a->actual_address = physical_address;

	input_a = physical_address;

	input_a = input_a >> byteOffsetWidth;		  // strip out the cache_offset


	if(!is_dimm_address) {
		temp_b = input_a;   	
		input_a = input_a >> channelBitWidth;
		temp_a  = input_a << channelBitWidth;
		this_a->channel = temp_a ^ temp_b; 		// strip out the channel address
		
		temp_b = input_a;   	
		input_a = input_a >> vaultBitWidth;
		temp_a  = input_a << vaultBitWidth;
		this_a->vault = temp_a ^ temp_b; 		// strip out the vault address


		temp_b = input_a;
		input_a = input_a >> bankBitWidth;
		temp_a  = input_a << bankBitWidth;
		this_a->bank = temp_a ^ temp_b;		// strip out the bank address 


		temp_b = input_a;
		input_a = input_a >> rankBitWidth;
		temp_a  = input_a << rankBitWidth;
		this_a->rank = temp_a ^ temp_b;     		// strip out the rank address


		temp_b = input_a;
		input_a = input_a >> colBitWidth;
		temp_a  = input_a << colBitWidth;
		this_a->column = temp_a ^ temp_b;		//strip out the column address


		temp_b = input_a;
		input_a = input_a >> rowBitWidth;
		temp_a  = input_a << rowBitWidth;
		this_a->row = temp_a ^ temp_b;			// strip out the row number
	}
	else if(is_dimm_address && ADDRESS_MAPPING == 0)
	{
		this_a->vault = 0;
			
		temp_b = input_a;				
		input_a = input_a >> colBitWidth;
		temp_a  = input_a << colBitWidth;
		this_a->column = temp_a ^ temp_b;		//strip out the column address
		

		temp_b = input_a;   				
		input_a = input_a >> channelBitWidth;
		temp_a  = input_a << channelBitWidth;
		this_a->channel = temp_a ^ temp_b; 		// strip out the channel address
		this_a->channel += NUM_HMCS;

		temp_b = input_a;				
		input_a = input_a >> bankBitWidth;
		temp_a  = input_a << bankBitWidth;
		this_a->bank = temp_a ^ temp_b;		// strip out the bank address 


		temp_b = input_a;			
		input_a = input_a >> rankBitWidth;
		temp_a  = input_a << rankBitWidth;
		this_a->rank = temp_a ^ temp_b;     		// strip out the rank address


		temp_b = input_a;		
		input_a = input_a >> rowBitWidth;
		temp_a  = input_a << rowBitWidth;
		this_a->row = temp_a ^ temp_b;		// strip out the row number
	}
	else if(is_dimm_address && ADDRESS_MAPPING == 1)
	{
		this_a->vault = 0;

		temp_b = input_a;   	
		input_a = input_a >> channelBitWidth;
		temp_a  = input_a << channelBitWidth;
		this_a->channel = temp_a ^ temp_b; 		// strip out the channel address
		this_a->channel += NUM_HMCS;


		temp_b = input_a;
		input_a = input_a >> bankBitWidth;
		temp_a  = input_a << bankBitWidth;
		this_a->bank = temp_a ^ temp_b;		// strip out the bank address 


		temp_b = input_a;
		input_a = input_a >> rankBitWidth;
		temp_a  = input_a << rankBitWidth;
		this_a->rank = temp_a ^ temp_b;     		// strip out the rank address


		temp_b = input_a;
		input_a = input_a >> colBitWidth;
		temp_a  = input_a << colBitWidth;
		this_a->column = temp_a ^ temp_b;		//strip out the column address


		temp_b = input_a;
		input_a = input_a >> rowBitWidth;
		temp_a  = input_a << rowBitWidth;
		this_a->row = temp_a ^ temp_b;			// strip out the row number
	}
	return(this_a);
}

// Function to create a new request node to be inserted into the read
// or write queue.
void * init_new_node(long long int physical_address, long long int arrival_time, optype_t type, int thread_id, int instruction_id, long long int instruction_pc)
{
	request_t * new_node = NULL;

	new_node = (request_t*)malloc(sizeof(request_t));

	if(new_node == NULL)
	{
		printf("FATAL : Malloc Error\n");

		exit(-1);
	}
	else
	{

		new_node->physical_address = physical_address;

		new_node->arrival_time = arrival_time;

		new_node->dispatch_time = -100;

		new_node->completion_time = -100;

		new_node->latency = -100;

		new_node->thread_id = thread_id;

		new_node->next_command = NOP;

		new_node->command_issuable = 0;

		new_node->operation_type = type;

		new_node->request_served = 0;

		new_node->instruction_id = instruction_id;

		new_node->instruction_pc = instruction_pc;

		new_node->next = NULL;

		dram_address_t * this_node_addr = calc_dram_addr(physical_address);

		new_node->dram_addr.actual_address = physical_address;
		new_node->dram_addr.channel = this_node_addr->channel;
		new_node->dram_addr.vault = this_node_addr->vault;
		new_node->dram_addr.rank = this_node_addr->rank;
		new_node->dram_addr.bank = this_node_addr->bank;
		new_node->dram_addr.row = this_node_addr->row;
		new_node->dram_addr.column = this_node_addr->column;

		free(this_node_addr);

		new_node->user_ptr = NULL;

		return (new_node);
	}
}

// Function that checks to see if an incoming read can be served by a
// write request pending in the write queue and return
// WQ_LOOKUP_LATENCY if there is a match. Also the function goes over
// the read_queue to see if there is a pending read for the same
// address and avoids duplication. The 2nd read is assumed to be
// serviced when the original request completes.

int read_matches_write_or_read_queue(long long int physical_address, int thread_id)
{
	//get channel info
	dram_address_t * this_addr = calc_dram_addr(physical_address);
	int channel = this_addr->channel;
	int vault = this_addr->vault;
	free(this_addr);

	request_t * wr_ptr = NULL;
	request_t * rd_ptr = NULL;

	if(channel < NUM_HMCS) {
		LL_FOREACH(write_queue_per_core_head[thread_id][channel], wr_ptr)
		{
			if(wr_ptr->dram_addr.actual_address == physical_address)
			{
			  num_read_merge ++;
			  stats_reads_merged_per_vault[channel][vault]++;
			  return WQ_LOOKUP_LATENCY[channel];
			}
		}
		LL_FOREACH(read_queue_per_core_head[thread_id][channel], rd_ptr)
		{
			if(rd_ptr->dram_addr.actual_address == physical_address)
			{
			  num_read_merge ++;
			  stats_reads_merged_per_vault[channel][vault]++;
			  return RQ_LOOKUP_LATENCY[channel];
			}
		}
	}

	else {
		LL_FOREACH(write_queue_head[channel][vault], wr_ptr)
		{
			if(wr_ptr->dram_addr.actual_address == physical_address)
			{
			  num_read_merge ++;
			  stats_reads_merged_per_vault[channel][vault]++;
			  return WQ_LOOKUP_LATENCY[channel];
			}
		}

		LL_FOREACH(read_queue_head[channel][vault], rd_ptr)
		{
			if(rd_ptr->dram_addr.actual_address == physical_address)
			{
			  num_read_merge ++;
			  stats_reads_merged_per_vault[channel][vault]++;
			  return RQ_LOOKUP_LATENCY[channel];
			}
		}
	}
	return 0;
}

// Function to merge writes to the same address
int write_exists_in_write_queue(long long int physical_address, int thread_id)
{
	//get channel info
	dram_address_t * this_addr = calc_dram_addr(physical_address);
	int channel = this_addr->channel;
	int vault = this_addr->vault;
	free(this_addr);
	
	request_t * wr_ptr = NULL;

	if(channel < NUM_HMCS) {
		LL_FOREACH(write_queue_per_core_head[thread_id][channel], wr_ptr)
		{
			if(wr_ptr->dram_addr.actual_address == physical_address)
			{
			  num_write_merge ++;
			  stats_writes_merged_per_vault[channel][vault]++;
			  return 1;
			}
		}
	}
	else {
		LL_FOREACH(write_queue_head[channel][vault], wr_ptr)
		{
			if(wr_ptr->dram_addr.actual_address == physical_address)
			{
			  num_write_merge ++;
			  stats_writes_merged_per_vault[channel][vault]++;
			  return 1;
			}
		}
	}
	return 0;

}


void transfer_request_to_HMCs(int channel)
{
	request_t * transfer_request = NULL;
	optype_t this_op = READ;
	int vault = 0;

	if(CYCLE_VAL>=next_request_schedule_time[channel] && CYCLE_VAL)
	{	
		// gets next request to be transmistted through Link to HMC
		transfer_request = schedule_to_hmc(channel);
		if(transfer_request != NULL)
		{
			dram_address_t * this_addr = calc_dram_addr(transfer_request->physical_address);
			vault = this_addr->vault;
			free(this_addr);
			this_op = transfer_request->operation_type;
			
			// updating the arrival time for vault of the request to next_request_schedule_time 
			transfer_request->arrival_time = next_request_schedule_time[channel];
			
			if(this_op == READ)
			{
				LL_DELETE(read_queue_per_core_head[transfer_request->thread_id][channel],transfer_request);
				read_queue_length_for_core[transfer_request->thread_id][channel]-- ;

				LL_APPEND(read_queue_head[channel][vault], transfer_request);
				read_queue_length[channel][vault] ++;

			}
			else if(this_op == WRITE)
			{
				LL_DELETE(write_queue_per_core_head[transfer_request->thread_id][channel],transfer_request);
				write_queue_length_for_core[transfer_request->thread_id][channel]-- ;

				LL_APPEND(write_queue_head[channel][vault], transfer_request);
				write_queue_length[channel][vault] ++;
			}
			else
			{
				printf("PANIC: SCHED_ERROR : Request selected is not defined with operation types:%lld.\n", CYCLE_VAL);
			}
		}
		else
		{

		}
	}
	else
	{
			// Wait unit next_request_schedule_time to make a new transfer
	}
	//printf(" Next SCheduled time : %lld \n", next_request_schedule_time);
}

void transfer_response_to_PROCESSOR(int channel)
{
	request_t * transfer_request = NULL;

	if(CYCLE_VAL>=next_respond_schedule_time[channel]  && CYCLE_VAL)
	{	
		// gets next request to be transmistted through Link to HMC
		transfer_request = schedule_completed_requests(channel);
		if(transfer_request != NULL)
		{			
			assert(transfer_request->operation_type==READ);
			assert(transfer_request->request_served==1);
			// updating the arrival time for vault of the request to next_request_schedule_time 
			//transfer_request->arrival_time = next_request_schedule_time;
			transfer_request->request_served = 2 ;
			ROB[transfer_request->thread_id].comptime[transfer_request->instruction_id] = next_respond_schedule_time[channel] + PIPELINEDEPTH ;
		}
		else
		{
			// No responses left
		}
	}
	else
	{
			// Wait unit next_request_schedule_time to make a new transfer
	}
	//printf(" Next response scheduled time : %lld \n", next_respond_schedule_time);
}

// Insert a new read to the read queue
request_t * insert_read(long long int physical_address, long long int arrival_time, int thread_id, int instruction_id, long long int instruction_pc)
{

	optype_t this_op = READ;

	//get channel info
	dram_address_t * this_addr = calc_dram_addr(physical_address);
	int channel = this_addr->channel;
	int vault = this_addr->vault;
	free(this_addr);

	stats_reads_seen[channel][vault]++;

	request_t * new_node = init_new_node(physical_address, arrival_time, this_op, thread_id, instruction_id, instruction_pc);

	if(channel < NUM_HMCS) {
		LL_APPEND(read_queue_per_core_head[thread_id][channel], new_node);
		read_queue_length_for_core[thread_id][channel]++;
	}
	else {
		LL_APPEND(read_queue_head[channel][vault], new_node);
		read_queue_length[channel][vault]++;
	}

	//UT_MEM_DEBUG("\nCyc: %lld New READ:%lld Core:%d Chan:%d Rank:%d Bank:%d Row:%lld RD_Q_Length:%lld\n", CYCLE_VAL, new_node->id, new_node->thread_id, new_node->dram_addr.channel,  new_node->dram_addr.rank,  new_node->dram_addr.bank,  new_node->dram_addr.row, read_queue_length[channel]);
	
	return new_node;
}

// Insert a new write to the write queue
request_t * insert_write(long long int physical_address, long long int arrival_time, int thread_id, int instruction_id)
{
	optype_t this_op = WRITE;

	dram_address_t * this_addr = calc_dram_addr(physical_address);
	int channel = this_addr->channel;
	int vault = this_addr->vault;
	free(this_addr);

	stats_writes_seen[channel][vault]++;

	request_t * new_node = init_new_node(physical_address, arrival_time, this_op, thread_id, instruction_id, 0);

	if(channel < NUM_HMCS) {
		LL_APPEND(write_queue_per_core_head[thread_id][channel], new_node);
		write_queue_length_for_core[thread_id][channel]++;
	}
	else {
		LL_APPEND(write_queue_head[channel][vault], new_node);
		write_queue_length[channel][vault]++;
	}

	//UT_MEM_DEBUG("\nCyc: %lld New WRITE:%lld Core:%d Chan:%d Rank:%d Bank:%d Row:%lld WR_Q_Length:%lld\n", CYCLE_VAL, new_node->id, new_node->thread_id, new_node->dram_addr.channel,  new_node->dram_addr.rank,  new_node->dram_addr.bank,  new_node->dram_addr.row, write_queue_length[channel]);

	return new_node;
}

// Function to update the states of the read queue requests.
// Each DRAM cycle, this function iterates over the read queue and
// updates the next_command and command_issuable fields to mark which
// commands can be issued this cycle
void update_read_queue_commands(int channel, int vault)
{
	request_t * curr = NULL;

	LL_FOREACH(read_queue_head[channel][vault],curr)
	{
		// ignore the requests whose completion time has been determined
		// these requests will be removed this very cycle 
		if(curr->request_served == 1)
			continue; 

		int bank = curr->dram_addr.bank;

		int rank = curr->dram_addr.rank;

		int row = curr->dram_addr.row;

		switch (dram_state[channel][vault][rank][bank].state)
		{
		  // if the DRAM bank has no rows open and the chip is
		  // powered up, the next command for the request
		  // should be ACT.
			case IDLE:
			case PRECHARGING:
			case REFRESHING:

			  
				curr->next_command = ACT_CMD;


				if(CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_act && is_T_FAW_met(channel, vault, rank, CYCLE_VAL))
					curr->command_issuable = 1;
				else
					curr->command_issuable = 0;
				
				// check if we are in OR too close to the forced refresh period
				if(forced_refresh_mode_on[channel][vault][rank] || ((CYCLE_VAL + T_RAS[channel]) > refresh_issue_deadline[channel][vault][rank]))
					curr->command_issuable = 0;
				break;

			case ROW_ACTIVE:

				// if the bank is active then check if this is a row-hit or not
				// If the request is to the currently
				// opened row, the next command should
				// be a COL_RD, else it should be a
				// PRECHARGE
				if(row == dram_state[channel][vault][rank][bank].active_row)
				{
					curr->next_command = COL_READ_CMD;
					
					if(CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_read)
						curr->command_issuable = 1;
					else
						curr->command_issuable = 0;
					
					if(forced_refresh_mode_on[channel][vault][rank] ||((CYCLE_VAL + T_RTP[channel]) > refresh_issue_deadline[channel][vault][rank]))
						curr->command_issuable = 0;
				}
				else
				{
					curr->next_command = PRE_CMD;

					if(CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_pre)
						curr->command_issuable = 1;
					else
						curr->command_issuable = 0;
					
					if(forced_refresh_mode_on[channel][vault][rank]|| ((CYCLE_VAL+T_RP[channel]) > refresh_issue_deadline[channel][vault][rank]))
						curr->command_issuable = 0;

				}
				
				break;
				// if the chip was powered, down the
				// next command required is power_up

			case PRECHARGE_POWER_DOWN_SLOW :
			case PRECHARGE_POWER_DOWN_FAST:
			case ACTIVE_POWER_DOWN :

				curr->next_command = PWR_UP_CMD;

				if(CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_powerup)
					curr->command_issuable = 1;
				else
					curr->command_issuable=0;
				
				if((dram_state[channel][vault][rank][bank].state == PRECHARGE_POWER_DOWN_SLOW) && ((CYCLE_VAL + T_XP_DLL[channel]) > refresh_issue_deadline[channel][vault][rank] ))
					curr->command_issuable = 0;
				else if(((dram_state[channel][vault][rank][bank].state == PRECHARGE_POWER_DOWN_FAST) || (dram_state[channel][vault][rank][bank].state == ACTIVE_POWER_DOWN)) && ((CYCLE_VAL + T_XP[channel]) > refresh_issue_deadline[channel][vault][rank] ))
					curr->command_issuable = 0;

				break;


			default : break;
		}
	}
}

// Similar to update_read_queue above, but for write queue
void update_write_queue_commands(int channel, int vault)
{
	request_t * curr = NULL;

	LL_FOREACH(write_queue_head[channel][vault], curr)
	{

		if(curr->request_served == 2 && channel < NUM_HMCS)
			continue; 
		else if(curr->request_served == 1 && channel >= NUM_HMCS)
			continue;
		
		int bank = curr->dram_addr.bank;

		int rank = curr->dram_addr.rank;

		int row = curr->dram_addr.row;

		switch (dram_state[channel][vault][rank][bank].state)
		{
			case IDLE:
			case PRECHARGING:
			case REFRESHING:
				curr->next_command = ACT_CMD;

				if(CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_act && is_T_FAW_met(channel, vault, rank, CYCLE_VAL))
					curr->command_issuable = 1;
				else
					curr->command_issuable = 0;
				
				// check if we are in or too close to the forced refresh period
				if(forced_refresh_mode_on[channel][vault][rank] || ((CYCLE_VAL + T_RAS[channel]) > refresh_issue_deadline[channel][vault][rank]))
					curr->command_issuable = 0;

				break;


			case ROW_ACTIVE:

				if(row == dram_state[channel][vault][rank][bank].active_row)
				{
					curr->next_command = COL_WRITE_CMD;

					if(CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_write)
						curr->command_issuable = 1;
					else
						curr->command_issuable = 0;

					if(forced_refresh_mode_on[channel][vault][rank]|| ((CYCLE_VAL+T_CWD[channel]+T_DATA_TRANS[channel]+T_WR[channel]) > refresh_issue_deadline[channel][vault][rank]))
						curr->command_issuable = 0;
				}
				else
				{
					curr->next_command = PRE_CMD;

					if(CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_pre)
						curr->command_issuable = 1;
					else
						curr->command_issuable = 0;

					if(forced_refresh_mode_on[channel][vault][rank]|| ((CYCLE_VAL+T_RP[channel]) > refresh_issue_deadline[channel][vault][rank]))
						curr->command_issuable = 0;

				}
				
				
				break;

			case PRECHARGE_POWER_DOWN_SLOW:
			case PRECHARGE_POWER_DOWN_FAST:
			case ACTIVE_POWER_DOWN :

				curr->next_command = PWR_UP_CMD;

				if(CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_powerup)
					curr->command_issuable = 1;
				else
					curr->command_issuable = 0;

				if(forced_refresh_mode_on[channel][vault][rank])
					curr->command_issuable= 0;

				if((dram_state[channel][vault][rank][bank].state == PRECHARGE_POWER_DOWN_SLOW) && ((CYCLE_VAL + T_XP_DLL[channel]) > refresh_issue_deadline[channel][vault][rank] ))
					curr->command_issuable = 0;
				else if(((dram_state[channel][vault][rank][bank].state == PRECHARGE_POWER_DOWN_FAST) || (dram_state[channel][vault][rank][bank].state == ACTIVE_POWER_DOWN)) && ((CYCLE_VAL + T_XP[channel]) > refresh_issue_deadline[channel][vault][rank] ))
					curr->command_issuable = 0;

				break;

			default : break;
		}
	}
}

// Remove finished requests from the queues.
void clean_queues(int channel, int vault)
{

	request_t * rd_ptr =  NULL;
	request_t * rd_tmp = NULL;
	request_t * wrt_ptr = NULL;
	request_t * wrt_tmp = NULL;

	// Delete all READ requests whose completion time has been determined i.e. COL_RD has been issued
	if(channel < NUM_HMCS) {
		LL_FOREACH_SAFE(read_return_queue_head[channel][vault],rd_ptr,rd_tmp) 
		{
			if(rd_ptr->request_served == 2)
			{
				assert(rd_ptr->next_command == COL_READ_CMD);
				assert(rd_ptr->completion_time != -100);
				LL_DELETE(read_return_queue_head[channel][vault],rd_ptr);
				if(rd_ptr->user_ptr)
					free(rd_ptr->user_ptr);

				free(rd_ptr);
				read_return_queue_length[channel][vault]--;
				assert(read_return_queue_length[channel][vault]>=0);
			}
		}
	}
	else {
		LL_FOREACH_SAFE(read_queue_head[channel][vault],rd_ptr,rd_tmp) 
		{
			if(rd_ptr->request_served == 1)
			{
				assert(rd_ptr->next_command == COL_READ_CMD);
				assert(rd_ptr->completion_time != -100);
				LL_DELETE(read_queue_head[channel][vault],rd_ptr);
				if(rd_ptr->user_ptr)
					free(rd_ptr->user_ptr);

				free(rd_ptr);
				read_queue_length[channel][vault]--;
				assert(read_queue_length[channel][vault]>=0);
			}
		}
	}

	// Delete all WRITE requests whose completion time has been determined i.e COL_WRITE has been issued
	LL_FOREACH_SAFE(write_queue_head[channel][vault],wrt_ptr,wrt_tmp) 
	{
		if((wrt_ptr->request_served == 2 && channel < NUM_HMCS) || (wrt_ptr->request_served == 1 && channel >= NUM_HMCS))
		{
			assert(wrt_ptr->next_command == COL_WRITE_CMD);

			LL_DELETE(write_queue_head[channel][vault],wrt_ptr);

			if(wrt_ptr->user_ptr)
				free(wrt_ptr->user_ptr);

			free(wrt_ptr);

			write_queue_length[channel][vault]--;

			assert(write_queue_length[channel][vault]>=0);
		}
	}
}

// function to update a completed read request to read return queue of each vault 
void update_read_return_queue(int channel, int vault)
{
	request_t * request = NULL;
	
	LL_FOREACH(read_queue_head[channel][vault],request)
	{
		if(request->request_served == 1 && (CYCLE_VAL >= request->completion_time))
		{
			assert(request->next_command == COL_READ_CMD);
	
			assert(request->completion_time != -100);
		
			LL_DELETE(read_queue_head[channel][vault],request);
		
			read_queue_length[channel][vault]--;
		
			assert(read_queue_length[channel][vault]>=0);
		
			LL_APPEND(read_return_queue_head[channel][vault],request);
		
			read_return_queue_length[channel][vault]++;

			//printf(" Updated Read Return queue with hmc id %d vault %d \n", hmc , vault );
		}
	}
}


// This affects state change
// Issue a valid command for a request in either the read or write
// queue.
// Upon issuing the request, the dram_state is changed and the
// next_"cmd" variables are updated to indicate when the next "cmd"
// can be issued to each bank
int issue_request_command(request_t * request) 
{
	long long int cycle =  CYCLE_VAL;
	if(request->command_issuable != 1 || command_issued_current_cycle[request->dram_addr.channel][request->dram_addr.vault] || CYCLE_VAL < request->arrival_time)
	{
		printf("PANIC: SCHED_ERROR : Command for request selected can not be issued in  cycle:%lld.\n", CYCLE_VAL);
		return 0;
	}

	int channel = request->dram_addr.channel;
	int vault = request->dram_addr.vault;
	int rank = request->dram_addr.rank;
	int bank = request->dram_addr.bank;
	long long int row = request->dram_addr.row;
	command_t cmd = request->next_command;

	switch(cmd)
	{
		case ACT_CMD :

			assert(dram_state[channel][vault][rank][bank].state == PRECHARGING || dram_state[channel][vault][rank][bank].state == IDLE || dram_state[channel][vault][rank][bank].state == REFRESHING);

			//UT_MEM_DEBUG("\nCycle: %lld Cmd:ACT Req:%lld Chan:%d Rank:%d Bank:%d Row:%lld\n", CYCLE_VAL, request->id, channel, vault, rank, bank, row);

			// open row
			dram_state[channel][vault][rank][bank].state = ROW_ACTIVE;

			dram_state[channel][vault][rank][bank].active_row = row;

			dram_state[channel][vault][rank][bank].next_pre = max((cycle + T_RAS[channel]) , dram_state[channel][vault][rank][bank].next_pre);
			
			dram_state[channel][vault][rank][bank].next_refresh = max((cycle + T_RAS[channel]) , dram_state[channel][vault][rank][bank].next_refresh);

			dram_state[channel][vault][rank][bank].next_read = max(cycle + T_RCD[channel], dram_state[channel][vault][rank][bank].next_read); 

			dram_state[channel][vault][rank][bank].next_write = max(cycle + T_RCD[channel],  dram_state[channel][vault][rank][bank].next_write);

			dram_state[channel][vault][rank][bank].next_act = max(cycle + T_RC[channel],  dram_state[channel][vault][rank][bank].next_act);

			dram_state[channel][vault][rank][bank].next_powerdown = max(cycle + T_RCD[channel], dram_state[channel][vault][rank][bank].next_powerdown);

			for(int i=0;i<NUM_BANKS[channel];i++)
				if(i!=bank)
					dram_state[channel][vault][rank][i].next_act = max(cycle+T_RRD[channel], dram_state[channel][vault][rank][i].next_act);

			record_activate(channel, vault, rank, cycle);

			if(request->operation_type == READ)
				stats_num_activate_read[channel][vault][rank][bank]++;
			else
				stats_num_activate_write[channel][vault][rank][bank]++;

			stats_num_activate[channel][vault][rank]++;

			average_gap_between_activates[channel][vault][rank] = ((average_gap_between_activates[channel][vault][rank]*(stats_num_activate[channel][vault][rank]-1)) + (CYCLE_VAL-last_activate[channel][vault][rank]))/stats_num_activate[channel][vault][rank];

			last_activate[channel][vault][rank] = CYCLE_VAL;

			command_issued_current_cycle[channel][vault] = 1;
			break;

		case COL_READ_CMD :

			assert(dram_state[channel][vault][rank][bank].state == ROW_ACTIVE) ;

			dram_state[channel][vault][rank][bank].next_pre = max(cycle + T_RTP[channel] , dram_state[channel][vault][rank][bank].next_pre);
			
			dram_state[channel][vault][rank][bank].next_refresh = max(cycle + T_RTP[channel] , dram_state[channel][vault][rank][bank].next_refresh);

			dram_state[channel][vault][rank][bank].next_powerdown = max (cycle+T_RTP[channel], dram_state[channel][vault][rank][bank].next_powerdown);

			for(int i=0;i<NUM_RANKS[channel];i++)
			{
				for(int j=0;j<NUM_BANKS[channel];j++)
				{
					if(i!=rank)
						dram_state[channel][vault][i][j].next_read = max(cycle+ T_DATA_TRANS[channel] + T_RTRS[channel], dram_state[channel][vault][i][j].next_read);

					else
						dram_state[channel][vault][i][j].next_read = max(cycle + max(T_CCD[channel], T_DATA_TRANS[channel]), dram_state[channel][vault][i][j].next_read); 

					dram_state[channel][vault][i][j].next_write = max(cycle + T_CAS[channel]+ T_DATA_TRANS[channel] + T_RTRS[channel]- T_CWD[channel] ,  dram_state[channel][vault][i][j].next_write);
				}
			}

			// set the completion time of this read request
			// in the ROB and the controller queue.
			request->completion_time = CYCLE_VAL+ T_CAS[channel] + T_DATA_TRANS[channel] ;
			request->latency = request->completion_time - request->arrival_time;
			request->dispatch_time = CYCLE_VAL;
			request->request_served = 1;

			// update the ROB with the completion time
			if(channel >= NUM_HMCS)
				ROB[request->thread_id].comptime[request->instruction_id] = request->completion_time+PIPELINEDEPTH;

			stats_reads_completed[channel][vault]++;
			stats_average_read_latency[channel][vault] = ((stats_reads_completed[channel][vault]-1)*stats_average_read_latency[channel][vault] + request->latency)/stats_reads_completed[channel][vault];
			stats_average_read_queue_latency[channel][vault] = ((stats_reads_completed[channel][vault]-1)*stats_average_read_queue_latency[channel][vault] + (request->dispatch_time - request->arrival_time))/stats_reads_completed[channel][vault];
			//UT_MEM_DEBUG("Req:%lld finishes at Cycle: %lld\n", request->id, request->completion_time);

			//printf("Cycle: %10lld, Reads  Completed = %5lld, this_latency= %5lld, latency = %f\n", CYCLE_VAL, stats_reads_completed[channel][vault], request->latency, stats_average_read_latency[channel][vault]);	

			stats_num_read[channel][vault][rank][bank]++;

			for(int i=0; i<NUM_RANKS[channel] ;i++)
			{
				if(i!=rank)
					stats_time_spent_terminating_reads_from_other_ranks[channel][vault][i] += T_DATA_TRANS[channel];
			}

			command_issued_current_cycle[channel][vault] = 1;
			cas_issued_current_cycle[channel][vault][rank][bank]=1;
			break;

		case COL_WRITE_CMD :


			assert(dram_state[channel][vault][rank][bank].state == ROW_ACTIVE);

			//UT_MEM_DEBUG("\nCycle: %lld Cmd: COL_WRITE Req:%lld Chan:%d Rank:%d Bank:%d \n", CYCLE_VAL, request->id, channel, vault, rank, bank);

			dram_state[channel][vault][rank][bank].next_pre = max(cycle + T_CWD[channel] +T_DATA_TRANS[channel] + T_WR[channel], dram_state[channel][vault][rank][bank].next_pre);
			
			dram_state[channel][vault][rank][bank].next_refresh = max(cycle + T_CWD[channel] +T_DATA_TRANS[channel] + T_WR[channel], dram_state[channel][vault][rank][bank].next_refresh);

			dram_state[channel][vault][rank][bank].next_powerdown = max (cycle + T_CWD[channel] + T_DATA_TRANS[channel] + T_WR[channel], dram_state[channel][vault][rank][bank].next_powerdown);

			for(int i=0;i<NUM_RANKS[channel];i++)
			{
				for(int j=0;j<NUM_BANKS[channel];j++)
				{
					if(i!=rank)
					{
						dram_state[channel][vault][i][j].next_write = max(cycle + T_DATA_TRANS[channel] + T_RTRS[channel], dram_state[channel][vault][i][j].next_write);

						dram_state[channel][vault][i][j].next_read = max(cycle + T_CWD[channel] + T_DATA_TRANS[channel] + T_RTRS[channel] - T_CAS[channel], dram_state[channel][vault][i][j].next_read);
					}
					else
					{
						dram_state[channel][vault][i][j].next_write = max(cycle + max(T_CCD[channel], T_DATA_TRANS[channel]), dram_state[channel][vault][i][j].next_write); 

						dram_state[channel][vault][i][j].next_read = max(cycle + T_CWD[channel] + T_DATA_TRANS[channel] + T_WTR[channel] ,  dram_state[channel][vault][i][j].next_read);
					}
				}
			}

			// set the completion time of this write request
			request->completion_time = CYCLE_VAL+ T_DATA_TRANS[channel] + T_WR[channel];
			request->latency = request->completion_time - request->arrival_time;
			request->dispatch_time = CYCLE_VAL;
			request->request_served = (channel<NUM_HMCS)?2:1;

			stats_writes_completed[channel][vault]++;

			stats_num_write[channel][vault][rank][bank]++;
			
			stats_average_write_latency[channel][vault] = ((stats_writes_completed[channel][vault]-1)*stats_average_write_latency[channel][vault] + request->latency)/stats_writes_completed[channel][vault];
			stats_average_write_queue_latency[channel][vault] = ((stats_writes_completed[channel][vault]-1)*stats_average_write_queue_latency[channel][vault] + (request->dispatch_time - request->arrival_time))/stats_writes_completed[channel][vault];
			//UT_MEM_DEBUG("Req:%lld finishes at Cycle: %lld\n", request->id, request->completion_time);

			//printf("Cycle: %10lld, Writes Completed = %5lld, this_latency= %5lld, latency = %f\n", CYCLE_VAL, stats_writes_completed[channel][vault], request->latency, stats_average_write_latency[channel][vault]);	


			for(int i=0; i<NUM_RANKS[channel] ;i++)
			{
				if(i!=rank)
					stats_time_spent_terminating_writes_to_other_ranks[channel][vault][i] += T_DATA_TRANS[channel];
			}

			command_issued_current_cycle[channel][vault] = 1;
			cas_issued_current_cycle[channel][vault][rank][bank] = 2;
			break;

		case PRE_CMD :

			assert(dram_state[channel][vault][rank][bank].state == ROW_ACTIVE || dram_state[channel][vault][rank][bank].state == PRECHARGING || dram_state[channel][vault][rank][bank].state == IDLE || dram_state[channel][vault][rank][bank].state ==  REFRESHING) ;

			//UT_MEM_DEBUG("\nCycle: %lld Cmd:PRE Req:%lld Chan:%d Rank:%d Bank:%d \n", CYCLE_VAL, request->id, channel, vault, rank, bank);

			dram_state[channel][vault][rank][bank].state = PRECHARGING ;

			dram_state[channel][vault][rank][bank].active_row = -1;

			dram_state[channel][vault][rank][bank].next_act = max(cycle+T_RP[channel], dram_state[channel][vault][rank][bank].next_act);

			dram_state[channel][vault][rank][bank].next_powerdown = max(cycle+T_RP[channel], dram_state[channel][vault][rank][bank].next_powerdown);

			dram_state[channel][vault][rank][bank].next_pre = max(cycle+T_RP[channel], dram_state[channel][vault][rank][bank].next_pre);

			dram_state[channel][vault][rank][bank].next_refresh = max(cycle+T_RP[channel], dram_state[channel][vault][rank][bank].next_refresh);

			stats_num_precharge[channel][vault][rank][bank] ++;

			command_issued_current_cycle[channel][vault] = 1;

			break;

		case PWR_UP_CMD :

			assert(dram_state[channel][vault][rank][bank].state == PRECHARGE_POWER_DOWN_SLOW || dram_state[channel][vault][rank][bank].state == PRECHARGE_POWER_DOWN_FAST || dram_state[channel][vault][rank][bank].state == ACTIVE_POWER_DOWN) ;

			//UT_MEM_DEBUG("\nCycle: %lld Cmd: PWR_UP_CMD Chan:%d Rank:%d \n", CYCLE_VAL, channel, vault, rank);

			for(int i=0; i<NUM_BANKS[channel] ; i++)
			{

				if(dram_state[channel][vault][rank][i].state == PRECHARGE_POWER_DOWN_SLOW || dram_state[channel][vault][rank][i].state == PRECHARGE_POWER_DOWN_FAST)
				{
					dram_state[channel][vault][rank][i].state = IDLE;
					dram_state[channel][vault][rank][i].active_row = -1;
				}
				else
				{
					dram_state[channel][vault][rank][i].state = ROW_ACTIVE;
				}

				if(dram_state[channel][vault][rank][i].state == PRECHARGE_POWER_DOWN_SLOW)
				{
					dram_state[channel][vault][rank][i].next_powerdown = max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_powerdown);

					dram_state[channel][vault][rank][i].next_pre= max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_pre);

					dram_state[channel][vault][rank][i].next_read = max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_read);

					dram_state[channel][vault][rank][i].next_write = max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_write);

					dram_state[channel][vault][rank][i].next_act = max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_act);

					dram_state[channel][vault][rank][i].next_refresh = max(cycle + T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_refresh);
				}
				else
				{

					dram_state[channel][vault][rank][i].next_powerdown = max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_powerdown);

					dram_state[channel][vault][rank][i].next_pre= max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_pre);

					dram_state[channel][vault][rank][i].next_read = max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_read);

					dram_state[channel][vault][rank][i].next_write = max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_write);

					dram_state[channel][vault][rank][i].next_act = max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_act);
					
					dram_state[channel][vault][rank][i].next_refresh = max(cycle + T_XP[channel], dram_state[channel][vault][rank][i].next_refresh);
				}
			}

			stats_num_powerup[channel][vault][rank]++;
			command_issued_current_cycle[channel][vault] =1;

			break ;
		case NOP :

			//UT_MEM_DEBUG("\nCycle: %lld Cmd: NOP Chan:%d\n", CYCLE_VAL, channel);
			break;

		default :
			break;
	}
	return 1;
}

int are_all_writes_completed()
{
	for(int channel=0; channel<NUM_CHANNELS; channel++)
	{
		if(channel<NUM_HMCS) {
			for(int core=0; core<NUMCORES; core++) {
				if(write_queue_length_for_core[core][channel])
					return 0;
			}
		}
		for(int vault=0; vault<NUM_VAULTS[channel]; vault++)
		{
				if(write_queue_length[channel][vault])
					return 0;
		}
	}
	
	return 1;
}

// Function called to see if the rank can be transitioned into a fast low
// power state - ACT_PDN or PRE_PDN_FAST. 
int is_powerdown_fast_allowed(int channel, int vault, int rank)
{
	int flag =0;

	// if already a command has been issued this cycle, or if
	// forced refreshes are underway, or if issuing this command
	// will cause us to miss the refresh deadline, do not allow it
	if (command_issued_current_cycle[channel][vault] || forced_refresh_mode_on[channel][vault][rank] || (CYCLE_VAL +T_PD_MIN[channel] + T_XP[channel] > refresh_issue_deadline[channel][vault][rank])) 
	  return 0;

	// command can be allowed if the next_powerdown is met for all banks in the rank
	for(int i =0; i < NUM_BANKS[channel] ; i++)
	{
	  if((dram_state[channel][vault][rank][i].state == PRECHARGING || dram_state[channel][vault][rank][i].state == ROW_ACTIVE || dram_state[channel][vault][rank][i].state == IDLE || dram_state[channel][vault][rank][i].state == REFRESHING) && CYCLE_VAL >= dram_state[channel][vault][rank][i].next_powerdown)
	    flag = 1;
	  else
	    return 0;
	}

	return flag;
}

// Function to see if the rank can be transitioned into a slow low
// power state - i.e. PRE_PDN_SLOW
int is_powerdown_slow_allowed(int channel, int vault, int rank)
{
	int flag =0;

	if (command_issued_current_cycle[channel][vault] || forced_refresh_mode_on[channel][vault][rank] || (CYCLE_VAL +T_PD_MIN[channel]+T_XP_DLL[channel] > refresh_issue_deadline[channel][vault][rank])) 
	  return 0;

	// Sleep command can be allowed if the next_powerdown is met for all banks in the rank
	// and if all the banks are precharged
	for(int i =0; i < NUM_BANKS[channel] ; i++)
	{
	  if(dram_state[channel][vault][rank][i].state == ROW_ACTIVE)
	    return 0;
	  else
	  {
	    if((dram_state[channel][vault][rank][i].state == PRECHARGING || dram_state[channel][vault][rank][i].state == IDLE || dram_state[channel][vault][rank][i].state == REFRESHING) && CYCLE_VAL >= dram_state[channel][vault][rank][i].next_powerdown)
	      flag = 1;
	    else
	      return 0;
	  }
	}
	return flag;
}

// Function to see if the rank can be powered up
int is_powerup_allowed(int channel, int vault, int rank)
{
	if (command_issued_current_cycle[channel][vault] || forced_refresh_mode_on[channel][vault][rank])
	  return 0;

	if(((dram_state[channel][vault][rank][0].state == PRECHARGE_POWER_DOWN_SLOW) ||(dram_state[channel][vault][rank][0].state == PRECHARGE_POWER_DOWN_FAST) || (dram_state[channel][vault][rank][0].state == ACTIVE_POWER_DOWN)) && (CYCLE_VAL >= dram_state[channel][vault][rank][0].next_powerup))
	{
	  // check if issuing it will cause us to miss the refresh
	  // deadline. If it does, don't allow it. The forced
	  // refreshes will issue an implicit power up anyway
		if((dram_state[channel][vault][rank][0].state == PRECHARGE_POWER_DOWN_SLOW) && ((CYCLE_VAL + T_XP_DLL[channel]) > refresh_issue_deadline[channel][vault][0]))
			return 0;
		if(((dram_state[channel][vault][rank][0].state == PRECHARGE_POWER_DOWN_FAST) || (dram_state[channel][vault][rank][0].state == ACTIVE_POWER_DOWN)) && (( CYCLE_VAL +T_XP[channel]) > refresh_issue_deadline[channel][vault][0]))
			return 0;
		return 1;
	}
	else
		return 0;
}

// Function to see if the bank can be activated or not
int is_activate_allowed(int channel, int vault, int rank, int bank)
{
	if (command_issued_current_cycle[channel][vault] || forced_refresh_mode_on[channel][vault][rank] || (CYCLE_VAL + T_RAS[channel] > refresh_issue_deadline[channel][vault][rank])) 
	  return 0;
	if ((dram_state[channel][vault][rank][bank].state == IDLE || dram_state[channel][vault][rank][bank].state == PRECHARGING || dram_state[channel][vault][rank][bank].state == REFRESHING) && (CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_act) && (is_T_FAW_met(channel,vault,rank,CYCLE_VAL)))
	  return 1;
	else 
	  return 0;
}

// Function to see if the rank can be precharged or not
int is_autoprecharge_allowed(int channel, int vault, int rank, int bank)
{
  long long int start_precharge = 0;
  if(cas_issued_current_cycle[channel][vault][rank][bank] == 1)
    start_precharge = max(CYCLE_VAL + T_RTP[channel], dram_state[channel][vault][rank][bank].next_pre);
  else
    start_precharge = max(CYCLE_VAL + T_CWD[channel] + T_DATA_TRANS[channel] + T_WR[channel], dram_state[channel][vault][rank][bank].next_pre);
  
  if(((cas_issued_current_cycle[channel][vault][rank][bank] == 1) && ((start_precharge+T_RP[channel]) <= refresh_issue_deadline[channel][vault][rank])) ||((cas_issued_current_cycle[channel][vault][rank][bank] == 2)&& ((start_precharge+T_RP[channel]) <= refresh_issue_deadline[channel][vault][rank])))
    return 1;
  else
    return 0;
}


// Function to see if the rank can be precharged or not
int is_precharge_allowed(int channel, int vault, int rank, int bank)
{
	if (command_issued_current_cycle[channel][vault] || forced_refresh_mode_on[channel][vault][rank] || (CYCLE_VAL + T_RP[channel] > refresh_issue_deadline[channel][vault][rank])) 
	    return 0;

	if((dram_state[channel][vault][rank][bank].state == ROW_ACTIVE || dram_state[channel][vault][rank][bank].state == IDLE || dram_state[channel][vault][rank][bank].state == PRECHARGING || dram_state[channel][vault][rank][bank].state ==  REFRESHING) && ( CYCLE_VAL >= dram_state[channel][vault][rank][bank].next_pre))
	  return 1;
	else
	  return 0;
}


// function to see if all banks can be precharged this cycle
int is_all_bank_precharge_allowed(int channel, int vault, int rank)
{
  	int flag = 0;
	if (command_issued_current_cycle[channel][vault] || forced_refresh_mode_on[channel][vault][rank] || (CYCLE_VAL + T_RP[channel] > refresh_issue_deadline[channel][vault][rank])) 
	  return 0;

	for(int i =0; i<NUM_BANKS[channel] ;i++)
	{
	  if((dram_state[channel][vault][rank][i].state == ROW_ACTIVE || dram_state[channel][vault][rank][i].state == IDLE || dram_state[channel][vault][rank][i].state == PRECHARGING || dram_state[channel][vault][rank][i].state ==  REFRESHING) && ( CYCLE_VAL >= dram_state[channel][vault][rank][i].next_pre))
	    flag = 1;
	  else
	    return 0;
	}
	return flag;
}

// function to see if refresh can be allowed this cycle

int is_refresh_allowed(int channel, int vault, int rank)
{
	if (command_issued_current_cycle[channel][vault] || forced_refresh_mode_on[channel][vault][rank]) 
	  return 0;

	for(int b=0; b< NUM_BANKS[channel]; b++)
	{
		if(CYCLE_VAL < dram_state[channel][vault][rank][b].next_refresh)
			return 0;
	}
	return 1;
}

// Function to put a rank into the low power mode
int issue_powerdown_command(int channel, int vault, int rank, command_t cmd)
{
	if(command_issued_current_cycle[channel][vault]) {
		printf("PANIC : SCHED_ERROR: Got beat. POWER_DOWN command not issuable in cycle:%lld\n", CYCLE_VAL);
		return 0;
	}

	// if right CMD has been used
	if ((cmd != PWR_DN_FAST_CMD) && (cmd != PWR_DN_SLOW_CMD)) {
		printf("PANIC: SCHED_ERROR : Only PWR_DN_SLOW_CMD or PWR_DN_FAST_CMD can be used to put DRAM rank to sleep\n");
		return 0;
	}
	// if the powerdown command can indeed be issued
	if(((cmd == PWR_DN_FAST_CMD) && !is_powerdown_fast_allowed(channel, vault, rank)) || ((cmd == PWR_DN_SLOW_CMD) && !is_powerdown_slow_allowed(channel, vault, rank))) {
		printf("PANIC : SCHED_ERROR: POWER_DOWN command not issuable in cycle:%lld\n", CYCLE_VAL);
		return 0;
	}

	for(int i=0; i<NUM_BANKS[channel] ; i++)
	{
	        // next_powerup and refresh times
		dram_state[channel][vault][rank][i].next_powerup = max(CYCLE_VAL+T_PD_MIN[channel], dram_state[channel][vault][rank][i].next_powerdown);
		dram_state[channel][vault][rank][i].next_refresh = max(CYCLE_VAL+T_PD_MIN[channel], dram_state[channel][vault][rank][i].next_refresh);
		
		// state change
		if(dram_state[channel][vault][rank][i].state == IDLE || dram_state[channel][vault][rank][i].state == PRECHARGING || dram_state[channel][vault][rank][i].state == REFRESHING)
		{
			if(cmd == PWR_DN_SLOW_CMD)
			{
				dram_state[channel][vault][rank][i].state= PRECHARGE_POWER_DOWN_SLOW;
				stats_num_powerdown_slow[channel][vault][rank]++;
			}
			else if(cmd == PWR_DN_FAST_CMD)
			{
				dram_state[channel][vault][rank][i].state = PRECHARGE_POWER_DOWN_FAST;
				stats_num_powerdown_fast[channel][vault][rank]++;
			}

			dram_state[channel][vault][rank][i].active_row = -1;
		}
		else if(dram_state[channel][vault][rank][i].state == ROW_ACTIVE)
		{
			dram_state[channel][vault][rank][i].state = ACTIVE_POWER_DOWN;
		}
	}
	command_issued_current_cycle[channel][vault] = 1;
	return 1;
}


// Function to power a rank up
int issue_powerup_command(int channel, int vault, int rank)
{
	if(!is_powerup_allowed(channel, vault, rank)) 
	{
		printf("PANIC : SCHED_ERROR: POWER_UP command not issuable in cycle:%lld\n", CYCLE_VAL);
		return 0;
	}
	else
	{
		long long int cycle =  CYCLE_VAL;
		for(int i=0; i<NUM_BANKS[channel] ; i++)
		{

			if(dram_state[channel][vault][rank][i].state == PRECHARGE_POWER_DOWN_SLOW || dram_state[channel][vault][rank][i].state == PRECHARGE_POWER_DOWN_FAST)
			{
				dram_state[channel][vault][rank][i].state = IDLE;
				dram_state[channel][vault][rank][i].active_row = -1;
			}
			else
			{
				dram_state[channel][vault][rank][i].state = ROW_ACTIVE;
			}

			if(dram_state[channel][vault][rank][i].state == PRECHARGE_POWER_DOWN_SLOW)
			{
				dram_state[channel][vault][rank][i].next_powerdown = max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_powerdown);

				dram_state[channel][vault][rank][i].next_pre= max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_pre);

				dram_state[channel][vault][rank][i].next_read = max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_read);

				dram_state[channel][vault][rank][i].next_write = max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_write);

				dram_state[channel][vault][rank][i].next_act = max(cycle+T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_act);

				dram_state[channel][vault][rank][i].next_refresh = max(cycle + T_XP_DLL[channel], dram_state[channel][vault][rank][i].next_refresh);
			}
			else
			{

				dram_state[channel][vault][rank][i].next_powerdown = max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_powerdown);

				dram_state[channel][vault][rank][i].next_pre= max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_pre);

				dram_state[channel][vault][rank][i].next_read = max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_read);

				dram_state[channel][vault][rank][i].next_write = max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_write);

				dram_state[channel][vault][rank][i].next_act = max(cycle+T_XP[channel], dram_state[channel][vault][rank][i].next_act);

				dram_state[channel][vault][rank][i].next_refresh = max(cycle + T_XP[channel], dram_state[channel][vault][rank][i].next_refresh);
			}
		}

		command_issued_current_cycle[channel][vault] = 1;
		return 1;

	}
}

// Function to issue a precharge command to a specific bank
int issue_autoprecharge(int channel, int vault, int rank, int bank)
{
  if(!is_autoprecharge_allowed(channel,vault,rank,bank))
    return 0;
  else
  {
    long long int start_precharge = 0;

    dram_state[channel][vault][rank][bank].active_row = -1;

    dram_state[channel][vault][rank][bank].state = PRECHARGING;

    if(cas_issued_current_cycle[channel][vault][rank][bank] == 1)
      start_precharge = max(CYCLE_VAL + T_RTP[channel], dram_state[channel][vault][rank][bank].next_pre);
    else
      start_precharge = max(CYCLE_VAL + T_CWD[channel] + T_DATA_TRANS[channel] + T_WR[channel], dram_state[channel][vault][rank][bank].next_pre);

    dram_state[channel][vault][rank][bank].next_act = max(start_precharge + T_RP[channel], dram_state[channel][vault][rank][bank].next_act);

    dram_state[channel][vault][rank][bank].next_powerdown = max(start_precharge + T_RP[channel], dram_state[channel][vault][rank][bank].next_powerdown);

    dram_state[channel][vault][rank][bank].next_pre = max(start_precharge + T_RP[channel], dram_state[channel][vault][rank][bank].next_pre);

    dram_state[channel][vault][rank][bank].next_refresh = max(start_precharge + T_RP[channel], dram_state[channel][vault][rank][bank].next_refresh);

    stats_num_precharge[channel][vault][rank][bank] ++;

    // reset the cas_issued_current_cycle 
    for(int r = 0; r < NUM_RANKS[channel] ; r++)
      for(int b = 0; b < NUM_BANKS[channel] ; b++)
	cas_issued_current_cycle[channel][vault][r][b]=0;
	  

    return 1;
  }
}

// Function to issue an activate command to a specific row
int issue_activate_command(int channel, int vault, int rank, int bank, long long int row)
{
  if(!is_activate_allowed(channel, vault, rank, bank))
  {
		printf("PANIC : SCHED_ERROR: ACTIVATE command not issuable in cycle:%lld\n", CYCLE_VAL);
		return 0;
  }
  else
  {
    long long int cycle = CYCLE_VAL;

    dram_state[channel][vault][rank][bank].state = ROW_ACTIVE;

    dram_state[channel][vault][rank][bank].active_row = row;

    dram_state[channel][vault][rank][bank].next_pre = max((cycle + T_RAS[channel]) , dram_state[channel][vault][rank][bank].next_pre);

    dram_state[channel][vault][rank][bank].next_refresh = max((cycle + T_RAS[channel]) , dram_state[channel][vault][rank][bank].next_refresh);

    dram_state[channel][vault][rank][bank].next_read = max(cycle + T_RCD[channel], dram_state[channel][vault][rank][bank].next_read); 

    dram_state[channel][vault][rank][bank].next_write = max(cycle + T_RCD[channel],  dram_state[channel][vault][rank][bank].next_write);

    dram_state[channel][vault][rank][bank].next_act = max(cycle + T_RC[channel],  dram_state[channel][vault][rank][bank].next_act);

    dram_state[channel][vault][rank][bank].next_powerdown = max(cycle + T_RCD[channel], dram_state[channel][vault][rank][bank].next_powerdown);

    for(int i=0;i<NUM_BANKS[channel];i++)
      if(i!=bank)
	dram_state[channel][vault][rank][i].next_act = max(cycle+T_RRD[channel], dram_state[channel][vault][rank][i].next_act);

    record_activate(channel, vault, rank, cycle);

    stats_num_activate[channel][vault][rank]++;
    stats_num_activate_spec[channel][vault][rank][bank]++;

    average_gap_between_activates[channel][vault][rank] = ((average_gap_between_activates[channel][vault][rank]*(stats_num_activate[channel][vault][rank]-1)) + (CYCLE_VAL-last_activate[channel][vault][rank]))/stats_num_activate[channel][vault][rank];

    last_activate[channel][vault][rank] = CYCLE_VAL;

    command_issued_current_cycle[channel][vault] = 1;

    return 1;

  }
}

// Function to issue a precharge command to a specific bank
int issue_precharge_command(int channel, int vault, int rank, int bank)
{
	if(!is_precharge_allowed(channel, vault, rank, bank))
	{
		printf("PANIC : SCHED_ERROR: PRECHARGE command not issuable in cycle:%lld\n", CYCLE_VAL);
		return 0;
	}
	else
	{
		dram_state[channel][vault][rank][bank].state = PRECHARGING;

		dram_state[channel][vault][rank][bank].active_row = -1;

		dram_state[channel][vault][rank][bank].next_act = max(CYCLE_VAL+T_RP[channel], dram_state[channel][vault][rank][bank].next_act);
		
		dram_state[channel][vault][rank][bank].next_powerdown = max(CYCLE_VAL+T_RP[channel], dram_state[channel][vault][rank][bank].next_powerdown);

		dram_state[channel][vault][rank][bank].next_pre = max(CYCLE_VAL+T_RP[channel], dram_state[channel][vault][rank][bank].next_pre);
		
		dram_state[channel][vault][rank][bank].next_refresh = max(CYCLE_VAL+T_RP[channel], dram_state[channel][vault][rank][bank].next_refresh);

		stats_num_precharge[channel][vault][rank][bank]++;
		
		command_issued_current_cycle[channel][vault] = 1;
		
		return 1;
	}
}

// Function to precharge a rank
int issue_all_bank_precharge_command(int channel, int vault, int rank)
{
	if(!is_all_bank_precharge_allowed(channel, vault, rank))
	{
		printf("PANIC : SCHED_ERROR: ALL_BANK_PRECHARGE command not issuable in cycle:%lld\n", CYCLE_VAL);
		return 0;
	}
	else
	{
		for(int i =0;i<NUM_BANKS[channel]; i++)
		{
			issue_precharge_command(channel, vault, rank, i);
			command_issued_current_cycle[channel][vault] = 0; /* Since issue_precharge_command would have set this, we need to reset it. */
		}
		command_issued_current_cycle[channel][vault] = 1;
		return 1;
	}
}

// Function to issue a refresh
int issue_refresh_command(int channel,int vault, int rank)
{

	if(!is_refresh_allowed(channel, vault, rank))
	{
		printf("PANIC : SCHED_ERROR: REFRESH command not issuable in cycle:%lld\n", CYCLE_VAL);
		return 0;
	}
	else
	{
		num_issued_refreshes[channel][vault][rank]++;
		long long int cycle = CYCLE_VAL;

		if(dram_state[channel][vault][rank][0].state == PRECHARGE_POWER_DOWN_SLOW)
		{
		  for(int b=0; b<NUM_BANKS[channel] ; b++)
		  {
		    dram_state[channel][vault][rank][b].next_act = max(cycle + T_XP_DLL[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_act);
		    dram_state[channel][vault][rank][b].next_pre = max(cycle + T_XP_DLL[channel]  + T_RFC[channel], dram_state[channel][vault][rank][b].next_pre);
		    dram_state[channel][vault][rank][b].next_refresh = max(cycle + T_XP_DLL[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_refresh);
		    dram_state[channel][vault][rank][b].next_powerdown = max(cycle + T_XP_DLL[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_powerdown);
		  }
		}
		else if(dram_state[channel][vault][rank][0].state == PRECHARGE_POWER_DOWN_FAST)
		{
		  for(int b=0; b<NUM_BANKS[channel] ; b++)
		  {
					dram_state[channel][vault][rank][b].next_act = max(cycle + T_XP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_act);
					dram_state[channel][vault][rank][b].next_pre = max(cycle + T_XP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_pre);
					dram_state[channel][vault][rank][b].next_refresh = max(cycle + T_XP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_refresh);
					dram_state[channel][vault][rank][b].next_powerdown = max(cycle + T_XP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_powerdown);
		  }
		}
		else if(dram_state[channel][vault][rank][0].state == ACTIVE_POWER_DOWN)
		{
		  for(int b=0; b<NUM_BANKS[channel] ; b++)
		  {
		    dram_state[channel][vault][rank][b].next_act = max(cycle + T_XP[channel] + T_RP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_act);
		    dram_state[channel][vault][rank][b].next_pre = max(cycle + T_XP[channel] + T_RP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_pre);
		    dram_state[channel][vault][rank][b].next_refresh = max(cycle + T_XP[channel] + T_RP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_refresh);
		    dram_state[channel][vault][rank][b].next_powerdown = max(cycle + T_XP[channel] + T_RP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_powerdown);
		  }
		}
		else // rank powered up
		{
		  int flag = 0;
		  for(int b=0; b<NUM_BANKS[channel] ; b++)
		  {
		    if(dram_state[channel][vault][rank][b].state == ROW_ACTIVE)
		    {
		      flag =1;
		      break;
		    }
		  }
		  if(flag) // at least a single bank is open
		  {
		    for(int b=0; b<NUM_BANKS[channel] ; b++)
		    {
		      dram_state[channel][vault][rank][b].next_act = max(cycle + T_RP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_act);
		      dram_state[channel][vault][rank][b].next_pre = max(cycle + T_RP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_pre);
		      dram_state[channel][vault][rank][b].next_refresh = max(cycle + T_RP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_refresh);
		      dram_state[channel][vault][rank][b].next_powerdown = max(cycle + T_RP[channel] + T_RFC[channel], dram_state[channel][vault][rank][b].next_powerdown);
		    }
		  }
		  else // everything precharged
		  {
		    for(int b=0; b<NUM_BANKS[channel] ; b++)
		    {
		    dram_state[channel][vault][rank][b].next_act = max(cycle + T_RFC[channel], dram_state[channel][vault][rank][b].next_act);
		    dram_state[channel][vault][rank][b].next_pre = max(cycle + T_RFC[channel], dram_state[channel][vault][rank][b].next_pre);
		    dram_state[channel][vault][rank][b].next_refresh = max(cycle + T_RFC[channel], dram_state[channel][vault][rank][b].next_refresh);
		    dram_state[channel][vault][rank][b].next_powerdown = max(cycle + T_RFC[channel], dram_state[channel][vault][rank][b].next_powerdown);
		    }
		  }

		}
		for(int b=0; b<NUM_BANKS[channel] ; b++)
		{
			dram_state[channel][vault][rank][b].active_row = -1;
			dram_state[channel][vault][rank][b].state = REFRESHING;
		}
		command_issued_current_cycle[channel][vault] = 1;
		return 1;
	}
}

void issue_forced_refresh_commands(int channel, int vault, int rank)
{
	for(int b=0; b < NUM_BANKS[channel]; b++)
	{

		dram_state[channel][vault][rank][b].state = REFRESHING;
		dram_state[channel][vault][rank][b].active_row = -1;

		dram_state[channel][vault][rank][b].next_act = next_refresh_completion_deadline[channel][vault][rank];
		dram_state[channel][vault][rank][b].next_pre = next_refresh_completion_deadline[channel][vault][rank];
		dram_state[channel][vault][rank][b].next_refresh = next_refresh_completion_deadline[channel][vault][rank];
		dram_state[channel][vault][rank][b].next_powerdown = next_refresh_completion_deadline[channel][vault][rank];
	}
}


void gather_stats(int channel, int vault)
{
	for(int i=0; i<NUM_RANKS[channel]; i++)
	{

		if(dram_state[channel][vault][i][0].state == PRECHARGE_POWER_DOWN_SLOW)
			stats_time_spent_in_precharge_power_down_slow[channel][vault][i]+=MEMORY_CLK_MULTIPLIER[channel];
		else if(dram_state[channel][vault][i][0].state == PRECHARGE_POWER_DOWN_FAST)
			stats_time_spent_in_precharge_power_down_fast[channel][vault][i]+=MEMORY_CLK_MULTIPLIER[channel];
		else if(dram_state[channel][vault][i][0].state == ACTIVE_POWER_DOWN)
			stats_time_spent_in_active_power_down[channel][vault][i]+=MEMORY_CLK_MULTIPLIER[channel];
		else 
		{
			for(int b=0; b<NUM_BANKS[channel]; b++)
			{
				if(dram_state[channel][vault][i][b].state == ROW_ACTIVE)
				{
					stats_time_spent_in_active_standby[channel][vault][i]+=MEMORY_CLK_MULTIPLIER[channel];
					break;
				}
			}
			stats_time_spent_in_power_up[channel][vault][i]+=MEMORY_CLK_MULTIPLIER[channel];
		}
	}
}

void print_stats()
{
	long long int activates_for_reads = 0;
	long long int activates_for_spec = 0;
	long long int activates_for_writes = 0;
	long long int read_cmds = 0;
	long long int write_cmds = 0;
	long long int hmc_read_cmds = 0;
	long long int hmc_write_cmds = 0;
	long long int total_read_cmds = 0;
	long long int total_write_cmds = 0;

	for(int c=0 ; c < NUM_CHANNELS ; c++)
	{
		hmc_read_cmds = 0;
		hmc_write_cmds = 0;
		for(int v=0; v < NUM_VAULTS[c]; v++) {
			
			activates_for_writes = 0;
			activates_for_reads = 0;
			activates_for_spec = 0;
			read_cmds = 0;
			write_cmds = 0;
			for(int r=0;r<NUM_RANKS[c] ;r++)
			{
				for(int b=0; b<NUM_BANKS[c] ; b++)
				{
					activates_for_writes += stats_num_activate_write[c][v][r][b];
					activates_for_reads += stats_num_activate_read[c][v][r][b];
					activates_for_spec += stats_num_activate_spec[c][v][r][b];
					read_cmds += stats_num_read[c][v][r][b];
					write_cmds += stats_num_write[c][v][r][b];
				}
			}
			
			if(NUM_VAULTS[c] > 1) {
				printf("-------- Vault %d Stats-----------\n",c);
				hmc_read_cmds += read_cmds;
				hmc_write_cmds += write_cmds;
			}
			else
				printf("-------- Channel %d Stats-----------\n",c);
			printf("Total Reads Serviced :          %-7lld\n", stats_reads_completed[c][v]);
			printf("Total Writes Serviced :         %-7lld\n", stats_writes_completed[c][v]);
			printf("Average Read Latency :          %7.5f\n", (double)stats_average_read_latency[c][v]);
			printf("Average Read Queue Latency :    %7.5f\n", (double)stats_average_read_queue_latency[c][v]);
			printf("Average Write Latency :         %7.5f\n", (double)stats_average_write_latency[c][v]);
			printf("Average Write Queue Latency :   %7.5f\n", (double)stats_average_write_queue_latency[c][v]);
			printf("Read Page Hit Rate :            %7.5f\n",((double)(read_cmds-activates_for_reads-activates_for_spec)/read_cmds));
			printf("Write Page Hit Rate :           %7.5f\n",((double)(write_cmds-activates_for_writes)/write_cmds));
			printf("------------------------------------\n");
			
			total_read_cmds += read_cmds;
			total_write_cmds += write_cmds;
		}
		printf("------------------------------------\n");
		printf("Total Reads Served for HMC %d :            %lld\n", c , hmc_read_cmds);
		printf("Total Writes Served for HMC %d :           %lld\n", c , hmc_write_cmds);
		printf("------------------------------------\n");	
	}
	printf("------------------------------------\n");
	printf("Total Reads Served :            %lld\n", total_read_cmds );
	printf("Total Writes Served :           %lld\n", total_write_cmds);
	printf("------------------------------------\n");
}

void update_issuable_commands(int channel, int vault)
{
	for(int rank = 0; rank < NUM_RANKS[channel]; rank++)
	{
		for(int bank = 0; bank < NUM_BANKS[channel] ; bank++)
			cmd_precharge_issuable[channel][vault][rank][bank] = is_precharge_allowed(channel, vault, rank, bank);

		cmd_all_bank_precharge_issuable[channel][vault][rank] = is_all_bank_precharge_allowed(channel, vault, rank) ;
		
		cmd_powerdown_fast_issuable[channel][vault][rank] =  is_powerdown_fast_allowed(channel, vault, rank);

		cmd_powerdown_slow_issuable[channel][vault][rank] =  is_powerdown_slow_allowed(channel, vault, rank);

		cmd_refresh_issuable[channel][vault][rank] = is_refresh_allowed(channel, vault, rank);

		cmd_powerup_issuable[channel][vault][rank] = is_powerup_allowed(channel, vault, rank);
	}
}

// function that updates the dram state and schedules auto-refresh if
// necessary. This is called every DRAM cycle
void update_memory(int channel)
{
	for(int vault=0;vault<NUM_VAULTS[channel];vault++)
	{
	        // make every channel ready to receive a new command
	  	command_issued_current_cycle[channel][vault] = 0;
		for(int rank =0;rank<NUM_RANKS[channel] ; rank++)
		{
			//reset variable
		    for(int bank=0; bank<NUM_BANKS[channel]; bank++)
				cas_issued_current_cycle[channel][vault][rank][bank] = 0;

		    // clean out the activate record for
			// CYCLE_VAL - T_FAW[channel]
			flush_activate_record(channel, vault, rank, CYCLE_VAL); 

			// if we are at the refresh completion
			// deadline
			if(CYCLE_VAL == next_refresh_completion_deadline[channel][vault][rank])
			{
			  	// calculate the next
				// refresh_issue_deadline
				num_issued_refreshes[channel][vault][rank] = 0;
				last_refresh_completion_deadline[channel][vault][rank] = CYCLE_VAL;
				next_refresh_completion_deadline[channel][vault][rank] = CYCLE_VAL + 8 * T_REFI[channel];
				refresh_issue_deadline[channel][vault][rank] = next_refresh_completion_deadline[channel][vault][rank] - T_RP[channel] - 8 * T_RFC[channel];
				forced_refresh_mode_on[channel][vault][rank] = 0;
				issued_forced_refresh_commands[channel][vault][rank] = 0;
			}
			else if((CYCLE_VAL == refresh_issue_deadline[channel][vault][rank]) && (num_issued_refreshes[channel][vault][rank] < 8))
			{
			    // refresh_issue_deadline has been
				// reached. Do the auto-refreshes
				forced_refresh_mode_on[channel][vault][rank] = 1;
				issue_forced_refresh_commands(channel, vault, rank);
			}
			else if(CYCLE_VAL < refresh_issue_deadline[channel][vault][rank])
			{
				//update the refresh_issue deadline
				refresh_issue_deadline[channel][vault][rank] = next_refresh_completion_deadline[channel][vault][rank] - T_RP[channel] - (8-num_issued_refreshes[channel][vault][rank]) * T_RFC[channel];
			}

		}

		// update the variables corresponding to the non-queue
		// variables
		update_issuable_commands(channel, vault);
		
		// update the request cmds in the queues
		update_read_queue_commands(channel, vault);

		update_write_queue_commands(channel, vault);
		
		update_read_return_queue(channel, vault);
		
		// remove finished requests
		clean_queues(channel, vault);
	}
}



//------------------------------------------------------------
// Calculate Power: It calculates and returns average power used by every Rank on Every 
// Channel during the course of the simulation 
// Units : Time- ns; Current mA; Voltage V; Power mW; 
//------------------------------------------------------------

 float calculate_power(int channel, int vault, int rank, int print_stats_type, int chips_per_rank)
{
	/*
	Power is calculated using the equations from Technical Note "TN-41-01: Calculating Memory System Power for DDR"
	The current values IDD* are taken from the data sheets. 
	These are average current values that the chip will draw when certain actions occur as frequently as possible.
	i.e., the worst case power consumption
	Eg: when ACTs happen every tRC
		pds_<component> is the power calculated by directly using the current values from the data sheet. 'pds' stands for 
	PowerDataSheet. This will the power drawn by the chip when operating under the activity that is assumed in the data 
	sheet. This mostly represents the worst case power
		These pds_<*> components need to be derated in accordance with the activity that is observed. Eg: If ACTs occur slower
	than every tRC, then pds_act will be derated to give "psch_act" (SCHeduled Power consumed by Activate) 
	*/

/*------------------------------------------------------------
// total_power is the sum of of 13 components listed below 
// Note: CKE is the ClocK Enable to every chip.
// Note: Even though the reads and write are to a different rank on the same channel, the Pull-Up and the Pull-Down resistors continue 
// 		to burn some power. psch_termWoth and psch_termWoth stand for the power dissipated in the rank in question when the reads and
// 		writes are to other ranks on the channel

	psch_act 						-> Power dissipated by activating a row
  psch_act_pdn 				-> Power dissipated when CKE is low (disabled) and all banks are precharged
  psch_act_stby 			-> Power dissipated when CKE is high (enabled) and at least one back is active (row is open)
  psch_pre_pdn_fast  	-> Power dissipated when CKE is low (disabled) and all banks are precharged and chip is in fast power down
  psch_pre_pdn_slow  	-> Power dissipated when CKE is low (disabled) and all banks are precharged and chip is in fast slow  down
  psch_pre_stby 			-> Power dissipated when CKE is high (enabled) and at least one back is active (row is open)
  psch_termWoth 			-> Power dissipated when a Write termiantes at the other set of chips.
  psch_termRoth 			-> Power dissipated when a Read  termiantes at the other set of chips
  psch_termW 					-> Power dissipated when a Write termiantes at the set of chips in question
  psch_dq 						-> Power dissipated when a Read  termiantes at the set of chips in question (Data Pins on the chip are called DQ)
  psch_ref 						-> Power dissipated during Refresh
  psch_rd 						-> Power dissipated during a Read  (does ot include power to open a row)
  psch_wr 						-> Power dissipated during a Write (does ot include power to open a row)

------------------------------------------------------------*/

	float pds_act ;						
	float pds_act_pdn;
	float pds_act_stby;
	float pds_pre_pdn_fast;
	float pds_pre_pdn_slow;
	float pds_pre_stby;
	float pds_wr;
	float pds_rd;
	float pds_ref;
	float pds_dq;
	float pds_termW;
	float pds_termRoth;
	float pds_termWoth;

	float psch_act;
	float psch_pre_pdn_slow;
	float psch_pre_pdn_fast;
	float psch_act_pdn;
	float psch_act_stby;
	float psch_pre_stby;
	float psch_rd;
	float psch_wr;
	float psch_ref;
	float psch_dq;
	float psch_termW;
	float psch_termRoth;
	float psch_termWoth;

	float total_chip_power;
	float total_rank_power;

	long long int writes =0 , reads=0;

	static int print_total_cycles=0;


	/*----------------------------------------------------
  //Calculating DataSheet Power
	----------------------------------------------------*/

	pds_act = (IDD0[channel] - (IDD3N[channel] * T_RAS[channel] + IDD2N[channel] *(T_RC[channel] - T_RAS[channel]))/T_RC[channel]) * VDD[channel];
	
	pds_pre_pdn_slow = IDD2P0[channel] * VDD[channel];

	pds_pre_pdn_fast = IDD2P1[channel] * VDD[channel];

	pds_act_pdn = IDD3P[channel] * VDD[channel];

	pds_pre_stby = IDD2N[channel] * VDD[channel];
	pds_act_stby = IDD3N[channel] * VDD[channel];

	pds_wr = (IDD4W[channel] - IDD3N[channel]) * VDD[channel];

	pds_rd = (IDD4R[channel] - IDD3N[channel]) * VDD[channel];

	pds_ref = (IDD5[channel] - IDD3N[channel]) * VDD[channel];


	/*----------------------------------------------------
	//On Die Termination (ODT) Power:
	//Values obtained from Micron Technical Note
	//This is dependent on the termination configuration of the simulated configuration
	//our simulator uses the same config as that used in the Tech Note
	----------------------------------------------------*/
	pds_dq = 3.2 * 10;

	pds_termW = 0;

	pds_termRoth = 24.9 * 10;

	pds_termWoth = 20.8 * 11;

	/*----------------------------------------------------
	//Derating worst case power to represent system activity
	----------------------------------------------------*/

	//average_gap_between_activates was initialised to 0. So if it is still
	//0, then no ACTs have happened to this rank.
	//Hence activate-power is also 0
	if (average_gap_between_activates[channel][vault][rank] == 0)
	{
		psch_act = 0;
	} else {
		psch_act = pds_act * T_RC[channel]/(average_gap_between_activates[channel][vault][rank]);
	}
	
	psch_act_pdn = pds_act_pdn * ((double)stats_time_spent_in_active_power_down[channel][vault][rank]/CYCLE_VAL);
	psch_pre_pdn_slow = pds_pre_pdn_slow * ((double)stats_time_spent_in_precharge_power_down_slow[channel][vault][rank]/CYCLE_VAL);
	psch_pre_pdn_fast = pds_pre_pdn_fast * ((double)stats_time_spent_in_precharge_power_down_fast[channel][vault][rank]/CYCLE_VAL);

	psch_act_stby = pds_act_stby * ((double)stats_time_spent_in_active_standby[channel][vault][rank]/CYCLE_VAL);

	/*----------------------------------------------------
  //pds_pre_stby assumes that the system is powered up and every 
	//row has been precharged during every cycle 
	// In reality, the chip could have been in a power-down mode
	//or a row could have been active. The time spent in these modes 
	//should be deducted from total time
	----------------------------------------------------*/
	psch_pre_stby = pds_pre_stby * ((double)(CYCLE_VAL - stats_time_spent_in_active_standby[channel][vault][rank]- stats_time_spent_in_precharge_power_down_slow[channel][vault][rank] - stats_time_spent_in_precharge_power_down_fast[channel][vault][rank] - stats_time_spent_in_active_power_down[channel][vault][rank]))/CYCLE_VAL;

	/*----------------------------------------------------
  //Calculate Total Reads ans Writes performed in the system
	----------------------------------------------------*/
	
	for(int i=0;i<NUM_BANKS[channel];i++)
	{
		writes+= stats_num_write[channel][vault][rank][i];
		reads+=stats_num_read[channel][vault][rank][i];
	}

	/*----------------------------------------------------
  // pds<rd/wr> assumes that there is rd/wr happening every cycle
	// T_DATA_TRANS is the number of cycles it takes for one rd/wr
	----------------------------------------------------*/
	psch_wr = pds_wr * (writes*T_DATA_TRANS[channel])/CYCLE_VAL;

	psch_rd = pds_rd * (reads*T_DATA_TRANS[channel])/CYCLE_VAL;

	/*----------------------------------------------------
  //pds_ref assumes that there is always a refresh happening.
	//in reality, refresh consumes only T_RFC out of every t_REFI
	----------------------------------------------------*/
	psch_ref = pds_ref * T_RFC[channel]/T_REFI[channel]; 

	psch_dq = pds_dq * (reads*T_DATA_TRANS[channel])/CYCLE_VAL;

	psch_termW = pds_termW * (writes*T_DATA_TRANS[channel])/CYCLE_VAL;


	psch_termRoth = pds_termRoth *  ((double)stats_time_spent_terminating_reads_from_other_ranks[channel][vault][rank]/CYCLE_VAL);
	psch_termWoth = pds_termWoth * ((double)stats_time_spent_terminating_writes_to_other_ranks[channel][vault][rank]/CYCLE_VAL);


	total_chip_power = psch_act + psch_termWoth + psch_termRoth + psch_termW + psch_dq + psch_ref + psch_rd + psch_wr + psch_pre_stby + psch_act_stby + psch_pre_pdn_fast + psch_pre_pdn_slow + psch_act_pdn  ;
	total_rank_power = total_chip_power * chips_per_rank;

	double time_in_pre_stby = (((double)(CYCLE_VAL - stats_time_spent_in_active_standby[channel][vault][rank]- stats_time_spent_in_precharge_power_down_slow[channel][vault][rank] - stats_time_spent_in_precharge_power_down_fast[channel][vault][rank] - stats_time_spent_in_active_power_down[channel][vault][rank]))/CYCLE_VAL);

	if (print_total_cycles ==0) {


		printf ("\n#-----------------------------Simulated Cycles Break-Up-------------------------------------------\n");
		printf ("Note:  1.(Read Cycles + Write Cycles + Read Other + Write Other) should add up to %% cycles during which\n");
		printf ("          the channel is busy. This should be the same for all Ranks on a Channel\n");
		printf ("       2.(PRE_PDN_FAST + PRE_PDN_SLOW + ACT_PDN + ACT_STBY + PRE_STBY) should add up to 100%%\n");
		printf ("       3.Power Down means Clock Enable, CKE = 0. In Standby mode, CKE = 1\n");
		printf ("#-------------------------------------------------------------------------------------------------\n");
		printf ("Total Simulation Cycles                      %11lld\n",CYCLE_VAL );
		printf ("---------------------------------------------------------------\n\n");

		print_total_cycles = 1;
	}

	if (print_stats_type == 0) {
		/*
		printf ("%3d %6d %13.2f %13.2f %13.2f %13.2f %15.2f %15.2f %15.2f %13.2f %11.2f \n",\
				channel,\
				rank,\
				(double)reads/CYCLE_VAL,\
				(double)writes/CYCLE_VAL,\
				psch_termRoth,\
				psch_termWoth,\
				((double)stats_time_spent_in_precharge_power_down_fast[channel][rank]/CYCLE_VAL),\
				((double)stats_time_spent_in_precharge_power_down_slow[channel][rank]/CYCLE_VAL),\
				((double)stats_time_spent_in_active_power_down[channel][rank]/CYCLE_VAL),\
				((double)stats_time_spent_in_active_standby[channel][rank]/CYCLE_VAL),\
				(((double)(CYCLE_VAL - stats_time_spent_in_active_standby[channel][rank]- stats_time_spent_in_precharge_power_down_slow[channel][rank] - stats_time_spent_in_precharge_power_down_fast[channel][rank] - stats_time_spent_in_active_power_down[channel][rank]))/CYCLE_VAL)
				);
	*/
		printf ("Channel %d Rank %d Read Cycles(%%)           %9.2f # %% cycles the Rank performed a Read\n",channel, rank, (double)reads*T_DATA_TRANS[channel]/CYCLE_VAL ); 
		printf ("Channel %d Rank %d Write Cycles(%%)          %9.2f # %% cycles the Rank performed a Write\n",channel, rank, (double)writes*T_DATA_TRANS[channel]/CYCLE_VAL ); 
		printf ("Channel %d Rank %d Read Other(%%)            %9.2f # %% cycles other Ranks on the channel performed a Read\n",channel, rank, \
					   ((double)stats_time_spent_terminating_reads_from_other_ranks[channel][vault][rank]/CYCLE_VAL) ); 
		printf ("Channel %d Rank %d Write Other(%%)           %9.2f # %% cycles other Ranks on the channel performed a Write\n",channel, rank,\
					  ((double)stats_time_spent_terminating_writes_to_other_ranks[channel][vault][rank]/CYCLE_VAL) ); 
		printf ("Channel %d Rank %d PRE_PDN_FAST(%%)          %9.2f # %% cycles the Rank was in Fast Power Down and all Banks were Precharged\n",channel, rank, \
						((double)stats_time_spent_in_precharge_power_down_fast[channel][vault][rank]/CYCLE_VAL) ); 
		printf ("Channel %d Rank %d PRE_PDN_SLOW(%%)          %9.2f # %% cycles the Rank was in Slow Power Down and all Banks were Precharged\n",channel, rank, \
						((double)stats_time_spent_in_precharge_power_down_slow[channel][vault][rank]/CYCLE_VAL) ); 
		printf ("Channel %d Rank %d ACT_PDN(%%)               %9.2f # %% cycles the Rank was in Active Power Down and atleast one Bank was Active\n",channel, rank, \
						((double)stats_time_spent_in_active_power_down[channel][vault][rank]/CYCLE_VAL) ); 
		printf ("Channel %d Rank %d ACT_STBY(%%)              %9.2f # %% cycles the Rank was in Standby and atleast one bank was Active\n",channel, rank,\
						 ((double)stats_time_spent_in_active_standby[channel][vault][rank]/CYCLE_VAL) ); 
		printf ("Channel %d Rank %d PRE_STBY(%%)              %9.2f # %% cycles the Rank was in Standby and all Banks were Precharged\n",channel, rank, time_in_pre_stby ); 
		printf ("---------------------------------------------------------------\n\n");


	} else if (print_stats_type == 1) {
		/*----------------------------------------------------
		// Total Power is the sum total of all the components calculated above
		----------------------------------------------------*/


		printf ("Channel %d Rank %d Background(mw)          %9.2f # depends only on Power Down time and time all banks were precharged\n",channel, rank, psch_act_pdn+psch_act_stby+psch_pre_pdn_slow+psch_pre_pdn_fast+psch_pre_stby); 
		printf ("Channel %d Rank %d Act(mW)                 %9.2f # power spend bringing data to the row buffer\n",channel, rank, psch_act); 
		printf ("Channel %d Rank %d Read(mW)                %9.2f # power spent doing a Read  after the Row Buffer is open\n",channel, rank, psch_rd); 
		printf ("Channel %d Rank %d Write(mW)               %9.2f # power spent doing a Write after the Row Buffer is open\n",channel, rank, psch_wr); 
		printf ("Channel %d Rank %d Read Terminate(mW)      %9.2f # power dissipated in ODT resistors during Read\n",channel, rank, psch_dq); 
		printf ("Channel %d Rank %d Write Terminate(mW)     %9.2f # power dissipated in ODT resistors during Write\n",channel, rank, psch_termW); 
		printf ("Channel %d Rank %d termRoth(mW)            %9.2f # power dissipated in ODT resistors during Reads  in other ranks\n",channel, rank, psch_termRoth); 
		printf ("Channel %d Rank %d termWoth(mW)            %9.2f # power dissipated in ODT resistors during Writes in other ranks\n",channel, rank, psch_termWoth); 
		printf ("Channel %d Rank %d Refresh(mW)             %9.2f # depends on frequency of Refresh (tREFI)\n",channel, rank, psch_ref); 
		printf ("---------------------------------------------------------------\n");
		printf ("Channel %d Rank %d Total Rank Power(mW)    %9.2f # (Sum of above components)*(num chips in each Rank)\n",channel, rank, total_rank_power);
		printf ("---------------------------------------------------------------\n\n");



/*

 printf("%3d %11d %16.2f %16.2f %17.2f %13.2f %13.2f %20.2f %21.2f %24.2f %12.2f %13.2f\n",\
				channel,\
				rank,\
				total_rank_power, \
				psch_act_pdn+psch_act_stby+psch_pre_pdn_slow+psch_pre_pdn_fast+psch_pre_stby, \
				psch_act, \
				psch_rd, \
				psch_wr, \
				psch_dq, \
				psch_termW, \
				psch_termRoth, \
				psch_termWoth, \
				psch_ref                                                                                  

				);
*/

/*
		printf("Channel:%d Rank:%d Total background power : %f mW\n", channel, rank, psch_act_pdn+psch_act_stby+psch_pre_pdn_slow+psch_pre_pdn_fast+psch_pre_stby);
		printf("Channel:%d, Rank:%d Total activate power : %f,%f,%d,%f mW\n", channel, rank, psch_act,pds_act,T_RC,average_gap_between_activates[channel][rank]);
		printf("Channel:%d, Rank:%d Total I/O and termination power: %f rd:%f wr:%f dq:%f termW:%f termRoth:%f termWoth:%f mW\n", channel, rank, psch_rd+psch_wr+psch_dq+psch_termW+psch_termRoth+psch_termWoth, psch_rd, psch_wr, psch_dq, psch_termW, psch_termRoth, psch_termWoth);

		printf("Channel:%d, Rank:%d Total refresh power: %f mW\n", channel, rank, psch_ref);
		printf("Channel:%d, Rank:%d Total Rank power: %f mW\n\n", channel, rank, total_rank_power);
		printf("------------------------------------------------\n");
		*/

	} else {
		printf ("PANIC: FN_CALL_ERROR: In calculate_power(), print_stats_type can only be 1 or 0\n");
		assert (-1);
	}

	return total_rank_power;

}
