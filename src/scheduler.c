#include <stdio.h>
#include <stdlib.h>
#include "utlist.h"
#include "utils.h"
#include "params.h"
#include "memory_controller.h"
#include "scheduler.h"

extern long long int CYCLE_VAL;

/* Keeping track of how many preemptive precharges are performed. */
long long int num_aggr_precharge = 0;

void init_scheduler_vars()
{
	for(int channel=0; channel<MAX_NUM_CHANNELS; channel++) {
		for(int vault=0; vault<MAX_NUM_VAULTS; vault++) {
			for(int rank=0; rank<MAX_NUM_RANKS; rank++) {
				for(int bank=0; bank<MAX_NUM_BANKS; bank++) {
					recent_colacc[channel][vault][rank][bank] = 0;
				}
			}
		}
	}
	return;
}

// write queue high water mark; begin draining writes if write queue exceeds this value
#define HI_WM 40

// end write queue drain once write queue has this many writes in it
#define LO_WM 20

// Read Return Queue Limit for each Vault
#define RRQ_LIMIT 10

// 1 means we are in write-drain mode for that vault
int core_to_be_served_next = 0;
int vault_to_be_served_next = 0;

// 1 means we are in write-drain mode for that channel
//int drain_writes[MAX_NUM_CHANNELS];

/* Each cycle it is possible to issue a valid command from the read or write queues
   OR
   a valid precharge command to any bank (issue_precharge_command())
   OR 
   a valid precharge_all bank command to a rank (issue_all_bank_precharge_command())
   OR
   a power_down command (issue_powerdown_command()), programmed either for fast or slow exit mode
   OR
   a refresh command (issue_refresh_command())
   OR
   a power_up command (issue_powerup_command())
   OR
   an activate to a specific row (issue_activate_command()).

   If a COL-RD or COL-WR is picked for issue, the scheduler also has the
   option to issue an auto-precharge in this cycle (issue_autoprecharge()).

   Before issuing a command it is important to check if it is issuable. For the RD/WR queue resident commands, checking the "command_issuable" flag is necessary. To check if the other commands (mentioned above) can be issued, it is important to check one of the following functions: is_precharge_allowed, is_all_bank_precharge_allowed, is_powerdown_fast_allowed, is_powerdown_slow_allowed, is_powerup_allowed, is_refresh_allowed, is_autoprecharge_allowed, is_activate_allowed.
   */

void schedule(int channel, int vault)
{
	request_t * rd_ptr = NULL;
	request_t * wr_ptr = NULL;
	request_t * ptr = NULL;

	// if in write drain mode, keep draining writes until the
	// write queue occupancy drops to LO_WM
	if (drain_writes[channel][vault] && (write_queue_length[channel][vault] > LO_WM)) {
	  drain_writes[channel][vault] = 1; // Keep draining.
	}
	else {
	  drain_writes[channel][vault] = 0; // No need to drain.
	}

	// initiate write drain if either the write queue occupancy
	// has reached the HI_WM , OR, if there are no pending read
	// requests
	if(write_queue_length[channel][vault] > HI_WM)
	{
		drain_writes[channel][vault] = 1;
	}
	else {
	  if (!read_queue_length[channel][vault])
	    drain_writes[channel][vault] = 1;
	}
	
	
	/*********************HMC-close-page*************************************/
	
	if(channel < NUM_HMCS) {
		if(drain_writes[channel][vault])
		{
			LL_FOREACH(write_queue_head[channel][vault], wr_ptr)
			{
				if(wr_ptr->command_issuable && (CYCLE_VAL >= wr_ptr->arrival_time) && !wr_ptr->request_served)
				{
					/* Before issuing the command, see if this bank is now a candidate for closure (if it just did a column-rd/wr).
					   If the bank just did an activate or precharge, it is not a candidate for closure. */
					if (wr_ptr->next_command == COL_WRITE_CMD) {
						recent_colacc[channel][vault][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 1;
					}
					if (wr_ptr->next_command == ACT_CMD) {
						recent_colacc[channel][vault][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 0;
					}
					if (wr_ptr->next_command == PRE_CMD) {
						recent_colacc[channel][vault][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank] = 0;
					}
					issue_request_command(wr_ptr);
					break;
				}
			}
		}

		// Draining Reads
		// look through the queue and find the first request whose
		// command can be issued in this cycle and issue it 
		// Simple FCFS 
		if(!drain_writes[channel][vault])
		{
			LL_FOREACH(read_queue_head[channel][vault],rd_ptr)
			{
				if(rd_ptr->command_issuable && (CYCLE_VAL >= rd_ptr->arrival_time) && !rd_ptr->request_served)
				{
					/* Before issuing the command, see if this bank is now a candidate for closure (if it just did a column-rd/wr).
					   If the bank just did an activate or precharge, it is not a candidate for closure. */
					if (rd_ptr->next_command == COL_READ_CMD) {
						recent_colacc[channel][vault][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 1;
					}
					if (rd_ptr->next_command == ACT_CMD) {
						recent_colacc[channel][vault][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 0;
					}
					if (rd_ptr->next_command == PRE_CMD) {
						recent_colacc[channel][vault][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank] = 0;
					}
					issue_request_command(rd_ptr);
					break;
				}
			}
		}

		/* If a command hasn't yet been issued to this channel in this cycle, issue a precharge. */
		if (!command_issued_current_cycle[channel][vault]) {
			for (int rank=0; rank<NUM_RANKS[channel]; rank++) {
				for (int bank=0; bank<NUM_BANKS[channel]; bank++) {  /* For all banks on the channel.. */
					if (recent_colacc[channel][vault][rank][bank]) {  /* See if this bank is a candidate. */
						if (is_precharge_allowed(channel,vault,rank,bank)) {  /* See if precharge is doable. */
							if (issue_precharge_command(channel,vault,rank,bank)) {
								num_aggr_precharge++;
								recent_colacc[channel][vault][rank][bank] = 0;
							}
						}
					}
				}
			}
		}
	}
	
	/*********************HMC-close-page*************************************/

	/*********************DIMM-FRFCFS-page*************************************/
	else {
		// If in write drain mode, look through all the write queue
        // elements (already arranged in the order of arrival), and
        // issue the command for the first request that is ready
        if(drain_writes[channel][vault])
        {
			LL_FOREACH(write_queue_head[channel][vault], wr_ptr)
			{
				if (!wr_ptr->command_issuable) continue;

				int cas_pending = 0;
				if (wr_ptr->next_command == PRE_CMD)
				{
					ptr=NULL;
					LL_FOREACH(write_queue_head[channel][vault], ptr)
					{
						if ((ptr->next_command == COL_WRITE_CMD) && (ptr->dram_addr.row == dram_state[wr_ptr->dram_addr.channel][wr_ptr->dram_addr.vault][wr_ptr->dram_addr.rank][wr_ptr->dram_addr.bank].active_row)
																 && (ptr->dram_addr.vault == wr_ptr->dram_addr.vault)
																 && (ptr->dram_addr.bank == wr_ptr->dram_addr.bank) 
																 && (ptr->dram_addr.rank == wr_ptr->dram_addr.rank)
																 && (ptr->dram_addr.channel == wr_ptr->dram_addr.channel))


						{
							cas_pending = 1;
							break;
						}
					}
				}
				if (cas_pending == 1)  
				{
					continue;
				}
				else
				{
					issue_request_command(wr_ptr);
					break;
				}
			}
            return;
        }
        
        // Draining Reads
        // look through the queue and find the first request whose
        // command can be issued in this cycle and issue it 
        // Simple FCFS 
        if(!drain_writes[channel][vault])
        {       
			LL_FOREACH(read_queue_head[channel][vault], rd_ptr)
			{
				if (!rd_ptr->command_issuable) continue;

				int cas_pending = 0;
				if (rd_ptr->next_command == PRE_CMD)
				{

					ptr = NULL;
					LL_FOREACH(read_queue_head[channel][vault], ptr)
					{
						if ((ptr->next_command == COL_READ_CMD) && (ptr->dram_addr.row == dram_state[rd_ptr->dram_addr.channel][rd_ptr->dram_addr.vault][rd_ptr->dram_addr.rank][rd_ptr->dram_addr.bank].active_row)
																&& (ptr->dram_addr.vault == rd_ptr->dram_addr.vault)
																&& (ptr->dram_addr.bank == rd_ptr->dram_addr.bank)
																&& (ptr->dram_addr.rank == rd_ptr->dram_addr.rank)
																&& (ptr->dram_addr.channel == rd_ptr->dram_addr.channel))
						{
							cas_pending = 1;
							break;
						}
					}
				}
				if (cas_pending == 1)
					continue;
				else
				{
					issue_request_command(rd_ptr);
					break;
				}
			}
			return;
        }
	}
	/*********************DIMM-FRFCFS-page*************************************/
}

int is_write_drain_required(int core, int channel)
{
	if (drain_write_for_core[core][channel] && (write_queue_length_for_core[core][channel] > LO_WM)) {
	  drain_write_for_core[core][channel] = 1; // Keep draining.
	}
	else {
	  drain_write_for_core[core][channel]  = 0; // No need to drain.
	}

	// initiate write drain if either the write queue occupancy
	// has reached the HI_WM , OR, if there are no pending read
	// requests
	if(write_queue_length_for_core[core][channel] > HI_WM)
	{
		drain_write_for_core[core][channel] = 1;
	}
	
	if(drain_write_for_core[core][channel])
		return 1;
	
	return 0;
}

request_t * schedule_to_hmc(int channel)
{
	request_t * request_ptr = NULL;
	request_t * wr_ptr = NULL;
	request_t * rd_ptr = NULL;
	int tmp=0;

	for(int core = (core_to_be_served_next + 1) % NUMCORES; core < NUMCORES ; core = (core+1)%NUMCORES)
	{
		tmp = is_write_drain_required(core, channel);
		if(tmp)
		{
			next_request_schedule_time[channel] = CYCLE_VAL + WQ_LINK_LATENCY ;
			LL_FOREACH(write_queue_per_core_head[core][channel], wr_ptr)
			{
				request_ptr = wr_ptr;
				break;
			}
			core_to_be_served_next = core;
			return request_ptr;
		}
		if(read_queue_length_for_core[core][channel])
		{
			LL_FOREACH(read_queue_per_core_head[core][channel], rd_ptr)
			{
				request_ptr = rd_ptr;
				dram_address_t * this_addr = calc_dram_addr(request_ptr->physical_address);
				int vault = this_addr->vault;
				free(this_addr);
				if(read_return_queue_length[channel][vault] < RRQ_LIMIT)
				{
					next_request_schedule_time[channel] = CYCLE_VAL + RQ_LINK_LATENCY ;
					core_to_be_served_next = core;
					return request_ptr;
				}
			}
		 }
		 if(core == core_to_be_served_next)
			break;
	}

	for(int core =  (core_to_be_served_next + 1) % NUMCORES; core < NUMCORES ; core = (core+1) % NUMCORES)
	{
		if(write_queue_length_for_core[core][channel])
		{
			LL_FOREACH(write_queue_per_core_head[core][channel], wr_ptr)
			{
				request_ptr = wr_ptr;
				break;
			}
			next_request_schedule_time[channel] = CYCLE_VAL + WQ_LINK_LATENCY ;
			core_to_be_served_next = core;
			return request_ptr;
		}
		if(core == core_to_be_served_next)
			break;
	}

	// Either the TIME for next transition is not reached or there are no pending READ/WRITE requests
	//core_to_be_served_next = (core_to_be_served_next+1) % NUMCORES;
	return NULL;
}

request_t * schedule_completed_requests(int channel)
{
	request_t * request_ptr = NULL;
	request_t * rd_ptr = NULL;
	int tmp_vault = vault_to_be_served_next;
	int condition_to_break = 1;
	int counter = 0;
	

	for(int vault = tmp_vault; vault < NUM_VAULTS[channel] && condition_to_break; vault = (vault+1)%NUM_VAULTS[channel])
	{
		if(read_return_queue_length[channel][vault])
		{	
			next_respond_schedule_time[channel] = CYCLE_VAL + WQ_LINK_LATENCY ;
			LL_FOREACH(read_return_queue_head[channel][vault], rd_ptr)
			{
				request_ptr = rd_ptr;
				break;
			}

			vault_to_be_served_next = (vault+1)%NUM_VAULTS[channel];
			return request_ptr;
		}
		if(vault==tmp_vault && counter>0)
			condition_to_break = 0;
			
		counter++;
	}

	vault_to_be_served_next = (vault_to_be_served_next+1) % NUM_VAULTS[channel];
	return NULL;
}


void scheduler_stats()
{
  /* Nothing to print for now. */
}

