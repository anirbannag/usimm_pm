#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

/* A data structure to see if a bank is a candidate for precharge. */
int recent_colacc[MAX_NUM_CHANNELS][MAX_NUM_VAULTS][MAX_NUM_RANKS][MAX_NUM_BANKS];

void init_scheduler_vars(); //called from main
void scheduler_stats(); //called from main
void schedule(int, int); // scheduler function called every cycle
request_t * schedule_to_hmc(int channel); //scheduler to transfer request through link to hmc
request_t * schedule_completed_requests(int channel); // scheduler to transfer read request response through link to processor

#endif //__SCHEDULER_H__

