#ifndef __META_CACHE_H__
#define __META_CACHE_H__

#include "memory_controller.h"

typedef struct tag
{
  unsigned long long int address;
  int lru;
  int dirty;
  int valid;
  int thread_id;
  int instruction_id;
}tag_t;

typedef struct cache
{
	int num_set;
	int num_way;
	int num_offset;
	int latency;
	double hit_num;
	double miss_num;
	tag_t *tag_array;
	
}cache_t;


#define find_tag(numoffset, numset, addr) ((addr) >> (log_base2(numoffset) + log_base2(numset)))
#define find_set(numoffset, numset, addr) (((addr) >> log_base2(numoffset)) & (((long long int)0x1 <<log_base2(numset)) -1))

// convert tag to address 
unsigned long long int tag_2_address(cache_t * my_cache, unsigned long long int in_tag, int set);

// to initialize a cache 
cache_t * init_cache (int set, int way, int offset, int latency);

// to look up the data inside the cache 
tag_t* look_up (cache_t * my_cache, unsigned long long int addr, int do_update, optype_t access);

//to replace a cache line and return back the replaced cache line  
int  replacement_cache (cache_t * my_cache, unsigned long long int addr);

//to insert a cache line in cache 
tag_t * insert_cache (cache_t * my_cache, unsigned long long int addr, int thread_id, int instruction_id, optype_t access);
													
													
void print_cache (cache_t * my_cache);


#endif //__META_CACHE_H__
