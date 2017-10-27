// meta data cache 

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "cache.h"
#include "params.h"
#include "memory_controller.h"

extern long long int CYCLE_VAL;

unsigned long long int tag_2_address(cache_t * my_cache, unsigned long long int in_tag, int set)
{
	return(in_tag << (log_base2(my_cache->num_set) + log_base2(my_cache->num_offset))) + (set << log_base2(my_cache->num_offset));
}

// to malloc a new cache and initialize it 
cache_t * init_cache(int set, int way, int offset, int latency)
{
	cache_t * my_cache = (cache_t *)malloc(sizeof(cache_t));
	my_cache->miss_num = 0;
	my_cache->hit_num = 0;
	my_cache->num_offset = offset;
	my_cache->num_way = way;
	my_cache->num_set = set;
	my_cache->latency = latency;
	tag_t * my_tag = (tag_t *)malloc(sizeof(tag_t)*set*way);
	for (int i=0; i<set*way; i++)
	{
		my_tag[i].address = 0x0;
		my_tag[i].lru = i%way;  		 // 0 to way-1
		my_tag[i].dirty = 0;    		 // not dirty
		my_tag[i].valid = 0;   			 // invalid
		my_tag[i].instruction_id = 0;
		my_tag[i].thread_id = 0;
	}
	my_cache->tag_array = my_tag;
	
	return (my_cache);
}

// to look up inside the cache, if it finds an invalid cache line,
// it returns it back but if it can not find it at all it returns NULL
// do_update is 1 means this access should be counted and if it is 0 
// this access is not counted as an official access
// if access is 1 it is a write request looking for this and if access
// is 0 the request is a read looking for this cache line 
tag_t * look_up (cache_t * my_cache, unsigned long long int addr, int do_update, optype_t access)
{
	unsigned long long int this_tag = find_tag(my_cache->num_offset, my_cache->num_set, addr);
	unsigned long long int this_set = find_set(my_cache->num_offset, my_cache->num_set, addr);
	int hit = 0;
	int found; // index of cache line 

	for (int i=(this_set*my_cache->num_way);i<(this_set*my_cache->num_way+my_cache->num_way); i++)
	{
		if ((my_cache->tag_array[i].address == this_tag) && (my_cache->tag_array[i].valid == 1))
		{
			// cache hit
			found = i;
			hit = 1;
			break;
		}
	}
	if (hit == 1)
	{
		if (do_update)
		{
			if (access == WRITE)
				my_cache->tag_array[found].dirty = 1;
			my_cache->hit_num++;
			for (int i=(this_set*my_cache->num_way);i<(this_set*my_cache->num_way+my_cache->num_way); i++)
			{
				if ((my_cache->tag_array[i].lru <= my_cache->tag_array[found].lru) && (i != found))
					my_cache->tag_array[i].lru++;
			}
			my_cache->tag_array[found].lru = 0;
		}
		return(&my_cache->tag_array[found]);
	}
	else
	{
		if (do_update)
		{
			my_cache->miss_num++;
		}
		return (NULL);
	}
}

//this function return the tag which is the candidate to be replaced
// based on LRU policy 
int  replacement_cache (cache_t * my_cache, unsigned long long int addr)
{
	int this_set = find_set(my_cache->num_offset, my_cache->num_set, addr);
	int index = 0;
	for (int i = this_set*my_cache->num_way; i<(this_set*my_cache->num_way+my_cache->num_way); i++)
	{
		if (my_cache->tag_array[index].lru <= my_cache->tag_array[i].lru)
		{
			index  = i;
		}
	}
	
	return(index);
}

// to insert a cache line into a cache, access shows it is a write request
// or a read one. it returns the cache line which is replaced 
tag_t * insert_cache (cache_t * my_cache, unsigned long long int addr, int thread_id, int instruction_id, optype_t access)
{
	tag_t * ltag = look_up (my_cache, addr, 0, access);

	//printf("%llx\t%d\n", addr, my_cache->num_set);
	assert (ltag == NULL);
	int this_set = find_set(my_cache->num_offset, my_cache->num_set, addr);

	int index = replacement_cache (my_cache, addr);
	tag_t * replaced = (tag_t *)malloc(sizeof(tag_t));
	replaced->address = my_cache->tag_array[index].address;
	replaced->lru = my_cache->tag_array[index].lru;
	replaced->valid = my_cache->tag_array[index].valid;
	replaced->dirty = my_cache->tag_array[index].dirty;
	replaced->thread_id = my_cache->tag_array[index].thread_id;
	replaced->instruction_id = my_cache->tag_array[index].instruction_id;
	my_cache->tag_array[index].address = find_tag(my_cache->num_offset,my_cache->num_set, addr);
	for (int i=(this_set*my_cache->num_way);i<(this_set*my_cache->num_way+my_cache->num_way); i++)
	{
		if ((my_cache->tag_array[i].lru <= my_cache->tag_array[index].lru) && (i != index))
			my_cache->tag_array[i].lru++;
	}
	my_cache->tag_array[index].lru = 0;
	my_cache->tag_array[index].valid = 1;
	if (access == WRITE)
		my_cache->tag_array[index].dirty = 1;
	else
		my_cache->tag_array[index].dirty = 0;
		
   return (replaced);
}

void print_cache (cache_t * my_cache)
{
	printf ("================================= cache content ===============\n");
	for (int i=0; i<(my_cache->num_way*my_cache->num_set); i++)
	{
		int prnt;
		if (i%my_cache->num_way==0)
		{
			prnt = 0;
			for (int t=i; t<i+my_cache->num_way; t++)
			{
				if (my_cache->tag_array[t].valid)
				{
					prnt = 1;
					break;
				}
			}
			if (prnt == 1)
				printf("\nset %d: ", i/my_cache->num_way);
		}
		if (my_cache->tag_array[i].valid)
		{
			printf ("[%d %llx]", i%my_cache->num_way, my_cache->tag_array[i].address);
		}
		
	}
	printf ("\n===============================================================\n");
	
}
