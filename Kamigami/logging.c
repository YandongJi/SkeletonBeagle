#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h> // for M_PI

#include <inttypes.h> // for PRIu64
#include <rc/time.h>

#include <robotcontrol.h>
#include "rc_balance_defs.h"
FILE *logfile;

extern long ticks;

extern core_state_t cstate;



// thread version
/**
 * prints telem data to file
 *
 * @return     nothing, NULL pointer
 */


void* telem_loop(__attribute__ ((unused)) void* ptr)
{	long old_tick=0;
	uint64_t initial_time, current_time;

	printf("telem thread\n");
    fflush(stdout); // empty buffer on every thread pass
    initial_time = rc_nanos_since_boot();
	while(rc_get_state()!=EXITING)
	{  // just data for csv format
		current_time = rc_nanos_since_boot();
		old_tick = ticks;
		fprintf(logfile, "%ld, ", old_tick);  // pass value which not changing by other process
		//fprintf(logfile, "%" PRIu64 ",",initial_time-current_time);
		fprintf(logfile, "%10.3f, ", (double)(current_time-initial_time)/1e6);
		fprintf(logfile,"%8.3f, ", cstate.gamma);
		fprintf(logfile,"%8.3f, %8.3f", cstate.dutyL, cstate.dutyR);
		fprintf(logfile,"\n");
		while(old_tick == ticks)
		{ rc_usleep(100); // sleep 100 us
		}
    }
	rc_usleep(1000000 / PRINTF_HZ);
	return NULL;
}