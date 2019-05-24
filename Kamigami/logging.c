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
extern setpoint_t setpoint;



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
		fprintf(logfile,"%8.3f, ", cstate.yaw);
		fprintf(logfile,"%8.3f, %8.3f", cstate.dutyL, cstate.dutyR);
		fprintf(logfile,"\n");
		while(old_tick == ticks)
		{ rc_usleep(100); // sleep 100 us
		}
    }
	rc_usleep(1000000 / PRINTF_HZ);
	return NULL;
}


/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
void* __printf_loop(__attribute__ ((unused)) void* ptr)
{
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	//last_rc_state = rc_get_state();
	last_rc_state = PAUSED; // get header to print when first run
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			printf("\nRUNNING:\n");
		//	printf("   theta |");
		//	printf("  θ_ref  |");
			printf(" leg vel |");
			printf(" leg ref |");
			printf("    yaw  |");
			printf("  uLeft  |");
			printf("  uRight |");
			printf("  vBatt  |");
			printf("arm_state|");
			printf("\n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_rc_state = new_rc_state;

		// decide what to print or exit
		if(new_rc_state == RUNNING){
			printf("\r");
		//	printf("%7.3f  |", cstate.theta);
		//	printf("%7.3f  |", setpoint.theta);
			printf("%7.3f  |", cstate.phi);
			printf("%7.3f  |", setpoint.phi);
			printf("%7.3f  |", cstate.yaw);
			printf("%7.3f  |", cstate.dutyL);
			printf("%7.3f  |", cstate.dutyR);
			printf("%7.3f  |", cstate.vBatt);

			if(setpoint.arm_state == ARMED) printf("  ARMED  |");
			else printf("DISARMED |");
			fflush(stdout);
		}
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

