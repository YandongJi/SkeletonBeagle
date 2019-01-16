/**
* quick test to see how fast a to d can be read if nothing else is running
**/

#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h> // for M_PI

#include <inttypes.h> // for PRIu64
#include <rc/time.h>

#include <robotcontrol.h>


// global variables
// Must be declared static so they get zero initalized
static uint64_t start_time, end_time, run_time;


int main(void)
{ int i;
 char c;
  double value= 0.0;
  long ivalue = 0;
          // initialize hardware first
        if(rc_adc_init()){
                fprintf(stderr,"ERROR: failed to run rc_init_adc()\n");
                return -1;
        }
	start_time = rc_nanos_thread_time();
	for(i = 0; i < 100000; i++)
	{  value = value + rc_adc_read_volt(0) ;
	}
	end_time = rc_nanos_thread_time();
	run_time = end_time - start_time;
	printf("A/D 100000 reads. Total = %lf takes: %" PRIu64 "us\n",value, run_time/1000);
	printf("waiting for input:");
	c = getchar();
	putchar(c);
	printf("\n");
	
	
	start_time = rc_nanos_thread_time();
	for(i = 0; i < 100000; i++)
	{  ivalue = ivalue + (long) rc_adc_read_raw(0) ;
	}
	end_time = rc_nanos_thread_time();
	run_time = end_time - start_time;
	printf("A/D 100000 raw reads. Total = %ld takes: %" PRIu64 "us\n",ivalue, run_time/1000);
	printf("waiting for input:");
	c = getchar();
	putchar(c);
	printf("\n");
	
	
	return 0;
}