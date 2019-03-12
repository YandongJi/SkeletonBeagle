/**
 * @file rc_project_template.c
 *
 * This is meant to be a skeleton program for Robot Control projects. Change
 * this description and file name before modifying for your own purpose.
 */

#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <inttypes.h> // for PRIu64
#define PRINTF_HZ 100 // rate for printf to run


static uint64_t start_time, min_time;
volatile long ticks = 0; // updated by another process, don't optimize
FILE *logfile;
char filename[80];

// function declarations
void on_pause_press();
void on_pause_release();
static void* telem_loop(void* ptr);		///< background thread
void controller();

/**
 * This template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 *
 * @return     0 during normal operation, -1 on error
 */
int main()
{ 	pthread_t telem_thread = 0;
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	printf("starting logging thread\n");
/* ********************************** */
/* figure start and end time */
/* log file initialization just before starting  */
/************************************ */
	start_time = rc_nanos_since_boot();
	min_time = 1000000000; // a big number, 1e9 ns
	strcpy(filename, "logfile.csv");
	logfile = fopen(filename,"w");
   printf(" returned from open. \n");
   if ( logfile == NULL)
   { printf("can't open file %s  \n", filename);
          return(-1);
   }
   fprintf(logfile," 'ticks','ms time since start', 'fprintf time','ns time since start', 'fprintf time'\n");	
   
   if(rc_pthread_create(&telem_thread, telem_loop, (void*) NULL, SCHED_OTHER, 0))
   {  fprintf(stderr, "failed to start telem thread\n");
			return -1; }
   


	printf("\nPress and release pause button to turn green LED on and off\n");
	printf("hold pause button down for 2 seconds to exit\n");

	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		// do things based on the state
		if(rc_get_state()==RUNNING){
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);
			controller(); // run controller 1 update cycle
		}
		else{
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
		}
		// always sleep at some point
		rc_usleep(10000); // this is 100 Hz update rate
	}
	// close logfile
	if(fclose(logfile) == 0)
	{ printf("%s closed successfully\n", filename); // close log file before exiting
	}
 	rc_usleep(200000); /* give time to print*/
	// turn off LEDs and close file descriptors
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}


/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}

/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
static void* telem_loop(__attribute__ ((unused)) void* ptr)
{	long old_tick=0;
	static uint64_t current_time, end_time, run_time = 0;
	double current_time_f, run_time_f;
	
	printf("telem thread\n");
    fflush(stdout); // empty buffer on every thread pass
	while(rc_get_state()!=EXITING)
	{  // just data for csv format
		current_time = rc_nanos_since_boot() - start_time;
		old_tick = ticks;
		fprintf(logfile, "%ld, ", old_tick);  // pass value which not changing by other process
		current_time_f = ((double) current_time)/ 1e6; // milliseconds
		run_time_f = ((double) run_time)/1000.0; // us
		fprintf(logfile,"%8.3lf, %8.3lf, ", current_time_f, run_time_f);
		fprintf(logfile, "%" PRIu64 ", ",current_time);
		fprintf(logfile, "%" PRIu64 "\n",run_time);
		end_time = rc_nanos_since_boot() - start_time;
		run_time = end_time - current_time;
		while(old_tick == ticks)
		{ rc_usleep(100); // sleep 100 us
		}
    }
	rc_usleep(1000000 / PRINTF_HZ);
	return NULL;
}

/**
 * discrete-time controller run off main loop 
 * 
 */
void controller(void)
{	// doesn't do anything but update tick counter for now
	ticks++;
}
	