/**
* @example rc_balance
*
* Reference solution for balancing EduMiP
* convert to try do differential drive on Kamigami, using z-rate
**/

#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h> // for M_PI

#include <inttypes.h> // for PRIu64
#include <rc/time.h>

#include <robotcontrol.h>

#include "rc_balance_defs.h"
#define DIFF_DRIVE_GAIN 0.5 // 50% duty cycle for 1 radian error
#define V_FORWARD 0.15 // nominal forward PWM


static void __print_usage(void);
static void __balance_controller(void);		///< mpu interrupt routine
extern void* __setpoint_manager(void* ptr);	///< background thread
static void* __battery_checker(void* ptr);	///< background thread
static void* __printf_loop(void* ptr);		///< background thread
extern void* telem_loop(void* ptr);		///< background thread

int __zero_out_controller(void);
int __disarm_controller(void);
int __arm_controller(void);


static void __on_pause_press(void);
static void __on_mode_release(void);
// static void writeTelemetry(long, uint64_t);  // not used if thread version used


// global variables
// Must be declared static so they get zero initalized
core_state_t cstate;
setpoint_t setpoint;
m_input_mode_t m_input_mode = DSM;
static rc_filter_t D1 = RC_FILTER_INITIALIZER;
static rc_filter_t D2 = RC_FILTER_INITIALIZER;
static rc_filter_t D3 = RC_FILTER_INITIALIZER;
static rc_mpu_data_t mpu_data;
static uint64_t start_time, end_time, run_time;
long ticks = 0;
static uint64_t max_time, min_time;
FILE *logfile;
char filename[80];
double duty; // temp: test duty cycle
int ch; // motor channel


/*
 * printed if some invalid argument was given
 */
static void __print_usage(void)
{
	printf("\n");
	printf("-i {dsm|stdin|none}     specify input\n");
	printf("-q                      Don't print diagnostic info\n");
	printf("-h                      print this help message\n");
	printf("\n");
}

/**
 * Initialize the filters, mpu, threads, & wait until shut down
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char *argv[])
{
	int c;
	pthread_t setpoint_thread = 0;
	pthread_t battery_thread = 0;
	pthread_t printf_thread = 0;
	pthread_t telem_thread = 0;
	bool adc_ok = true;
	bool quiet = false;

	printf("Kamigami controller 4/25/2019\n");

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "i:qh")) != -1){
		switch (c){
		case 'i': // input option
			if(!strcmp("dsm", optarg)) {
				m_input_mode = DSM;
			} else if(!strcmp("stdin", optarg)) {
				m_input_mode = STDIN;
			} else if(!strcmp("none", optarg)){
				m_input_mode = NONE;
			} else {
				__print_usage();
				return -1;
			}
			break;
		case 'q':
			quiet = true;
			break;
		case 'h':
			__print_usage();
			return -1;
			break;
		default:
			__print_usage();
			return -1;
			break;
		}
	}

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

	// initialize buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize mode button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,__on_pause_press,NULL);
	rc_button_set_callbacks(RC_BTN_PIN_MODE,NULL,__on_mode_release);

	// initialize enocders
	if(rc_encoder_eqep_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
		return -1;
	}

	// initialize motors
	if(rc_motor_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize motors\n");
		return -1;
	}
	rc_motor_standby(1); // start with motors in standby
	
	// start dsm listener *NOT USED FOR KAMIGAMI*
	if(m_input_mode == DSM){
		if(rc_dsm_init()==-1){
			fprintf(stderr,"failed to start initialize DSM\n");
			return -1;
		}
	}

	// initialize adc
	if(rc_adc_init()==-1){
		fprintf(stderr, "failed to initialize adc\n");
		adc_ok = false;
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();




	printf("\n q to quit in stdin mode\n");
	printf("stdin mode: l +left, r +right, f +fwd, b +bkwd\n");
	printf("Press and release MODE button to toggle DSM drive mode\n");
	printf("Press and release PAUSE button to pause/start the motors\n");
	printf("hold pause button down for 2 seconds to exit\n");

	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 1)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
		return -1;
	}

	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Y_UP;

	// if gyro isn't calibrated, run the calibration routine
	if(!rc_mpu_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}

	// make sure setpoint starts at normal values
	setpoint.arm_state = DISARMED;
	setpoint.drive_mode = NOVICE;

	// set up D1 Theta controller
	double D1_num[] = D1_NUM;
	double D1_den[] = D1_DEN;
	if(rc_filter_alloc_from_arrays(&D1, DT, D1_num, D1_NUM_LEN, D1_den, D1_DEN_LEN)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D1\n");
		return -1;
	}
	D1.gain = D1_GAIN;
	rc_filter_enable_saturation(&D1, -1.0, 1.0);
	rc_filter_enable_soft_start(&D1, SOFT_START_SEC);

	// set up D2 Phi controller
	double D2_num[] = D2_NUM;
	double D2_den[] = D2_DEN;
	if(rc_filter_alloc_from_arrays(&D2, DT, D2_num, D2_NUM_LEN, D2_den, D2_DEN_LEN)){
		fprintf(stderr,"ERROR in rc_balance, failed to make filter D2\n");
		return -1;
	}
	D2.gain = D2_GAIN;
	rc_filter_enable_saturation(&D2, -THETA_REF_MAX, THETA_REF_MAX);
	rc_filter_enable_soft_start(&D2, SOFT_START_SEC);

	// printf("Inner Loop controller D1:\n");
	// rc_filter_print(D1);
	// printf("\nOuter Loop controller D2:\n");
	// rc_filter_print(D2);

	// set up D3 gamma (steering) controller
	if(rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, 4*DT, DT)){
		fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
		return -1;
	}
	rc_filter_enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);

	// start a thread to slowly sample battery
	if (adc_ok) {
		if(rc_pthread_create(&battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}
	else { // If we can't get the battery voltage
		cstate.vBatt = V_NOMINAL; // Set to a nominal value
	}

	// wait for the battery thread to make the first read
	while(cstate.vBatt<1.0 && rc_get_state()!=EXITING) rc_usleep(10000);



      
      
/*  * ********************************** */
/* figure start and end time */
/* log file initialization just before starting  */
/************************************ */

	min_time = 1000000000; // a big number, 1e9 ns
	strcpy(filename, "logfile.csv");
	logfile = fopen(filename,"w");
   printf(" returned from open %s. \n",filename);
   if ( logfile == NULL)
   { printf("can't open file %s  \n", filename);
          return(-1);
   }
   fprintf(logfile," 'ticks','ms time', 'gamma', 'dutyL', 'dutyR'\n");	

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout)) && (quiet == false)){
		if(rc_pthread_create(&printf_thread, __printf_loop, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start printf thread\n");
			return -1;
		}
	}

	// start mpu
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}

	// start balance stack to control setpoints
	if(rc_pthread_create(&setpoint_thread, __setpoint_manager, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start battery thread\n");
		return -1;
	}


/* see if motor will turn on here */
	    rc_motor_cleanup();  // close if already opened
	    if(rc_motor_init()) return -1;
        rc_motor_standby(0); // make sure not in standby
        duty = 0.2; ch =2;
        printf("initialization done.\n");
        printf("sending motor %d duty cycle %0.4f\n",ch, duty);
        rc_motor_set(ch,duty);
        rc_usleep(1500000);
        rc_motor_set(ch,0.0);
        
        duty = 0.2; ch =3;
        printf("sending motor %d duty cycle %0.4f\n",ch, duty);
        rc_motor_set(ch,duty);
        rc_usleep(1500000);
        rc_motor_set(ch,0.0); // turn off again
        /*                */
   
   if(rc_pthread_create(&telem_thread, telem_loop, (void*) NULL, SCHED_OTHER, 0))
   {  fprintf(stderr, "failed to start telem thread\n");
			return -1; }
   
 /* ************************     
***********************************************
* ************************ */

/**** this starts interrupts, so should be last init step ****/
	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	rc_mpu_set_dmp_callback(&__balance_controller);

/* */

 /******************************************************** */
	// start in the RUNNING state, pressing the pause button will swap to
	// the PAUSED state then back again.

	rc_set_state(RUNNING);
	
	// // see if threads are set up
	// printf("printf thread properties:");
	// rc_pthread_print_properties(printf_thread);
	// printf("telemetry thread properties:");
	// rc_pthread_print_properties(telem_thread);

	// chill until something exits the program
	rc_set_state(RUNNING);  // turns on all threads, etc
	rc_motor_standby(0); // make sure motor is running
	
/*  wait here until exiting control loop */	
	while(rc_get_state()!=EXITING){
		rc_usleep(200000);
	}

/* exiting and clean up code */
/*******************************************/

	if(fclose(logfile) == 0)
	{ printf("%s closed successfully\n", filename); // close log file before exiting
	}
 	rc_usleep(200000); /* give time to print*/

	// join threads
	// pthread_join - wait for thread termination
	rc_pthread_timed_join(setpoint_thread, NULL, 1.5);
	if (battery_thread) rc_pthread_timed_join(battery_thread, NULL, 1.5);
	if (printf_thread) rc_pthread_timed_join(printf_thread, NULL, 1.5);
	if (telem_thread) rc_pthread_timed_join(telem_thread, NULL, 1.5);
    
	// cleanup
	rc_filter_free(&D1);
	rc_filter_free(&D2);
	rc_filter_free(&D3);
	rc_mpu_power_off();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
    
 //	printf("control thread time: %" PRIu64 "ns\n",run_time);
   	printf("min control thread latency: %10.3f ms\n",((double)min_time)/1e6);
	printf("max control thread latency: %10.3f ms\n",((double) max_time)/1e6);
	printf("# of ticks %ld \n",ticks);
	
	return 0;
}


/**
 * discrete-time balance controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
static void __balance_controller(void)
{	double dutyL, dutyR;
	
	/* test timing of interrupt in Linux environment */
	ticks++;
	end_time = rc_nanos_since_boot();
	run_time = end_time - start_time;  // time since previous interrupt
	start_time = rc_nanos_since_boot();
	
	if (ticks>10) // ignore initial startup of asynch process
	{	if (run_time < min_time) 
		{min_time = run_time;}
		if (run_time > max_time) 
		{max_time = run_time;}
	}	
	
	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	******************************************************************/
	
	// yaw steering angle gamma estimate- direct from gyro, need to integrate?
	cstate.gamma = mpu_data.dmp_TaitBryan[TB_YAW_Z];

	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(rc_get_state()==EXITING){
		rc_motor_set(0,0.0);
		return;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(rc_get_state()!=RUNNING && setpoint.arm_state==ARMED){
		__disarm_controller();
		return;
	}
	// exit if the controller is disarmed
	if(setpoint.arm_state==DISARMED){
		return;
	}


	/**********************************************************
	* Send signal to motors
	* multiply by polarity to make sure direction is correct.
	***********************************************************/
	// gamma dot is turning rate, phi_dot is forward rate 
	// dutyL = DIFF_DRIVE_GAIN * cstate.gamma + V_FORWARD;
	// dutyR = -DIFF_DRIVE_GAIN * cstate.gamma + V_FORWARD;
	dutyL = DIFF_DRIVE_GAIN * cstate.gamma + setpoint.phi_dot + setpoint.gamma_dot;
	dutyR = -DIFF_DRIVE_GAIN * cstate.gamma + setpoint.phi_dot - setpoint.gamma_dot;
	dutyL = fmax(-0.25, fmin(dutyL,0.25));
	dutyR = fmax(-0.25, fmin(dutyR,0.25));
	cstate.dutyL = dutyL;
	cstate.dutyR = dutyR;
	
	//fprintf(logfile, "%6.3f , %6.3f  ", dutyL, dutyR);  // only for debugging- file IO slows down control
	rc_motor_set(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL);
	rc_motor_set(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR);
	// writeTelemetry(ticks,start_time); /* uncomment for direct telem rather than threaded */
	return;
}

/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
int __zero_out_controller(void)
{
	rc_filter_reset(&D1);
	rc_filter_reset(&D2);
	rc_filter_reset(&D3);
	setpoint.theta = 0.0;
	setpoint.phi   = 0.0;
	setpoint.gamma = 0.0;
	rc_motor_set(0,0.0);
	return 0;
}

/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */

int __disarm_controller(void)
{
	rc_motor_standby(1);
	rc_motor_free_spin(0);
	setpoint.arm_state = DISARMED;
	/**** keep running for debugging purposes ***/
	setpoint.arm_state = ARMED;
	return 0;
}

/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
int __arm_controller(void)
{
	__zero_out_controller();
	rc_encoder_eqep_write(ENCODER_CHANNEL_L,0);
	rc_encoder_eqep_write(ENCODER_CHANNEL_R,0);
	// prefill_filter_inputs(&D1,cstate.theta);
	rc_motor_standby(0);
	setpoint.arm_state = ARMED;
	return 0;
}


/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL pointer
 */
static void* __battery_checker(__attribute__ ((unused)) void* ptr)
{
	double new_v;
	while(rc_get_state()!=EXITING){
		new_v = rc_adc_batt();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		cstate.vBatt = new_v;
		rc_usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
static void* __printf_loop(__attribute__ ((unused)) void* ptr)
{
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	last_rc_state = rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			printf("\nRUNNING:\n");
			printf("    θ    |");
			printf("  θ_ref  |");
			printf("    φ    |");
			printf("  φ_ref  |");
			printf("    γ    |");
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
			printf("%7.3f  |", cstate.theta);
			printf("%7.3f  |", setpoint.theta);
			printf("%7.3f  |", cstate.phi);
			printf("%7.3f  |", setpoint.phi);
			printf("%7.3f  |", cstate.gamma);
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



/**
 * Disarm the controller and set system state to paused. If the user holds the
 * pause button for 2 seconds, exit cleanly
 */
static void __on_pause_press(void)
{
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	switch(rc_get_state()){
	// pause if running
	case EXITING:
		return;
	case RUNNING:
		rc_set_state(PAUSED);
		__disarm_controller();
		rc_led_set(RC_LED_RED,1);
		rc_led_set(RC_LED_GREEN,0);
		break;
	case PAUSED:
		rc_set_state(RUNNING);
		__disarm_controller();
		rc_led_set(RC_LED_GREEN,1);
		rc_led_set(RC_LED_RED,0);
		break;
	default:
		break;
	}

	// now wait to see if the user want to shut down the program
	while(i<samples){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
			return; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	rc_led_blink(RC_LED_RED,5,2);
	rc_set_state(EXITING);  // turns off all threads
	return;
}

/**
 * toggle between position and angle modes if MiP is paused
 */
static void __on_mode_release(void)
{
	// toggle between position and angle modes
	if(setpoint.drive_mode == NOVICE){
		setpoint.drive_mode = ADVANCED;
		printf("using drive_mode = ADVANCED\n");
	}
	else {
		setpoint.drive_mode = NOVICE;
		printf("using drive_mode = NOVICE\n");
	}

	rc_led_blink(RC_LED_GREEN,5,1);
	return;
}



