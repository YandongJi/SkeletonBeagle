#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <termios.h> // for unbuffered stdin
#include <math.h> // for M_PI
#include <inttypes.h> // for PRIu64
#include <rc/time.h>

#include <robotcontrol.h>
#include "rc_balance_defs.h"
FILE *logfile;

extern long ticks;

extern core_state_t cstate;
extern int __zero_out_controller(void);
extern int __arm_controller(void);
extern int __disarm_controller(void);

extern setpoint_t setpoint;
extern m_input_mode_t m_input_mode;


/**
 * This thread is in charge of adjusting the controller setpoint based on user
 * inputs from dsm radio control. Also detects pickup to control arming the
 * controller.
 *
 * @param      ptr   The pointer
 *
 * @return     { description_of_the_return_value }
 */
void* __setpoint_manager(__attribute__ ((unused)) void* ptr)
{
	double drive_stick = 0.1; // duty cycle
	double  turn_stick = 0.05; // input sticks
	int ch, stdin_timeout = 0; // for stdin input
	struct termios old_tio, new_tio;

	// wait for mpu to settle
	__disarm_controller();
	rc_usleep(2500000);
	rc_set_state(RUNNING);
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);
	
	
	// set up character input without buffer
	/* get the terminal settings for stdin */
	tcgetattr(STDIN_FILENO,&old_tio);
	/* we want to keep the old setting to restore them a the end */
	new_tio=old_tio;
	/* disable canonical mode (buffered i/o) and local echo */
	new_tio.c_lflag &=(~ICANON & ~ECHO);
	/* set the new settings immediately */
	if(m_input_mode == STDIN) 
	{ tcsetattr(STDIN_FILENO,TCSANOW,&new_tio);}


	while(rc_get_state()!=EXITING){
		// clear out input of old data before waiting for new data
		if(m_input_mode == STDIN) fseek(stdin,0,SEEK_END);

		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000000/SETPOINT_MANAGER_HZ);

		// nothing to do if paused, go back to beginning of loop
		if(rc_get_state() != RUNNING || m_input_mode == NONE) continue;

		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to pick MIP up
		// which will we detected by wait_for_starting_condition()
		if(setpoint.arm_state == DISARMED)
		{		__zero_out_controller();
				__arm_controller();
		}

		// if dsm is active, update the setpoint rates
		switch(m_input_mode){
		case NONE:
			continue;
		case DSM:
			if(rc_dsm_is_new_data()){
				// Read normalized (+-1) inputs from RC radio stick and multiply by
				// polarity setting so positive stick means positive setpoint
				turn_stick  = rc_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
				drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;

				// saturate the inputs to avoid possible erratic behavior
				rc_saturate_double(&drive_stick,-1,1);
				rc_saturate_double(&turn_stick,-1,1);

				// use a small deadzone to prevent slow drifts in position
				if(fabs(drive_stick)<DSM_DEAD_ZONE) drive_stick = 0.0;
				if(fabs(turn_stick)<DSM_DEAD_ZONE)  turn_stick  = 0.0;

				// translate normalized user input to real setpoint values
				switch(setpoint.drive_mode){
				case NOVICE:
					setpoint.phi_dot   = DRIVE_RATE_NOVICE * drive_stick;
					setpoint.yaw_dot =  TURN_RATE_NOVICE * turn_stick;
					break;
				case ADVANCED:
					setpoint.phi_dot   = DRIVE_RATE_ADVANCED * drive_stick;
					setpoint.yaw_dot = TURN_RATE_ADVANCED  * turn_stick;
					break;
				default: break;
				}
			}
			// if dsm had timed out, put setpoint rates back to 0
			else if(rc_dsm_is_connection_active()==0){
				setpoint.theta = 0;
				setpoint.phi_dot = 0;
				setpoint.yaw_dot = 0;
				continue;
			}
			break;
		// stdin mode: l +left, r +right, f +fwd, b +bkwd
		case STDIN:
			
			while ((ch = getchar()) != EOF){
				putchar(ch); // echo character
			    if( (ch == 'q') || (ch == 'Q')) {
			    	/* restore the former STDIN settings */
    				tcsetattr(STDIN_FILENO,TCSANOW,&old_tio);
    				printf("\nEnding. Restored STDIN\n");
			    	rc_set_state(EXITING); } // turns off all threads
			    	
				if( ch == 'z')   // turn everything off
				{ setpoint.theta = 0;
				  setpoint.phi_dot = 0;
				  setpoint.yaw_dot = 0;
				}
				
				if(ch == 'l') setpoint.yaw_dot += turn_stick; // turning rate
				if(ch == 'r') setpoint.yaw_dot -= turn_stick;
				if(ch == 'f') setpoint.phi_dot += drive_stick; // forward speed rate
				if(ch == 'b') setpoint.phi_dot -= drive_stick;
				stdin_timeout = 0;
			}

			// if it has been more than 1 second since getting data
			if(stdin_timeout >= SETPOINT_MANAGER_HZ){
				setpoint.theta = 0;
				setpoint.phi_dot = 0;
				setpoint.yaw_dot = 0;
				putchar('*'); // timeout
			}
			else{
				stdin_timeout++;
			}
			continue;
			break;
		default:
			fprintf(stderr,"ERROR in setpoint manager, invalid input mode\n");
			break;
		}
	}
// if state becomes EXITING the above loop exists and we disarm here

	
	__disarm_controller();
	return NULL;
}
