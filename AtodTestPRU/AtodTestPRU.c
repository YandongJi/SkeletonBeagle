/*
 * @file rc_test_encoders_pru.c
 *
 * @example    rc_test_encoders_pru
 *
 * Prints out current quadrature position for channel 4 which is counted
 * using the PRU. Channels 1-3 are counted by the eQEP modules, test those with
 * rc_test_encoders_eqep instead.
 */

#include <stdio.h>
#include <signal.h>
#include <inttypes.h> // for PRIu64
#include <rc/encoder_pru.h>
#include <rc/time.h>
#include <rc/pru.h>

#include <robotcontrol.h> // includes ALL Robot Control subsystems
#include <rc/pinmux.h>
// this should give use GPIO0 port at 
// /sys/devices/platform/ocp/44e07000.gpio/gpio/gpiochip0
#define CHIP 0 

#define ENCODER_MEM_OFFSET	16

// pru shared memory pointer
static volatile unsigned int* shared_mem_32bit_ptr = NULL;

// global variables
// Must be declared static so they get zero initalized
static uint64_t start_time, end_time, run_time;

static int running = 0;

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


int main()
{	long value = 0;
	long i, j;
	int pin1 = UART1_HEADER_PIN_3;	///< P9_26, normally UART1 RX
	int pin2 = UART1_HEADER_PIN_4;	///< P9_24, normally UART1 TX
	
	int linescan[128]; // array to hold line scan data
	
	// initialize hardware first
	if(rc_encoder_pru_init()){
		fprintf(stderr,"ERROR: failed to run rc_encoder_pru_init\n");
		return -1;
	}
	
// setup SI and CLK pins for output
	if (rc_pinmux_set(pin1, PINMUX_GPIO) == -1)
		{printf("Failed to set pin %d", pin1); }
	if(rc_gpio_init (CHIP, pin1, GPIOHANDLE_REQUEST_OUTPUT) ==-1)
		{printf("Failed to init gpio pin %d", pin1); }
	
	if (rc_pinmux_set(pin2, PINMUX_GPIO) == -1)
		{printf("Failed to set pin %d", pin2); }
	if(rc_gpio_init (CHIP, pin2, GPIOHANDLE_REQUEST_OUTPUT) ==-1)
		{printf("Failed to init gpio pin %d", pin2); }
	

	// set signal handler so the loop can exit cleanly
	signal(SIGINT, __signal_handler);
	running=1;

	printf("\nRaw encoder position\n");
	printf("      E4   |");
	printf(" \n");
	rc_encoder_pru_write(1); //set to non-zero value so can verify read cycles
	start_time = rc_nanos_thread_time();
	for(i = 0; i < 1000000; i++)
	{  value = value + rc_encoder_pru_read();
	}
	end_time = rc_nanos_thread_time();
	run_time = end_time - start_time;
	printf("Encoder 1e6 reads. Total = %ld. Takes: %" PRIu64 "us\n",value, run_time/1000);

	shared_mem_32bit_ptr = rc_pru_shared_mem_ptr();
	if(shared_mem_32bit_ptr==NULL){
		fprintf(stderr, "ERROR in rc_encoder_pru_init, failed to map shared memory pointer\n");
		return -1;
	}
	
// //	while(running){
// 		printf("\r%10d |", rc_encoder_pru_read());
// 		printf("  r3=%x", 
// 		(int) shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1]);  // supposed to be written from PRU, next word above
// 		printf("  shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2]=%x", 
// 		(int) shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2]);  // supposed to be written from PRU, next word above
// 		fflush(stdout);
// 		rc_usleep(50000);
//	}

	printf("\n");
	start_time = rc_nanos_thread_time();
	for(j = 0; j< 1000; j++)
	{	shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] = 1; // set flag to start conversion by PRU
		while(shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] == 1); // wait for PRU to zero word
		for(i = 0; i< 128; i++){
			linescan[i] = (int) shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2+i]; // copy data
		}
	}
	end_time = rc_nanos_thread_time();
	run_time = end_time - start_time;
	printf("AtoD buffer 1e3 reads. Takes: %" PRIu64 "us\n", run_time/1000);
	
	printf("Memory buffer with A/D reads\n");
// now print out memory buffer which holds A/D readings
// use locations [ENCODER_MEM_OFFSET] + 2....129 to hold line scan from A/D
	for(i = 0; i< 128; i++){
		// printf("%8x ", (int) shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2+i]); 
		printf("%8x ", linescan[i]); 
	}
	printf("\n");
	rc_encoder_pru_cleanup();  // should shut down PRU, maybe release A/D?
	return 0;
}

