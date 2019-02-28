/*
 * @file LineCamera.c
 (R. Fearing 27 Feb. 2019)
 * based on rc_test_encoders_pru which reads PRU through shared memory interface
 *
 * Prints out last line read, assuming TSL1401 line camera is connected to UART1 on BeagleBoneBlue
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
int linescan[128]; // array to hold line scan data
static int running = 0;

// prototypes
void log_scan(uint64_t, FILE*);

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
	running=0;
	return;
}


int main()
{	long i, j;
	int pin1 = UART1_HEADER_PIN_3;	///< P9_26, normally UART1 RX
	int pin2 = UART1_HEADER_PIN_4;	///< P9_24, normally UART1 TX
	int num_scans;
	FILE *fp;
	char filename[80]; // should bound check
	

	
	printf("LineScan camera test. Using Debian and PRU, not working with Ubuntu. \n");
	printf("Make sure pru_firmware is installed. v1.00 2/27/2019\n");
	
	
	// initialize PRU hardware first
	// rc_encoder_pru_init sets up several things, checks that PRU is running
	// could be cleaned up later.
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

	//fetches a pointer to the beginning of the PRU shared memory where line scan data is stored
	shared_mem_32bit_ptr = rc_pru_shared_mem_ptr();
	if(shared_mem_32bit_ptr==NULL){
		fprintf(stderr, "ERROR in rc_encoder_pru_init, failed to map shared memory pointer\n");
		return -1;
	} 
	printf("Initialization Complete\n");
	printf("How many scans:");
	scanf("%d", &num_scans);
	printf("Doing %d scans\n", num_scans);
	strcpy(filename, "linescans.csv");
	printf("Opening file %s for writing\n", filename);
	fp = fopen(filename,"w");
	if ( fp == NULL)
	{ printf("can't open file %s  \n", filename);
          return(-1);
	}
	fprintf(fp,"time (ms), linecsan, velocity\n");
	
	start_time = rc_nanos_since_boot();
	for(j = 0; j< num_scans; j++)
	{	shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] = 1; // set flag to start conversion by PRU
		while(shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] == 1); // wait for PRU to zero word
		for(i = 0; i< 128; i++){
			linescan[i] = (int) shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2+i]; // copy data
		}
		log_scan(start_time, fp); // log the scan to file
		rc_usleep(5000); // allow 5 ms for exposure before reading out camera. Minimum exposure (with 0 usleep) will be set by 8us A/D read, so about 1 ms.
	}
	end_time = rc_nanos_since_boot();
	run_time = end_time - start_time;
	printf("End_time %" PRIu64 " start_time %" PRIu64 "\n", end_time, start_time);
	printf("AtoD buffer 1e3 reads. Takes: %" PRIu64 "us\n", run_time/1000);
	
	printf("Memory buffer with A/D reads\n");
// now print out memory buffer which holds A/D readings
// use locations [ENCODER_MEM_OFFSET] + 2....129 to hold line scan from A/D
	for(i = 0; i< 128; i++){
		// printf("%8x ", (int) shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2+i]); 
		printf("%8x ", linescan[i]); 
	}
	printf("\n");
	fclose(fp);
	rc_encoder_pru_cleanup();  // should shut down PRU, maybe release A/D?
	return 0;
}

void log_scan(uint64_t start_time, FILE *fp)
{ uint64_t present_time, elapsed_time;
  int i;
  present_time = rc_nanos_since_boot();
  elapsed_time = (present_time - start_time)/1e6;
  fprintf(fp,"%" PRIu64 ", ", elapsed_time);
  fprintf(fp,"\"[");
  for(i=0; i< 128; i++)
	fprintf(fp,"%d,", linescan[i]);
  fprintf(fp,"]\", ");
  fprintf(fp," 0\n"); // velocity in m/s
  return;
}
  