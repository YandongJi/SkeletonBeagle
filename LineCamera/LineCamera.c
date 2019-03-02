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
#include <stdlib.h> // for malloc
#include <time.h> // for date/time
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

// structure to hold line scan data
struct telem_struct {
	int time_ms;
	int linescan[128];
	};
struct telem_struct telemdata[2000]; // statically define. malloc is risky in embedded applications
static int running = 0;

// prototypes
void log_scan(int, FILE*);

// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{	running=0;
	return; }


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
		return -1;	}
	
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
		return -1;	} 
		
	printf("Initialization Complete\n");
	printf("How many scans (2000 max):");
	scanf("%d", &num_scans);
	printf("Doing %d scans\n", num_scans);

	
	time_t t = time(NULL);
	struct tm tm_struct = *localtime(&t);
	snprintf(filename, sizeof("linescans-1999-12-31-23:59:59.csv"), "linescans.%d%02d%02d-%02d%02d%02d.csv", 
			tm_struct.tm_year + 1900, tm_struct.tm_mon + 1, tm_struct.tm_mday, tm_struct.tm_hour, tm_struct.tm_min, tm_struct.tm_sec);
	// strcpy(filename, "linescans.csv");
	printf("Opening file %s for writing\n", filename);
	fp = fopen(filename,"w");
	if ( fp == NULL)
	{ printf("can't open file %s  \n", filename);
          return(-1);
	}
	fprintf(fp,"time (ms), linescan_near, velocity\n");
	
	start_time = rc_nanos_since_boot();
	
	// throw out first scan- it is probably saturated
	shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] = 1; // set flag to start conversion by PRU
	while(shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] == 1); // wait for PRU to zero word
		
	for(j = 0; j< num_scans; j++)
	{	shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] = 1; // set flag to start conversion by PRU
		while(shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] == 1); // wait for PRU to zero word
		
		end_time = rc_nanos_since_boot();
		run_time = end_time - start_time; // get time increment since starting scans
		telemdata[j].time_ms = (int) (run_time/1e3); // convert ns to us
		
		for(i = 0; i< 128; i++)
		{			telemdata[j].linescan[i] = (int) shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2+i]; // copy data
		}

		rc_usleep(100); // allow 0.1 ms for exposure before reading out camera. Minimum exposure (with 0 usleep) will be set by 8us A/D read, so about 1 ms.
	}
	end_time = rc_nanos_since_boot();
	run_time = end_time - start_time;
	printf("End_time %" PRIu64 " start_time %" PRIu64 "\n", end_time, start_time);
	printf("AtoD buffer %d reads. Each scan takes: %" PRIu64 "us\n", num_scans, (run_time/1000)/num_scans);
	log_scan(num_scans, fp); // log the scan to file -- this takes 1 ms per line (slow printf?)
	
	
// now print out memory buffer which holds A/D readings
// use locations [ENCODER_MEM_OFFSET] + 2....129 to hold line scan from A/D
	printf("Memory buffer with A/D reads\n");
	for(i = 0; i< 128; i++)
	{ printf("%8x ", telemdata[num_scans-1].linescan[i]); }
	printf("\n");
	fclose(fp);

	rc_encoder_pru_cleanup();  // should shut down PRU, maybe release A/D?
	return 0;
}

// watch out for spaces, which also work as delimiters for python
void log_scan(int num_scans, FILE *fp)
{ int i, j;
  
  for(j = 0; j< num_scans; j++)
  {
  	  fprintf(fp,"%d,",telemdata[j].time_ms); // just index, would be nice to have time stamp 
	  fprintf(fp,"\"[");
	  for(i=0; i< 127; i++)
	  {	fprintf(fp,"%d,", telemdata[j].linescan[i]); }
	  
	  fprintf(fp," %d", telemdata[j].linescan[127]); //end data without comma
	  fprintf(fp,"]\", ");
	  fprintf(fp," 0\n"); // velocity in m/s
  }
  return;
}
  