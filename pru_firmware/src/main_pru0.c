/*
 * Source Modified by Zubeen Tolani < ZeekHuge - zeekhuge@gmail.com >
 * Based on the examples distributed by TI
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *	* Redistributions of source code must retain the above copyright
 *	  notice, this list of conditions and the following disclaimer.
 *
 *	* Redistributions in binary form must reproduce the above copyright
 *	  notice, this list of conditions and the following disclaimer in the
 *	  documentation and/or other materials provided with the
 *	  distribution.
 *
 *	* Neither the name of Texas Instruments Incorporated nor the names of
 *	  its contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <pru_cfg.h>
#include <pru_ctrl.h>
#include "resource_table_pru1.h"
#include <sys_tscAdcSs.h>  // touch screen A/D?

#define ENCODER_MEM_OFFSET	16
#define PRU_SHAREDMEM 0x10000 // shared memory with Cortex A8?

#define GPIO_CLEARDATAOUT 0x190
#define GPIO_SETDATAOUT 0x194

#define GPIO0 0x44e07000
// gpio[14] = TSL1401 clock, UART1_RX
#define CLK (1 << 14)  
// gpio[15] = start integration, UART1_TX
#define SI (1 << 15) 
unsigned int volatile * const GPIO0_CLEAR = (unsigned int *) (GPIO0 + GPIO_CLEARDATAOUT);
unsigned int volatile * const GPIO0_SET   = (unsigned int *) (GPIO0 + GPIO_SETDATAOUT);

#define GPIO1 0x4804C000
#define USR0 (1<<21)
#define USR1 (1<<22)
#define USR2 (1<<23)
#define USR3 (1<<24)
unsigned int volatile * const GPIO1_CLEAR = (unsigned int *) (GPIO1 + GPIO_CLEARDATAOUT);
unsigned int volatile * const GPIO1_SET   = (unsigned int *) (GPIO1 + GPIO_SETDATAOUT);


/* shared pins are controlled by pinmux and kernel level process/device drivers.
* try to write directly to ouput port, using pins already defined as output
*/

//#define CLK (1 << 5)  // should be r30_5, CH8BIT
//#define SI (1 << 4)  // start integration, r30_4, CH7BIT

// pru shared memory pointer
static volatile unsigned int* shared_mem_32bit_ptr = NULL;

//volatile register uint32_t __R30;  // general purpose output register

/* prototypes */
void init_adc(void);
uint16_t read_adc(uint16_t);


// The function is defined in pru0-encodery.asm in same dir
// We just need to add a declaration here, the definition can be
// separately linked
extern void start(void);

void main(void)
{	// int atod_value =0x7779;
	int i;
	/* Clear SYSCFG[STANDBY_INIT] to enable OCP master port */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
	/* Enable OCP Master ports (STANDBY_INIT) */
	// CT_CFG.SYSCFG_bit.IDLE_MODE =0;
	// CT_CFG.SYSCFG_bit.STANDBY_MODE =0;
	// CT_CFG.SYSCFG_bit.SUB_MWAIT =0;
	
	// Access PRU Shared RAM using Constant Table                    */
	// C28 defaults to 0x00000000, we need to set bits 23:8 to 0x0100 in order to have it point to 0x00010000	 */
	PRU0_CTRL.CTPPR0_bit.C28_BLK_POINTER = 0x0100;
	
	*GPIO0_CLEAR = CLK | SI ; // reset CLK and SI
	// __R30 &= ~CLK; // reset CLK
	// __R30 &= ~SI; // reset SI
	
	// try to access shared memory from C 
	shared_mem_32bit_ptr = (volatile unsigned int *) PRU_SHAREDMEM;
	
	init_adc();
	shared_mem_32bit_ptr[ENCODER_MEM_OFFSET] = 0x0;  // signal to Linux process that PRU is running/ready
	
	// atod_value = read_adc(0);  // channels are 0 to 3
	// shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2] = atod_value;
	
	// use location [ENCODER_MEM_OFFSET] + 1 to signal starting to read adc
	while(1)
	{	while(shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] == 0); // loop until command to read 128 times
			// use locations [ENCODER_MEM_OFFSET] + 2....129 to hold line scan from A/D
		*GPIO0_SET =  SI ; // set SI
		__delay_cycles(200);
		*GPIO0_SET = CLK; // set CLK. This rising edge gives first pixel
		__delay_cycles(200);
		*GPIO0_CLEAR = SI; // reset SI
		__delay_cycles(200);
		
		
		*GPIO1_SET = USR3 | USR2 |USR1 | USR0;      // Turn the USR3..0 LEDs on
	// ok to have a few extra clocks- should just be HiZ read	
		for(i = 0; i< 130; i++)
		{ 	*GPIO0_CLEAR = CLK; //  reset CLK line	
			shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+2+i] = read_adc(0); 
			*GPIO0_SET = CLK; // set CLK. This rising edge gives next pixel
			__delay_cycles(200); // allow hold time - 1 us, with ~8 us for A/D
		}	
		*GPIO0_CLEAR = CLK; // reset CLK line	- this should be clock 129 falling edge
	
		
		shared_mem_32bit_ptr[ENCODER_MEM_OFFSET+1] = 0; // reset to zero to indicate read complete
		*GPIO1_CLEAR = USR3 | USR2 |USR1 | USR0;      // Turn the USR3..0 LEDs on
	}
	
	//start(); // start assembly language routine if there is one
}




/* *******************************************************
*/

/* A to D converter code from pru_adc_firmware.c
 /usr/lib/ti/pru-software-support-package/examples/am335x/PRU_ADC
 Jan. 29, 2019- converted from message passing to basic memory port interface for speed
 and simplicity */

/* Control Module registers to enable the ADC peripheral */
#define CM_WKUP_CLKSTCTRL  (*((volatile unsigned int *)0x44E00400))
#define CM_WKUP_ADC_TSC_CLKCTRL  (*((volatile unsigned int *)0x44E004BC))


void init_adc()
{	/* set the always on clock domain to NO_SLEEP. Enable ADC_TSC clock */
	while (!(CM_WKUP_ADC_TSC_CLKCTRL == 0x02)) {
		CM_WKUP_CLKSTCTRL = 0;
		CM_WKUP_ADC_TSC_CLKCTRL = 0x02;
		/* Optional: implement timeout logic. */
	}

	/* 
	 * Set the ADC_TSC CTRL register. 
	 * Disable TSC_ADC_SS module so we can program it.
	 * Set step configuration registers to writable.
	 */
	ADC_TSC.CTRL_bit.ENABLE = 0;
	ADC_TSC.CTRL_bit.STEPCONFIG_WRITEPROTECT_N_ACTIVE_LOW = 1;

	/* 
	* make sure no step is enabled, until channel is read with read_adc(chan)  
	disable TS_CHARGE and Steps 1-16
	*/
	ADC_TSC.STEPENABLE = ADC_TSC.STEPENABLE & 0xfffe0000;
	
	
	// seems to read these channels, even if STEPENABLE is not set, so just go for channel 0.
	/* 
	 * set the ADC_TSC STEPCONFIG1 register for channel 0  
	 * Mode = 0; SW enabled, one-shot
	 * Averaging = 0x3; 8 sample average
	 * SEL_INP_SWC_3_0 = 0x0 = Channel 0
	 * use FIFO0
	 */
	ADC_TSC.STEPCONFIG1_bit.MODE = 0;
	ADC_TSC.STEPCONFIG1_bit.AVERAGING = 0;
	ADC_TSC.STEPCONFIG1_bit.SEL_INP_SWC_3_0 = 0;
	ADC_TSC.STEPCONFIG1_bit.FIFO_SELECT = 0;

	/* make sure delay is also minimum. This is set to zero at reset, but just in case see if it makes any difference. */
	/* sped up by factor of 6 by setting to zero. Perhaps it was set by other rc library code being run. 
	*  Now get 7.9 us per conversion
	*/

	ADC_TSC.STEPDELAY1_bit.OPENDELAY = 0x0;
	ADC_TSC.STEPDELAY1_bit.SAMPLEDELAY = 0x0;

	/*
	 * set the ADC_TSC STEPCONFIG2 register for channel 1
	 * Mode = 0; SW enabled, one-shot
	 * Averaging = 0x3; 8 sample average
	 * SEL_INP_SWC_3_0 = 0x1 = Channel 1
	 * use FIFO0
	 */
	ADC_TSC.STEPCONFIG2_bit.MODE = 0;
	ADC_TSC.STEPCONFIG2_bit.AVERAGING = 0;
	ADC_TSC.STEPCONFIG2_bit.SEL_INP_SWC_3_0 = 0;  // chan 0
	ADC_TSC.STEPCONFIG2_bit.FIFO_SELECT = 0;

	/* 
	 * set the ADC_TSC STEPCONFIG3 register for channel 2
	 * Mode = 0; SW enabled, one-shot
	 * Averaging = 0x3; 8 sample average
	 * SEL_INP_SWC_3_0 = 0x2 = Channel 2
	 * use FIFO0
	 */
	ADC_TSC.STEPCONFIG3_bit.MODE = 0;
	ADC_TSC.STEPCONFIG3_bit.AVERAGING = 0;
	ADC_TSC.STEPCONFIG3_bit.SEL_INP_SWC_3_0 = 0; // chan 0
	ADC_TSC.STEPCONFIG3_bit.FIFO_SELECT = 0;

	/* 
	 * set the ADC_TSC STEPCONFIG4 register for channel 3
	 * Mode = 0; SW enabled, one-shot
	 * Averaging = 0x3; 8 sample average
	 * SEL_INP_SWC_3_0 = 0x3= Channel 3
	 * use FIFO0
	 */
	ADC_TSC.STEPCONFIG4_bit.MODE = 0;
	ADC_TSC.STEPCONFIG4_bit.AVERAGING = 0;
	ADC_TSC.STEPCONFIG4_bit.SEL_INP_SWC_3_0 = 0; // chan 0
	ADC_TSC.STEPCONFIG4_bit.FIFO_SELECT = 0;

	/* 
	 * set the ADC_TSC CTRL register
	 * set step configuration registers to protected
	 * store channel ID tag if needed for debug
	 * Enable TSC_ADC_SS module
	 */
	ADC_TSC.CTRL_bit.STEPCONFIG_WRITEPROTECT_N_ACTIVE_LOW = 0;
	ADC_TSC.CTRL_bit.STEP_ID_TAG = 1;
	ADC_TSC.CTRL_bit.ENABLE = 1;
}

uint16_t read_adc(uint16_t adc_chan)
{
	/* 
	 * Clear FIFO0 by reading from it
	 * We are using single-shot mode. 
	 * It should not usually enter the for loop
	 */
	uint32_t count = ADC_TSC.FIFO0COUNT;
	uint32_t data;
	uint32_t i;
	for (i = 0; i < count; i++) {
		data = ADC_TSC.FIFO0DATA;
	}
	ADC_TSC.STEPENABLE_bit.STEP1 = 1;
	
	/* read from the specified ADC channel */
	/* are these ever disabled? maybe should be set to zero at init time... */
	
	/* **** SWITCH DOES NOT COMPILE CORRECTLY- DO NOT USE !!!!!!!!!!!! */
	
	// switch (adc_chan) {
	// 	case 0 :
	// 		ADC_TSC.STEPENABLE_bit.STEP1 = 1;
	// 		break; // break is needed to keep other steps from being enabled
	// 	case 1 :
	// 		ADC_TSC.STEPENABLE_bit.STEP2 = 1;
	// 		break;
	// 	case 2 :
	// 		ADC_TSC.STEPENABLE_bit.STEP3 = 1;
	// 		break;
	// 	case 3 :
	// 		ADC_TSC.STEPENABLE_bit.STEP4 = 1;
	// 		break;
	// 	default :
	// 		/* 
	// 		 * this error condition should not occur because of
	// 		 * checks in userspace code
	// 		 */
	// 		ADC_TSC.STEPENABLE_bit.STEP1 = 1;
	// 		break;
	// }

	while (ADC_TSC.FIFO0COUNT == 0) {
		/*
		 * loop until value placed in fifo.
		 * Optional: implement timeout logic.
		 *
		 * Other potential options include: 
		 *   - configure PRU to receive an ADC interrupt. Note that 
		 *     END_OF_SEQUENCE does not mean that the value has been
		 *     loaded into the FIFO yet
		 *   - perform other actions, and periodically poll for 
		 *     FIFO0COUNT > 0
		 */
	}

	/* read the voltage */
	uint16_t voltage = ADC_TSC.FIFO0DATA_bit.ADCDATA;

	return voltage;
}

