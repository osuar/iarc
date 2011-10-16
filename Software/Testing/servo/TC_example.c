/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  XMEGA Timer/Counter example source file.
 *
 *      This file contains an example application that demonstrates the Timer/
 *      Counter driver.
 *
 * \par Application note:
 *      AVR1306: Using the XMEGA Timer/Counter
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 1569 $
 * $Date: 2008-04-22 13:03:43 +0200 (ti, 22 apr 2008) $  \n
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

/* Definition of the CPU clock speed and TC prescaler setting. */
#define F_CPU           2000000UL
#define CPU_PRESCALER   1

#include "avr_compiler.h"
#include "TC_driver.h"

/* Prototyping of functions. */
void Example1( void );
void Example2( void );
void Example3( void );
void Example4( void );
void Example5( void );
void Example6( void );


/* \brief This is the main function for executing one of the examples.
 *
 *  Uncomment one of the function calls to show the example.
 */
int main( void )
{
	//Example1();
	/*Example2();*/
	/*Example3();*/
	Example4();
	/*Example5();*/
	/*Example6();*/
}


/*! \brief This example shows how to configure TCC0 for basic timer operation.
 *
 *  This function implements example 1, "Basic Timer/Counter Operation" from
 *  the "Getting Started" section of application note AVR1306.
 */
void Example1( void )
{
	/* Set period/TOP value. */
	TC_SetPeriod( &TCC0, 0x1000 );

	/* Select clock source. */
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1_gc );

	do {
		/* Wait while the timer counts. */
	} while (1);
}


/*! \brief This function shows how to use the Input Capture functionality.
 *
 *  This function implements example 2, "Using the Input Capture Functionality"
 *  from the "Getting Started" section of application note AVR1306.
 *
 */
void Example2( void )
{
	uint16_t inputCaptureTime;

	/* Configure PC0 for input, triggered on falling edge. */
	PORTC.PIN0CTRL = PORT_ISC_FALLING_gc;
	PORTC.DIRCLR = 0x01;

	/* Configure Port D for output. */
	PORTD.DIRSET = 0xFF;

	/* Select PC0 as input to event channel 2. */
	EVSYS.CH2MUX = EVSYS_CHMUX_PORTC_PIN0_gc;

	/* Configure TCC0 for Input Capture using event channel 2. */
	TC0_ConfigInputCapture( &TCC0, TC_EVSEL_CH2_gc );

	/* Enable Input Capture channel A. */
	TC0_EnableCCChannels( &TCC0, TC0_CCAEN_bm );

	/* Start timer by selecting a clock source. */
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1_gc );

	 do {
		do {
			/* Wait for Input Capture. */
		} while ( TC_GetCCAFlag( &TCC0 ) == 0 );

		inputCaptureTime = TC_GetCaptureA( &TCC0 );
		PORTD.OUT = (uint8_t) (inputCaptureTime >> 8);
	} while (1);
}


/*! \brief This example shows how to configure TCC0 for measurement of Frequency
 *         and Duty cycle of a signal applied to PC0.
 *
 *  This function implements example 3, "Using Input Capture to Calculate
 *  Frequency and Duty Cycle of a Signal" from the "Getting Started" section of
 *  application note AVR1306.
 */
void Example3( void )
{
	/* Configure PC0 for input, triggered on both edges. */
	PORTC.PIN0CTRL = PORT_ISC_BOTHEDGES_gc;
	PORTC.DIRCLR = 0x01;

	/* Select PC0 as input to event channel 0. */
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN0_gc;

	/* Configure TCC0 for Input Capture using event channel 2. */
	TC0_ConfigInputCapture( &TCC0, TC_EVSEL_CH0_gc );

	/* Enable Input "Capture or Compare" channel A. */
	TC0_EnableCCChannels( &TCC0, TC0_CCAEN_bm );

	/* Clear MSB of PER[H:L] to allow for propagation of edge polarity. */
	TC_SetPeriod( &TCC0, 0x7FFF );

	/* Start timer by selecting a clock source. */
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1_gc );

	/* Enable CCA interrupt. */
	TC0_SetCCAIntLevel( &TCC0, TC_CCAINTLVL_LO_gc );
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	sei();

	do {
		/* Wait while interrupt measure Frequency and Duty cycle. */
	} while (1);
}


ISR(TCC0_CCA_vect)
{
	static uint32_t frequency;
	static uint32_t dutyCycle;
	static uint16_t totalPeriod;
	static uint16_t highPeriod;

	uint16_t thisCapture = TC_GetCaptureA( &TCC0 );

	/*  Save total period based on rising edge and reset counter. */
	if ( thisCapture & 0x8000 ) {
		totalPeriod = thisCapture & 0x7FFF;
		TC_Restart( &TCC0 );
	}
	 /* Calculate duty cycle based on time from reset and falling edge. */
	else {
		highPeriod = thisCapture;
	}

	dutyCycle = ( ( ( highPeriod * 100 ) / totalPeriod ) + dutyCycle ) / 2;
	frequency = ( ( ( F_CPU / CPU_PRESCALER ) / totalPeriod ) + frequency ) / 2;
}


/*! \brief This example shows how to configure TCC0 for pulse width modulation
 *         output with varying duty cycle on channel A.
 *
 *  This function implements example 4, "Using a Timer/Counter for PWM
 *  Generation" from the "Getting Started" section of application note AVR1306.
 */
void Example4( void )
{
	uint16_t compareValue = 0x0000;

	/* Enable output on PC0. */
	PORTC.DIR = 0x01;

	/* Set the TC period. */
	TC_SetPeriod( &TCC0, 0xFFFF );

	/* Configure the TC for single slope mode. */
	TC0_ConfigWGM( &TCC0, TC_WGMODE_SS_gc );

	/* Enable Compare channel A. */
	TC0_EnableCCChannels( &TCC0, TC0_CCAEN_bm );

	/* Start timer by selecting a clock source. */
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1_gc );

	do {
		/* Calculate new compare value. */
		compareValue += 32;

		/* Output new compare value. */
		TC_SetCompareA( &TCC0, compareValue );

		do {
			/*  Wait for the new compare value to be latched
			 *  from CCABUF[H:L] to CCA[H:L]. This happens at
			 *  TC overflow (UPDATE ).
			 */
		} while( TC_GetOverflowFlag( &TCC0 ) == 0 );

		/* Clear overflow flag. */
		TC_ClearOverflowFlag( &TCC0 );

	} while (1);
}


/*! \brief This example shows how to configure TCC0 to count events.
 *
 *  This function implements example 5, "Event Counting" from the "Getting
 *  Started" section of application note AVR1306.
 *
 *  This example shows how to configure TCC0 to count the number of switch
 *  presses for a switch connected to PC0. PD0 is used as output and will
 *  be toggled for every 5 switch presses on PC0.
 */
void Example5( void )
{
	/* Configure PORTC as input on PC0, sense on falling edge. */
	PORTC.PIN0CTRL = PORT_ISC_RISING_gc;
	PORTC.DIRCLR = 0x01;

	/* Configure PORTD as output on PD0. */
	PORTD.DIRSET = 0x01;

	/* Select PC0 as input to event channel 0, enable filtering. */
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTC_PIN0_gc;
	EVSYS.CH0CTRL = EVSYS_DIGFILT_8SAMPLES_gc;

	/* Set period ( TOP value ). */
	TC_SetPeriod( &TCC0, 4 );

	/* Enable overflow interrupt at low level */
	TC0_SetOverflowIntLevel( &TCC0, TC_OVFINTLVL_LO_gc );
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	sei();

	/* Start Timer/Counter. */
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_EVCH0_gc );

	do {
		/* Wait for user input. */
	} while (1);
}


ISR(TCC0_OVF_vect)
{
	/* Toggle PD0 output after 5 switch presses. */
	PORTD.OUTTGL = 0x01;
}


/*! \brief This example shows how to configure Timer/Counter for 32-bit counting
 *         with input capture.
 *
 *  This function implements example 6, "Setting up a 32-bit Timer/Counter With
 *  Input Capture" from the "Getting Started" section of application note
 *  AVR1306.
 *
 *  This example shows how to configure TCC0 and TCC1 for 32-bit Timer/Counter
 *  operation with input capture. The overflow from TCC0 is routed through
 *  event channel 0 to TCC1. An input capture is triggered by a falling edge on
 *  PC0, routed through event channel 1.
 */
void Example6( void )
{
	uint32_t inputCaptureTime;

	/* Configure PC0 for input, triggered on falling edge. */
	PORTC.PIN0CTRL = PORT_ISC_FALLING_gc;
	PORTC.DIRCLR = 0x01;

	/* Configure PORTD as output. */
	PORTD.DIRSET = 0xFF;

	/* Use PC0 as multiplexer input for event channel 1. */
	EVSYS.CH1MUX = EVSYS_CHMUX_PORTC_PIN0_gc;

	/* Use TCC0 overflow as input for event channel 0. */
	EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc;

	/*  Configure TCC0 and TCC1 for input capture with event channel 1 as
	 *  trigger source.
	 */
	TC0_ConfigInputCapture( &TCC0, TC_EVSEL_CH1_gc );
	TC1_ConfigInputCapture( &TCC1, TC_EVSEL_CH1_gc );

	/* Enable event delay on TCC1. */
	TC_EnableEventDelay( &TCC1 );

	/* Enable input capture channel A on TCC0 and TCC1 */
	TC0_EnableCCChannels( &TCC0, TC0_CCAEN_bm );
	TC1_EnableCCChannels( &TCC1, TC1_CCAEN_bm );

	/* Use event channel 0 as clock source for TCC1. */
	TC1_ConfigClockSource( &TCC1, TC_CLKSEL_EVCH0_gc );

	/* Select system clock as TCC0 clock source. */
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV1_gc );

	do {
		do {
			/* Wait for Input Capture. */
		} while ( TC_GetCCAFlag( &TCC0 ) == 0  );

		uint16_t highWord = TC_GetCaptureA( &TCC1 );
		uint16_t lowWord = TC_GetCaptureA( &TCC0 );
		inputCaptureTime = ( (uint32_t) highWord << 16 ) | lowWord;

		PORTD.OUT = (uint8_t) (inputCaptureTime >> 24);

	} while (1);

}
