/*
 ***************************************************************************
 ses_timer V1 - Copyright (C) 2018 MOSTAFA HASSAN & HAZEM ABAZA.
 ***************************************************************************
 This file is part of the SES_TUHH library.

 ses_scheduler is a library that allows the library user to control two timers.
 the timer library has functions to start , stop and set the call back  for
 Timer 1 (16 Bit timer) and Timer 2 (8 bit timer)

 ***************************************************************************
 */

/* INCLUDES ******************************************************************/
#include "ses_timer.h"
#include "ses_button.h"
#include "ses_common.h"
#include <avr/interrupt.h>
#include "ses_led.h"
#include <stdbool.h>
#include "ses_lcd.h"
#include "ses_scheduler.h"

/* DEFINES & MACROS **********************************************************/

/*************** Macros configuration for timer 2******************************/

/*
 * Setting number of cycles for the time to count
 */
#define TIMER2_CYC_FOR_1MILLISEC	     250

#define TIMER2_REGISTER					 TCNT2

/*
 * Initial Value for Timer Reg
 */
#define INITIALIZE_TIMER_2				 0

/*
 * Control Registers A and B for the Timer
 */

#define TIMER2_CONTROL_REGISTER_2A   	 TCCR2A
#define TIMER2_CONTROL_REGISTER_2B		 TCCR2B

/*
 *Operation Mode for the timer
 */

#define WAVEFORM_GENRATION_MODE_2_0   	 WGM20
#define WAVEFORM_GENRATION_MODE_2_1	 	 WGM21
#define WAVEFORM_GENRATION_MODE_2_2	 	 WGM22

/*
 * Clock Selection
 */

#define CLOCK_SELSECT_2_0			     CS20
#define CLOCK_SELSECT_2_1				 CS21
#define CLOCK_SELSECT_2_2				 CS22

#define TIMER_INTERRUPT_MASK_2			 TIMSK2

#define TIMER_OUTPUT_COMPARE_MATCH_2A	 OCIE2A

#define TIMER_INTERRUPT_FLAG_REGISTER2	 TIFR2

#define OUTPUT_COMPARE_FLAG_2_A			 OCF2A

#define TIMER2_OUTPUT_COMPARE_REG        OCR2A

/*************** Macros configuration for timer 1******************************/

/*
 *Setting number of cycles for the time to count
 */
#define TIMER1_CYC_FOR_5MILLISEC    	 1250
#define TIMER1_REGISTER					 TCNT1
/*
 * Initial Value for Timer Reg
 */
#define INITIALIZE_TIMER				 0
/*
 * Control Registers A and B for the Timer
 */
#define TIMER1_CONTROL_REGISTER_1A   	 TCCR1A
#define TIMER1_CONTROL_REGISTER_1B   	 TCCR1B

/*
 *Operation Mode for the timer
 */

#define WAVEFORM_GENRATION_MODE_1_0   	 WGM10
#define WAVEFORM_GENRATION_MODE_1_1   	 WGM11
#define WAVEFORM_GENRATION_MODE_1_2   	 WGM12
#define WAVEFORM_GENRATION_MODE_1_3   	 WGM13

/*
 * Clock Selection
 */
#define CLOCK_SELSECT_1_0			     CS10
#define CLOCK_SELSECT_1_1			     CS11
#define CLOCK_SELSECT_1_2			     CS12

#define TIMER_INTERRUPT_MASK_1			 TIMSK1
#define TIMER_OUTPUT_COMPARE_MATCH_1A	 OCIE1A
#define TIMER_INTERRUPT_FLAG_REGISTER1	 TIFR1
#define TIMER1_OUTPUT_COMPARE_REG        OCR1A

/*************** Macros configuration for timer 5******************************/

#define TIMER5_REGISTER					 TCNT5
#define TIMER5_CONTROL_REGISTER_5A   	 TCCR5A
#define TIMER5_CONTROL_REGISTER_5B   	 TCCR5B
#define TIMER_INTERRUPT_MASK_5		 	 TIMSK5
#define TIMER_OUTPUT_COMPARE_MATCH_5A	 OCIE5A
#define TIMER_INTERRUPT_FLAG_REGISTER5	 TIFR5
#define TIMER5_OUTPUT_COMPARE_REG        OCR5A

/*************** Macros configuration for timer 0******************************/
#define POWER_REDUCTION_REGISTER    PRR0
#define Timer0_CONTROL_REGISTER_A   TCCR0A
#define Timer0_CONTROL_REGISTER_B   TCCR0B
#define TIMER_INTERRUPT_MASK_2		TIMSK2

/*Variables ********************************************************/

volatile pTimerCallback myTimerCallback1 = NULL;
volatile pTimerCallback myTimerCallback5 = NULL;

/*FUNCTION DEFINITION ********************************************************/

void timer0_start(void) {

	POWER_REDUCTION_REGISTER &= ~(1 << PRTIM0);

	Timer0_CONTROL_REGISTER_A |= (1 << WGM01) | (1 << WGM00);
	Timer0_CONTROL_REGISTER_B &= ~(1 << WGM02);

	Timer0_CONTROL_REGISTER_A |= (1 << COM0B0); // Set OC0B on Compare Match, clear OC0B at BOTTOM, (invertingmode).
	Timer0_CONTROL_REGISTER_A |= (1 << COM0B1);

	// disable the prescaler

	Timer0_CONTROL_REGISTER_B |= (1 << CS00);
	Timer0_CONTROL_REGISTER_B &= ~(1 << CS01);
	Timer0_CONTROL_REGISTER_B &= ~(1 << CS02);

}

void timer0_stop() {

	TCNT0 = 0;
	PORTG |= (1 << PG5);

	POWER_REDUCTION_REGISTER |= (1 << PRTIM0);

	Timer0_CONTROL_REGISTER_A |= (0 << COM0B0); // Clear OC0B on Compare Match, clear OC0B at BOTTOM, (invertingmode).
	Timer0_CONTROL_REGISTER_A |= (1 << COM0B1);
	/*
	 * The clock is disabled.
	 */
	Timer0_CONTROL_REGISTER_B &= ~(1 << CS00);
	Timer0_CONTROL_REGISTER_B &= ~(1 << CS01);
	Timer0_CONTROL_REGISTER_B &= ~(1 << CS02);

	/*
	 * The Timer register is reset to its initialization value.
	 */

	//TIMER2_REGISTER = INITIALIZE_TIMER_2;
}



void timer2_start() {

	/*
	 *Selecting the clock mode of the timer to internal clock
	 *with pre-scaler 64 .This is done by setting clock
	 *selection bits in Control register B
	 */
	TIMER2_CONTROL_REGISTER_2B &= ~(1 << CLOCK_SELSECT_2_0);
	TIMER2_CONTROL_REGISTER_2B &= ~(1 << CLOCK_SELSECT_2_1);
	TIMER2_CONTROL_REGISTER_2B |= (1 << CLOCK_SELSECT_2_2);

	/*
	 * Interrupts are allowed . It occurs when
	 * the content in the compare register is same of
	 * the timer register.
	 */

	TIMER_INTERRUPT_MASK_2 |= (1 << TIMER_OUTPUT_COMPARE_MATCH_2A);

	/*
	 * The interrupt flag is cleared  to avoid any unexpected behavior.
	 * The Flag is cleared by setting its value to One.
	 */

	TIMER_INTERRUPT_FLAG_REGISTER2 |= (1 << OCF2A);

	/*
	 * Setting the compare register to allow interrupt
	 * each 1 mili-second.
	 */

	TIMER2_OUTPUT_COMPARE_REG = TIMER2_CYC_FOR_1MILLISEC;

	/*
	 *Setting the operation mode of the timer to clear the
	 *compare register when it matched the value of the timer.
	 */

	TIMER2_CONTROL_REGISTER_2A &= ~(1 << WAVEFORM_GENRATION_MODE_2_0);
	TIMER2_CONTROL_REGISTER_2A |= (1 << WAVEFORM_GENRATION_MODE_2_1);
	TIMER2_CONTROL_REGISTER_2B &= ~(1 << WAVEFORM_GENRATION_MODE_2_2);

	/*
	 * Global Interrupts are enabled.
	 */

	sei();
}

void timer2_stop() {

	/*
	 * Interrupts are disabled.
	 */

	TIMER_INTERRUPT_MASK_2 &= ~(1 << TIMER_OUTPUT_COMPARE_MATCH_2A);

	/*
	 * The clock is disabled.
	 */
	TIMER2_CONTROL_REGISTER_2B &= ~(1 << CLOCK_SELSECT_2_0);
	TIMER2_CONTROL_REGISTER_2B &= ~(1 << CLOCK_SELSECT_2_1);
	TIMER2_CONTROL_REGISTER_2B &= ~(1 << CLOCK_SELSECT_2_2);

	/*
	 * The Timer register is reset to its initialization value.
	 */

	TIMER2_REGISTER = INITIALIZE_TIMER_2;

}

void timer1_setCallback(pTimerCallback cb) {

	/*
	 * myTimerCallback pointer is set by the passed
	 * function from the user
	 */
	myTimerCallback1 = cb;

}

void timer1_start() {

	/*
	 *Selecting the clock mode of the timer to internal clock
	 *with pre-scaler 64 .This is done by setting clock
	 *selection bits in Control register B
	 */
	TIMER1_CONTROL_REGISTER_1B |= (1 << CLOCK_SELSECT_1_0);
	TIMER1_CONTROL_REGISTER_1B |= (1 << CLOCK_SELSECT_1_1);
	TIMER1_CONTROL_REGISTER_1B &= ~(1 << CLOCK_SELSECT_1_2);

	/*
	 * Interrupts are allowed . It occurs when
	 * the content in the compare register is same of
	 * the timer register.
	 */
	TIMER_INTERRUPT_MASK_1 |= (1 << TIMER_OUTPUT_COMPARE_MATCH_1A);

	/*
	 * The interrupt flag is cleared  to avoid any unexpected behavior.
	 * The Flag is cleared by setting its value to One.
	 */

	TIMER_INTERRUPT_FLAG_REGISTER1 |= (1 << OCF1A);

	/*
	 *Setting the operation mode of the timer to clear the
	 *compare register when it matched the value of the timer.
	 *This is done atomically because we are accessing 16 bit
	 *register (i.e any interrupt that may occur is disable
	 *to preserve the same value when reading the 2 8 bit registers)
	 *
	 */

	unsigned char sreg = SREG;
	cli();
	TIMER1_OUTPUT_COMPARE_REG = 25000;
	SREG = sreg;

	/*
	 *setting the operation mode for CTC
	 */
	TIMER1_CONTROL_REGISTER_1A &= ~(1 << WAVEFORM_GENRATION_MODE_1_0);
	TIMER1_CONTROL_REGISTER_1A &= ~(1 << WAVEFORM_GENRATION_MODE_1_1);
	TIMER1_CONTROL_REGISTER_1B |= (1 << WAVEFORM_GENRATION_MODE_1_2);
	TIMER1_CONTROL_REGISTER_1B &= ~(1 << WAVEFORM_GENRATION_MODE_1_3);

	/*
	 * Global Interrupts are enabled.
	 */

	sei();

}

void timer1_stop() {

	/*
	 * Interrupts are disabled.
	 */

	TIMER_INTERRUPT_MASK_1 &= ~(1 << OCIE1A); // disable the mask register to disable interrupts from the timer

	/*
	 * The clock is disabled.
	 */

	TIMER1_CONTROL_REGISTER_1B &= ~(1 << CLOCK_SELSECT_1_0);
	TIMER1_CONTROL_REGISTER_1B &= ~(1 << CLOCK_SELSECT_1_1);
	TIMER1_CONTROL_REGISTER_1B &= ~(1 << CLOCK_SELSECT_1_2);

	/*
	 * The Timer register is reset to its initialization value.
	 * This is done atomically because we are writing to 16 bit
	 * register.
	 */

	unsigned char sreg = SREG;
	cli();
	TIMER1_REGISTER = INITIALIZE_TIMER;   // set timer to 0
	SREG = sreg;

}

void timer5_start(void) {

	/*
	 * configure the prescaler
	 */

	TIMER5_CONTROL_REGISTER_5B &= ~(1 << CLOCK_SELSECT_1_0);
	TIMER5_CONTROL_REGISTER_5B &= ~(1 << CLOCK_SELSECT_1_1);
	TIMER5_CONTROL_REGISTER_5B |= (1 << CLOCK_SELSECT_1_2);

	TIMER_INTERRUPT_MASK_5 |= (1 << OCIE5A);

	TIMER_INTERRUPT_FLAG_REGISTER5 |= (1 << OCF5A);

	unsigned char sreg = SREG;
	cli();
	TIMER5_OUTPUT_COMPARE_REG = 13200;        //0.5 sec
	SREG = sreg;

	/*
	 *setting the operation mode for CTC
	 */
	TIMER5_CONTROL_REGISTER_5A &= ~(1 << WAVEFORM_GENRATION_MODE_1_0);
	TIMER5_CONTROL_REGISTER_5A &= ~(1 << WAVEFORM_GENRATION_MODE_1_1);
	TIMER5_CONTROL_REGISTER_5B |= (1 << WAVEFORM_GENRATION_MODE_1_2);
	TIMER5_CONTROL_REGISTER_5B &= ~(1 << WAVEFORM_GENRATION_MODE_1_3);

	/*
	 * Global Interrupts are enabled.
	 */

	sei();
}

void timer5_setCallback(pTimerCallback cb) {

	/*
	 * myTimerCallback pointer is set by the passed
	 * function from the user
	 */
	myTimerCallback5 = cb;
}

void timer5_stop(void) {
	/*
	 * Interrupts are disabled.
	 */

	/*
	 * The clock is disabled.
	 */
	TIMER5_CONTROL_REGISTER_5B &= ~(1 << CLOCK_SELSECT_1_0);
	TIMER5_CONTROL_REGISTER_5B &= ~(1 << CLOCK_SELSECT_1_1);
	TIMER5_CONTROL_REGISTER_5B &= ~(1 << CLOCK_SELSECT_1_2);

	/*
	 * The Timer register is reset to its initialization value.
	 * This is done atomically because we are writing to 16 bit
	 * register.
	 */

	unsigned char sreg = SREG;
	cli();
	TIMER5_REGISTER = INITIALIZE_TIMER;   // set timer to 0
	SREG = sreg;

}

ISR(TIMER1_COMPA_vect) {
	/*
	 * If the global myTimerCallback1 is not NULL.
	 * the ISR calls the callback function for Timer1
	 */

	if (myTimerCallback1 != NULL) {
		myTimerCallback1(NULL);

	}
}



ISR(TIMER5_COMPA_vect) {

	/*
	 * If the global myTimerCallback1 is not NULL.
	 * the ISR calls the callback function for Timer1
	 */
	if (myTimerCallback5 != NULL) {
		myTimerCallback5(NULL);
	}

}

