/*
 ***************************************************************************
 ses_led V1 - Copyright (C) 2018 MOSTAFA HASSAN & HAZEM ABAZA.
 ***************************************************************************
 This file is part of the SES_TUHH library.

 ses_motorFrequency is a library that allows reading the value of the Motor Frequency.

 ***************************************************************************
 */

/* INCLUDES ******************************************************************/


#include "ses_motorFrequency.h"
#include "ses_led.h"
#include "ses_timer.h"
#include "ses_lcd.h"
#include "util/atomic.h"

/* DEFINES & MACROS **********************************************************/

#define EXTERNAL_INTERUPT_MASK_REGISTER   	   EIMSK
#define EXTERNAL_INTERUPT_CONTROL_REGISTER	   EICRA
#define EXTERNAL_INTERRUPT_FLAG_REGISTER	   EIFR
#define TIMER5							   	   TCNT5
#define ARRAY_SIZE                             21
#define TIMER_RESOLUTION                       62500
#define SPIKES                                  5
#define SCALE_FACTOR                           100000

/* Variables **********************************************************/



uint8_t spikesCounter = SPIKES ;

uint8_t arrayCounter = 0;

uint8_t arrayLength = ARRAY_SIZE;

uint8_t array[ARRAY_SIZE];

bool motorOn = false;

uint16_t frequency = 0;




/*-------------------------------------------------------------
 * Implementation of functions defined in ses_motorFrequency.c *
 *-------------------------------------------------------------*/

void motorFrequency_init() {

	DDR_REGISTER(PORTD) &= ~(1 << PD0);

	PORTD |= (1 << PD0);

	/*
	 * disable interrupt to prevent accidental call during setup
	 */

	EXTERNAL_INTERUPT_MASK_REGISTER &= ~(1 << INT0);

	 /*
	  * trigger raising edges of signal
	  */

	EXTERNAL_INTERUPT_CONTROL_REGISTER |= (1 << ISC00);

	EXTERNAL_INTERUPT_CONTROL_REGISTER |= (1 << ISC01);

	/*
	 * clear pending interrupt (INT2)
	 */

	EXTERNAL_INTERRUPT_FLAG_REGISTER |= (1 << INTF0);

	/*
	* enable interrupt
	*/

	EXTERNAL_INTERUPT_MASK_REGISTER |= (1 << INT0);

	sei();
}


void motorSet(bool value) {

	motorOn = value;
}

uint16_t motorFrequency_getRecent() {

uint16_t timerValue;


	if (!motorOn) {
	/*
	 * If the motor is off ,frequency should be zero
	 */
		return 0;

	} else {


		if (spikesCounter == 5) {

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){

			   /*
			    * Timer5 starts
				*/

				TCNT5 = 0;
			}
		}



		if (spikesCounter >= 0) {
			spikesCounter--;
		}

		if (spikesCounter == 0) {


			ATOMIC_BLOCK(ATOMIC_RESTORESTATE){

				   /*
					* Timer for One revolution is calculated
					*/

				timerValue = SCALE_FACTOR * TCNT5 / TIMER_RESOLUTION;
			}

			   /*
				* frequency is calculated
				*/
			frequency = SCALE_FACTOR / timerValue;
		}

	}

	return frequency;
}

uint16_t motorFrequency_getMedian() {

	uint16_t temp = 0;
	uint16_t median = 0;
	uint16_t arrayCopy[arrayLength];

	if (!motorOn) {
		/*
		 * If the motor is off ,frequency should be zero
		 */
			return 0;

		}
	else
	{
		/*
		* Copy the Array to avoid calculation mistakes
		*/
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
			for (int i = 0; i < arrayLength; i++) {
				arrayCopy[i] = array[i];
			}
		}

		for (int i = 0; i < arrayLength; i++) {
			for (int j = 0; j < arrayLength - 1; j++) {
				if (arrayCopy[j] > arrayCopy[j + 1]) {
					temp = arrayCopy[j];
					arrayCopy[j] = arrayCopy[j + 1];
					arrayCopy[j + 1] = temp;
				}
			}
		}

		/*
		 * function to calculate the median of the array of odd elements
		 */

		median = arrayCopy[arrayLength / 2];

		return median;

	}



}

ISR(INT0_vect) {


	led_greenOff();

	motorSet(true);

	/*
	 * If array is full , we
	 * go back to the first element
	 */

	if (arrayCounter == arrayLength) {
		arrayCounter = 0;
	}

	led_yellowToggle();

	/*
	 * motor frequency is saved in array
	 */

	array[arrayCounter] = motorFrequency_getRecent();

	arrayCounter++;

}

