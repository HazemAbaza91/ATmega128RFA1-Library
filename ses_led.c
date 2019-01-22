/*
 ***************************************************************************
 ses_led V1 - Copyright (C) 2018 MOSTAFA HASSAN & HAZEM ABAZA.
 ***************************************************************************
 This file is part of the SES_TUHH library.

 ses_led is a library that allows controlling the GREEN , YELLOW and
 RED led on the board.It has functions to switch each led ON or OFF and
 also functions to toggle each led independently.

 ***************************************************************************
 */

/* INCLUDES ******************************************************************/

#include "ses_common.h"
#include "ses_led.h"
#include <stdbool.h>

/*-----------------------------------------------------------
 * Implementation of functions defined in ses_led.h *
 *----------------------------------------------------------*/

/* DEFINES & MACROS **********************************************************/

/*
 * Perform hardware setup to match LED wiring on SES board.
 */

#define LED_RED_PORT       	PORTG
#define LED_RED_PIN         PG1

#define LED_YELLOW_PORT 	PORTF
#define LED_YELLOW_PIN     	PF7

#define LED_GREEN_PORT 		PORTF
#define LED_GREEN_PIN       PF6

/* VARIBALE DEFINITION *******************************************************/

/* FUNCTION DEFINITION *******************************************************/

void led_redInit(void) {
	DDR_REGISTER(PORTG) |= (1 << LED_RED_PIN);

	/* Initializing the RED led to
	 * be OFF at the begining . */

	led_redOff();

}

void led_redToggle(void) {
	PORTG ^= (1 << LED_RED_PIN);
}

void led_redOn(void) {

	PORTG &= ~(1 << LED_RED_PIN);
}

void led_redOff(void) {

	PORTG |= (1 << LED_RED_PIN);

}

void led_yellowInit(void) {
	DDR_REGISTER(PORTF) |= (1 << LED_YELLOW_PIN);

	/* Initializing the YELLOW led to
	 * be OFF at the begining . */

	led_yellowOff();

}

void led_yellowToggle(void) {

	PORTF ^= (1 << LED_YELLOW_PIN);

}

void led_yellowOn(void) {

	PORTF &= ~(1 << LED_YELLOW_PIN);

}

void led_yellowOff(void) {

	PORTF |= (1 << LED_YELLOW_PIN);

}

void led_greenInit(void) {

	DDR_REGISTER(PORTF) |= (1 << LED_GREEN_PIN);

	/* Initializing the GREEN led to
	 * be OFF at the begining . */

	led_greenOff();

}

void led_greenToggle(void) {

	PORTF ^= (1 << LED_GREEN_PIN);

}

void led_greenOn(void) {

	PORTF &= ~(1 << LED_GREEN_PIN);

}

void led_greenOff(void) {

	PORTF |= (1 << LED_GREEN_PIN);

}
