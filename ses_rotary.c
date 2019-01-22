/*
 ***************************************************************************
 ses_rotary V1 - Copyright (C) 2018 MOSTAFA HASSAN & HAZEM ABAZA.
 ***************************************************************************
 This file is part of the SES_TUHH library.

 ses_rotartis a library that allows to control the rotating action of the
 encoder.

 ***************************************************************************
 */
/* INCLUDES ******************************************************************/
#include <avr/interrupt.h>

#include <stdbool.h>

#include "ses_rotary.h"

/*-----------------------------------------------------------
 * Implementation of functions defined in scheduler.h *
 *----------------------------------------------------------*/

#define ROTARY_BIN_1            5
#define ROTARY_BIN_2            2
#define BUTTON_NUM_DEBOUNCE_CHECKS  5

/* PRIVATE VARIABLES **************************************************/

pTypeRotaryCallback CWcallback = NULL;
pTypeRotaryCallback CCWcallback = NULL;

uint8_t CW = 0;
uint8_t CCW = 0;

void rotary_init() {

	DDR_REGISTER(PORTB) &= ~(1 << ROTARY_BIN_1);

	PORTB |= (1 << ROTARY_BIN_1);

	DDR_REGISTER(PORTG) &= ~(1 << ROTARY_BIN_2);

	PORTB |= (1 << ROTARY_BIN_2);

}

void rotary_setClockwiseCallback(pTypeRotaryCallback callback) {
	CWcallback = callback;
}

void rotary_setCounterClockwiseCallback(pTypeRotaryCallback callback) {
	CCWcallback = callback;
}

bool bin1_state() {
	if ((PIN_REGISTER(PORTB) & (1 << ROTARY_BIN_1)) == 0) {
		return true;
	}

	else {
		return false;
	}

}

bool bin2_state() {
	if ((PIN_REGISTER(PORTG) & (1 << ROTARY_BIN_2)) == 0) {
		return true;
	}

	else {
		return false;
	}

}

void rotary_checkState(void* checkP) {

	static uint8_t state[BUTTON_NUM_DEBOUNCE_CHECKS] = { };
	static uint8_t index = 0;
	static uint8_t debouncedState = 0;
	uint8_t lastDebouncedState = debouncedState;

	/* each bit in every state byte represents one button*/

	state[index] = 0;

	/*This indicates that there is a change in BIN_1
	 */

	if (bin1_state()) {
		state[index] |= 1;
	}

	else {
		state[index] |= 2;

	}

	index++;
	if (index == BUTTON_NUM_DEBOUNCE_CHECKS) {
		index = 0;
	}

	uint8_t j = 0xFF;
	for (uint8_t i = 0; i < BUTTON_NUM_DEBOUNCE_CHECKS; i++) {
		j = j & state[i];
	}
	debouncedState = j;

	/*// If the previous and the current state of ROTARY_BIN_1
	 * are different, that means a Pulse has happened*/

	if (lastDebouncedState != debouncedState) {

		if (bin1_state() != bin2_state()) {
			CWcallback();
		}

		else {

			CCWcallback();
		}

		lastDebouncedState = debouncedState;

	}

}

