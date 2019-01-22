/* INCLUDES ******************************************************************/
#include "ses_button.h"
#include "ses_common.h"
#include <avr/interrupt.h>
#include "ses_led.h"
#include <stdbool.h>
#include "ses_timer.h"
#include "ses_lcd.h"

/* DEFINES & MACROS **********************************************************/

#define JOYSTICK_PIN         	         7
#define ROTARY_ENCODER_PIN               6

#define PIN_CHANGE_INTERRUPT_REGISTER    PCICR
#define PIN_Change_Interrupt_Enable_0    PCIE0
#define PIN_CHANGE_MASK_REGISTER_0       PCMSK0
#define PIN_CHANGE_ENABLE_MASK_JOYSTICK  7
#define PIN_CHANGE_ENABLE_MASK_ROTARY    6
#define EXTERNAL_INTERRUPT_FLAG_REGISTER EIFR
#define BUTTON_NUM_DEBOUNCE_CHECKS       5

/* GLOBAL VARIABLES *******************************************************/

volatile pButtonCallback myRotaryCallback = NULL;
volatile pButtonCallback myJoystickCallback = NULL;

bool externalInterrupt = false;

/* FUNCTION DEFINITION *******************************************************/

void button_init(bool debouncing) {

	DDR_REGISTER(PORTB) &= ~(1 << JOYSTICK_PIN);    // declare PINB7 is an input
	PORTB |= (1 << JOYSTICK_PIN);   // make the pull up resistor of the Joystick
	DDR_REGISTER(PORTB) &= ~(1 << ROTARY_ENCODER_PIN); // declare PINB6 is an input
	PORTB |= (1 << ROTARY_ENCODER_PIN);	// make the pull up resistor of the Rotary

	if (debouncing) {
		timer1_start();
		timer1_setCallback(&button_checkState);
	} else {

		PIN_CHANGE_MASK_REGISTER_0 &= ~(1 << PIN_CHANGE_ENABLE_MASK_JOYSTICK); // disable the mask first to prevent accidentall calls during setups
		PIN_CHANGE_MASK_REGISTER_0 &= ~(1 << PIN_CHANGE_ENABLE_MASK_ROTARY);

		EXTERNAL_INTERRUPT_FLAG_REGISTER |= (1
				<< PIN_CHANGE_ENABLE_MASK_JOYSTICK); // clear pending interrupt
		EXTERNAL_INTERRUPT_FLAG_REGISTER |=
				(1 << PIN_CHANGE_ENABLE_MASK_ROTARY); // clear pending interrupt

// now enable the external Interrupt
		if (externalInterrupt) {

			PIN_CHANGE_INTERRUPT_REGISTER |=
					(1 << PIN_Change_Interrupt_Enable_0);
			PIN_CHANGE_MASK_REGISTER_0 |=
					(1 << PIN_CHANGE_ENABLE_MASK_JOYSTICK);
			PIN_CHANGE_MASK_REGISTER_0 |= (1 << PIN_CHANGE_ENABLE_MASK_ROTARY);

		}
		sei();

	}
}

bool button_isJoystickPressed(void) {

	if ((PIN_REGISTER(PORTB) & 1 << JOYSTICK_PIN) == 0) { // if PORTB and operation with 0b1000000 = 0 then it is pressed
		return true;
	} else {									// it is not pressed
		return false;
	}
}
bool button_isRotaryPressed(void) {
	if ((PIN_REGISTER(PORTB) & 1 << ROTARY_ENCODER_PIN) == 0) { // if PORTB and operation with 0b0100000 = 0 then it is pressed
		return true;
	} else {
		return false;                                       // it is not pressed
	}
}

ISR(PCINT0_vect) { // both buttons are connected to the same external interrupt.

	if (button_isJoystickPressed() && (myJoystickCallback != NULL)
			&& (PIN_CHANGE_MASK_REGISTER_0
					& 1 << PIN_CHANGE_ENABLE_MASK_JOYSTICK) != 0) {
		myJoystickCallback();

	} else if (button_isRotaryPressed() && (myRotaryCallback != NULL)
			&& (PIN_CHANGE_MASK_REGISTER_0 & 1 << PIN_CHANGE_ENABLE_MASK_ROTARY)
					!= 0) {
		myRotaryCallback();
	}

}

void button_setRotaryButtonCallback(pButtonCallback callback) {
	// implement the race condition here.
	myRotaryCallback = callback;
}

void button_setJoystickButtonCallback(pButtonCallback callback) {
	// implement the race condition here.
	myJoystickCallback = callback;

}

void button_checkState(void* checkP) {

	static uint8_t state[BUTTON_NUM_DEBOUNCE_CHECKS] = { };
	static uint8_t index = 0;
	static uint8_t debouncedState = 0;
	uint8_t lastDebouncedState = debouncedState;

	// each bit in every state byte represents one button
	state[index] = 0;

	if (button_isJoystickPressed()) {
		state[index] |= 1;
	}

	if (button_isRotaryPressed()) {
		state[index] |= 2;
	}

	index++;

	if (index == BUTTON_NUM_DEBOUNCE_CHECKS) {
		index = 0;
	}

	// init compare value and compare with ALL reads, only if
	// we read BUTTON_NUM_DEBOUNCE_CHECKS consistent "1" in the state
	// array, the button at this position is considered pressed

	uint8_t j = 0xFF;

	for (uint8_t i = 0; i < BUTTON_NUM_DEBOUNCE_CHECKS; i++) {
		j = j & state[i];
	}
	debouncedState = j;

	if (lastDebouncedState != debouncedState) { // if debounce state = 0 and last debouncestate not

		if (debouncedState == 1) {  // check 3al bit
			myJoystickCallback();

		}
		if (debouncedState == 2) { // check 3al bit

			myRotaryCallback();

		}

		lastDebouncedState = debouncedState;
	}
}
