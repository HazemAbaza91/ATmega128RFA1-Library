/*
 * ses_pwm.c
 *
 *  Created on: 16 Jun 2018
 *      Author: Hassan
 */
#include "ses_pwm.h"
#include "ses_timer.h"




void pwm_init(void) {

	DDR_REGISTER(PORTG) |= (1 << PG5);    //set the pin as O/P

	timer0_start();
}

void pwm_setDutyCycle(uint8_t dutyCycle) {

	OCR0B = dutyCycle;
}
