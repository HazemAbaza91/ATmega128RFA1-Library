/*
 * ses_rotary.h
 *
 *  Created on: Jul 3, 2018
 *      Author: lenovo
 */
#include "ses_common.h"
#include <stdbool.h>


#ifndef SES_ROTARY_H_
#define SES_ROTARY_H_


/**type of function pointer used as callback
 */
typedef void (*pTypeRotaryCallback)();

/**
 * Initializes rotary encoder.
 */

void rotary_init();

/**
 * Sets a function to be called when rotary encoder rotate clockwise.
 *
 * @param cb  pointer to the callback function; if NULL, no callback
 *            will be executed.
 */
void rotary_setClockwiseCallback(pTypeRotaryCallback);

/**
 * Sets a function to be called when rotary encoder rotate c-clockwise.
 *
 * @param cb  pointer to the callback function; if NULL, no callback
 *            will be executed.
 */
void rotary_setCounterClockwiseCallback(pTypeRotaryCallback);

/**
 * function to determin whether the rotary is rotaed CW or CCW.
 *
 * @param void pointer.
 */


void rotary_checkState(void* checkP);


/**
 *  a function that returns the state of BIN_ROTART1.
 *
 * @param void
 *
 */

bool bin1_state();


/**
 *  a function that returns the state of BIN_ROTART2.
 *
 * @param void
 *
 */


bool bin2_state();




#endif /* SES_ROTARY_H_ */
