#ifndef SES_BUTTON_H_
#define SES_BUTTON_H_

/* INCLUDES ******************************************************************/

#include "ses_common.h"
#include <stdbool.h>


extern bool externalInterrupt;

/* FUNCTION PROTOTYPES *******************************************************/
void button_checkState(void*);
/**
 * Initializes rotary encoder and joystick button
 */
void button_init(bool debounce);

/** 
 * Get the state of the joystick button.
 */
bool button_isJoystickPressed(void);

/** 
 * Get the state of the rotary button.
 */
bool button_isRotaryPressed(void);

typedef void (*pButtonCallback)();

void button_setRotaryButtonCallback(pButtonCallback);

void button_setJoystickButtonCallback(pButtonCallback);
//void button_checkState();

#endif /* SES_BUTTON_H_ */
