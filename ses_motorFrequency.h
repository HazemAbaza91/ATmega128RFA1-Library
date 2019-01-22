#ifndef SES_MOTORFREQUENCY_H_
#define SES_MOTORFREQUENCY_H_

#include <inttypes.h>
#include <avr/io.h>
#include "ses_common.h"

void motorFrequency_init();
uint16_t motorFrequency_getRecent();
uint16_t motorFrequency_getMedian();
void motorFrequency_set(uint16_t);
void motorSet(bool);

#endif /* SES_MOTORFREQUENCY_H_ */
