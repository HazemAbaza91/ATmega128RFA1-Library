// INCLUDES *****************************************************************/
#include "ses_adc.h"
#include "ses_common.h"
#include "ses_lcd.h"
#include "util/atomic.h"
// DEFINES & MACROS *********************************************************/
#define TEMP_SENSOR_PORT           			PORTF
#define TEMP_SENSOR_PIN		        		2

#define LIGHT_SENSOR_PORT          			PORTF
#define LIGHT_SENSOR_PIN            		4

#define JOYSTICK_PORT              			PORTF
#define JOYSTICK_PIN                		5

#define MICROPHONE_PORT            			PORTF
#define MICROPHONE_PIN_0             		0
#define MICROPHONE_PIN_1             		1

#define POWER_REDUCTION_MODE_REGISTER       PRR0
#define POWER_REDUCTION_MODE_PIN            0

#define ADC_VREF_SRC           			    (1<<REFS1)|(1<<REFS0)
#define ADC_VREF_REGISTER               	ADMUX

#define ADC_RESULT_ADJ_REGISTER         	ADMUX
#define ADC_RESULT_TORIGHT_PIN            	5

#define ADC_PRESCALE          				((1<<ADPS0)|(1<<ADPS1))|((0<<ADPS2))
#define ADC_CLOCK_REGISTER              	ADCSRA

#define ADC_AUTO_TRIGGRERER_REGISTER    	ADCSRA
#define ADC_AUTO_TRIGGRERER_PIN           	5

#define ADC_ENABLE_REGISTER             	ADCSRA
#define ADC_ENABLE_PIN                    	ADEN

#define ADC_START_REGISTER               	ADCSRA
#define ADC_START_PIN                     	6

#define ADC_ISCALCULATING_REGISTER         	ADCSRA
#define ADC_ISCALCULATING_PIN               6

#define ADC_CHANNEL_SELECT_REGISTER       	ADMUX
#define ADC_CHANNEL_SELECT_PIN              0

#define NO_DIRECTION_POS                    1000
#define RIGHT_RAW                           200
#define LEFT_RAW                            600
#define UP_RAW                              400
#define DOWN_RAW                            800
#define TOLERANCE                           50

#define ADMUX_REGISTER                      ADMUX
#define ADMUX_CLEAR_VALUE                   0xf0

#define ADC_TEMP_MAX                        40
#define ADC_TEMP_MIN                        20
#define ADC_TEMP_RAW_MAX                    257
#define ADC_TEMP_RAW_MIN                    482
#define ADC_TEMP_FACTOR                     100

// FUNCTION DEFINITION ******************************************************/

void adc_init(void) {

	DDR_REGISTER(TEMP_SENSOR_PORT) &= ~(1 << TEMP_SENSOR_PIN); // configure data register for temperature sensor
	TEMP_SENSOR_PORT &= ~(1 << TEMP_SENSOR_PIN); // Deactivate pull up resistor for temperature

	DDR_REGISTER(LIGHT_SENSOR_PORT) &= ~(1 << LIGHT_SENSOR_PIN); // configure data register for light sensor
	LIGHT_SENSOR_PORT &= ~(1 << LIGHT_SENSOR_PIN); // Deactivate pull up resistor for light

	DDR_REGISTER(JOYSTICK_PORT) &= ~(1 << JOYSTICK_PIN); // configure data register for joystick sensor
	JOYSTICK_PORT &= ~(1 << JOYSTICK_PIN); // Deactivate pull up resistor for joystick

	DDR_REGISTER( MICROPHONE_PORT) &= ~((1 << MICROPHONE_PIN_0)
			| (1 << MICROPHONE_PIN_1)); // configure data register for microphone sensor at pin 0 and 1
	MICROPHONE_PORT &= ~((1 << MICROPHONE_PIN_0) | (1 << MICROPHONE_PIN_1)); // Deactivate pull up resistor for microphone

	POWER_REDUCTION_MODE_REGISTER &= ~(1 << POWER_REDUCTION_MODE_PIN); // Disable power reduction mode for the ADC

	ADC_RESULT_ADJ_REGISTER &= ~(1 << ADC_RESULT_TORIGHT_PIN); //set the ADC result right adjusted

	ADC_VREF_REGISTER |= ADC_VREF_SRC; // set Ref.Volatage to Internal 1.6V Voltage Reference

	ADC_CLOCK_REGISTER |= ADC_PRESCALE;   // Setting ADC Clock to 2MHz

	ADC_AUTO_TRIGGRERER_REGISTER &= ~(1 << ADC_AUTO_TRIGGRERER_PIN); //Deactivate Auto Triggrer

	ADC_ENABLE_REGISTER |= (1 << ADC_ENABLE_PIN);                 // Enable ADC

}

uint16_t adc_read(uint8_t adc_channel) {

	uint16_t ADC_RESULT;
	uint8_t sreg;        // Variable to save the value of the SREG Register

	adc_init();

	if ((adc_channel > ADC_NUM) | (adc_channel < 0)) {
		return ADC_INVALID_CHANNEL;
	}

	else {

		ADMUX_REGISTER &= ADMUX_CLEAR_VALUE;

		ADC_CHANNEL_SELECT_REGISTER |= (adc_channel << ADC_CHANNEL_SELECT_PIN); //Select the ADC channels that will be used

		ADC_START_REGISTER |= (1 << ADC_START_PIN);    //Starting the Conversion

		while ((ADC_ISCALCULATING_REGISTER & (1 << ADC_ISCALCULATING_PIN)));     // Waiting Until Conversion Terminates

		ADC_START_REGISTER &= ~(1 << ADC_START_PIN);

		/* Reading the ADC
		 * register atomically
		 */

		sreg = SREG; /* Save global interrupt flag */

		cli();
		/* Disable interrupts */

		ADC_RESULT = ADC; /* Read the ADC Values*/

		SREG = sreg; /* Restore global interrupt flag */

		__asm__ __volatile__ ("" ::: "memory");

		return (ADC_RESULT);

	}
}

uint8_t adc_getJoystickDirection() {

	uint16_t a = adc_read(ADC_JOYSTICK_CH);    // check

	if (a < (RIGHT_RAW + TOLERANCE)) {
		return RIGHT;
	} else if (a < (UP_RAW + TOLERANCE)) {
		return UP;
	} else if (a < (LEFT_RAW + TOLERANCE)) {
		return LEFT;
	} else if (a < (DOWN_RAW + TOLERANCE)) {
		return DOWN;
	} else {
		return NO_DIRECTION;
	}

}

int16_t adc_getTemperature() {

	int16_t adc = adc_read(ADC_TEMP_CH);

	int16_t slope = (ADC_TEMP_MAX - ADC_TEMP_MIN)
			/ (ADC_TEMP_RAW_MAX - ADC_TEMP_RAW_MIN);
	int16_t offset = ADC_TEMP_MAX - (ADC_TEMP_RAW_MAX * slope);
	return (adc * slope + offset) / ADC_TEMP_FACTOR;
}

int16_t adc_getLight() {
	int16_t adc = adc_read(ADC_LIGHT_CH);

	return adc;
}

void adc_print(void) {
	lcd_clear();
	lcd_setCursor(0, 0);

	fprintf(lcdout, "*the Temp=%d C*", adc_getTemperature());

	lcd_setCursor(1, 1);

	fprintf(lcdout, "**the light=%d**", adc_getLight());

	lcd_setCursor(2, 2);

	fprintf(lcdout, "***position=%d", adc_getJoystickDirection());

	lcd_setCursor(3, 3);

	fprintf(lcdout, "*temp raw=%d", adc_read(ADC_TEMP_CH));

	_delay_ms(1000);

}

