#include <avr/io.h>
#include "adc.h"
#include "common_macros.h"
#include "std_types.h"
void adc_config(void)
{
	ADMUX  = (1<<REFS0);
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

uint16 adc_read(uint8 ch)
{
	uint16 r;

	ch &= 0x01;
	ADMUX = (ADMUX & 0xFE) | ch;

	SET_BIT(ADCSRA, ADSC);
	while(BIT_IS_SET(ADCSRA, ADSC));

	r  = ADCL;
	r |= ((uint16)ADCH << 8);

	return r;
}
