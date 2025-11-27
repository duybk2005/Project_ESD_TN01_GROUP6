#include "timer.h"
#include <avr/io.h>
#include "common_macros.h"

void timer_int(void)
{
	SET_BIT(DDRD, PD4);   /* PD4 = OC1B */

	TCCR1A = (1<<COM1B1) | (1<<WGM11) | (1<<WGM10);
	TCCR1B = (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (1<<CS10);  /* prescaler 1024 */

	OCR1A = 0;
	OCR1B = 0;
}

void timer_make_wave(uint16 *ocra, uint16 *ocrb)
{
	OCR1A = *ocra;
	OCR1B = *ocrb;
}
