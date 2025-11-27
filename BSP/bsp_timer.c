#include "bsp_timer.h"
#include "timer.h"
#include "board.h"          /* ch?a CPU_FREQ */
#include <avr/io.h>

#define TIMER1_PRESCALER   1024UL

static void compute_ocr(uint16 cycle_ms, uint8 duty, uint16 *ocra, uint16 *ocrb)
{
	uint32 ticks;

	if(cycle_ms == 0U || duty > 100U)
	{
		*ocra = 0;
		*ocrb = 0;
		return;
	}

	ticks = (CPU_FREQ / TIMER1_PRESCALER) * (uint32)cycle_ms / 1000UL;
	ticks /= 2;  
	if(ticks > 65535UL) ticks = 65535UL;

	if(ticks <= 1UL)
	{
		*ocra = 0;
		*ocrb = 0;
		return;
	}

	*ocra = (uint16)(ticks - 1U);
	*ocrb = (uint16)(((uint32)(*ocra) * duty) / 100U);
}

void bsp_timer_int(void)
{
	timer_int();
}

void bsp_make_wave(uint16 cycle_ms, uint8 duty_cycle)
{
	uint16 ocra, ocrb;

	compute_ocr(cycle_ms, duty_cycle, &ocra, &ocrb);

	/* g?i driver b?ng pointer */
	timer_make_wave(&ocra, &ocrb);
}
