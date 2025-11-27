#include "bsp_timer.h"
#include "alarm.h"

void high_alarm(void)
{
	bsp_make_wave(500, 90u);
}

void low_alarm(void)
{
	bsp_make_wave(500, 50u);
}

void pause_wave(void)
{
	bsp_make_wave(0u, 0u);
}