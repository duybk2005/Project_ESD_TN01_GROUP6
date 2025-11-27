#include "bsp_timer.h"
#include "std_types.h"

void test_timer(void)
{
	bsp_timer_int();

	while(1)
	{
		bsp_make_wave(1000, 50);   /* 1000ms – 50% */
	}
}
