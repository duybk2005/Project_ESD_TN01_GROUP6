#ifndef BSP_TIMER_H_
#define BSP_TIMER_H_

#include "std_types.h"

void bsp_timer_int(void);
void bsp_make_wave(uint16 cycle_ms, uint8 duty_cycle);

#endif /* BSP_TIMER_H_ */
