#ifndef BSP_INTERRUPT_H_
#define BSP_INTERRUPT_H_

#include "std_types.h"
#include "interrupt.h"

typedef enum
{
	BTN_INT0 = 0,
	BTN_INT1 = 1,
	BTN_INT2 = 2
} BSP_Button;

void bsp_interrupt_init(BSP_Button btn, INT_EdgeType edge);

#endif
