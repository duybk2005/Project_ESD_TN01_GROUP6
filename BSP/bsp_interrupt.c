#include "bsp_interrupt.h"

void bsp_interrupt_init(BSP_Button btn, INT_EdgeType edge)
{
	switch(btn)
	{
		case BTN_INT0: INT0_init(edge); break;
		case BTN_INT1: INT1_init(edge); break;
		case BTN_INT2: INT2_init(edge); break;
		default: break;
	}
}
