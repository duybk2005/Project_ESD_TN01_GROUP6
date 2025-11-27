#include "test_interrupt.h"
#include "bsp_interrupt.h"
#include "gpio.h"
#include <avr/interrupt.h>

void test_interrupt(void)
{
	GPIO_setupPortDirection(PORTA_ID, PORT_OUTPUT);
	bsp_interrupt_init(BTN_INT2, INT_EDGE_FALL);
	sei();

	while(1){}
}

