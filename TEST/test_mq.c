#include "test_mq.h"
#include "std_types.h"
#include "gpio.h"
#include "mq.h"
#include "bsp_adc.h"    // c?n ?? có prototype bsp_adc_init()

void test_mq(uint8 channel)
{
	uint16 v;

	bsp_adc_init();

	GPIO_setupPortDirection(PORTB_ID, PORT_OUTPUT);
	GPIO_setupPortDirection(PORTD_ID, PORT_OUTPUT);

	while(1)
	{
		v = mq_read(channel, 5000u);

		GPIO_writePort(PORTB_ID, (uint8)(v >> 8));
		GPIO_writePort(PORTD_ID, (uint8)v);
	}
}
