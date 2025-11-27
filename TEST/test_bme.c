#include "bme.h"
#include "gpio.h"
#include "std_types.h"

void test_bme(void)
{
	uint32 t;
	
	bme_config();
	GPIO_setupPortDirection(PORTB_ID, PORT_OUTPUT);

	while(1)
	{
		t = bme_read_oC()/100 ;      /* °C x 100 */
		GPIO_writePort(PORTB_ID, (uint8)t);
	}
}
	