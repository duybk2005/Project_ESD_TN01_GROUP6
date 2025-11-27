#include "bsp_adc.h"
#include "adc.h"
#include "std_types.h"

void bsp_adc_init(void)
{
	adc_config();
}

uint16 bsp_adc_read(uint8 ch)
{
	return adc_read(ch);
}
