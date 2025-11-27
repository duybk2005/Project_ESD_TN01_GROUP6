#include "mq.h"
#include "bsp_adc.h"
#include "std_types.h"

uint16 mq_read(uint8 ch, uint16 vref_mV)
{
	uint16 adc_raw = bsp_adc_read(ch);
	uint32 voltage = ((uint32)adc_raw * (uint32)vref_mV) / 1024UL;
	return (uint16)voltage;
}
