#ifndef BSP_ADC_H_
#define BSP_ADC_H_

#include "std_types.h"

void   bsp_adc_init(void);
uint16 bsp_adc_read(uint8 ch);

#endif
