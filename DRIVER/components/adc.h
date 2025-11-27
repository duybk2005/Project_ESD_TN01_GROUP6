#ifndef ADC_H_
#define ADC_H_

#include "std_types.h"

void   adc_config(void);
uint16 adc_read(uint8 ch);

#endif