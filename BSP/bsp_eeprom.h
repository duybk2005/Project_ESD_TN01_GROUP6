#ifndef BSP_EEPROM_H_
#define BSP_EEPROM_H_

#include "std_types.h"

void bsp_eeprom_loadThreshold(uint8 *thres);
void bsp_eeprom_saveThreshold(const uint8 *thres);

#endif
