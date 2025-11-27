#include "bsp_eeprom.h"
#include "eeprom.h"

#define EE_ADDR_THRESH  0x00u   /* 8 byte */

void bsp_eeprom_loadThreshold(uint8 *thres)
{
	eeprom_readBlock(EE_ADDR_THRESH, thres, 8);
}

void bsp_eeprom_saveThreshold(const uint8 *thres)
{
	eeprom_writeBlock(EE_ADDR_THRESH, thres, 8);
}
