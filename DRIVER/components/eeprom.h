#ifndef EEPROM_H_
#define EEPROM_H_

#include "std_types.h"

uint8  eeprom_read8(uint16 addr);
void   eeprom_write8(uint16 addr, uint8 value);

void   eeprom_readBlock(uint16 addr, uint8 *buf, uint16 len);
void   eeprom_writeBlock(uint16 addr, const uint8 *buf, uint16 len);

#endif
