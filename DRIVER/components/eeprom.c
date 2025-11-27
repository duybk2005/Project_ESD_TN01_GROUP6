#include "eeprom.h"
#include <avr/eeprom.h>

uint8 eeprom_read8(uint16 addr)
{
	return eeprom_read_byte((uint8*)addr);
}

void eeprom_write8(uint16 addr, uint8 value)
{
	eeprom_update_byte((uint8*)addr, value);
}

void eeprom_readBlock(uint16 addr, uint8 *buf, uint16 len)
{
	uint16 i;
	for(i = 0; i < len; i++)
	buf[i] = eeprom_read8(addr + i);
}

void eeprom_writeBlock(uint16 addr, const uint8 *buf, uint16 len)
{
	uint16 i;
	for(i = 0; i < len; i++)
	eeprom_write8(addr + i, buf[i]);
}
