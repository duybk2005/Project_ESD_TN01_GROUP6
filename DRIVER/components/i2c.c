#include "gpio.h"
#include "std_types.h"
#include "common_macros.h"
#include <avr/io.h>


void TWI_init()
{
	TWBR = 0x02;
	TWSR = 0x00; //bit-rate=400.000kbps
	TWCR = 1<<TWEN;
}
void TWI_start(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while(BIT_IS_CLEAR(TWCR,TWINT));
}
void TWI_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}
void TWI_writeByte(uint8 data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(BIT_IS_CLEAR(TWCR,TWINT));
}
uint8 TWI_readByteWithACK(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while(BIT_IS_CLEAR(TWCR,TWINT));
    return TWDR;
}
uint8 TWI_readByteWithNACK(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN);
    while(BIT_IS_CLEAR(TWCR,TWINT));
    return TWDR;
}
uint8 TWI_getStatus(void)
{
	uint8 status;
	status = TWSR & 0xF8;
	return status;
}