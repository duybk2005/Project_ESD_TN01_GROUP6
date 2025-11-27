#include "bsp_i2c.h"
#include "i2c.h"

void bsp_i2c_init(void)
{
	TWI_init();
}

void bsp_i2c_start(void)
{
	TWI_start();
}

void bsp_i2c_restart(void)
{
	TWI_start();
}

void bsp_i2c_stop(void)
{
	TWI_stop();
}

uint8 bsp_i2c_readAck(void)
{
	return TWI_readByteWithACK();
}

uint8 bsp_i2c_readNack(void)
{
	return TWI_readByteWithNACK();
}

void bsp_i2c_write(uint8 data)
{
	TWI_writeByte(data);
}

void bsp_i2c_read_frame(uint8 addr_slave, uint8 addr_reg, uint8 *buf, uint8 len)
{
	uint8 i;
	if(len == 0) return;

	bsp_i2c_start();
	bsp_i2c_write((addr_slave << 1) | 0);
	bsp_i2c_write(addr_reg);

	bsp_i2c_restart();
	bsp_i2c_write((addr_slave << 1) | 1);

	for(i = 0; i < len - 1; i++)
	buf[i] = bsp_i2c_readAck();

	buf[len - 1] = bsp_i2c_readNack();
	bsp_i2c_stop();
}

void bsp_i2c_write_frame(uint8 addr_slave, uint8 addr_reg, const uint8 *data, uint8 len)
{
	uint8 i;
	if(len == 0) return;

	bsp_i2c_start();
	bsp_i2c_write((addr_slave << 1) | 0);
	bsp_i2c_write(addr_reg);

	for(i = 0; i < len; i++)
	bsp_i2c_write(data[i]);

	bsp_i2c_stop();
}
