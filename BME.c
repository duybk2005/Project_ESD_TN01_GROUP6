
#include <avr/io.h>
#include "bsp.h"

void bme_write(unsigned char addr, unsigned char data)
{
	i2c_start();
	i2c_write(0x77<<1|0);
	i2c_write(addr);
	i2c_write(data);
	i2c_stop();
}
uint32_t bme_read(unsigned char addr)
{
	unsigned char data[3];
	i2c_start();
	i2c_write(0x77<<1|0);
	i2c_write(addr);
	i2c_start();
	i2c_write(0x77<<1|1);
	
	data[0] = i2c_read();
	data[1] = i2c_read();
	data[2] = i2c_read();
	i2c_nack();
	i2c_stop();
	return ( ((uint32_t)data[0]<<12) | ((uint32_t)data[1]<<4) | ((uint32_t)data[2]>>4) );
}

void bme_config()
{
	i2c_init();
	bme_write(0xE0,0xB6);		//reset
	bme_write(0xF2,0x00);		//skip humidity
	bme_write(0xF4,0x27);		//ov_samp_tx1,px1,mode normal
	bme_write(0xF5,0x00);		//not use in normal mode
}
//convert
uint16_t bme_read16u(uint8_t addr)
{
	uint8_t data[2];

	i2c_start();
	i2c_write(0x77<<1 | 0);   // SLA+W
	i2c_write(addr);          // reg addr

	i2c_start();
	i2c_write(0x77<<1 | 1);   // SLA+R

	data[0] = i2c_read();     // LSB
	data[1] = i2c_read();     // MSB
	i2c_nack();
	i2c_stop();

	// hi = data[1], lo = data[0]  (little-endian)
	return (uint16_t)(((uint16_t)data[1] << 8) | (uint16_t)data[0]);
}

int16_t bme_read16s(uint8_t addr)
{
	return (int16_t)bme_read16u(addr);
}


int32_t t_fine;
int32_t convertCelcius
(int32_t adc_T)
{
	uint16_t T1 = bme_read16u(0x88);
	int16_t T2 = bme_read16s(0x8A);
	int16_t T3 = bme_read16s(0x8C);
	
	int32_t var1 = ((((int32_t)adc_T >> 3) - ((int32_t)T1 << 1)) * (int32_t)T2) >> 11;
	int32_t var2 = (((((int32_t)adc_T >> 4) - (int32_t)T1) *(((int32_t)adc_T >> 4) - (int32_t)T1)) >> 12) *	(int32_t)T3 >> 14;
	(t_fine) = var1 + var2;
	return ((t_fine) * 5 + 128) >> 8;   //  C * 100
}
uint32_t convertPascal(uint32_t adc_P)
{
	uint16_t P1=bme_read16u(0x8E);
	int32_t P2=bme_read16s(0x90);
	int32_t P3=bme_read16s(0x92);
	int32_t P4=bme_read16s(0x94);
	int32_t P5=bme_read16s(0x96);
	int32_t P6=bme_read16s(0x98);
	int32_t P7=bme_read16s(0x9A);
	int32_t P8=bme_read16s(0x9C);
	int32_t P9=bme_read16s(0x9E);
	
	int32_t var1 = (t_fine >> 1) - (int32_t)64000;
	int32_t var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * P6;
	var2 += ((var1 * P5) << 1);
	var2 = (var2 >> 2) + (P4 << 16);
	var1 = (((P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + (((P2)*var1)>>1))>>18;
	var1 = ((32768 + var1) * (int32_t)P1) >> 15;
	if (var1 == 0) return 0;

	uint32_t p = (((uint32_t)((int32_t)1048576 - (int32_t)adc_P) - (var2 >> 12)) * 3125u);
	if (p < 0x80000000u)
	p = (p << 1) / (uint32_t)var1;
	else
	p = (p / (uint32_t)var1) << 1;

	var1 = ((int32_t)P9 * (int32_t)(((p >> 3) * (p >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(p >> 2) * (int32_t)P8) >> 13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + P7) >> 4));
	return p;   // Pascal
}
