#include <avr/io.h>
#include "std_types.h"
#include "bsp_i2c.h"
#include "bme.h"

#define BME280_ADDR  0x77u

static sint32 t_fine;

/* ??c calib 16-bit unsigned t?i addr (LSB tr??c, MSB sau) */
static uint16 bme_read16u(uint8 addr)
{
	uint8 buf[2];
	bsp_i2c_read_frame(BME280_ADDR, addr, buf, 2);
	return (uint16)(((uint16)buf[1] << 8) | (uint16)buf[0]);
}

/* ??c calib 16-bit signed */
static sint16 bme_read16s(uint8 addr)
{
	return (sint16)bme_read16u(addr);
}

/* ================== 1) CONFIG ================== */

void bme_config(void)
{
	uint8 data;

	bsp_i2c_init();

	data = 0xB6;                           /* reset */
	bsp_i2c_write_frame(BME280_ADDR, 0xE0, &data, 1);

	data = 0x00;                           /* skip humidity (oversampling x0) */
	bsp_i2c_write_frame(BME280_ADDR, 0xF2, &data, 1);

	data = 0x27;                           /* temp x1, press x1, normal mode */
	bsp_i2c_write_frame(BME280_ADDR, 0xF4, &data, 1);

	data = 0x00;                           /* config default */
	bsp_i2c_write_frame(BME280_ADDR, 0xF5, &data, 1);
}

/* ================== 2) READ TEMPERATURE (°C * 100) ================== */

uint32 bme_read_oC(void)
{
	uint8 buf[3];
	uint32 adc_T;
	uint16 T1;
	sint16  T2, T3;
	sint32  var1, var2;

	/* ??c raw temperature (0xFA..0xFC) */
	bsp_i2c_read_frame(BME280_ADDR, 0xFA, buf, 3);
	adc_T = ((uint32)buf[0] << 12) | ((uint32)buf[1] << 4) | ((uint32)buf[2] >> 4);

	/* calib temperature */
	T1 = bme_read16u(0x88);
	T2 = bme_read16s(0x8A);
	T3 = bme_read16s(0x8C);

	var1 = ((((sint32)adc_T >> 3) - ((sint32)T1 << 1)) * (sint32)T2) >> 11;
	var2 = (((((sint32)adc_T >> 4) - (sint32)T1) *
	(((sint32)adc_T >> 4) - (sint32)T1)) >> 12) * (sint32)T3 >> 14;

	t_fine = var1 + var2;

	/* tr? v? °C * 100 */
	return (t_fine * 5 + 128) >> 8;
}

/* ================== 3) READ PRESSURE (hPa) ================== */

uint32 bme_read_hPa(void)
{
	uint8 buf[3];
	uint32 adc_P;
	uint16 P1;
	sint32  P2, P3, P4, P5, P6, P7, P8, P9;
	sint32  var1, var2;
	uint32 p_Pa;

	/* ph?i ??c nhi?t ?? tr??c ?? có t_fine */
	(void)bme_read_oC();

	/* ??c raw pressure (0xF7..0xF9) */
	bsp_i2c_read_frame(BME280_ADDR, 0xF7, buf, 3);
	adc_P = ((uint32)buf[0] << 12) | ((uint32)buf[1] << 4) | ((uint32)buf[2] >> 4);

	/* calib pressure */
	P1 = bme_read16u(0x8E);
	P2 = bme_read16s(0x90);
	P3 = bme_read16s(0x92);
	P4 = bme_read16s(0x94);
	P5 = bme_read16s(0x96);
	P6 = bme_read16s(0x98);
	P7 = bme_read16s(0x9A);
	P8 = bme_read16s(0x9C);
	P9 = bme_read16s(0x9E);

	var1 = (t_fine >> 1) - (sint32)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * P6;
	var2 += (var1 * P5) << 1;
	var2 = (var2 >> 2) + (P4 << 16);
	var1 = (((P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) +
	((P2 * var1) >> 1)) >> 18;
	var1 = ((32768 + var1) * (sint32)P1) >> 15;
	if (var1 == 0)
	return 0;

	p_Pa = (uint32)((sint32)1048576 - (sint32)adc_P);
	p_Pa = (p_Pa - (uint32)(var2 >> 12)) * 3125u;

	if (p_Pa < 0x80000000u)
	p_Pa = (p_Pa << 1) / (uint32)var1;
	else
	p_Pa = (p_Pa / (uint32)var1) << 1;

	var1 = ((sint32)P9 * (sint32)(((p_Pa >> 3) * (p_Pa >> 3)) >> 13)) >> 12;
	var2 = ((sint32)(p_Pa >> 2) * (sint32)P8) >> 13;
	p_Pa = (uint32)((sint32)p_Pa + ((var1 + var2 + P7) >> 4));

	/* ??i Pascal ? hPa */
	return p_Pa / 100u;
}
