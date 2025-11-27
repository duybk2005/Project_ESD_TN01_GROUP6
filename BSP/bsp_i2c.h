#ifndef BSP_I2C_H_
#define BSP_I2C_H_

#include "std_types.h"

void  bsp_i2c_init(void);

uint8 bsp_i2c_readAck(void);
uint8 bsp_i2c_readNack(void);
void  bsp_i2c_write(uint8 data);

/* ??c len byte t? addr_reg c?a slave vào buf */
void  bsp_i2c_read_frame(uint8 addr_slave, uint8 addr_reg, uint8 *buf, uint8 len);

/* Ghi len byte t? data vào addr_reg c?a slave */
void  bsp_i2c_write_frame(uint8 addr_slave, uint8 addr_reg, const uint8 *data, uint8 len);

#endif /* BSP_I2C_H_ */
