#ifndef BSP_UART_H_
#define BSP_UART_H_

#include "std_types.h"

void bsp_uart_init(uint32 baud);

uint8 bsp_uart_rx(void);
void  bsp_uart_tx(uint8 data);
void  bsp_uart_txString(uint8 *s);

#endif /* BSP_UART_H_ */
