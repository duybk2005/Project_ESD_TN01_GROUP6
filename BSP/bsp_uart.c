#include "bsp_uart.h"
#include "uart.h"

void bsp_uart_init(uint32 baud)
{
	UART0_init(baud);
	UART1_init(baud);
}

uint8 bsp_uart_rx(void)
{
	return UART0_recv();
}

void bsp_uart_tx(uint8 data)
{
	UART1_send(data);
}

void bsp_uart_txString(uint8 *s)
{
	UART1_sendString(s);
}
