#include "print_uart.h"
#include "bsp_uart.h"
#include "std_types.h"

void print_uart1_str(const char *s)
{
	while(*s)
	{
		bsp_uart_tx((uint8)*s);
		s++;
	}
}

void print_uart1_int(uint16 value)
{
	char buf[5];
	uint8 i = 0;

	if(value == 0)
	{
		bsp_uart_tx('0');
		return;
	}

	while(value > 0 && i < 5)
	{
		buf[i++] = (char)('0' + (value % 10u));
		value /= 10u;
	}

	while(i > 0)
	{
		bsp_uart_tx((uint8)buf[--i]);
	}
}
