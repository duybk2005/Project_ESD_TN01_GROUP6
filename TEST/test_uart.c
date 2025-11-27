#include "bsp_uart.h"
#include "print_uart.h"
#include "std_types.h"

void test_uart(void)
{
	uint8 v;

	bsp_uart_init(9600);

	while(1)
	{
		v = bsp_uart_rx();

		print_uart1_str("value : <");
		print_uart1_int(v);
		bsp_uart_tx('>');
		bsp_uart_tx('\n');
	}
}
