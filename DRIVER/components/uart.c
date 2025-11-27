#include "uart.h"
#include <avr/io.h>
#include "common_macros.h"
#include "board.h"

void uart_set(uint8_t idx, uint32 baud)
{
	uint16 ubrr = (CPU_FREQ / (8UL * baud)) - 1;

	if(idx == 0)
	{
		SET_BIT(UCSR0A, U2X0);
		SET_BIT(UCSR0B, RXEN0);
		SET_BIT(UCSR0B, TXEN0);
		SET_BIT(UCSR0C, UCSZ01);
		SET_BIT(UCSR0C, UCSZ00);
		UBRR0H = ubrr >> 8;
		UBRR0L = ubrr;
	}
	else
	{
		SET_BIT(UCSR1A, U2X1);
		SET_BIT(UCSR1B, RXEN1);
		SET_BIT(UCSR1B, TXEN1);
		SET_BIT(UCSR1C, UCSZ11);
		SET_BIT(UCSR1C, UCSZ10);
		UBRR1H = ubrr >> 8;
		UBRR1L = ubrr;
	}
}

void UART0_init(uint32 baud){ uart_set(0, baud); }
void UART1_init(uint32 baud){ uart_set(1, baud); }

void UART0_send(uint8 data)
{
	while(BIT_IS_CLEAR(UCSR0A, UDRE0));
	UDR0 = data;
}

void UART1_send(uint8 data)
{
	while(BIT_IS_CLEAR(UCSR1A, UDRE1));
	UDR1 = data;
}

uint8 UART0_recv(void)
{
	while(BIT_IS_CLEAR(UCSR0A, RXC0));
	return UDR0;
}

uint8 UART1_recv(void)
{
	while(BIT_IS_CLEAR(UCSR1A, RXC1));
	return UDR1;
}

void UART0_sendString(uint8 *s)
{
	while(*s) UART0_send(*s++);
}

void UART1_sendString(uint8 *s)
{
	while(*s) UART1_send(*s++);
}

void UART0_recvString(uint8 *s)
{
	uint8 c;
	do { c = UART0_recv(); *s++ = c; } while(c != '#');
	*(s-1) = 0;
}

void UART1_recvString(uint8 *s)
{
	uint8 c;
	do { c = UART1_recv(); *s++ = c; } while(c != '#');
	*(s-1) = 0;
}
