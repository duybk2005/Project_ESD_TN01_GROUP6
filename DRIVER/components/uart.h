#ifndef UART_H_
#define UART_H_

#include "std_types.h"

void UART0_init(uint32 baud);
void UART1_init(uint32 baud);

void UART0_send(uint8 data);
void UART1_send(uint8 data);

uint8 UART0_recv(void);
uint8 UART1_recv(void);

void UART0_sendString(uint8 *s);
void UART1_sendString(uint8 *s);

void UART0_recvString(uint8 *s);
void UART1_recvString(uint8 *s);

#endif /* UART_H_ */
