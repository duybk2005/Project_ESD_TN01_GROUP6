#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "bsp.h"


#define EE_BASE_ADDR ((uint8_t*)0x0100)

// ---------------- UART1 (TX) ----------------


void usart1_message_tx(const char *s)
{
	while (*s) usart1_char_tx(*s++);
}

// ---------------- UART0 (RX) ----------------

void send_int(int32_t x)
{
	char b[12];
	uint8_t i = 0;

	if (x == 0) {
		usart1_char_tx('0');
		return;
	}

	if (x < 0) {
		usart1_char_tx('-');
		x = -x;
	}

	while (x > 0) {
		b[i++] = (x % 10) + '0';
		x /= 10;
	}

	while (i--) {
		usart1_char_tx(b[i]);
	}
}
ISR(INT2_vect)
{
	usart1_message_tx("CONFIG THRESHOLD\r\n");

	// ===== TEMP =====
	usart1_message_tx("temp_threshold:   [");
	uint8_t v0 = usart0_char_rx();
	eeprom_update_byte(EE_BASE_ADDR + 0, v0);
	send_int(v0);

	usart1_message_tx(" ; ");

	uint8_t v1 = usart0_char_rx();
	eeprom_update_byte(EE_BASE_ADDR + 1, v1);
	send_int(v1);

	usart1_message_tx("]\r\n");

	// ===== PRESS =====
	usart1_message_tx("press_threshold:  [");
	uint8_t v2 = usart0_char_rx();
	eeprom_update_byte(EE_BASE_ADDR + 2, v2);
	send_int(v2);

	usart1_message_tx(" ; ");

	uint8_t v3 = usart0_char_rx();
	eeprom_update_byte(EE_BASE_ADDR + 3, v3);
	send_int(v3);

	usart1_message_tx("]\r\n");

	// ===== MQ4 =====
	usart1_message_tx("mq4_threshold:    [");
	uint8_t v4 = usart0_char_rx();
	eeprom_update_byte(EE_BASE_ADDR + 4, v4);
	send_int(v4);

	usart1_message_tx(" ; ");

	uint8_t v5 = usart0_char_rx();
	eeprom_update_byte(EE_BASE_ADDR + 5, v5);
	send_int(v5);

	usart1_message_tx("]\r\n");

	// ===== MQ7 =====
	usart1_message_tx("mq7_threshold:    [");
	uint8_t v6 = usart0_char_rx();
	eeprom_update_byte(EE_BASE_ADDR + 6, v6);
	send_int(v6);

	usart1_message_tx(" ; ");

	uint8_t v7 = usart0_char_rx();
	eeprom_update_byte(EE_BASE_ADDR + 7, v7);
	send_int(v7);

	usart1_message_tx("]\r\n");

	while ((PINB & (1<<PB2)) == 0);
}




void thresRead(uint8_t *dst)
{
	for (uint8_t i = 0; i < 8; i++)
	{
		dst[i] = eeprom_read_byte(EE_BASE_ADDR + i);
	}
}


