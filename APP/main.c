#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

#include "std_types.h"
#include "bsp_uart.h"
#include "print_uart.h"
#include "bsp_adc.h"
#include "bsp_timer.h"
#include "bme.h"
#include "alarm.h"
#include "interrupt.h"

#define EE_BASE_ADDR 0x00u

int main(void)
{
	uint8 thres[8];
	uint8 state = 0;
	uint8 warning_found, danger_found;
	uint8 i;

	bsp_uart_init(9600);
	bsp_adc_init();
	bsp_timer_int();
	bme_config();
	INT2_init(INT_EDGE_FALL);

	sei();

	while (1)
	{
		for(i = 0u; i < 8u; i++)
		{
			thres[i] = eeprom_read_byte((uint8_t*)(EE_BASE_ADDR + i));
		}

		/* BME */
		sint32 temp_x100  = (sint32)bme_read_oC();
		uint32 press_hPa  = bme_read_hPa();
		sint32 tempReal   = temp_x100 / 100;
		uint32 pressReal  = press_hPa;

		/* MQ ADC */
		uint16 metanValue = (uint16)((uint32)bsp_adc_read(0) * 5u / 1024u);
		uint16 coValue    = (uint16)((uint32)bsp_adc_read(1) * 5u / 1024u);

		danger_found  = 0u;
		warning_found = 0u;

		/* ===== DANGER ===== */
		if (tempReal > thres[1])
		{
			print_uart1_str("DANGER at temperature ");
			print_uart1_int((uint16)tempReal);
			print_uart1_str("\r\n");
			high_alarm();
			state = 2u;
			danger_found = 1u;
		}
		else if (pressReal > thres[3])
		{
			print_uart1_str("DANGER at pressure ");
			print_uart1_int((uint16)pressReal);
			print_uart1_str("\r\n");
			high_alarm();
			state = 2u;
			danger_found = 1u;
		}
		else if (metanValue > thres[5])
		{
			print_uart1_str("DANGER at metan ");
			print_uart1_int(metanValue);
			print_uart1_str("\r\n");
			high_alarm();
			state = 2u;
			danger_found = 1u;
		}
		else if (coValue > thres[7])
		{
			print_uart1_str("DANGER at carbonmonoxide ");
			print_uart1_int(coValue);
			print_uart1_str("\r\n");
			high_alarm();
			state = 2u;
			danger_found = 1u;
		}

		if (danger_found)
		{
			continue;
		}

		/* ===== WARNING ===== */
		if (tempReal >= thres[0] && tempReal <= thres[1])
		{
			print_uart1_str("WARNING at temperature ");
			print_uart1_int((uint16)tempReal);
			print_uart1_str("\r\n");
			low_alarm();
			state = 1u;
			warning_found = 1u;
		}
		else if (pressReal >= thres[2] && pressReal <= thres[3])
		{
			print_uart1_str("WARNING at pressure ");
			print_uart1_int((uint16)pressReal);
			print_uart1_str("\r\n");
			low_alarm();
			state = 1u;
			warning_found = 1u;
		}
		else if (metanValue >= thres[4] && metanValue <= thres[5])
		{
			print_uart1_str("WARNING at metan ");
			print_uart1_int(metanValue);
			print_uart1_str("\r\n");
			low_alarm();
			state = 1u;
			warning_found = 1u;
		}
		else if (coValue >= thres[6] && coValue <= thres[7])
		{
			print_uart1_str("WARNING at carbonmonoxide ");
			print_uart1_int(coValue);
			print_uart1_str("\r\n");
			low_alarm();
			state = 1u;
			warning_found = 1u;
		}

		/* ===== SAFE ===== */
		if (!danger_found && !warning_found)
		{
			if (state != 0u)
			{
				print_uart1_str("SAFE\r\n");
				pause_wave();
			}
			state = 0u;
		}
	}
}
ISR(INT2_vect)
{
	uint8 v0, v1, v2, v3, v4, v5, v6, v7;

	print_uart1_str("CONFIG THRESHOLD\r\n");

	/* ===== TEMP ===== */
	print_uart1_str("temp_threshold:   [");
	v0 = bsp_uart_rx();
	eeprom_update_byte((uint8*)(EE_BASE_ADDR + 0u), v0);
	print_uart1_int(v0);

	print_uart1_str(" ; ");
	v1 = bsp_uart_rx();
	eeprom_update_byte((uint8*)(EE_BASE_ADDR + 1u), v1);
	print_uart1_int(v1);
	print_uart1_str("]\r\n");

	/* ===== PRESS ===== */
	print_uart1_str("press_threshold:  [");
	v2 = bsp_uart_rx();
	eeprom_update_byte((uint8*)(EE_BASE_ADDR + 2u), v2);
	print_uart1_int(v2);

	print_uart1_str(" ; ");
	v3 = bsp_uart_rx();
	eeprom_update_byte((uint8*)(EE_BASE_ADDR + 3u), v3);
	print_uart1_int(v3);
	print_uart1_str("]\r\n");

	/* ===== MQ4 ===== */
	print_uart1_str("mq4_threshold:    [");
	v4 = bsp_uart_rx();
	eeprom_update_byte((uint8*)(EE_BASE_ADDR + 4u), v4);
	print_uart1_int(v4);

	print_uart1_str(" ; ");
	v5 = bsp_uart_rx();
	eeprom_update_byte((uint8*)(EE_BASE_ADDR + 5u), v5);
	print_uart1_int(v5);
	print_uart1_str("]\r\n");

	/* ===== MQ7 ===== */
	print_uart1_str("mq7_threshold:    [");
	v6 = bsp_uart_rx();
	eeprom_update_byte((uint8*)(EE_BASE_ADDR + 6u), v6);
	print_uart1_int(v6);

	print_uart1_str(" ; ");
	v7 = bsp_uart_rx();
	eeprom_update_byte((uint8*)(EE_BASE_ADDR + 7u), v7);
	print_uart1_int(v7);
	print_uart1_str("]\r\n");

	/* Wait release button */
	while (!(PINB & (1u << PB2))) { }
}
