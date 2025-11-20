#include <avr/io.h>
#include <avr/interrupt.h>
#include "UART.h"
#include "bsp.h"
#include "BME.h"




int main(void)
{
	uint8_t thres[8];

	usart1_init();
	usart0_init();
	timer1_init();
	adc_config();
	
	thresConfig();
	bme_config();
	
	sei();                 

int state = 0;   // 0 = SAFE, 1 = WARNING, 2 = DANGER

while (1)
{
	thresRead(thres);

	int32_t tempRaw   = bme_read(0xFA);
	uint32_t pressRaw = bme_read(0xF7);

	int32_t  tempReal  = convertCelcius(tempRaw)/100;
	uint32_t pressReal = convertPascal(pressRaw)/100;

	uint16_t metanValue = adc_read(0)*5/1024;
	uint16_t coValue    = adc_read(1)*5/1024;

	int danger_found = 0;
	int warning_found = 0;

	// ================== DANGER CHECK ==================
	if (tempReal > thres[1]) {
		usart1_message_tx("DANGER at temperature ");
		send_int(tempReal);
		usart1_char_tx('\r'); usart1_char_tx('\n');
		high_alarm();
		state = 2;
		danger_found = 1;
	}
	else if (pressReal > thres[3]) {
		usart1_message_tx("DANGER at pressure ");
		send_int(pressReal);
		usart1_char_tx('\r'); usart1_char_tx('\n');
		high_alarm();
		state = 2;
		danger_found = 1;
	}
	else if (metanValue > thres[5]) {
		usart1_message_tx("DANGER at metan ");
		send_int(metanValue);
		usart1_char_tx('\r'); usart1_char_tx('\n');
		high_alarm();
		state = 2;
		danger_found = 1;
	}
	else if (coValue > thres[7]) {
		usart1_message_tx("DANGER at carbonmonoxide ");
		send_int(coValue);
		usart1_char_tx('\r'); usart1_char_tx('\n');
		high_alarm();
		state = 2;
		danger_found = 1;
	}

	if (danger_found)
	continue;

	// ================== WARNING CHECK ==================
	if (tempReal >= thres[0] && tempReal <= thres[1]) {
		usart1_message_tx("WARNING at temperature ");
		send_int(tempReal);
		usart1_char_tx('\r'); usart1_char_tx('\n');
		low_alarm();
		state = 1;
		warning_found = 1;
	}
	else if (pressReal >= thres[2] && pressReal <= thres[3]) {
		usart1_message_tx("WARNING at pressure ");
		send_int(pressReal);
		usart1_char_tx('\r'); usart1_char_tx('\n');
		low_alarm();
		state = 1;
		warning_found = 1;
	}
	else if (metanValue >= thres[4] && metanValue <= thres[5]) {
		usart1_message_tx("WARNING at metan ");
		send_int(metanValue);
		usart1_char_tx('\r'); usart1_char_tx('\n');
		low_alarm();
		state = 1;
		warning_found = 1;
	}
	else if (coValue >= thres[6] && coValue <= thres[7]) {
		usart1_message_tx("WARNING at carbonmonoxide ");
		send_int(coValue);
		usart1_char_tx('\r'); usart1_char_tx('\n');
		low_alarm();
		state = 1;
		warning_found = 1;
	}

	// ================== SAFETY CHECK ==================
	if (!danger_found && !warning_found) {

		if (state != 0) {
			usart1_message_tx("SAFE");
			usart1_char_tx('\r');
			usart1_char_tx('\n');
			pause_wave();
		}

		state = 0;
	}
}

}
