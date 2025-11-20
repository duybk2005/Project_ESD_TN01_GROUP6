#include <avr/io.h>
void adc_config(void)
{
	ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}
uint16_t adc_read(uint8_t ch)
{
	ch &= 0x01;
	ADMUX = (ADMUX & 0xFE) | ch;
	ADCSRA |= (1<<ADSC);
	while (ADCSRA & (1<<ADSC));
	uint16_t r = ADCL;
	r |= (ADCH << 8);
	return r;
}
void i2c_init()
{
	TWBR = 8;
	TWSR = 1<<TWPS0; //bit-rate=4
	TWCR = 1<<TWEN;
}
void i2c_start()
{
	TWCR = (1<<TWEN)|(1<<TWSTA)|(1<<TWINT);
	while (!(TWCR & (1<<TWINT) )) ; //WAIT FINISH START
}
void i2c_write (unsigned char x)
{
	TWDR = x;
	TWCR = (1<<TWEN)|(1<<TWINT);
	while(!(TWCR & (1<<TWINT)));
}
void i2c_stop()
{
	TWCR = (1<<TWEN)|(1<<TWSTO)|(1<<TWINT);
}
unsigned char i2c_read()
{
	TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWEA);
	while (!(TWCR&(1<<TWINT)));
	return (TWDR);
}
void i2c_nack()
{
	TWCR=(1<<TWEN)|(1<<TWINT);
	while (!(TWCR & (1<<TWINT)));
}
#define OCR1A_VAL   3906U
#define OCR1B_HIGH  3125U
#define OCR1B_LOW   1953U

void timer1_init(void)
{
	DDRD |= (1<<PD4);              // PD4 = OC1B

	// Fast PWM, TOP = OCR1A
	TCCR1A = (1<<COM1B1) | (1<<WGM11) | (1<<WGM10);
	TCCR1B = (1<<WGM13) | (0<<WGM12) | (1<<CS12) | (1<<CS10);

	OCR1A = OCR1A_VAL;
	OCR1B = OCR1B_HIGH;
}
void high_alarm(void)
{

	TCCR1A |= (1<<COM1B1);
	TCCR1A &= ~(1<<COM1B0);
	OCR1B = OCR1B_HIGH;
}

void low_alarm(void)
{
	TCCR1A |= (1<<COM1B1);
	TCCR1A &= ~(1<<COM1B0);
	OCR1B = OCR1B_LOW;
}
void pause_wave(void)
{
	TCCR1A &= ~((1<<COM1B1) | (1<<COM1B0));
}
void usart1_init(void)
{
	UBRR1H = 0;
	UBRR1L = 51;                       // 9600 @ 8 MHz
	UCSR1A = 0;
	UCSR1B = (1<<TXEN1);
	UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);
}

void usart1_char_tx(uint8_t c)
{
	while (!(UCSR1A & (1<<UDRE1)));
	UDR1 = c;
}
void usart0_init(void)
{
	UBRR0H = 0;
	UBRR0L = 51;                       // 9600 @ 8 MHz
	UCSR0A = 0;
	UCSR0B = (1<<RXEN0);
	UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
}
uint8_t usart0_char_rx(void)
{
	while (!(UCSR0A & (1<<RXC0)));
	return UDR0;
}
void thresConfig(void)
{
	DDRB  &= ~(1<<PB2);     // PB2 input
	PORTB |=  (1<<PB2);     // pull-up
	EICRA  =  (1<<ISC21);   // falling edge
	EIMSK  =  (1<<INT2);    // enable INT2
}
