#include "interrupt.h"
#include "common_macros.h"
#include <avr/io.h>

void INT0_init(INT_EdgeType edge)
{
	CLEAR_BIT(DDRD, 2);
	SET_BIT(PORTD, 2);

	EICRA &= ~((1<<ISC01)|(1<<ISC00));
	EICRA |=  (edge << ISC00);

	SET_BIT(EIMSK, INT0);
}

void INT1_init(INT_EdgeType edge)
{
	CLEAR_BIT(DDRD, 3);
	SET_BIT(PORTD, 3);

	EICRA &= ~((1<<ISC11)|(1<<ISC10));
	EICRA |=  (edge << ISC10);

	SET_BIT(EIMSK, INT1);
}

void INT2_init(INT_EdgeType edge)
{
	CLEAR_BIT(DDRB, 2);
	SET_BIT(PORTB, 2);

	EICRA &= ~((1<<ISC21)|(1<<ISC20));
	EICRA |=  (edge << ISC20);

	SET_BIT(EIMSK, INT2);
}
