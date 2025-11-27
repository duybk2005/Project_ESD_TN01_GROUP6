#ifndef INTERRUPT_H_
#define INTERRUPT_H_

#include "std_types.h"

typedef enum
{
	INT_EDGE_LOW  = 0,
	INT_EDGE_ANY  = 1,
	INT_EDGE_FALL = 2,
	INT_EDGE_RISE = 3
} INT_EdgeType;

void INT0_init(INT_EdgeType edge);
void INT1_init(INT_EdgeType edge);
void INT2_init(INT_EdgeType edge);

#endif
