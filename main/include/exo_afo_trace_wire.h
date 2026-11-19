#ifndef EXO_AFO_TRACE_WIRE_H
#define EXO_AFO_TRACE_WIRE_H
#include <stddef.h>
#include "exo_adaptive_runtime.h"
/* Explicit versioned CSV, not the C ABI. No transport or state mutation. */
int exo_afo_trace_format(char *buffer,size_t size,const exo_afo_trace_t *row,unsigned side,bool first);
#endif
