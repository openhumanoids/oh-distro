#ifndef STATE_H
#define STATE_H

#include "palm_state.h"

extern palm_state_t g_state;

void state_init();
void state_idle();
void state_systick();
void state_tc0_irq();
uint32_t state_get_time();

#endif

