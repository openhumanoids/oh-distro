#ifndef HALLS_H
#define HALLS_H

#include <stdint.h>

extern volatile int32_t g_hall_count_0, g_hall_count_1, g_hall_count_2;
extern uint32_t g_hall_0, g_hall_1, g_hall_2, g_hall_prev[3];
extern volatile int g_hall_state_0, g_hall_state_1, g_hall_state_2;
extern volatile int32_t g_hall_vel[3];

void halls_init();
void halls_pioa_irq();
void halls_piob_irq();

#endif

