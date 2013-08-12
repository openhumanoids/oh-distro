#ifndef TACTILE_H
#define TACTILE_H

void tactile_init();
void tactile_scan(uint8_t *data_buf, const uint8_t with_leds);
void tactile_idle();
void tactile_systick();

#define TACTILE_NUM_TAXELS 6
extern uint16_t g_tactile_last_scan[TACTILE_NUM_TAXELS];

extern uint32_t g_tactile_strain_last_scan;

void thermal_scan();

#endif

