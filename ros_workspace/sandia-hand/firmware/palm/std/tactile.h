#ifndef TACTILE_H
#define TACTILE_H

void tactile_init();
void tactile_scan(uint8_t *data_buf, const uint8_t with_leds);
void tactile_systick();
void tactile_idle();

#define TACTILE_NUM_TAXELS 32
extern uint16_t g_tactile_last_scan[TACTILE_NUM_TAXELS];
extern uint32_t g_tactile_last_scan_time;

#define TACTILE_NUM_THERM 4

#endif

