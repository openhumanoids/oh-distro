#ifndef TACTILE_H
#define TACTILE_H

void tactile_init();
void tactile_scan(uint8_t *data_buf, const uint8_t with_leds);
void tactile_systick();
void tactile_idle();

#define TACTILE_NUM_TAXELS 12
extern uint16_t g_tactile_last_scan[TACTILE_NUM_TAXELS];

#define TACTILE_NUM_THERM 4
// todo: global buffer for last tactile scan data
void thermal_scan(uint8_t *data_buf);

#endif

