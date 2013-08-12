#ifndef STRAIN_H
#define STRAIN_H

void strain_init();
void strain_scan(uint8_t *data_buf, const uint8_t with_leds);
void strain_idle();
void strain_systick();

extern uint32_t g_strain_last_scan;
#define STRAIN_NUM_TEMP 2
extern uint32_t g_strain_temp_last_scan[STRAIN_NUM_TEMP];

#endif

