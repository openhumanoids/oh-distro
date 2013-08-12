#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

uint8_t config_init();
uint8_t config_is_left_hand();
uint32_t config_get_hw_version();
const uint8_t *config_get_hand_mac();
uint32_t config_get_hand_ip();
uint32_t config_get_master_ip();

#endif

