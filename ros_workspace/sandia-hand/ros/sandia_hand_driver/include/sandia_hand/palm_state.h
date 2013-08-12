#ifndef PALM_STATE_H
#define PALM_STATE_H

#define PALM_STATE_NUM_TAXELS 32
#define PALM_STATE_NUM_TEMPS   7

#include <stdint.h>
typedef struct
{
  uint32_t palm_time;
  int16_t  palm_accel[3];
  int16_t  palm_gyro[3];
  int16_t  palm_mag[3];
  uint16_t palm_temps[7]; // four thermistors and temp sensors on ARM,gyro,mag
  uint16_t palm_tactile[PALM_STATE_NUM_TAXELS];
} palm_state_t;

#endif
