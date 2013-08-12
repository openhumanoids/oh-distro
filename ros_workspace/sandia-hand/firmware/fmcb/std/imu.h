#ifndef IMU_H
#define IMU_H

#include <stdint.h>

void imu_init();
void imu_accel_reg_write(uint8_t addr, uint8_t val);
uint8_t imu_accel_reg_read(uint8_t addr);

#endif

