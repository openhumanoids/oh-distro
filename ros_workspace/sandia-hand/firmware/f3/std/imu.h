#ifndef IMU_H
#define IMU_H

#include <stdint.h>

void imu_init();
void imu_reg_write(uint8_t device, uint8_t reg_addr, uint8_t reg_payload);
uint8_t imu_reg_read(uint8_t device, uint8_t reg_addr);
void imu_systick();
void imu_idle();
void imu_twi_irq();
void imu_power(uint8_t on);
extern volatile int16_t g_imu_data[6];

#endif

