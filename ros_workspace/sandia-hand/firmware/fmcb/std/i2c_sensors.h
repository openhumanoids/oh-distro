#ifndef I2C_SENSORS_H
#define I2C_SENSORS_H

#include <stdint.h>

void i2c_sensors_init();
void i2c_sensors_systick();
void i2c_sensors_idle();
void i2c_sensors_twi_irq();
extern volatile uint16_t g_i2c_sensors_data[8]; 
  // temp sensor: 1 reading
  // ST combo sensor: 3x accel, 3x mag, 1 temperature

#endif

