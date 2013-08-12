#ifndef PP_PINS_H
#define PP_PINS_H

#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"

extern const Pin pin_led;
extern const Pin pin_rs485_de, pin_rs485_di, pin_rs485_ro;
extern const Pin pin_spi_sck, pin_spi_mosi, pin_spi_miso;
extern const Pin pin_cs_adc_strain;
extern const Pin pin_cs_adc_tactile;
extern const Pin pin_i2c_sda, pin_i2c_scl;
extern const Pin pin_leds[6];
extern const Pin pin_strain_power;

#endif
