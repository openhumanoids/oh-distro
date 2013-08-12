#ifndef DP0_PINS_H
#define DP0_PINS_H

#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"

#define PINS_NUM_CS_ADC 2
#define PINS_NUM_LEDS 20
#define PINS_NUM_MUX 5
#define PINS_MUX_ADDR_BITS 3

// these are set at startup to different values depending on if it's a right
// or left palm
extern Pin pin_led;
extern Pin pin_rs485_de, pin_rs485_di, pin_rs485_ro;
extern Pin pin_cs_adc[PINS_NUM_CS_ADC];
extern Pin pin_spi_sck, pin_spi_mosi, pin_spi_miso;
extern Pin pin_i2c_sda, pin_i2c_scl;
extern Pin pin_leds[PINS_NUM_LEDS];
extern Pin pin_mux[PINS_NUM_MUX][PINS_MUX_ADDR_BITS];

void pins_assign(); // 'L' for left hand, 'R' for right hand
void pins_init();
extern char g_pins_hand;

#endif
