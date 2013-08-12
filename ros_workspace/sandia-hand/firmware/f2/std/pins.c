#include "pins.h"

const Pin pin_led = { PIO_PA22, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT };

const Pin pin_rs485_de = { PIO_PC9, PIOC, ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT };
const Pin pin_rs485_di = { PIO_PA6, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_rs485_ro = { PIO_PA5, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

const Pin pin_spi_mosi = { PIO_PA13, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_spi_miso = { PIO_PA12, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_spi_sck =  { PIO_PA14, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

const Pin pin_cs_adc_strain={PIO_PA7, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT};
const Pin pin_cs_adc_tactile={PIO_PA8,PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT};
const Pin pin_strain_power = {PIO_PC0,PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT};

const Pin pin_i2c_scl = { PIO_PA4, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_i2c_sda = { PIO_PA3, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

const Pin pin_leds[6] = {{PIO_PC5 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                         {PIO_PC12, PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                         {PIO_PA20, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
                         {PIO_PC2 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                         {PIO_PC3 , PIOC, ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT},
                         {PIO_PA9 , PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}};


