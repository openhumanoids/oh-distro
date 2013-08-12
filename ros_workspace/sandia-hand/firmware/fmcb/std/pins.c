#include "pins.h"

const Pin pin_led = { PIO_PA31, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT };
const Pin pin_rs485_de = { PIO_PA7, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT };
const Pin pin_rs485_di = { PIO_PA6, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_rs485_ro = { PIO_PA5, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };

const Pin pin_phal_di = { PIO_PA22, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_phal_ro = { PIO_PA21, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT };
const Pin pin_phal_de = { PIO_PA8, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT };
/*
const Pin pin_accel_mosi = {PIO_PA13, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT};
const Pin pin_accel_miso = {PIO_PA12, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT};
const Pin pin_accel_sck  = {PIO_PA14, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT};
const Pin pin_accel_cs   = {PIO_PA11, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT};
*/
const Pin pin_m0_hall0 = {PIO_PA16, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT};
const Pin pin_m0_hall1 = {PIO_PA23, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT};
const Pin pin_m0_hall2 = {PIO_PA20, PIOA, ID_PIOA, PIO_INPUT, PIO_DEFAULT};

const Pin pin_m1_hall0 = {PIO_PB13, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT};
const Pin pin_m1_hall1 = {PIO_PB10, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT};
const Pin pin_m1_hall2 = {PIO_PB14, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT};

const Pin pin_m2_hall0 = {PIO_PB1, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT};
const Pin pin_m2_hall1 = {PIO_PB2, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT};
const Pin pin_m2_hall2 = {PIO_PB11, PIOB, ID_PIOB, PIO_INPUT, PIO_DEFAULT};

const Pin pin_5v_reg = {PIO_PA15, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin pin_phal_pwr = {PIO_PA27, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};

const Pin pin_m_brake = {PIO_PA30, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};

//const Pin pin_m0_en =  {PIO_PA24, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT};
const Pin pin_m0_en =  {PIO_PA25, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT};
const Pin pin_m1_en =  {PIO_PA2 , PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT};
const Pin pin_m2_en =  {PIO_PA26, PIOA, ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT};

//const Pin pin_m0_dir = {PIO_PA25, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin pin_m0_dir = {PIO_PA24, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin pin_m1_dir = {PIO_PA0 , PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin pin_m2_dir = {PIO_PA18, PIOA, ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};

const Pin pin_m0_vref = {PIO_PA19, PIOA, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT};
const Pin pin_m1_vref = {PIO_PA1 , PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT};
const Pin pin_m2_vref = {PIO_PA17, PIOA, ID_PIOA, PIO_PERIPH_C, PIO_DEFAULT};

const Pin pin_i2c_scl = {PIO_PA4, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT};
const Pin pin_i2c_sda = {PIO_PA3, PIOA, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT};

// phalange current sense: PB0
// (scaled) motor bus voltage: PB3

