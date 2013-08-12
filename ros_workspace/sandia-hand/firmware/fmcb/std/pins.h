#ifndef FMCB_PINS_H
#define FMCB_PINS_H

#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
extern const Pin pin_led;
extern const Pin pin_rs485_de, pin_rs485_di, pin_rs485_ro, pin_rs485_re;
extern const Pin pin_phal_di, pin_phal_ro, pin_phal_de, pin_phal_pwr;
extern const Pin pin_m0_hall0, pin_m0_hall1, pin_m0_hall2; 
extern const Pin pin_m1_hall0, pin_m1_hall1, pin_m1_hall2; 
extern const Pin pin_m2_hall0, pin_m2_hall1, pin_m2_hall2; 
extern const Pin pin_5v_reg, pin_m_brake;
extern const Pin pin_m0_en, pin_m1_en, pin_m2_en;
extern const Pin pin_m0_dir, pin_m1_dir, pin_m2_dir;
extern const Pin pin_m0_vref, pin_m1_vref, pin_m2_vref;
extern const Pin pin_i2c_scl, pin_i2c_sda;

#endif
