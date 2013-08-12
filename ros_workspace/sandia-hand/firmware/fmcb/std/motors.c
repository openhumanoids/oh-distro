#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "motors.h"
#include "pins.h"

void motors_init()
{
  PIO_Configure(&pin_m0_en, 1);
  PIO_Configure(&pin_m1_en, 1);
  PIO_Configure(&pin_m2_en, 1);
  PIO_Configure(&pin_m_brake, 1);
  PIO_Configure(&pin_m0_dir, 1);
  PIO_Configure(&pin_m1_dir, 1);
  PIO_Configure(&pin_m2_dir, 1);
  PMC_EnablePeripheral(ID_PWM); // power up pwm controller plz
  PIO_Configure(&pin_m0_vref, 1); // PWML0
  PIO_Configure(&pin_m1_vref, 1); // PWMH1
  PIO_Configure(&pin_m2_vref, 1); // PWMH3
  PWM->PWM_CH_NUM[0].PWM_CMR = 0; // pass-through divider. 64 MHz clock.
  PWM->PWM_CH_NUM[0].PWM_CPRD = 255; // 8-bit current control, 125 kHz cycle 
  PWM->PWM_CH_NUM[0].PWM_CDTY = 0; // zero duty for now
  PWM->PWM_CH_NUM[1].PWM_CMR = 0; 
  PWM->PWM_CH_NUM[1].PWM_CPRD = 255; 
  PWM->PWM_CH_NUM[1].PWM_CDTY = 255;
  PWM->PWM_CH_NUM[3].PWM_CMR = 0;
  PWM->PWM_CH_NUM[3].PWM_CPRD = 255;
  PWM->PWM_CH_NUM[3].PWM_CDTY = 255;
  PWM->PWM_ENA = 0xb; // enable channels 0,1,3
  PIO_Set(&pin_m_brake);
  PIO_Set(&pin_m0_en);
  PIO_Set(&pin_m1_en);
  PIO_Set(&pin_m2_en);
}

void motors_set_all(int en, int dir, int torque[3])
{
  if (en & 0x01)
    PIO_Clear(&pin_m0_en);
  else
    PIO_Set(&pin_m0_en);

  if (en & 0x02)
    PIO_Clear(&pin_m1_en);
  else 
    PIO_Set(&pin_m1_en);

  if (en & 0x04)
    PIO_Clear(&pin_m2_en);
  else 
    PIO_Set(&pin_m2_en);


  if (dir & 0x01)
    PIO_Set(&pin_m0_dir);
  else
    PIO_Clear(&pin_m0_dir);

  if (dir & 0x02)
    PIO_Set(&pin_m1_dir);
  else
    PIO_Clear(&pin_m1_dir);

  if (dir & 0x04)
    PIO_Set(&pin_m2_dir);
  else
    PIO_Clear(&pin_m2_dir);

  PWM->PWM_CH_NUM[0].PWM_CDTYUPD = torque[0];
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = 255-torque[1];
  PWM->PWM_CH_NUM[3].PWM_CDTYUPD = 255-torque[2];
}


