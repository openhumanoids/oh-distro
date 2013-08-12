#include "gpio.h"
#include "params.h"
#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"

struct params_t g_params;

const struct registered_param_t const g_registered_params[] = 
{ {&g_params.motor_kp[0], "fm0_kp"},
  {&g_params.motor_kp[1], "fm1_kp"},
  {&g_params.motor_kp[2], "fm2_kp"},
  {&g_params.motor_ki[0], "fm0_ki"},
  {&g_params.motor_ki[1], "fm1_ki"},
  {&g_params.motor_ki[2], "fm2_ki"},
  {&g_params.motor_kd[0], "fm0_kd"},
  {&g_params.motor_kd[1], "fm1_kd"},
  {&g_params.motor_kd[2], "fm2_kd"},
  {&g_params.torque_deadband[0], "im0_deadband"},
  {&g_params.torque_deadband[1], "im1_deadband"},
  {&g_params.torque_deadband[2], "im2_deadband"},
  {&g_params.encoder_offset[0], "Im0_offset"},
  {&g_params.encoder_offset[1], "Im1_offset"},
  {&g_params.encoder_offset[2], "Im2_offset"},
  {&g_params.gear[0], "fm0_gear"},
  {&g_params.gear[1], "fm1_gear"},
  {&g_params.gear[2], "fm2_gear"},
  {&g_params.lower_joint_limit[0], "fj0_lower_limit"},
  {&g_params.lower_joint_limit[1], "fj1_lower_limit"},
  {&g_params.lower_joint_limit[2], "fj2_lower_limit"},
  {&g_params.upper_joint_limit[0], "fj0_upper_limit"},
  {&g_params.upper_joint_limit[1], "fj1_upper_limit"},
  {&g_params.upper_joint_limit[2], "fj2_upper_limit"},
  {&g_params.torque_limit[0], "im0_torque_limit"},
  {&g_params.torque_limit[1], "im1_torque_limit"},
  {&g_params.torque_limit[2], "im2_torque_limit"},
  {&g_params.accel_bias[0], "imm_accel_bias_x"},
  {&g_params.accel_bias[1], "imm_accel_bias_y"},
  {&g_params.accel_bias[2], "imm_accel_bias_z"},
  {&g_params.accel_bias[3], "ipp_accel_bias_x"},
  {&g_params.accel_bias[4], "ipp_accel_bias_y"},
  {&g_params.accel_bias[5], "ipp_accel_bias_z"},
  {&g_params.accel_bias[6], "idp_accel_bias_x"},
  {&g_params.accel_bias[7], "idp_accel_bias_y"},
  {&g_params.accel_bias[8], "idp_accel_bias_z"},
  {&g_params.accel_scale[0], "imm_accel_scale_x"},
  {&g_params.accel_scale[1], "imm_accel_scale_y"},
  {&g_params.accel_scale[2], "imm_accel_scale_z"},
  {&g_params.accel_scale[3], "ipp_accel_scale_x"},
  {&g_params.accel_scale[4], "ipp_accel_scale_y"},
  {&g_params.accel_scale[5], "ipp_accel_scale_z"},
  {&g_params.accel_scale[6], "idp_accel_scale_x"},
  {&g_params.accel_scale[7], "idp_accel_scale_y"},
  {&g_params.accel_scale[8], "idp_accel_scale_z"},
};

uint16_t g_num_registered_params = sizeof(g_registered_params) / 
                                   sizeof(struct registered_param_t);

// this leaves the top 4 pages of flash set up for params
#define PARAM_FLASH_START_PAGE 1020

void params_set_all_default()
{
  int i;
  for (i = 0; i < 3; i++)
  {
    g_params.motor_kp[i] = 0.15;
    g_params.motor_ki[i] = 0.0;
    g_params.motor_kd[i] = 6.0;
    g_params.torque_deadband[i] = 50;
    g_params.encoder_offset[i] = 0;
    g_params.torque_limit[i] = 50;
  }
  for (i = 0; i < 9; i++)
  {
    g_params.accel_bias[i] = 0;
    g_params.accel_scale[i] = 1023;
  }
  g_params.gear[0] = 173.4f;
  g_params.gear[1] = 196.7f;
  g_params.gear[2] = 231.0f;
  g_params.lower_joint_limit[0] = -0.5;
  g_params.upper_joint_limit[0] =  0.5;
  g_params.lower_joint_limit[1] = -0.5;
  g_params.upper_joint_limit[1] =  0.5;
  g_params.lower_joint_limit[2] = -0.5;
  g_params.upper_joint_limit[2] =  0.5;
}

void params_load_all_from_flash()
{
  if (*((uint32_t *)(0x400000 + PARAM_FLASH_START_PAGE*256)) != 0xdeadbeef)
  {
    // flash is bogus.
    params_set_all_default();
    //params_save_all_to_flash();
    gpio_led(true);
    return; 
  }
  for (int i = 0; i < g_num_registered_params; i++)
  {
    uint32_t *param_addr = g_registered_params[i].val;
    if (g_registered_params[i].name[0] >= 'a') // uppercase = no auto-load
      *param_addr = *((uint32_t *)(0x400004+PARAM_FLASH_START_PAGE*256+4*i));
    else
      *param_addr = 0;
  }
}

void params_save_all_to_flash()
{
  uint32_t page_buf[64];
  uint32_t *dest;
  int i;
  #define CHIP_FLASH_IAP_ADDRESS (0x00800008)
  const static uint32_t (*IAP_PerformCommand)( uint32_t, uint32_t ) ;
  // NOTE: this code only deals with a single page of params. generalize later
  // if/when we have more than 64 params.
  __disable_irq();
  EFC->EEFC_FMR = EEFC_FMR_FWS(6); // as per errata in datasheet
  page_buf[0] = 0xdeadbeef;
  dest = page_buf + 1;
  for (i = 0; i < g_num_registered_params; i++)
    *dest++ = *((uint32_t *)g_registered_params[i].val);
  for (i = g_num_registered_params; i < 63; i++) // first slot is the sentinel
    *dest++ = 0;
  // now, copy to aligned landing spot in address space
  dest = (uint32_t *)(0x400000 + 256 * PARAM_FLASH_START_PAGE);
  for (i = 0; i < 64; i++)
    *dest++ = page_buf[i];
  typedef uint32_t (*iap_fp)(uint32_t, uint32_t);
  iap_fp iap = *((iap_fp *)0x00800008); // magic IAP pointer in ROM.
  iap(0, EEFC_FCR_FKEY(0x5A) | EEFC_FCR_FARG(PARAM_FLASH_START_PAGE) |
         EEFC_FCR_FCMD(EFC_FCMD_EWP));
  while ((EFC->EEFC_FSR & EEFC_FSR_FRDY) != EEFC_FSR_FRDY) { } // spin...
  EFC->EEFC_FMR = EEFC_FMR_FWS(2); // reset it so we can run faster now
  __enable_irq();
}

