#ifndef FMCB_PARAMS_H
#define FMCB_PARAMS_H

#include <stdint.h>

#define HALL_STATES (36.0f)
#define ANGLE_TO_HALL (HALL_STATES / (3.1415926f * 2.0f))
#define CAPSTAN_RATIO 0.89f

struct params_t
{
  float motor_kp[3];
  float motor_ki[3];
  float motor_kd[3];
  int   torque_deadband[3];
  int   encoder_offset[3];
  float gear[3];
  float lower_joint_limit[3];
  float upper_joint_limit[3];
  int   torque_limit[3];
  int   accel_bias[9];  // mm, pp, dp
  int   accel_scale[9]; // mm, pp, dp
};
extern struct params_t g_params;

struct registered_param_t
{
  void *val;
  const char *name;
};
extern const struct registered_param_t const g_registered_params[];
extern uint16_t g_num_registered_params;

void params_save_all_to_flash();
void params_load_all_from_flash();
void params_set_all_default();

#endif

