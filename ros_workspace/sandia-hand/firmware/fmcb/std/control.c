#include "sam3s/chip.h"
#include "sam3s/core_cm3.h"
#include "control.h"
#include "params.h"
#include "halls.h"
#include "gpio.h"
#include "motors.h"
#include "pins.h"

volatile enum control_mode_t g_control_mode;
volatile int32_t g_control_hall_tgt[3];
volatile float g_control_joint_tgt[3];
volatile uint8_t g_control_joint_max_effort[3], g_control_max_effort_mobo;
volatile int16_t g_control_effort[3];

inline void control_systick_disable()
{
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
}

inline void control_systick_enable()
{
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}

#define CONTROL_FREQ 1000
void control_init()
{
  for (int i = 0; i < 3; i++)
  {
    g_control_hall_tgt[i] = 0;
    g_control_joint_tgt[i] = 0;
    g_control_joint_max_effort[i] = 255; // use overall effort limit at first
    g_control_max_effort_mobo = 255;     // mobo will clamp this later
    g_control_effort[i] = 0;
  }
  g_control_mode = CM_IDLE;
  SysTick_Config(F_CPU / CONTROL_FREQ); // 1 ms tick clock
  NVIC_SetPriority(SysTick_IRQn, 3); // control loop priority lower than halls
}

void control_systick()
{
  volatile int32_t hall[3];
  int torque[3];
  gpio_led(true);
  NVIC_DisableIRQ(PIOA_IRQn);
  NVIC_DisableIRQ(PIOB_IRQn);
  hall[0] = g_hall_count_0;
  hall[1] = g_hall_count_1;
  hall[2] = g_hall_count_2;
  NVIC_EnableIRQ(PIOA_IRQn);
  NVIC_EnableIRQ(PIOB_IRQn);
  for (int i = 0; i < 3; i++)
    hall[i] -= g_params.encoder_offset[i];
  if (g_control_mode == CM_MOTOR_SPACE || g_control_mode == CM_JOINT_SPACE)
  {
    static volatile int32_t prev_err[3] = {0, 0, 0};
    for (int i = 0; i < 3; i++)
    {
      int32_t err = g_control_hall_tgt[i] - hall[i];
      torque[i] = (int)(255.0f * (g_params.motor_kp[i] * err +
                                  g_params.motor_kd[i] * (err - prev_err[i])));
      prev_err[i] = err;
    }
    // send to motors
    int en = 0, dir = 0;
    for (int i = 0; i < 3; i++)
    {
      if (torque[i] <  g_params.torque_deadband[i] &&
          torque[i] > -g_params.torque_deadband[i])
        torque[i] = 0;

      if (torque[i] != 0)
        en |= (1 << i);
      if (torque[i] < 0)
        dir |= (1 << i);
      if (torque[i] < 0)
        torque[i] *= -1;
      if (torque[i] > g_params.torque_limit[i]) // from param table
        torque[i] = g_params.torque_limit[i];
      if (torque[i] > g_control_joint_max_effort[i]) // if specified in msg
        torque[i] = g_control_joint_max_effort[i];
      if (torque[i] > g_control_max_effort_mobo) // used by mobo to clamp
        torque[i] = g_control_max_effort_mobo;
      // reconstruct a bipolar effort to send up in the status packets
      g_control_effort[i] = torque[i];
      if (dir & (1 << i))
        g_control_effort[i] *= -1;
    }
    motors_set_all(en, dir, torque);
  }
  gpio_led(false);
}

void control_set_motorspace(const int16_t *targets)
{
  control_systick_disable();
  g_control_mode = CM_JOINT_SPACE;
  for (int i = 0; i < 3; i++)
    g_control_hall_tgt[i] = (int32_t)targets[i];
  control_systick_enable();
}

void control_set_jointspace(const float *targets)
{
  control_systick_disable();
  g_control_mode = CM_JOINT_SPACE;
  for (int i = 0; i < 3; i++)
  {
    g_control_joint_tgt[i] = targets[i];
    if (g_control_joint_tgt[i] < g_params.lower_joint_limit[i])
      g_control_joint_tgt[i] = g_params.lower_joint_limit[i];
    else if (g_control_joint_tgt[i] > g_params.upper_joint_limit[i])
      g_control_joint_tgt[i] = g_params.upper_joint_limit[i];
  }
  g_control_hall_tgt[2] = ANGLE_TO_HALL * 
                            g_params.gear[2] * g_control_joint_tgt[2];
  g_control_hall_tgt[1] = ANGLE_TO_HALL * 
                                (  g_params.gear[1] * g_control_joint_tgt[1]
                 - CAPSTAN_RATIO * g_params.gear[2] * g_control_joint_tgt[2]);
  g_control_hall_tgt[0] = ANGLE_TO_HALL * 
                                (  g_params.gear[0] * g_control_joint_tgt[0]
                                 + g_params.gear[1] * g_control_joint_tgt[1]
                 + CAPSTAN_RATIO * g_params.gear[2] * g_control_joint_tgt[2]);
  control_systick_enable();
}

void control_set_jointspace_fp(const int16_t *fp_targets)
{
  float targets[3];
  for (int i = 0; i < 3; i++)
    targets[i] = (float)fp_targets[i] / 10000.0f;
  control_set_jointspace(targets);
}

void control_set_relative_jointspace(const float *relative_targets,
                                     const uint8_t *efforts)
{
  float targets[3] = {0};
  control_systick_disable();
  for (int i = 0; i < 3; i++)
  {
    targets[i] = g_control_joint_tgt[i] + relative_targets[i];
    g_control_joint_max_effort[i]  = efforts[i];
  }
  control_systick_enable();
  control_set_jointspace(targets);
}

void control_halt()
{
  control_systick_disable();
  g_control_mode = CM_IDLE;
  PIO_Set(&pin_m0_en);
  PIO_Set(&pin_m1_en);
  PIO_Set(&pin_m2_en);
  PWM->PWM_CH_NUM[0].PWM_CDTYUPD = 0;
  PWM->PWM_CH_NUM[1].PWM_CDTYUPD = 255;
  PWM->PWM_CH_NUM[3].PWM_CDTYUPD = 255;
  control_systick_enable();
}

void control_set_jointspace_with_max_effort(const float   *targets, 
                                            const uint8_t *efforts)
{
  for (int i = 0; i < 3; i++)
    g_control_joint_max_effort[i] = efforts[i];
  control_set_jointspace(targets);
}

void control_set_max_effort_mobo(const uint8_t effort)
{
  g_control_max_effort_mobo = effort;
}

