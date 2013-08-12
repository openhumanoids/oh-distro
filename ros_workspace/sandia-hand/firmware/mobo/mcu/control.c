#include "control.h"

float   control_target_joint_angles[12];
uint8_t control_target_max_efforts [12];

void control_init()
{
  for (int i = 0; i < 12; i++)
  {
    control_target_joint_angles[i] = 0;
    control_target_max_efforts[i] = 10;
  }
}

