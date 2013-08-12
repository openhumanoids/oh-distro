#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>

extern float   control_target_joint_angles[12];
extern uint8_t control_target_max_efforts [12];

void control_init();

#endif

