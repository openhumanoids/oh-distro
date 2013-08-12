/*  Software License Agreement (Apache License)
 *
 *  Copyright 2012 Open Source Robotics Foundation
 *  Author: Morgan Quigley
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 * 
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef POWER_H
#define POWER_H

#include <stdint.h>

typedef enum { POWER_OFF = 0, POWER_LOW = 1, POWER_ON = 2} power_state_t;

void power_init();
void power_set(const uint8_t finger_idx, const power_state_t power_state);
void power_idle();
void power_systick();
void power_adc_vector();
void power_set_mobo_status_rate(const uint16_t rate);
void power_twi1_vector();
void power_enable_lowvolt_regulator(const uint8_t enable);
void power_set_mobo_current_limit(const float max_amps);

#endif

