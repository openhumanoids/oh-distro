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

#ifndef CAM_H
#define CAM_H

#include <stdint.h>

void cam_init();
uint16_t cam_read_register(uint8_t cam_idx, uint8_t reg_idx);
void cam_write_register(const uint8_t cam_idx, 
                        const uint8_t reg_idx, const uint16_t reg_val);
void cam_set_streams(const uint8_t stream_0, const uint8_t stream_1);

#define CAM_CAPTURE_0 0x0040
#define CAM_CAPTURE_1 0x0080

#endif

