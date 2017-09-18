/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_H_
#define DYNAMIXEL_WORKBENCH_H_

#include "dynamixel_driver.h"

class DynamixelWorkbench
{
 private:
  DynamixelDriver driver_;

 public:
  DynamixelWorkbench();
  ~DynamixelWorkbench();

  bool begin(char* model_series, char* device_name = "/dev/ttyUSB0", uint32_t baud_rate = 57600);

  uint8_t  scan(uint8_t *get_id);
  uint16_t ping(uint8_t id);

  bool reboot(uint8_t id);
  bool reset(uint8_t id);

  bool jointMode(uint8_t id, uint32_t accel = 100, uint32_t vel = 10);

  bool goalPosition(uint8_t id, uint32_t goal);
};

#endif /*DYNAMIXEL_WORKBENCH_H_*/
