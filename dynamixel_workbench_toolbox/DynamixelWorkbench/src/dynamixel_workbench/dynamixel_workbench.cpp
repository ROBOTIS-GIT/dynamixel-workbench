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

#include "../../include/dynamixel_workbench/dynamixel_workbench.h"

DynamixelWorkbench::DynamixelWorkbench()
{

}

DynamixelWorkbench::~DynamixelWorkbench()
{

}

bool DynamixelWorkbench::begin(char* model_series, char* device_name, uint32_t baud_rate)
{
  bool error = false;

  error = driver_.begin(model_series, device_name, baud_rate);

  return error;
}

uint8_t  DynamixelWorkbench::scan(uint8_t *get_id)
{
  uint8_t id_cnt = 0;

  id_cnt = driver_.scan(get_id, 16);

  return id_cnt;
}

uint16_t DynamixelWorkbench::ping(uint8_t id)
{
  uint16_t model_num = 0;

  model_num = driver_.ping(id);

  return model_num;
}

bool DynamixelWorkbench::reboot(uint8_t id)
{
  bool error = false;

  error = driver_.reboot(id);

  return error;
}

bool DynamixelWorkbench::reset(uint8_t id)
{
  bool error = false;

  error = driver_.reset(id);

  return error;
}

bool DynamixelWorkbench::jointMode(uint8_t id, uint32_t accel, uint32_t vel)
{
  driver_.writeRegister(id, "Operating Mode", 3);

  driver_.writeRegister(id, "Torque Enable", 1);

  driver_.writeRegister(id, "Profile Acceleration", accel);

  driver_.writeRegister(id, "Profile Velocity", vel);
}

bool DynamixelWorkbench::goalPosition(uint8_t id, uint32_t goal)
{
  driver_.writeRegister(id, "Goal Position", goal);  
}
