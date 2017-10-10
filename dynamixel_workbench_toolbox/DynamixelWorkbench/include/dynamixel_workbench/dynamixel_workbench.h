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

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_H_
#define DYNAMIXEL_WORKBENCH_H_

#include "dynamixel_driver.h"

#define XL320_POSITION_CONTROL_MODE 0
#define XL320_VELOCITY_CONTROL_MODE 1

#define X_SERIES_CURRENT_CONTROL_MODE                0
#define X_SERIES_VELOCITY_CONTROL_MODE               1
#define X_SERIES_POSITION_CONTROL_MODE               3
#define X_SERIES_EXTENDED_POSITION_CONTROL_MODE      4
#define X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE 5
#define X_SERIES_VOLTAGE_CONTROL_MODE                16

#define PRO_SERIES_TORQUE_CONTROL_MODE                 0
#define PRO_SERIES_VELOCITY_CONTROL_MODE               1
#define PRO_SERIES_POSITION_CONTROL_MODE               3
#define PRO_SERIES_EXTENDED_POSITION_CONTROL_MODE      4

class DynamixelWorkbench
{
 private:
  DynamixelDriver driver_;
  char dxl_[64];
  float version_;

 public:
  DynamixelWorkbench();
  ~DynamixelWorkbench();

  bool begin(char* model_series, char* device_name = "/dev/ttyUSB0", uint32_t baud_rate = 57600, float protocol_version = 1.0);
 
  uint8_t  scan(uint8_t *get_id);
  uint16_t ping(uint8_t id);

  bool reboot(uint8_t id);
  bool reset(uint8_t id);

  bool setID(uint8_t id, uint8_t new_id);
  bool setBaud(uint8_t id, uint32_t new_baud);
  bool setProtocolVersion(uint8_t id, uint8_t new_version);

  bool jointMode(uint8_t id, uint16_t vel = 0, uint16_t acc = 0);
  bool wheelMode(uint8_t id, uint16_t vel = 0, uint16_t acc = 0);

  bool goalPosition(uint8_t id, uint16_t goal);
  bool goalSpeed(uint8_t id, int32_t goal);

 private:
  bool torque(uint8_t id, bool onoff);
  bool setPositionControlMode(uint8_t id);
  bool setVelocityControlMode(uint8_t id);
};

#endif /*DYNAMIXEL_WORKBENCH_H_*/
