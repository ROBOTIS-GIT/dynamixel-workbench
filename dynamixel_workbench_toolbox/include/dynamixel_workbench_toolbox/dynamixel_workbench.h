/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

class DynamixelWorkbench : public DynamixelDriver
{
 public:
  DynamixelWorkbench();
  ~DynamixelWorkbench();

  bool torque(uint8_t id, bool onoff, const char **log = NULL);
  bool torqueOn(uint8_t id, const char **log = NULL);
  bool torqueOff(uint8_t id, const char **log = NULL);

  bool setID(uint8_t id, uint8_t new_id, const char **log = NULL);
  bool setBaud(uint8_t id, uint32_t new_baudrate, const char **log = NULL);
  bool setProtocolVersion(uint8_t id, uint8_t version, const char **log = NULL);

  bool led(uint8_t id, bool onoff, const char **log = NULL);
  bool ledOn(uint8_t id, const char **log = NULL);
  bool ledOff(uint8_t id, const char **log = NULL);

  bool setNormalDirection(uint8_t id, const char **log = NULL);
  bool setReverseDirection(uint8_t id, const char **log = NULL);
  
  bool setVelocityBasedProfile(uint8_t id, const char **log = NULL);
  bool setTimeBasedProfile(uint8_t id, const char **log = NULL);

  bool setSecondaryID(uint8_t id, uint8_t secondary_id, const char **log = NULL);

  bool setCurrentControlMode(uint8_t id, const char **log = NULL);
  bool setTorqueControlMode(uint8_t id, const char **log = NULL);
  bool setVelocityControlMode(uint8_t id, const char **log = NULL);  
  bool setPositionControlMode(uint8_t id, const char **log = NULL);  
  bool setExtendedPositionControlMode(uint8_t id, const char **log = NULL);
  bool setCurrentBasedPositionControlMode(uint8_t id, const char **log = NULL);
  bool setPWMControlMode(uint8_t id, const char **log = NULL);

  bool setOperatingMode(uint8_t id, uint8_t index, const char **log = NULL);

  bool jointMode(uint8_t id, uint32_t velocity = 0, uint32_t acceleration = 0, const char **log = NULL);
  bool wheelMode(uint8_t id, uint32_t acceleration = 0, const char **log = NULL);
  bool CurrentBasedPositionMode(uint8_t id, uint32_t current = 0, const char **log = NULL);

  bool goalPosition(uint8_t id, uint32_t goal, const char **log = NULL);
  bool goalVelocity(uint8_t id, uint32_t goal, const char **log = NULL);

  uint32_t convertRadian2Value(uint8_t id, float radian, const char **log = NULL);
  float convertValue2Radian(uint8_t id, uint32_t value, const char **log = NULL);

  uint32_t convertVelocity2Value(uint8_t id, float velocity, const char **log = NULL);
  float convertValue2Velocity(uint8_t id, uint32_t value, const char **log = NULL);
};

#endif /*DYNAMIXEL_WORKBENCH_H_*/
