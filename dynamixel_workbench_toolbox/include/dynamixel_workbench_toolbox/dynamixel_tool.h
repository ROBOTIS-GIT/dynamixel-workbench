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

/* Authors: zerom, Taehoon Lim (Darby) */

#ifndef DYNAMIXEL_TOOL_H
#define DYNAMIXEL_TOOL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>

namespace dynamixel_tool
{
enum ACCESS_TYPE {
  READ,
  READ_WRITE
};

enum MEMORY_TYPE {
  EEPROM,
  RAM
};

struct ControlTableItem
{
 std::string item_name;
 uint16_t address;
 ACCESS_TYPE access_type;
 MEMORY_TYPE memory_type;
 uint8_t data_length;
};

class DynamixelTool
{
 public:
  uint8_t id_;
  uint16_t model_number_;
  std::string model_name_;

  float velocity_to_value_ratio_;
  float torque_to_current_value_ratio_;
  int32_t value_of_0_radian_position_;
  int32_t value_of_min_radian_position_;
  int32_t value_of_max_radian_position_;
  float  min_radian_;
  float  max_radian_;

  std::string item_path_;
  std::string name_path_;

  std::map<std::string, ControlTableItem *> ctrl_table_;
  std::map<std::string, ControlTableItem *>::iterator it_ctrl_;

  std::map<uint32_t, uint32_t> baud_rate_table_;
  std::map<uint32_t, uint32_t>::iterator it_baud_;

  ControlTableItem *item_;

 public:
  DynamixelTool(uint8_t id, uint16_t model_number);
  DynamixelTool(uint8_t id, std::string model_name);
  ~DynamixelTool();

 private:
  bool getModelPath();
  bool getModelName(uint16_t model_number);
  bool getModelItem();
};
}
#endif //DYNAMIXEL_TOOL_H
