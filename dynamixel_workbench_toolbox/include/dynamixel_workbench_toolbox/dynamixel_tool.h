/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Taehoon Lim (Darby), zerom */

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

  bool getModelPath(void);
  bool getModelName(uint16_t model_number);
  bool getModelItem();
};
}
#endif //DYNAMIXEL_TOOL_H
