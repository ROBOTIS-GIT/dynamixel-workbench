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

#ifndef DYNAMIXEL_TOOL_H
#define DYNAMIXEL_TOOL_H

#if defined(__OPENCR__) || defined(__OPENCM904__)
  #include <Arduino.h>
  #define CONTROL_TABLE_ITEM_NUM 25
#elif defined(__linux__)
  #include <stdio.h>
  #include <stdint.h>
  #include <string.h>
  #define CONTROL_TABLE_ITEM_NUM 70
#endif

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
  uint16_t    address;
  char*       item_name;  
  uint8_t     data_length;
  ACCESS_TYPE access_type;
  MEMORY_TYPE memory_type;  
};

class DynamixelTool
{
 private:
  char* model_name_;
  uint8_t id_;

  float velocity_to_value_ratio_;
  float torque_to_current_value_ratio_;

  int32_t value_of_min_radian_position_;
  int32_t value_of_0_radian_position_;
  int32_t value_of_max_radian_position_;

  float  min_radian_;
  float  max_radian_;

  uint8_t control_table_size_;
  uint8_t baud_rate_table_size_;

  ControlTableItem item_[CONTROL_TABLE_ITEM_NUM];

 public:
  DynamixelTool();
  ~DynamixelTool();

  bool begin(char* model_name);
  bool begin(uint16_t model_num);

  void setControlTable(char* name);
  void setControlTable(uint16_t num);

  void setModelInfo(char* name);
  void setModelInfo(uint16_t num);

  char* getModelName();

  void setID(uint8_t id);
  uint8_t getID();

  float getVelocityToValueRatio();  
  float getTorqueToCurrentValueRatio();  

  int32_t getValueOfMinRadianPosition();  
  int32_t getValueOfMaxRadianPosition();  
  int32_t getValueOfZeroRadianPosition();  

  float getMinRadian();  
  float getMaxRadian();  

  uint8_t getControlTableSize();  
  uint8_t getBaudRateTableSize();  
  
  ControlTableItem getControlItem(char* item_name);  
};
#endif //DYNAMIXEL_TOOL_H
