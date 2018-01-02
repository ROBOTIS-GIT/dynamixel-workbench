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

#ifndef DYNAMIXEL_TOOL_H
#define DYNAMIXEL_TOOL_H

#include <string.h>

#include "dynamixel_item.h"

typedef struct
{
  char model_name[64];
  uint8_t id;
} DXLInfo;

class DynamixelTool
{
 private:
  DXLInfo dxl_info_[10];
  uint8_t dxl_info_cnt_;

  ControlTableItem* item_ptr_;
  ModelInfo* info_ptr_;

#if defined(__OPENCR__) || defined(__OPENCM904__)
  ControlTableItem item_[14];
#else
  ControlTableItem item_[60];
#endif
  ModelInfo info_;
  uint8_t control_table_size_;

 public:
  DynamixelTool();
  ~DynamixelTool();

  bool begin(const char* model_name, uint8_t id);
  bool begin(uint16_t model_num, uint8_t id);

  void setControlTable(const char* name);
  void setControlTable(uint16_t num);

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
  ControlTableItem* getControlItem(const char *item_name);
  ControlTableItem* getControlItemPtr(); 
  ModelInfo* getModelInfoPtr();
};
#endif //DYNAMIXEL_TOOL_H
