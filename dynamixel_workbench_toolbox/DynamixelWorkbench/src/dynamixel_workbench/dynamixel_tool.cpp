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

#include "../../include/dynamixel_workbench/dynamixel_tool.h"

DynamixelTool::DynamixelTool(){}

DynamixelTool::~DynamixelTool(){}

bool DynamixelTool::begin(char* model_name)
{
  model_name_ = model_name;  
  setControlTable(model_name);
}

bool DynamixelTool::begin(uint16_t model_num)
{
  setControlTable(model_num);
}

void DynamixelTool::setControlTable(char* name)
{  
  if (!strncmp(name, "XM430-W350", strlen(name)))
  { 
    setControlTable(1020);
  }
  else if (!strncmp(name, "XM430-W210", strlen(name)))
  { 
    setControlTable(1030);
  }
  else if (!strncmp(name, "XL430-W250", strlen(name)))
  { 
    setControlTable(1060);
  }
}

void DynamixelTool::setControlTable(uint16_t num)
{
  if (num == 1020)
  {
    model_name_ = "XM430-W350";

    item_               = getItem(num);
    control_table_size_ = getSize();
    model_info_         = getInfo(num);
  }
  if (num == 1030)
  {
    model_name_ = "XM430-W210";
    item_               = getItem(num);
    control_table_size_ = getSize();
    model_info_         = getInfo(num);
  }
  else if (num == 1060)
  {
    model_name_ = "XL430-W250";
    item_               = getItem(num);
    control_table_size_ = getSize();
    model_info_         = getInfo(num);
  }
}

char* DynamixelTool::getModelName()
{
  return model_name_;
}

float DynamixelTool::getVelocityToValueRatio()
{
  return model_info_->velocity_to_value_ratio;
}

float DynamixelTool::getTorqueToCurrentValueRatio()
{
  return model_info_->torque_to_current_value_ratio;
}

int32_t DynamixelTool::getValueOfMinRadianPosition()
{
  return model_info_->value_of_min_radian_position;
}

int32_t DynamixelTool::getValueOfMaxRadianPosition()
{
  return model_info_->value_of_max_radian_position;
}

int32_t DynamixelTool::getValueOfZeroRadianPosition()
{
  return model_info_->value_of_0_radian_position;
}

float DynamixelTool::getMinRadian()
{
  return model_info_->min_radian;
}

float DynamixelTool::getMaxRadian()
{
  return model_info_->max_radian;
}

uint8_t DynamixelTool::getControlTableSize()
{
  return control_table_size_;
}

void DynamixelTool::setID(uint8_t id)
{
  id_ = id;
}

uint8_t DynamixelTool::getID()
{
  return id_;
}

ControlTableItem* DynamixelTool::getControlItem(char* item_name)
{
  ControlTableItem* cti;

  for (int num = 0; num < control_table_size_; num++)
  {
    if (!strncmp(item_name, item_[num].item_name, strlen(item_name)))
    {
      cti = &item_[num];
      return cti;
    }
  }
}
