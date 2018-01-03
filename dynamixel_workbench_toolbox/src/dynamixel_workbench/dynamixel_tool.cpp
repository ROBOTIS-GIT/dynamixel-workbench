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

#include "../../include/dynamixel_workbench/dynamixel_tool.h"

DynamixelTool::DynamixelTool() : dxl_info_cnt_(0), control_table_size_(0){}

DynamixelTool::~DynamixelTool(){}

bool DynamixelTool::begin(const char* model_name, uint8_t id)
{
  ++dxl_info_cnt_;

  strcpy(dxl_info_[dxl_info_cnt_].model_name, model_name);
  dxl_info_[dxl_info_cnt_].id = id;

  setControlTable(model_name);
}

bool DynamixelTool::begin(uint16_t model_num, uint8_t id)
{
  ++dxl_info_cnt_;

  setModelName(model_num);
  dxl_info_[dxl_info_cnt_].id = id;


  printf("%d,    %s\n", dxl_info_cnt_, dxl_info_[dxl_info_cnt_].model_name);

  setControlTable(model_num);
}

bool DynamixelTool::addDXL(const char* model_name, uint8_t id)
{
  ++dxl_info_cnt_;

  strcpy(dxl_info_[dxl_info_cnt_].model_name, model_name);
  dxl_info_[dxl_info_cnt_].id = id;
}

bool DynamixelTool::addDXL(uint16_t model_num, uint8_t id)
{
  ++dxl_info_cnt_;

  setModelName(model_num);
  dxl_info_[dxl_info_cnt_].id = id;
}

void DynamixelTool::setControlTable(const char *model_name)
{  
  const char* name = model_name;

  if (!strncmp(name, "AX-12A", strlen(name))) 
    setControlTable(AX_12A);
  else if (!strncmp(name, "AX-12W", strlen(name)))
    setControlTable(AX_12W);
  else if (!strncmp(name, "AX-18A", strlen(name)))
    setControlTable(AX_18A);

  else if (!strncmp(name, "RX-24F", strlen(name)))
    setControlTable(RX_24F);
  else if (!strncmp(name, "RX-28", strlen(name)))
    setControlTable(RX_28);
  else if (!strncmp(name, "RX-64", strlen(name)))
    setControlTable(RX_64);

  else if (!strncmp(name, "EX-106", strlen(name)))
    setControlTable(EX_106);

  else if (!strncmp(name, "MX-12W", strlen(name)))
    setControlTable(MX_12W);
  else if (!strncmp(name, "MX-28", strlen(name)))
    setControlTable(MX_28);
  else if (!strncmp(name, "MX-28-2", strlen(name)))
    setControlTable(MX_28_2);
  else if (!strncmp(name, "MX-64", strlen(name)))
    setControlTable(MX_64);
  else if (!strncmp(name, "MX-64-2", strlen(name)))
    setControlTable(MX_64_2);
  else if (!strncmp(name, "MX-106", strlen(name)))
    setControlTable(MX_106);
  else if (!strncmp(name, "MX-106-2", strlen(name)))
    setControlTable(MX_106_2);

  else if (!strncmp(name, "XL-320", strlen(name)))
    setControlTable(XL_320);
  else if (!strncmp(name, "XL430-W250", strlen(name)))
    setControlTable(XL430_W250);

  else if (!strncmp(name, "XM430-W210", strlen(name)))
    setControlTable(XM430_W210);
  else if (!strncmp(name, "XM430-W350", strlen(name)))
    setControlTable(XM430_W350);
  else if (!strncmp(name, "XM540-W150", strlen(name)))
    setControlTable(XM540_W150);
  else if (!strncmp(name, "XM540-W270", strlen(name)))
    setControlTable(XM540_W270);

  else if (!strncmp(name, "XH430-V210", strlen(name)))
    setControlTable(XH430_V210);
  else if (!strncmp(name, "XH430-V350", strlen(name)))
    setControlTable(XH430_V350);
  else if (!strncmp(name, "XH430-W210", strlen(name)))
    setControlTable(XH430_W210);
  else if (!strncmp(name, "XH430-W350", strlen(name)))
    setControlTable(XH430_W350);

  else if (!strncmp(name, "PRO-L42-10-S300-R", strlen(name)))
    setControlTable(PRO_L42_10_S300_R);
  else if (!strncmp(name, "PRO-L54-30-S400-R", strlen(name)))
    setControlTable(PRO_L54_30_S400_R);
  else if (!strncmp(name, "PRO-L54-30-S500-R", strlen(name)))
    setControlTable(PRO_L54_30_S500_R);
  else if (!strncmp(name, "PRO-L54-50-S290-R", strlen(name)))
    setControlTable(PRO_L54_50_S290_R);
  else if (!strncmp(name, "PRO-L54-50-S500-R", strlen(name)))
    setControlTable(PRO_L54_50_S500_R);

  else if (!strncmp(name, "PRO-M42-10-S260-R", strlen(name)))
    setControlTable(PRO_M42_10_S260_R);
  else if (!strncmp(name, "PRO-M54-40-S250-R", strlen(name)))
    setControlTable(PRO_M54_40_S250_R);
  else if (!strncmp(name, "PRO-M54-60-S250-R", strlen(name)))
    setControlTable(PRO_M54_60_S250_R);

  else if (!strncmp(name, "PRO-H42-20-S300-R", strlen(name)))
    setControlTable(PRO_H42_20_S300_R);
  else if (!strncmp(name, "PRO-H54-100-S500-R", strlen(name)))
    setControlTable(PRO_H54_100_S500_R);
  else if (!strncmp(name, "PRO-H54-200-S500-R", strlen(name)))
    setControlTable(PRO_H54_200_S500_R);
}

void DynamixelTool::setControlTable(uint16_t model_num)
{
  item_ptr_           = getItem(model_num);
  control_table_size_ = getSize();
  info_ptr_           = getInfo(model_num);

  for (int index = 0; index < control_table_size_; index++)
    item_[index] = item_ptr_[index];

  info_.velocity_to_value_ratio         = info_ptr_->velocity_to_value_ratio;
  info_.torque_to_current_value_ratio   = info_ptr_->torque_to_current_value_ratio;

  info_.value_of_0_radian_position      = info_ptr_->value_of_0_radian_position;
  info_.value_of_min_radian_position    = info_ptr_->value_of_min_radian_position;
  info_.value_of_max_radian_position    = info_ptr_->value_of_max_radian_position;

  info_.min_radian                      = info_ptr_->min_radian;
  info_.max_radian                      = info_ptr_->max_radian;
}

void DynamixelTool::setModelName(uint16_t model_num)
{
  uint16_t num = model_num;

  if (num == AX_12A)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "AX-12A");
  else if (num == AX_12W)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "AX-12W");
  else if (num == AX_18A)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "AX-18A");

  else if (num == RX_24F)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "RX-24F");
  else if (num == RX_28)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "RX-28");
  else if (num == RX_64)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "RX-64");

  else if (num == EX_106)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "EX-106");

  else if (num == MX_12W)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "MX-12W");
  else if (num == MX_28)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "MX-28");
  else if (num == MX_28_2)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "MX-28-2");
  else if (num == MX_64)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "MX-64");
  else if (num == MX_64_2)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "MX-64-2");
  else if (num == MX_106)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "MX-106");
  else if (num == MX_106_2)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "MX-106-2");

  else if (num == XL_320)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XL-320");
  else if (num == XL430_W250)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XL430-W250");

  else if (num == XM430_W210)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XM430-W210");
  else if (num == XM430_W350)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XM430-W350");
  else if (num == XM540_W150)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XM540-W150");
  else if (num == XM540_W270)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XM540-W270");

  else if (num == XH430_V210)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XH430-V210");
  else if (num == XH430_V350)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XH430-V350");
  else if (num == XH430_W210)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XH430-W210");
  else if (num == XH430_W350)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "XH430-W350");

  else if (num == PRO_L42_10_S300_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-L42-10-S300-R");
  else if (num == PRO_L54_30_S400_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-L54-30-S400-R");
  else if (num == PRO_L54_30_S500_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-L54-30-S500-R");
  else if (num == PRO_L54_50_S290_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-L54-50-S290-R");
  else if (num == PRO_L54_50_S500_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-L54-50-S500-R");

  else if (num == PRO_M42_10_S260_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-M42-10-S260-R");
  else if (num == PRO_M54_40_S250_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-M54-40-S250-R");
  else if (num == PRO_M54_60_S250_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-M54-60-S250-R");

  else if (num == PRO_H42_20_S300_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-H42-20-S300-R");
  else if (num == PRO_H54_100_S500_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-H54-100-S500-R");
  else if (num == PRO_H54_200_S500_R)
    strcpy(dxl_info_[dxl_info_cnt_].model_name, "PRO-H54-200-S500-R");
}

//const char *DynamixelTool::getModelName(uint8_t id)
//{
//  for (int num = 0; num < dxl_info_cnt_; num++)
//  {
//    if (dxl_info_[num].id == id)
//      return dxl_info_[num].model_name;
//  }

//  return NULL;
//}

float DynamixelTool::getVelocityToValueRatio()
{
  return info_.velocity_to_value_ratio;
}

float DynamixelTool::getTorqueToCurrentValueRatio()
{
  return info_.torque_to_current_value_ratio;
}

int32_t DynamixelTool::getValueOfMinRadianPosition()
{
  return info_.value_of_min_radian_position;
}

int32_t DynamixelTool::getValueOfMaxRadianPosition()
{
  return info_.value_of_max_radian_position;
}

int32_t DynamixelTool::getValueOfZeroRadianPosition()
{
  return info_.value_of_0_radian_position;
}

float DynamixelTool::getMinRadian()
{
  return info_.min_radian;
}

float DynamixelTool::getMaxRadian()
{
  return info_.max_radian;
}

uint8_t DynamixelTool::getControlTableSize()
{
  return control_table_size_;
}

//void DynamixelTool::setID(uint8_t id)
//{
//  id_ = id;
//}

//uint8_t DynamixelTool::getID()
//{
//  return id_;
//}

ControlTableItem* DynamixelTool::getControlItem(const char* item_name)
{
  ControlTableItem* cti;

  for (int num = 0; num < control_table_size_; num++)
  {
    if (!strncmp(item_name, item_[num].item_name, strlen(item_[num].item_name)))
    {
      cti = &item_[num];
      return cti;
    }
  }
}

ControlTableItem* DynamixelTool::getControlItemPtr()
{
  return item_ptr_;
}

ModelInfo* DynamixelTool::getModelInfoPtr()
{
  return info_ptr_;
}
