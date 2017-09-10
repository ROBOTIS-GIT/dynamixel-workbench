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

#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include "control_table_item.h"

typedef struct
{
  float velocity_to_value_ratio;
  float torque_to_current_value_ratio;
  
  int32_t value_of_min_radian_position;
  int32_t value_of_0_radian_position;
  int32_t value_of_max_radian_position; 
  
  float  min_radian;
  float  max_radian;
} ModelInfo;

static void setAXItem();
static void setAXInfo();

static void setRXItem();
static void setRXInfo();

static void setEXItem();
static void setEXInfo();

static void setMXItem();
static void setMXInfo();

static void setXLItem();
static void setXLInfo();

static void setXMItem();
static void setXMInfo();

static void setXHItem();
static void setXHInfo();

static void setPROItem();
static void setPROInfo();

ControlTableItem* getItem(uint16_t num);
uint8_t getSize();
ModelInfo* getInfo(uint16_t num);

#endif //DYNAMIXEL_H