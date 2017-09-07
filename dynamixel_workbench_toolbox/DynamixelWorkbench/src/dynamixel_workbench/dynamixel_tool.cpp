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

DynamixelTool::DynamixelTool() {}

DynamixelTool::~DynamixelTool(){}

bool DynamixelTool::begin(char* model_name)
{
  model_name_ = model_name;
  
  setControlTable(model_name);
  setModelInfo(model_name);
}

bool DynamixelTool::begin(uint16_t model_num)
{
  setControlTable(model_num);
  setModelInfo(model_num);
}

void DynamixelTool::setControlTable(char* name)
{  
  if (!strncmp(name, "XM430-W350", strlen(name)))
  { 
#if defined(__OPENCR__) || defined(__OPENCM904__)
    item_[0]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
    item_[1]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
    item_[2]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};

    item_[3]  = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
    item_[4]  = {65 , "LED"                   , 1 , READ_WRITE , RAM};
    item_[5]  = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
    item_[6]  = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
    item_[7]  = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
    item_[8]  = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
    item_[9]  = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
    item_[10] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
    item_[11] = {120, "Realtime Tick"         , 2 , READ       , RAM};
    item_[12] = {122, "Moving"                , 1 , READ       , RAM};
    item_[13] = {123, "Moving Status"         , 1 , READ       , RAM};
    item_[14] = {124, "Present PWM"           , 2 , READ       , RAM};
    item_[15] = {126, "Present Current"       , 2 , READ       , RAM};
    item_[16] = {128, "Present Velocity"      , 4 , READ       , RAM};
    item_[17] = {132, "Present Position"      , 4 , READ       , RAM};
    item_[18] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
    item_[19] = {140, "Position Trajectory"   , 4 , READ       , RAM};
    item_[20] = {144, "Present Input Voltage" , 2 , READ       , RAM};
    item_[21] = {146, "Present Temperature"   , 1 , READ       , RAM};

    control_table_size_ = 22;
#else
    item_[0]  = {0  , "Model Number"          , 2 , READ       , EEPROM};
    item_[1]  = {6  , "Version of Firmware"   , 1 , READ       , EEPROM};
    item_[2]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
    item_[3]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
    item_[4]  = {9  , "Return Delay Time"     , 1 , READ_WRITE , EEPROM};
    item_[5]  = {10 , "Drive Mode"            , 1 , READ_WRITE , EEPROM};
    item_[6]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};
    item_[7]  = {12 , "Secondary ID"          , 1 , READ_WRITE , EEPROM};
    item_[8]  = {13 , "Protocol Version"      , 1 , READ_WRITE , EEPROM};
    item_[9]  = {20 , "Homing Offset"         , 4 , READ_WRITE , EEPROM};
    item_[10] = {24 , "Moving Threshold"      , 4 , READ_WRITE , EEPROM};
    item_[11] = {31 , "Temperature Limit"     , 1 , READ_WRITE , EEPROM};
    item_[12] = {32 , "Max Voltage Limit"     , 2 , READ_WRITE , EEPROM};
    item_[13] = {34 , "Min Voltage Limit"     , 2 , READ_WRITE , EEPROM};
    item_[14] = {36 , "PWM Limit"             , 2 , READ_WRITE , EEPROM};
    item_[15] = {38 , "Current Limit"         , 2 , READ_WRITE , EEPROM};
    item_[16] = {40 , "Acceleration Limit"    , 4 , READ_WRITE , EEPROM};
    item_[17] = {44 , "Velocity Limit"        , 4 , READ_WRITE , EEPROM};
    item_[18] = {48 , "Max Position Limit"    , 4 , READ_WRITE , EEPROM};
    item_[19] = {52 , "Min Position Limit"    , 4 , READ_WRITE , EEPROM};
    item_[20] = {63 , "Shutdown"              , 1 , READ_WRITE , EEPROM};

    item_[21] = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
    item_[22] = {65 , "LED"                   , 1 , READ_WRITE , RAM};
    item_[23] = {68 , "Status Return Level"   , 1 , READ_WRITE , RAM};
    item_[24] = {69 , "Registered Instruction", 1 , READ       , RAM};
    item_[25] = {70 , "Hardware Error Status" , 1 , READ       , RAM};
    item_[26] = {76 , "Velocity I Gain"       , 2 , READ_WRITE , RAM};
    item_[27] = {78 , "Velocity P Gain"       , 2 , READ_WRITE , RAM};
    item_[28] = {80 , "Position D Gain"       , 2 , READ_WRITE , RAM};
    item_[29] = {82 , "Position I Gain"       , 2 , READ_WRITE , RAM};
    item_[30] = {84 , "Position P Gain"       , 2 , READ_WRITE , RAM};
    item_[31] = {88 , "Feedforward 2nd Gain"  , 2 , READ_WRITE , RAM};
    item_[32] = {90 , "Feedforward 1st Gain"  , 2 , READ_WRITE , RAM};
    item_[33] = {98 , "Bus Watchdog"          , 1 , READ_WRITE , RAM};
    item_[34] = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
    item_[35] = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
    item_[36] = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
    item_[37] = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
    item_[38] = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
    item_[39] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
    item_[40] = {120, "Realtime Tick"         , 2 , READ       , RAM};
    item_[41] = {122, "Moving"                , 1 , READ       , RAM};
    item_[42] = {123, "Moving Status"         , 1 , READ       , RAM};
    item_[43] = {124, "Present PWM"           , 2 , READ       , RAM};
    item_[44] = {126, "Present Current"       , 2 , READ       , RAM};
    item_[45] = {128, "Present Velocity"      , 4 , READ       , RAM};
    item_[46] = {132, "Present Position"      , 4 , READ       , RAM};
    item_[47] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
    item_[48] = {140, "Position Trajectory"   , 4 , READ       , RAM};
    item_[49] = {144, "Present Input Voltage" , 2 , READ       , RAM};
    item_[50] = {146, "Present Temperature"   , 1 , READ       , RAM};

    control_table_size_ = 51;
#endif   
  }
}

void DynamixelTool::setControlTable(uint16_t num)
{
  if (num == 1020)
  {
    model_name_ = "XM430-W350";
    setControlTable(model_name_);
  }
  if (num == 1030)
  {
    model_name_ = "XM430-W350";
    setControlTable(model_name_);
  }
  else if (num == 1060)
  {
    model_name_ = "XM430-W350";
    setControlTable(model_name_);
  }
}

void DynamixelTool::setModelInfo(char* name)
{
  if (!strncmp(name, "XM430-W350", strlen(name)));
  {  
    velocity_to_value_ratio_       = 41.71;
    torque_to_current_value_ratio_ = 149.795386991;

    value_of_min_radian_position_  = 0;
    value_of_0_radian_position_    = 2048;
    value_of_max_radian_position_  = 4095;

    min_radian_ = -3.14159265;
    max_radian_ = 3.14159265;
  }
}

void DynamixelTool::setModelInfo(uint16_t num)
{
  if (num == 1020)
  {
    model_name_ = "XM430-W350";
    setModelInfo(model_name_);
  }
  if (num == 1030)
  {
    model_name_ = "XM430-W350";
    setModelInfo(model_name_);
  }
  else if (num == 1060)
  {
    model_name_ = "XM430-W350";
    setModelInfo(model_name_);
  }
}

char* DynamixelTool::getModelName()
{
  return model_name_;
}

float DynamixelTool::getVelocityToValueRatio()
{
  return velocity_to_value_ratio_;
}

float DynamixelTool::getTorqueToCurrentValueRatio()
{
  return torque_to_current_value_ratio_;
}

int32_t DynamixelTool::getValueOfMinRadianPosition()
{
  return value_of_min_radian_position_;
}

int32_t DynamixelTool::getValueOfMaxRadianPosition()
{
  return value_of_max_radian_position_;
}

int32_t DynamixelTool::getValueOfZeroRadianPosition()
{
  return value_of_0_radian_position_;
}

float DynamixelTool::getMinRadian()
{
  return min_radian_;
}

float DynamixelTool::getMaxRadian()
{
  return max_radian_;
}

uint8_t DynamixelTool::getControlTableSize()
{
  return control_table_size_;
}

uint8_t DynamixelTool::getBaudRateTableSize()
{
  return baud_rate_table_size_;
}

void DynamixelTool::setID(uint8_t id)
{
  id_ = id;
}

uint8_t DynamixelTool::getID()
{
  return id_;
}

ControlTableItem DynamixelTool::getControlItem(char* item_name)
{
  ControlTableItem cti;

  for (int num = 0; num < control_table_size_; num++)
  {
    if (!strncmp(item_name, item_[num].item_name, strlen(item_name)))
    {
      cti = item_[num];
      return cti;
    }
  }
}
