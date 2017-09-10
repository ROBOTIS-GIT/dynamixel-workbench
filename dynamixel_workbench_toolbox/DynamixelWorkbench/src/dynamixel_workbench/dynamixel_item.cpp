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

#include "../../include/dynamixel_workbench/dynamixel_item.h"

static uint8_t control_table_size = 0;
static ControlTableItem item[60]    = {0.0, };

static ModelInfo model_info       = {0.0, };

void setAXItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[1]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[2]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[3]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};

  item[4] = {24 , "Torque ON/OFF"          , 1 , READ_WRITE , RAM};
  item[5] = {25 , "LED"                    , 1 , READ_WRITE , RAM};
  item[6] = {26 , "CW Compliance Margin"   , 1 , READ_WRITE , RAM};
  item[7] = {27 , "CCW Compliance Margin"  , 1 , READ_WRITE , RAM};
  item[8] = {28 , "CW Compliance Slope"    , 1 , READ_WRITE , RAM};
  item[9] = {29 , "CCW Compliance Slope"   , 1 , READ_WRITE , RAM};
  item[10] = {30 , "Goal Position"         , 2 , READ_WRITE , RAM};
  item[11] = {32 , "Moving Speed"          , 2 , READ_WRITE , RAM};
  item[12] = {35 , "Torque Limit"          , 2 , READ_WRITE , RAM};
  item[13] = {36 , "Present Position"      , 2 , READ       , RAM};
  item[14] = {38 , "Present Speed"         , 2 , READ       , RAM};
  item[15] = {40 , "Present Load"          , 2 , READ       , RAM};
  item[16] = {42 , "Present Voltage"       , 1 , READ       , RAM};
  item[17] = {43 , "Present Temperature"   , 1 , READ       , RAM};
  item[18] = {44 , "Registered"            , 1 , READ       , RAM};
  item[19] = {46 , "Moving"                , 1 , READ       , RAM};
  item[20] = {47 , "Lock"                  , 1 , READ_WRITE , RAM};
  item[21] = {48 , "Punch"                 , 2 , READ_WRITE , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"                  , 2 , READ       , EEPROM};
  item[1]  = {2  , "Version of Firmware"           , 1 , READ       , EEPROM};
  item[2]  = {3  , "ID"                            , 1 , READ_WRITE , EEPROM};
  item[3]  = {4  , "Baud Rate"                     , 1 , READ_WRITE , EEPROM};
  item[4]  = {5  , "Return Delay Time"             , 1 , READ_WRITE , EEPROM};
  item[5]  = {6  , "CW Angle Limit"                , 2 , READ_WRITE , EEPROM};
  item[6]  = {8  , "CCW Angle Limit"               , 2 , READ_WRITE , EEPROM};
  item[7]  = {11 , "the Highest Limit Temperature" , 1 , READ_WRITE , EEPROM};
  item[8]  = {12 , "the Lowest Limit Voltage"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {13 , "the Highest Limit Voltage"     , 1 , READ_WRITE , EEPROM};
  item[10] = {14 , "Max Torque"                    , 2 , READ_WRITE , EEPROM};
  item[11] = {16 , "Status Return Level"           , 1 , READ_WRITE , EEPROM};
  item[12] = {17 , "Alarm LED"                     , 1 , READ_WRITE , EEPROM};
  item[13] = {18 , "Alarm Shutdown"                , 1 , READ_WRITE , EEPROM};

  item[14] = {24 , "Torque ON/OFF"         , 1 , READ_WRITE , RAM};
  item[15] = {25 , "LED"                   , 1 , READ_WRITE , RAM};
  item[16] = {26 , "CW Compliance Margin"  , 1 , READ_WRITE , RAM};
  item[17] = {27 , "CCW Compliance Margin" , 1 , READ_WRITE , RAM};
  item[18] = {28 , "CW Compliance Slope"   , 1 , READ_WRITE , RAM};
  item[19] = {29 , "CCW Compliance Slope"  , 1 , READ_WRITE , RAM};
  item[20] = {30 , "Goal Position"         , 2 , READ_WRITE , RAM};
  item[21] = {32 , "Moving Speed"          , 2 , READ_WRITE , RAM};
  item[22] = {35 , "Torque Limit"          , 2 , READ_WRITE , RAM};
  item[23] = {36 , "Present Position"      , 2 , READ       , RAM};
  item[24] = {38 , "Present Speed"         , 2 , READ       , RAM};
  item[25] = {40 , "Present Load"          , 2 , READ       , RAM};
  item[26] = {42 , "Present Voltage"       , 1 , READ       , RAM};
  item[27] = {43 , "Present Temperature"   , 1 , READ       , RAM};
  item[28] = {44 , "Registered"            , 1 , READ       , RAM};
  item[29] = {46 , "Moving"                , 1 , READ       , RAM};
  item[30] = {47 , "Lock"                  , 1 , READ_WRITE , RAM};
  item[31] = {48 , "Punch"                 , 2 , READ_WRITE , RAM};

  control_table_size = 32;
#endif  
}

void setAXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03;
  
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setRXItem()
{

}

void setRXInfo()
{

}

void setEXItem()
{

}

void setEXInfo()
{

}

void setMXItem()
{

}

void setMXInfo()
{
  
}

void setXMItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[1]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[2]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};

  item[3]  = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[4]  = {65 , "LED"                   , 1 , READ_WRITE , RAM};
  item[5]  = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
  item[6]  = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
  item[7]  = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[8]  = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
  item[9]  = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
  item[10] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[11] = {120, "Realtime Tick"         , 2 , READ       , RAM};
  item[12] = {122, "Moving"                , 1 , READ       , RAM};
  item[13] = {123, "Moving Status"         , 1 , READ       , RAM};
  item[14] = {124, "Present PWM"           , 2 , READ       , RAM};
  item[15] = {126, "Present Current"       , 2 , READ       , RAM};
  item[16] = {128, "Present Velocity"      , 4 , READ       , RAM};
  item[17] = {132, "Present Position"      , 4 , READ       , RAM};
  item[18] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
  item[19] = {140, "Position Trajectory"   , 4 , READ       , RAM};
  item[20] = {144, "Present Input Voltage" , 2 , READ       , RAM};
  item[21] = {146, "Present Temperature"   , 1 , READ       , RAM};

  control_table_size = 22;
#else
  item[0]  = {0  , "Model Number"          , 2 , READ       , EEPROM};
  item[1]  = {6  , "Version of Firmware"   , 1 , READ       , EEPROM};
  item[2]  = {7  , "ID"                    , 1 , READ_WRITE , EEPROM};
  item[3]  = {8  , "Baud Rate"             , 1 , READ_WRITE , EEPROM};
  item[4]  = {9  , "Return Delay Time"     , 1 , READ_WRITE , EEPROM};
  item[5]  = {10 , "Drive Mode"            , 1 , READ_WRITE , EEPROM};
  item[6]  = {11 , "Operating Mode"        , 1 , READ_WRITE , EEPROM};
  item[7]  = {12 , "Secondary ID"          , 1 , READ_WRITE , EEPROM};
  item[8]  = {13 , "Protocol Version"      , 1 , READ_WRITE , EEPROM};
  item[9]  = {20 , "Homing Offset"         , 4 , READ_WRITE , EEPROM};
  item[10] = {24 , "Moving Threshold"      , 4 , READ_WRITE , EEPROM};
  item[11] = {31 , "Temperature Limit"     , 1 , READ_WRITE , EEPROM};
  item[12] = {32 , "Max Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[13] = {34 , "Min Voltage Limit"     , 2 , READ_WRITE , EEPROM};
  item[14] = {36 , "PWM Limit"             , 2 , READ_WRITE , EEPROM};
  item[15] = {38 , "Current Limit"         , 2 , READ_WRITE , EEPROM};
  item[16] = {40 , "Acceleration Limit"    , 4 , READ_WRITE , EEPROM};
  item[17] = {44 , "Velocity Limit"        , 4 , READ_WRITE , EEPROM};
  item[18] = {48 , "Max Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[19] = {52 , "Min Position Limit"    , 4 , READ_WRITE , EEPROM};
  item[20] = {63 , "Shutdown"              , 1 , READ_WRITE , EEPROM};

  item[21] = {64 , "Torque Enable"         , 1 , READ_WRITE , RAM};
  item[22] = {65 , "LED"                   , 1 , READ_WRITE , RAM};
  item[23] = {68 , "Status Return Level"   , 1 , READ_WRITE , RAM};
  item[24] = {69 , "Registered Instruction", 1 , READ       , RAM};
  item[25] = {70 , "Hardware Error Status" , 1 , READ       , RAM};
  item[26] = {76 , "Velocity I Gain"       , 2 , READ_WRITE , RAM};
  item[27] = {78 , "Velocity P Gain"       , 2 , READ_WRITE , RAM};
  item[28] = {80 , "Position D Gain"       , 2 , READ_WRITE , RAM};
  item[29] = {82 , "Position I Gain"       , 2 , READ_WRITE , RAM};
  item[30] = {84 , "Position P Gain"       , 2 , READ_WRITE , RAM};
  item[31] = {88 , "Feedforward 2nd Gain"  , 2 , READ_WRITE , RAM};
  item[32] = {90 , "Feedforward 1st Gain"  , 2 , READ_WRITE , RAM};
  item[33] = {98 , "Bus Watchdog"          , 1 , READ_WRITE , RAM};
  item[34] = {100, "Goal PWM"              , 2 , READ_WRITE , RAM};
  item[35] = {102, "Goal Current"          , 2 , READ_WRITE , RAM};
  item[36] = {104, "Goal Velocity"         , 4 , READ_WRITE , RAM};
  item[37] = {108, "Profile Acceleration"  , 4 , READ_WRITE , RAM};
  item[38] = {112, "Profile Velocity"      , 4 , READ_WRITE , RAM};
  item[39] = {116, "Goal Position"         , 4 , READ_WRITE , RAM};
  item[40] = {120, "Realtime Tick"         , 2 , READ       , RAM};
  item[41] = {122, "Moving"                , 1 , READ       , RAM};
  item[42] = {123, "Moving Status"         , 1 , READ       , RAM};
  item[43] = {124, "Present PWM"           , 2 , READ       , RAM};
  item[44] = {126, "Present Current"       , 2 , READ       , RAM};
  item[45] = {128, "Present Velocity"      , 4 , READ       , RAM};
  item[46] = {132, "Present Position"      , 4 , READ       , RAM};
  item[47] = {136, "Velocity Trajectory"   , 4 , READ       , RAM};
  item[48] = {140, "Position Trajectory"   , 4 , READ       , RAM};
  item[49] = {144, "Present Input Voltage" , 2 , READ       , RAM};
  item[50] = {146, "Present Temperature"   , 1 , READ       , RAM};

  control_table_size = 51;
#endif  
}

void setXMInfo()
{
  model_info.velocity_to_value_ratio       = 41.71;
  model_info.torque_to_current_value_ratio = 149.795386991;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4095;

  model_info.min_radian = -3.14159265;
  model_info.max_radian = 3.14159265;
}

ControlTableItem* getItem(uint16_t num)
{
  if (num == 12 || num == 300 || num == 18)
  {
    setAXItem();
  }
  else if (num == 24 || num == 28 || num == 64)
  {
    setRXItem();
  }
  else if (num == 107)
  {
    setEXItem();
  }
  else if (num == 29 || num == 310 || num == 320 || num == 360)
  {
    setMXItem();
  }
  else if (num == 1020 || num == 1030 || num == 1060)
  {
    setXMItem();
  }

  return item;
}

ModelInfo* getInfo(uint16_t num)
{  
  if (num == 12 || num == 300 || num == 18)
  {
    setAXInfo();
  }
  else if (num == 24 || num == 28 || num == 64)
  {
    setRXInfo();
  }
  else if (num == 107)
  {
    setEXInfo();
  }
  else if (num == 29 || num == 310 || num == 320 || num == 360)
  {
    setMXInfo();
  }
  else if (num == 1020 || num == 1030 || num == 1060)
  {
    setXMInfo();
  }

  return &model_info;
}

uint8_t getSize()
{
  return control_table_size;
}