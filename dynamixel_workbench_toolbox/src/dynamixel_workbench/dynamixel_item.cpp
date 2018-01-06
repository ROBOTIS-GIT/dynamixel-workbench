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

#include "../../include/dynamixel_workbench/dynamixel_item.h"

static uint8_t the_number_of_item = 0;

#if defined(__OPENCR__) || defined(__OPENCM904__)
static ControlTableItem item[15]  = {0, };
#else
static ControlTableItem item[60]  = {0, };
#endif

static ModelInfo model_info         = {0.0, };

void setAXItem(void)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud Rate"                     , 1};
  item[2]  = {6  , "CW Angle Limit"                , 2};
  item[3]  = {8  , "CCW Angle Limit"               , 2};

  item[4]  = {24 , "Torque Enable"                 , 1};
  item[5]  = {25 , "LED"                           , 1};
  item[6]  = {30 , "Goal Position"                 , 2};
  item[7]  = {32 , "Moving Speed"                  , 2};
  item[8]  = {34 , "Torque Limit"                  , 2};
  item[9]  = {36 , "Present Position"              , 2};
  item[10] = {38 , "Present Speed"                 , 2};
  item[11] = {40 , "Present Load"                  , 2};
  item[12] = {46 , "Moving"                        , 1};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model Number"                  , 2};
  item[1]  = {2  , "Firmware Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud Rate"                     , 1};
  item[4]  = {5  , "Return Delay Time"             , 1};
  item[5]  = {6  , "CW Angle Limit"                , 2};
  item[6]  = {8  , "CCW Angle Limit"               , 2};
  item[7]  = {11 , "Temperature Limit"             , 1};
  item[8]  = {12 , "Min Voltage Limit"             , 1};
  item[9]  = {13 , "Max Voltage Limit"             , 1};
  item[10] = {14 , "Max Torque"                    , 2};
  item[11] = {16 , "Status Return Level"           , 1};
  item[12] = {17 , "Alarm LED"                     , 1};
  item[13] = {18 , "Shutdown"                      , 1};

  item[14] = {24 , "Torque Enable"                 , 1};
  item[15] = {25 , "LED"                           , 1};
  item[16] = {26 , "CW Compliance Margin"          , 1};
  item[17] = {27 , "CCW Compliance Margin"         , 1};
  item[18] = {28 , "CW Compliance Slope"           , 1};
  item[19] = {29 , "CCW Compliance Slope"          , 1};
  item[20] = {30 , "Goal Position"                 , 2};
  item[21] = {32 , "Moving Speed"                  , 2};
  item[22] = {34 , "Torque Limit"                  , 2};
  item[23] = {36 , "Present Position"              , 2};
  item[24] = {38 , "Present Speed"                 , 2};
  item[25] = {40 , "Present Load"                  , 2};
  item[26] = {42 , "Present Voltage"               , 1};
  item[27] = {43 , "Present Temperature"           , 1};
  item[28] = {44 , "Registered"                    , 1};
  item[29] = {46 , "Moving"                        , 1};
  item[30] = {47 , "Lock"                          , 1};
  item[31] = {48 , "Punch"                         , 2};

  the_number_of_item = 32;
#endif  
}

void setAXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03; // AX series don't support exact speed in wheel mode.
  
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 1023;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

void setRXItem()
{
  #if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud Rate"                     , 1};
  item[2]  = {6  , "CW Angle Limit"                , 2};
  item[3]  = {8  , "CCW Angle Limit"               , 2};

  item[4]  = {24 , "Torque Enable"                 , 1};
  item[5]  = {25 , "LED"                           , 1};
  item[6]  = {30 , "Goal Position"                 , 2};
  item[7]  = {32 , "Moving Speed"                  , 2};
  item[8]  = {34 , "Torque Limit"                  , 2};
  item[9]  = {36 , "Present Position"              , 2};
  item[10] = {38 , "Present Speed"                 , 2};
  item[11] = {40 , "Present Load"                  , 2};
  item[12] = {46 , "Moving"                        , 1};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model Number"                  , 2};
  item[1]  = {2  , "Firmware Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud Rate"                     , 1};
  item[4]  = {5  , "Return Delay Time"             , 1};
  item[5]  = {6  , "CW Angle Limit"                , 2};
  item[6]  = {8  , "CCW Angle Limit"               , 2};
  item[7]  = {11 , "Temperature Limit"             , 1};
  item[8]  = {12 , "Min Voltage Limit"             , 1};
  item[9]  = {13 , "Max Voltage Limit"             , 1};
  item[10] = {14 , "Max Torque"                    , 2};
  item[11] = {16 , "Status Return Level"           , 1};
  item[12] = {17 , "Alarm LED"                     , 1};
  item[13] = {18 , "Shutdown"                      , 1};

  item[14] = {24 , "Torque Enable"                 , 1};
  item[15] = {25 , "LED"                           , 1};
  item[16] = {26 , "CW Compliance Margin"          , 1};
  item[17] = {27 , "CCW Compliance Margin"         , 1};
  item[18] = {28 , "CW Compliance Slope"           , 1};
  item[19] = {29 , "CCW Compliance Slope"          , 1};
  item[20] = {30 , "Goal Position"                 , 2};
  item[21] = {32 , "Moving Speed"                  , 2};
  item[22] = {34 , "Torque Limit"                  , 2};
  item[23] = {36 , "Present Position"              , 2};
  item[24] = {38 , "Present Speed"                 , 2};
  item[25] = {40 , "Present Load"                  , 2};
  item[26] = {42 , "Present Voltage"               , 1};
  item[27] = {43 , "Present Temperature"           , 1};
  item[28] = {44 , "Registered"                    , 1};
  item[29] = {46 , "Moving"                        , 1};
  item[30] = {47 , "Lock"                          , 1};
  item[31] = {48 , "Punch"                         , 2};

  the_number_of_item = 32;
#endif  
}

void setRXInfo(void)
{
  model_info.velocity_to_value_ratio         = 86.03; // RX series don't support exact speed in wheel mode.
  
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 1023;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

void setEXItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud Rate"                     , 1};
  item[2]  = {6  , "CW Angle Limit"                , 2};
  item[3]  = {8  , "CCW Angle Limit"               , 2};
  item[4]  = {10 , "Drive Mode"                    , 1};

  item[5]  = {24 , "Torque Enable"                 , 1};
  item[6]  = {25 , "LED"                           , 1};
  item[7]  = {30 , "Goal Position"                 , 2};
  item[8]  = {32 , "Moving Speed"                  , 2};
  item[9]  = {34 , "Torque Limit"                  , 2};
  item[10] = {36 , "Present Position"              , 2};
  item[11] = {38 , "Present Speed"                 , 2};
  item[12] = {40 , "Present Load"                  , 2};
  item[13] = {46 , "Moving"                        , 1};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"                  , 2};
  item[1]  = {2  , "Firmware Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud Rate"                     , 1};
  item[4]  = {5  , "Return Delay Time"             , 1};
  item[5]  = {6  , "CW Angle Limit"                , 2};
  item[6]  = {8  , "CCW Angle Limit"               , 2};
  item[7]  = {10 , "Drive Mode"                    , 1};
  item[8]  = {11 , "Temperature Limit"             , 1};
  item[9]  = {12 , "Min Voltage Limit"             , 1};
  item[10] = {13 , "Max Voltage Limit"             , 1};
  item[11] = {14 , "Max Torque"                    , 2};
  item[12] = {16 , "Status Return Level"           , 1};
  item[13] = {17 , "Alarm LED"                     , 1};
  item[14] = {18 , "Shutdown"                      , 1};

  item[15] = {24 , "Torque Enable"                 , 1};
  item[16] = {25 , "LED"                           , 1};
  item[17] = {26 , "CW Compliance Margin"          , 1};
  item[18] = {27 , "CCW Compliance Margin"         , 1};
  item[19] = {28 , "CW Compliance Slope"           , 1};
  item[20] = {29 , "CCW Compliance Slope"          , 1};
  item[21] = {30 , "Goal Position"                 , 2};
  item[22] = {34 , "Moving Speed"                  , 2};
  item[23] = {35 , "Torque Limit"                  , 2};
  item[24] = {36 , "Present Position"              , 2};
  item[25] = {38 , "Present Speed"                 , 2};
  item[26] = {40 , "Present Load"                  , 2};
  item[27] = {42 , "Present Voltage"               , 1};
  item[28] = {43 , "Present Temperature"           , 1};
  item[29] = {44 , "Registered"                    , 1};
  item[30] = {46 , "Moving"                        , 1};
  item[31] = {47 , "Lock"                          , 1};
  item[32] = {48 , "Punch"                         , 2};
  item[33] = {56 , "Sensored Current"              , 2};

  the_number_of_item = 34;
#endif  
}

void setEXInfo()
{
  model_info.velocity_to_value_ratio         = 86.03; // EX series don't support exact speed in wheel mode.
  
  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -2.18969008;
  model_info.max_radian                      =  2.18969008;
}

void setMXItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud Rate"                     , 1};
  item[2]  = {6  , "CW Angle Limit"                , 2};
  item[3]  = {8  , "CCW Angle Limit"               , 2};

  item[4]  = {24 , "Torque Enable"                 , 1};
  item[5]  = {25 , "LED"                           , 1};
  item[6]  = {30 , "Goal Position"                 , 2};
  item[7]  = {32 , "Moving Speed"                  , 2};
  item[8]  = {34 , "Torque Limit"                  , 2};
  item[9]  = {36 , "Present Position"              , 2};
  item[10] = {38 , "Present Speed"                 , 2};
  item[11] = {40 , "Present Load"                  , 2};
  item[12] = {46 , "Moving"                        , 1};
  item[13] = {73 , "Goal Acceleration"             , 1};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"                  , 2};
  item[1]  = {2  , "Firmware Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud Rate"                     , 1};
  item[4]  = {5  , "Return Delay Time"             , 1};
  item[5]  = {6  , "CW Angle Limit"                , 2};
  item[6]  = {8  , "CCW Angle Limit"               , 2};
  item[7]  = {11 , "Temperature Limit"             , 1};
  item[8]  = {12 , "Min Voltage Limit"             , 1};
  item[9]  = {13 , "Max Voltage Limit"             , 1};
  item[10] = {14 , "Max Torque"                    , 2};
  item[11] = {16 , "Status Return Level"           , 1};
  item[12] = {17 , "Alarm LED"                     , 1};
  item[13] = {18 , "Shutdown"                      , 1};
  item[14] = {20 , "Multi Turn Offset"             , 2};
  item[15] = {22 , "Resolution Divider"            , 1};

  item[16] = {24 , "Torque Enable"                 , 1};
  item[17] = {25 , "LED"                           , 1};
  item[18] = {26 , "D gain"                        , 1};
  item[19] = {27 , "I gain"                        , 1};
  item[20] = {28 , "P gain"                        , 1};
  item[21] = {30 , "Goal Position"                 , 2};
  item[22] = {32 , "Moving Speed"                  , 2};
  item[23] = {34 , "Torque Limit"                  , 2};
  item[24] = {36 , "Present Position"              , 2};
  item[25] = {38 , "Present Speed"                 , 2};
  item[26] = {40 , "Present Load"                  , 2};
  item[27] = {42 , "Present Voltage"               , 1};
  item[28] = {43 , "Present Temperature"           , 1};
  item[29] = {44 , "Registered"                    , 1};
  item[30] = {46 , "Moving"                        , 1};
  item[31] = {47 , "Lock"                          , 1};
  item[32] = {48 , "Punch"                         , 2};
  item[33] = {73 , "Goal Acceleration"             , 1};

  the_number_of_item = 34;
#endif  
}

void setMXInfo()
{
  model_info.velocity_to_value_ratio         = 86.81;
  
  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setMX2Item(void)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud Rate"             , 1};
  item[2]  = {10 , "Drive Mode"            , 1};
  item[3]  = {11 , "Operating Mode"        , 1};

  item[4]  = {64 , "Torque Enable"         , 1};
  item[5]  = {65 , "LED"                   , 1};
  item[6]  = {104, "Goal Velocity"         , 4};
  item[7]  = {108, "Profile Acceleration"  , 4};
  item[8]  = {112, "Profile Velocity"      , 4};
  item[9]  = {116, "Goal Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present Load"          , 2};
  item[12] = {128, "Present Velocity"      , 4};
  item[13] = {132, "Present Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"          , 2};
  item[1]  = {6  , "Firmware Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud Rate"             , 1};
  item[4]  = {9  , "Return Delay Time"     , 1};
  item[5]  = {10 , "Drive Mode"            , 1};
  item[6]  = {11 , "Operating Mode"        , 1};
  item[7]  = {12 , "Secondary(Shadow) ID"  , 1};
  item[8]  = {13 , "Protocol Version"      , 1};
  item[9]  = {20 , "Homing Offset"         , 4};
  item[10] = {24 , "Moving Threshold"      , 4};
  item[11] = {31 , "Temperature Limit"     , 1};
  item[12] = {32 , "Max Voltage Limit"     , 2};
  item[13] = {34 , "Min Voltage Limit"     , 2};
  item[14] = {36 , "PWM Limit"             , 2};
  item[15] = {40 , "Acceleration Limit"    , 4};
  item[16] = {44 , "Velocity Limit"        , 4};
  item[17] = {48 , "Max Position Limit"    , 4};
  item[18] = {52 , "Min Position Limit"    , 4};
  item[19] = {63 , "Shutdown"              , 1};

  item[20] = {64 , "Torque Enable"         , 1};
  item[21] = {65 , "LED"                   , 1};
  item[22] = {68 , "Status Return Level"   , 1};
  item[23] = {69 , "Registered Instruction", 1};
  item[24] = {70 , "Hardware Error Status" , 1};
  item[25] = {76 , "Velocity I Gain"       , 2};
  item[26] = {78 , "Velocity P Gain"       , 2};
  item[27] = {80 , "Position D Gain"       , 2};
  item[28] = {82 , "Position I Gain"       , 2};
  item[29] = {84 , "Position P Gain"       , 2};
  item[30] = {88 , "Feedforward 2nd Gain"  , 2};
  item[31] = {90 , "Feedforward 1st Gain"  , 2};
  item[32] = {98 , "Bus Watchdog"          , 1};
  item[33] = {100, "Goal PWM"              , 2};
  item[34] = {104, "Goal Velocity"         , 4};
  item[35] = {108, "Profile Acceleration"  , 4};
  item[36] = {112, "Profile Velocity"      , 4};
  item[37] = {116, "Goal Position"         , 4};
  item[38] = {120, "Realtime Tick"         , 2};
  item[39] = {122, "Moving"                , 1};
  item[40] = {123, "Moving Status"         , 1};
  item[41] = {124, "Present PWM"           , 2};
  item[42] = {126, "Present Load"          , 2};
  item[43] = {128, "Present Velocity"      , 4};
  item[44] = {132, "Present Position"      , 4};
  item[45] = {136, "Velocity Trajectory"   , 4};
  item[46] = {140, "Position Trajectory"   , 4};
  item[47] = {144, "Present Input Voltage" , 2};
  item[48] = {146, "Present Temperature"   , 1};

  the_number_of_item = 49;
#endif  
}

void setMX2Info(void)
{
  model_info.velocity_to_value_ratio         = 41.70;
  
  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setExtMXItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud Rate"                     , 1};
  item[2]  = {6  , "CW Angle Limit"                , 2};
  item[3]  = {8  , "CCW Angle Limit"               , 2};

  item[4]  = {24 , "Torque Enable"                 , 1};
  item[5]  = {25 , "LED"                           , 1};
  item[6]  = {30 , "Goal Position"                 , 2};
  item[7]  = {32 , "Moving Speed"                  , 2};
  item[8]  = {34 , "Torque Limit"                  , 2};
  item[9]  = {36 , "Present Position"              , 2};
  item[10] = {38 , "Present Speed"                 , 2};
  item[11] = {40 , "Present Load"                  , 2};
  item[12] = {46 , "Moving"                        , 1};
  // item[13] = {68 , "Current"                       , 2};
  // item[14] = {70 , "Torque Control Mode Enable"    , 1};
  // item[15] = {71 , "Goal Torque"                   , 2};
  // item[16] = {73 , "Goal Acceleration"             , 1};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model Number"                  , 2};
  item[1]  = {2  , "Firmware Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud Rate"                     , 1};
  item[4]  = {5  , "Return Delay Time"             , 1};
  item[5]  = {6  , "CW Angle Limit"                , 2};
  item[6]  = {8  , "CCW Angle Limit"               , 2};
  item[7]  = {11 , "Temperature Limit"             , 1};
  item[8]  = {12 , "Min Voltage Limit"             , 1};
  item[9]  = {13 , "Max Voltage Limit"             , 1};
  item[10] = {14 , "Max Torque"                    , 2};
  item[11] = {16 , "Status Return Level"           , 1};
  item[12] = {17 , "Alarm LED"                     , 1};
  item[13] = {18 , "Shutdown"                      , 1};
  item[14] = {20 , "Multi Turn Offset"             , 2};
  item[15] = {22 , "Resolution Divider"            , 1};

  item[16] = {24 , "Torque Enable"                 , 1};
  item[17] = {25 , "LED"                           , 1};
  item[18] = {26 , "D gain"                        , 1};
  item[19] = {27 , "I gain"                        , 1};
  item[20] = {28 , "P gain"                        , 1};
  item[21] = {30 , "Goal Position"                 , 2};
  item[22] = {32 , "Moving Speed"                  , 2};
  item[23] = {34 , "Torque Limit"                  , 2};
  item[24] = {36 , "Present Position"              , 2};
  item[25] = {38 , "Present Speed"                 , 2};
  item[26] = {40 , "Present Load"                  , 2};
  item[27] = {42 , "Present Voltage"               , 1};
  item[28] = {43 , "Present Temperature"           , 1};
  item[29] = {44 , "Registered"                    , 1};
  item[30] = {46 , "Moving"                        , 1};
  item[31] = {47 , "Lock"                          , 1};
  item[32] = {48 , "Punch"                         , 2};
  item[33] = {68 , "Current"                       , 2};
  item[34] = {70 , "Torque Control Mode Enable"    , 1};
  item[35] = {71 , "Goal Torque"                   , 2};
  item[36] = {73 , "Goal Acceleration"             , 1};

  the_number_of_item = 37;
#endif  
}

void setExtMXInfo()
{
  model_info.velocity_to_value_ratio         = 86.81;
  
  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setExtMX2Item(void)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud Rate"             , 1};
  item[2]  = {11 , "Operating Mode"        , 1};

  item[3]  = {64 , "Torque Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal Current"          , 2};
  item[6]  = {104, "Goal Velocity"         , 4};
  item[7]  = {108, "Profile Acceleration"  , 4};
  item[8]  = {112, "Profile Velocity"      , 4};
  item[9]  = {116, "Goal Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present Current"       , 2};
  item[12] = {128, "Present Velocity"      , 4};
  item[13] = {132, "Present Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"          , 2};
  item[1]  = {6  , "Firmware Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud Rate"             , 1};
  item[4]  = {9  , "Return Delay Time"     , 1};
  item[5]  = {10 , "Drive Mode"            , 1};
  item[6]  = {11 , "Operating Mode"        , 1};
  item[7]  = {12 , "Secondary(Shadow) ID"  , 1};
  item[8]  = {13 , "Protocol Version"      , 1};
  item[9]  = {20 , "Homing Offset"         , 4};
  item[10] = {24 , "Moving Threshold"      , 4};
  item[11] = {31 , "Temperature Limit"     , 1};
  item[12] = {32 , "Max Voltage Limit"     , 2};
  item[13] = {34 , "Min Voltage Limit"     , 2};
  item[14] = {36 , "PWM Limit"             , 2};
  item[15] = {40 , "Current Limit"         , 2};
  item[16] = {40 , "Acceleration Limit"    , 4};
  item[17] = {44 , "Velocity Limit"        , 4};
  item[18] = {48 , "Max Position Limit"    , 4};
  item[19] = {52 , "Min Position Limit"    , 4};
  item[20] = {63 , "Shutdown"              , 1};

  item[21] = {64 , "Torque Enable"         , 1};
  item[22] = {65 , "LED"                   , 1};
  item[23] = {68 , "Status Return Level"   , 1};
  item[24] = {69 , "Registered Instruction", 1};
  item[25] = {70 , "Hardware Error Status" , 1};
  item[26] = {76 , "Velocity I Gain"       , 2};
  item[27] = {78 , "Velocity P Gain"       , 2};
  item[28] = {80 , "Position D Gain"       , 2};
  item[29] = {82 , "Position I Gain"       , 2};
  item[30] = {84 , "Position P Gain"       , 2};
  item[31] = {88 , "Feedforward 2nd Gain"  , 2};
  item[32] = {90 , "Feedforward 1st Gain"  , 2};
  item[33] = {98 , "Bus Watchdog"          , 1};
  item[34] = {100, "Goal PWM"              , 2};
  item[35] = {102, "Goal Current"          , 2};
  item[36] = {104, "Goal Velocity"         , 4};
  item[37] = {108, "Profile Acceleration"  , 4};
  item[38] = {112, "Profile Velocity"      , 4};
  item[39] = {116, "Goal Position"         , 4};
  item[40] = {120, "Realtime Tick"         , 2};
  item[41] = {122, "Moving"                , 1};
  item[42] = {123, "Moving Status"         , 1};
  item[43] = {124, "Present PWM"           , 2};
  item[44] = {126, "Present Current"       , 2};
  item[45] = {128, "Present Velocity"      , 4};
  item[46] = {132, "Present Position"      , 4};
  item[47] = {136, "Velocity Trajectory"   , 4};
  item[48] = {140, "Position Trajectory"   , 4};
  item[49] = {144, "Present Input Voltage" , 2};
  item[50] = {146, "Present Temperature"   , 1};

  the_number_of_item = 51;
#endif  
}

void setExtMX2Info(void)
{
  model_info.velocity_to_value_ratio         = 41.70;
  
  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setXL320Item()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {3  , "ID"                            , 1};
  item[1]  = {4  , "Baud Rate"                     , 1};
  item[2]  = {6  , "CW Angle Limit"                , 2};
  item[3]  = {8  , "CCW Angle Limit"               , 2};
  item[4]  = {11 , "Control Mode"                  , 1};

  item[5]  = {24 , "Torque ON/OFF"                 , 1};
  item[6]  = {25 , "LED"                           , 1};
  item[7]  = {30 , "Goal Position"                 , 2};
  item[8]  = {32 , "Moving Speed"                  , 2};
  item[9]  = {34 , "Torque Limit"                  , 2};
  item[10] = {37 , "Present Position"              , 2};
  item[11] = {39 , "Present Speed"                 , 2};
  item[12] = {41 , "Present Load"                  , 2};
  item[13] = {49 , "Moving"                        , 1};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"                  , 2};
  item[1]  = {2  , "Firmware Version"              , 1};
  item[2]  = {3  , "ID"                            , 1};
  item[3]  = {4  , "Baud Rate"                     , 1};
  item[4]  = {5  , "Return Delay Time"             , 1};
  item[5]  = {6  , "CW Angle Limit"                , 2};
  item[6]  = {8  , "CCW Angle Limit"               , 2};
  item[7]  = {11 , "Control Mode"                  , 1};
  item[8]  = {12 , "Temperature Limit"             , 1};
  item[9]  = {13 , "Min Voltage Limit"             , 1};
  item[10] = {14 , "Max Voltage Limit"             , 1};
  item[11] = {15 , "Max Torque"                    , 2};
  item[12] = {17 , "Return Level"                  , 2};
  item[13] = {18 , "Alarm Shutdown"                , 2};

  item[14] = {24 , "Torque Enable"                 , 1};
  item[15] = {25 , "LED"                           , 1};
  item[16] = {27 , "D gain"                        , 1};
  item[17] = {28 , "I gain"                        , 1};
  item[18] = {29 , "P gain"                        , 1};
  item[19] = {30 , "Goal Position"                 , 2};
  item[20] = {32 , "Moving Speed"                  , 2};
  item[21] = {34 , "Torque Limit"                  , 2};
  item[22] = {37 , "Present Position"              , 2};
  item[23] = {39 , "Present Speed"                 , 2};
  item[24] = {41 , "Present Load"                  , 2};
  item[25] = {45 , "Present Voltage"               , 1};
  item[26] = {46 , "Present Temperature"           , 1};
  item[27] = {47 , "Registered"                    , 1};
  item[28] = {49 , "Moving"                        , 1};
  item[29] = {50 , "Hardware Error Status"         , 1};
  item[30] = {51 , "Punch"                         , 2};

  the_number_of_item = 31;
#endif  
}

void setXL320Info()
{
  model_info.velocity_to_value_ratio         = 86.03; // XL320 don't support exact speed in wheel mode.
  
  model_info.value_of_0_radian_position      = 512;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 1024;

  model_info.min_radian                      = -2.61799;
  model_info.max_radian                      =  2.61799;
}

void setXLItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud Rate"             , 1};
  item[2]  = {11 , "Operating Mode"        , 1};

  item[3]  = {64 , "Torque Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal Current"          , 2};
  item[6]  = {104, "Goal Velocity"         , 4};
  item[7]  = {108, "Profile Acceleration"  , 4};
  item[8]  = {112, "Profile Velocity"      , 4};
  item[9]  = {116, "Goal Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present Load"          , 2};
  item[12] = {128, "Present Velocity"      , 4};
  item[13] = {132, "Present Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"          , 2};
  item[1]  = {6  , "Firmware Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud Rate"             , 1};
  item[4]  = {9  , "Return Delay Time"     , 1};
  item[5]  = {10 , "Drive Mode"            , 1};
  item[6]  = {11 , "Operating Mode"        , 1};
  item[7]  = {12 , "Secondary(Shadow) ID"  , 1};
  item[8]  = {13 , "Protocol Version"      , 1};
  item[9]  = {20 , "Homing Offset"         , 4};
  item[10] = {24 , "Moving Threshold"      , 4};
  item[11] = {31 , "Temperature Limit"     , 1};
  item[12] = {32 , "Max Voltage Limit"     , 2};
  item[13] = {34 , "Min Voltage Limit"     , 2};
  item[14] = {36 , "PWM Limit"             , 2};
  item[15] = {40 , "Acceleration Limit"    , 4};
  item[16] = {44 , "Velocity Limit"        , 4};
  item[17] = {48 , "Max Position Limit"    , 4};
  item[18] = {52 , "Min Position Limit"    , 4};
  item[19] = {63 , "Shutdown"              , 1};

  item[20] = {64 , "Torque Enable"         , 1};
  item[21] = {65 , "LED"                   , 1};
  item[22] = {68 , "Status Return Level"   , 1};
  item[23] = {69 , "Registered Instruction", 1};
  item[24] = {70 , "Hardware Error Status" , 1};
  item[25] = {76 , "Velocity I Gain"       , 2};
  item[26] = {78 , "Velocity P Gain"       , 2};
  item[27] = {80 , "Position D Gain"       , 2};
  item[28] = {82 , "Position I Gain"       , 2};
  item[29] = {84 , "Position P Gain"       , 2};
  item[30] = {88 , "Feedforward 2nd Gain"  , 2};
  item[31] = {90 , "Feedforward 1st Gain"  , 2};
  item[32] = {98 , "Bus Watchdog"          , 1};
  item[33] = {100, "Goal PWM"              , 2};
  item[34] = {102, "Goal Current"          , 2};
  item[35] = {104, "Goal Velocity"         , 4};
  item[36] = {108, "Profile Acceleration"  , 4};
  item[37] = {112, "Profile Velocity"      , 4};
  item[38] = {116, "Goal Position"         , 4};
  item[39] = {120, "Realtime Tick"         , 2};
  item[40] = {122, "Moving"                , 1};
  item[41] = {123, "Moving Status"         , 1};
  item[42] = {124, "Present PWM"           , 2};
  item[43] = {126, "Present Load"          , 2};
  item[44] = {128, "Present Velocity"      , 4};
  item[45] = {132, "Present Position"      , 4};
  item[46] = {136, "Velocity Trajectory"   , 4};
  item[47] = {140, "Position Trajectory"   , 4};
  item[48] = {144, "Present Input Voltage" , 2};
  item[49] = {146, "Present Temperature"   , 1};

  the_number_of_item = 50;
#endif  
}

void setXLInfo()
{
  model_info.velocity_to_value_ratio         = 41.70;

  model_info.value_of_0_radian_position      = 2048;
  model_info.value_of_min_radian_position    = 0;
  model_info.value_of_max_radian_position    = 4095;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

void setXMItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud Rate"             , 1};
  item[2]  = {11 , "Operating Mode"        , 1};

  item[3]  = {64 , "Torque Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal Current"          , 2};
  item[6]  = {104, "Goal Velocity"         , 4};
  item[7]  = {108, "Profile Acceleration"  , 4};
  item[8] =  {112, "Profile Velocity"      , 4};
  item[9] = {116, "Goal Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present Current"       , 2};
  item[12] = {128, "Present Velocity"      , 4};
  item[13] = {132, "Present Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"          , 2};
  item[1]  = {6  , "Firmware Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud Rate"             , 1};
  item[4]  = {9  , "Return Delay Time"     , 1};
  item[5]  = {10 , "Drive Mode"            , 1};
  item[6]  = {11 , "Operating Mode"        , 1};
  item[7]  = {12 , "Secondary(Shadow) ID"  , 1};
  item[8]  = {13 , "Protocol Version"      , 1};
  item[9]  = {20 , "Homing Offset"         , 4};
  item[10] = {24 , "Moving Threshold"      , 4};
  item[11] = {31 , "Temperature Limit"     , 1};
  item[12] = {32 , "Max Voltage Limit"     , 2};
  item[13] = {34 , "Min Voltage Limit"     , 2};
  item[14] = {36 , "PWM Limit"             , 2};
  item[15] = {40 , "Current Limit"         , 2};
  item[16] = {40 , "Acceleration Limit"    , 4};
  item[17] = {44 , "Velocity Limit"        , 4};
  item[18] = {48 , "Max Position Limit"    , 4};
  item[19] = {52 , "Min Position Limit"    , 4};
  item[20] = {63 , "Shutdown"              , 1};

  item[21] = {64 , "Torque Enable"         , 1};
  item[22] = {65 , "LED"                   , 1};
  item[23] = {68 , "Status Return Level"   , 1};
  item[24] = {69 , "Registered Instruction", 1};
  item[25] = {70 , "Hardware Error Status" , 1};
  item[26] = {76 , "Velocity I Gain"       , 2};
  item[27] = {78 , "Velocity P Gain"       , 2};
  item[28] = {80 , "Position D Gain"       , 2};
  item[29] = {82 , "Position I Gain"       , 2};
  item[30] = {84 , "Position P Gain"       , 2};
  item[31] = {88 , "Feedforward 2nd Gain"  , 2};
  item[32] = {90 , "Feedforward 1st Gain"  , 2};
  item[33] = {98 , "Bus Watchdog"          , 1};
  item[34] = {100, "Goal PWM"              , 2};
  item[35] = {102, "Goal Current"          , 2};
  item[36] = {104, "Goal Velocity"         , 4};
  item[37] = {108, "Profile Acceleration"  , 4};
  item[38] = {112, "Profile Velocity"      , 4};
  item[39] = {116, "Goal Position"         , 4};
  item[40] = {120, "Realtime Tick"         , 2};
  item[41] = {122, "Moving"                , 1};
  item[42] = {123, "Moving Status"         , 1};
  item[43] = {124, "Present PWM"           , 2};
  item[44] = {126, "Present Current"       , 2};
  item[45] = {128, "Present Velocity"      , 4};
  item[46] = {132, "Present Position"      , 4};
  item[47] = {136, "Velocity Trajectory"   , 4};
  item[48] = {140, "Position Trajectory"   , 4};
  item[49] = {144, "Present Input Voltage" , 2};
  item[50] = {146, "Present Temperature"   , 1};

  the_number_of_item = 51;
#endif  
}

void setXMInfo()
{
  model_info.velocity_to_value_ratio       = 41.70;
  model_info.torque_to_current_value_ratio = 149.795386991;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4095;

  model_info.min_radian = -3.14159265;
  model_info.max_radian =  3.14159265;
}

void setExtXMItem(void)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud Rate"             , 1};
  item[2]  = {11 , "Operating Mode"        , 1};

  item[3]  = {64 , "Torque Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal Current"          , 2};
  item[6]  = {104, "Goal Velocity"         , 4};
  item[7]  = {108, "Profile Acceleration"  , 4};
  item[8]  = {112, "Profile Velocity"      , 4};
  item[9] = {116, "Goal Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present Current"       , 2};
  item[12] = {128, "Present Velocity"      , 4};
  item[13] = {132, "Present Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"          , 2};
  item[1]  = {6  , "Firmware Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud Rate"             , 1};
  item[4]  = {9  , "Return Delay Time"     , 1};
  item[5]  = {10 , "Drive Mode"            , 1};
  item[6]  = {11 , "Operating Mode"        , 1};
  item[7]  = {12 , "Secondary(Shadow) ID"  , 1};
  item[8]  = {13 , "Protocol Version"      , 1};
  item[9]  = {20 , "Homing Offset"         , 4};
  item[10] = {24 , "Moving Threshold"      , 4};
  item[11] = {31 , "Temperature Limit"     , 1};
  item[12] = {32 , "Max Voltage Limit"     , 2};
  item[13] = {34 , "Min Voltage Limit"     , 2};
  item[14] = {36 , "PWM Limit"             , 2};
  item[15] = {40 , "Current Limit"         , 2};
  item[16] = {40 , "Acceleration Limit"    , 4};
  item[17] = {44 , "Velocity Limit"        , 4};
  item[18] = {48 , "Max Position Limit"    , 4};
  item[19] = {52 , "Min Position Limit"    , 4};
  item[20] = {56 , "External Port Mode 1"  , 1};
  item[21] = {57 , "External Port Mode 2"  , 1};
  item[22] = {58 , "External Port Mode 3"  , 1};
  item[23] = {63 , "Shutdown"              , 1};

  item[24] = {64 , "Torque Enable"         , 1};
  item[25] = {65 , "LED"                   , 1};
  item[26] = {68 , "Status Return Level"   , 1};
  item[27] = {69 , "Registered Instruction", 1};
  item[28] = {70 , "Hardware Error Status" , 1};
  item[29] = {76 , "Velocity I Gain"       , 2};
  item[30] = {78 , "Velocity P Gain"       , 2};
  item[31] = {80 , "Position D Gain"       , 2};
  item[32] = {82 , "Position I Gain"       , 2};
  item[33] = {84 , "Position P Gain"       , 2};
  item[34] = {88 , "Feedforward 2nd Gain"  , 2};
  item[35] = {90 , "Feedforward 1st Gain"  , 2};
  item[36] = {98 , "Bus Watchdog"          , 1};
  item[37] = {100, "Goal PWM"              , 2};
  item[38] = {102, "Goal Current"          , 2};
  item[39] = {104, "Goal Velocity"         , 4};
  item[40] = {108, "Profile Acceleration"  , 4};
  item[41] = {112, "Profile Velocity"      , 4};
  item[42] = {116, "Goal Position"         , 4};
  item[43] = {120, "Realtime Tick"         , 2};
  item[44] = {122, "Moving"                , 1};
  item[45] = {123, "Moving Status"         , 1};
  item[46] = {124, "Present PWM"           , 2};
  item[47] = {126, "Present Current"       , 2};
  item[48] = {128, "Present Velocity"      , 4};
  item[49] = {132, "Present Position"      , 4};
  item[50] = {136, "Velocity Trajectory"   , 4};
  item[51] = {140, "Position Trajectory"   , 4};
  item[52] = {144, "Present Input Voltage" , 2};
  item[53] = {146, "Present Temperature"   , 1};

  the_number_of_item = 54;
#endif  
}

void setExtXMInfo(void)
{
  model_info.velocity_to_value_ratio       = 41.70;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4095;

  model_info.min_radian = -3.14159265;
  model_info.max_radian =  3.14159265;
}

void setXHItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud Rate"             , 1};
  item[2]  = {11 , "Operating Mode"        , 1};

  item[3]  = {64 , "Torque Enable"         , 1};
  item[4]  = {65 , "LED"                   , 1};
  item[5]  = {102, "Goal Current"          , 2};
  item[6]  = {104, "Goal Velocity"         , 4};
  item[7]  = {108, "Profile Acceleration"  , 4};
  item[8]  = {112, "Profile Velocity"      , 4};
  item[9]  = {116, "Goal Position"         , 4};
  item[10] = {122, "Moving"                , 1};
  item[11] = {126, "Present Current"       , 2};
  item[12] = {128, "Present Velocity"      , 4};
  item[13] = {132, "Present Position"      , 4};

  the_number_of_item = 14;
#else
  item[0]  = {0  , "Model Number"          , 2};
  item[1]  = {6  , "Firmware Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud Rate"             , 1};
  item[4]  = {9  , "Return Delay Time"     , 1};
  item[5]  = {10 , "Drive Mode"            , 1};
  item[6]  = {11 , "Operating Mode"        , 1};
  item[7]  = {12 , "Secondary(Shadow) ID"  , 1};
  item[8]  = {13 , "Protocol Version"      , 1};
  item[9]  = {20 , "Homing Offset"         , 4};
  item[10] = {24 , "Moving Threshold"      , 4};
  item[11] = {31 , "Temperature Limit"     , 1};
  item[12] = {32 , "Max Voltage Limit"     , 2};
  item[13] = {34 , "Min Voltage Limit"     , 2};
  item[14] = {36 , "PWM Limit"             , 2};
  item[15] = {40 , "Current Limit"         , 2};
  item[16] = {40 , "Acceleration Limit"    , 4};
  item[17] = {44 , "Velocity Limit"        , 4};
  item[18] = {48 , "Max Position Limit"    , 4};
  item[19] = {52 , "Min Position Limit"    , 4};
  item[20] = {63 , "Shutdown"              , 1};

  item[21] = {64 , "Torque Enable"         , 1};
  item[22] = {65 , "LED"                   , 1};
  item[23] = {68 , "Status Return Level"   , 1};
  item[24] = {69 , "Registered Instruction", 1};
  item[25] = {70 , "Hardware Error Status" , 1};
  item[26] = {76 , "Velocity I Gain"       , 2};
  item[27] = {78 , "Velocity P Gain"       , 2};
  item[28] = {80 , "Position D Gain"       , 2};
  item[29] = {82 , "Position I Gain"       , 2};
  item[30] = {84 , "Position P Gain"       , 2};
  item[31] = {88 , "Feedforward 2nd Gain"  , 2};
  item[32] = {90 , "Feedforward 1st Gain"  , 2};
  item[33] = {98 , "Bus Watchdog"          , 1};
  item[34] = {100, "Goal PWM"              , 2};
  item[35] = {102, "Goal Current"          , 2};
  item[36] = {104, "Goal Velocity"         , 4};
  item[37] = {108, "Profile Acceleration"  , 4};
  item[38] = {112, "Profile Velocity"      , 4};
  item[39] = {116, "Goal Position"         , 4};
  item[40] = {120, "Realtime Tick"         , 2};
  item[41] = {122, "Moving"                , 1};
  item[42] = {123, "Moving Status"         , 1};
  item[43] = {124, "Present PWM"           , 2};
  item[44] = {126, "Present Current"       , 2};
  item[45] = {128, "Present Velocity"      , 4};
  item[46] = {132, "Present Position"      , 4};
  item[47] = {136, "Velocity Trajectory"   , 4};
  item[48] = {140, "Position Trajectory"   , 4};
  item[49] = {144, "Present Input Voltage" , 2};
  item[50] = {146, "Present Temperature"   , 1};

  the_number_of_item = 51;
#endif  
}

void setXHInfo()
{
  model_info.velocity_to_value_ratio       = 41.71;

  model_info.value_of_min_radian_position  = 0;
  model_info.value_of_0_radian_position    = 2048;
  model_info.value_of_max_radian_position  = 4095;

  model_info.min_radian = -3.14159265;
  model_info.max_radian = 3.14159265;
}

void setPROItem()
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
  item[0]  = {7  , "ID"                    , 1};
  item[1]  = {8  , "Baud Rate"             , 1};
  item[2]  = {11 , "Operating Mode"        , 1};

  item[3]  = {562, "Torque Enable"         , 1};
  item[4]  = {563, "LED RED"               , 1};
  item[5]  = {596, "Goal Position"         , 4};
  item[6]  = {600, "Goal Velocity"         , 4};
  item[7]  = {604, "Goal Torque"           , 2};
  item[8]  = {606, "Goal Acceleration"     , 4};
  item[9]  = {610, "Moving"                , 1};
  item[10] = {611, "Present Position"      , 4};
  item[11] = {615, "Present Velocity"      , 4};
  item[12] = {621, "Present Current"       , 2};

  the_number_of_item = 13;
#else
  item[0]  = {0  , "Model Number"          , 2};
  item[1]  = {6  , "Firmware Version"      , 1};
  item[2]  = {7  , "ID"                    , 1};
  item[3]  = {8  , "Baud Rate"             , 1};
  item[4]  = {9  , "Return Delay Time"     , 1};
  item[5]  = {11 , "Operating Mode"        , 1};
  item[6]  = {13 , "Homing Offset"         , 4};
  item[7]  = {17 , "Moving Threshold"      , 4};
  item[8]  = {21 , "Temperature Limit"     , 1};
  item[9]  = {22 , "Max Voltage Limit"     , 2};
  item[10] = {24 , "Min Voltage Limit"     , 2};
  item[11] = {26 , "Acceleration Limit"    , 4};
  item[12] = {30 , "Torque Limit"          , 2};
  item[13] = {32 , "Velocity Limit"        , 4};
  item[14] = {36 , "Max Position Limit"    , 4};
  item[15] = {40 , "Min Position Limit"    , 4};
  item[16] = {44 , "External Port Mode 1"  , 1};
  item[17] = {45 , "External Port Mode 2"  , 1};
  item[18] = {46 , "External Port Mode 3"  , 1};
  item[19] = {47 , "External Port Mode 4"  , 1};
  item[20] = {48 , "Shutdown"              , 1};

  item[20] = {562, "Torque Enable"         , 1};
  item[21] = {563, "LED RED"               , 1};
  item[22] = {564, "LED GREEN"             , 1};
  item[23] = {565, "LED BLUE"              , 1};
  item[24] = {586, "Velocity I Gain"       , 2};
  item[25] = {588, "Velocity P Gain"       , 2};
  item[26] = {594, "Position P Gain"       , 2};
  item[27] = {596, "Goal Position"         , 4};
  item[28] = {600, "Goal Velocity"         , 4};
  item[29] = {604, "Goal Torque"           , 2};
  item[30] = {606, "Goal Acceleration"     , 4};
  item[31] = {610, "Moving"                , 1};
  item[32] = {611, "Present Position"      , 4};
  item[33] = {615, "Present Velocity"      , 4};
  item[34] = {621, "Present Current"       , 2};
  item[35] = {623, "Present Input Voltage" , 2};
  item[36] = {625, "Present Temperature"   , 1};
  item[37] = {626, "External Port Mode 1"  , 1};
  item[38] = {628, "External Port Mode 2"  , 1};
  item[39] = {630, "External Port Mode 3"  , 1};
  item[40] = {632, "External Port Mode 4"  , 1};
  item[41] = {890, "Registered Instruction", 1};
  item[42] = {891, "Status Return Level"   , 1};
  item[43] = {892, "Hardware Error Status" , 1};

  the_number_of_item = 44;
#endif  
}

void setPROInfo()
{
  model_info.velocity_to_value_ratio         = 4792.8;
  
  model_info.value_of_0_radian_position      = 0;
  model_info.value_of_min_radian_position    = -250961;
  model_info.value_of_max_radian_position    =  250961;

  model_info.min_radian                      = -3.14159265;
  model_info.max_radian                      =  3.14159265;
}

ControlTableItem* getConrolTableItem(uint16_t model_number)
{
  uint16_t num = model_number;

  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    setAXItem();
  }
  else if (num == RX_24F || num == RX_28 || num == RX_64)
  {
    setRXItem();
  }
  else if (num == EX_106)
  {
    setEXItem();
  }
  else if (num == MX_12W || num == MX_28)
  {
    setMXItem();
  }
  else if (num == MX_64 || num == MX_106)
  {
    setExtMXItem();
  }
  else if (num == MX_28_2)
  {
    setMX2Item();
  }
  else if (num == MX_64_2 || num == MX_106_2)
  {
    setExtMX2Item();
  }
  else if (num == XL_320)
  {
    setXL320Item();
  }
  else if (num == XL430_W250)
  {
    setXLItem();
  }
  else if (num == XM430_W210 || num == XM430_W350)
  {
    setXMItem();
  }
  else if (num == XM540_W150 || num == XM540_W270)
  {
    setExtXMItem();
  }
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    setXHItem();
  }
  else if (num == PRO_L42_10_S300_R  || num == PRO_L54_30_S400_R || num == PRO_L54_30_S500_R || num == PRO_L54_50_S290_R || num == PRO_L54_50_S500_R  ||
           num == PRO_M42_10_S260_R  || num == PRO_M54_40_S250_R || num == PRO_M54_60_S250_R || num == PRO_H42_20_S300_R || num == PRO_H54_100_S500_R ||
           num == PRO_H54_200_S500_R)
  {
    setPROItem();
  }
  else
  {
    setXMItem();
  }

  return item;
}

ModelInfo* getModelInfo(uint16_t model_number)
{  
  uint16_t num = model_number;

  if (num == AX_12A || num == AX_12W || num == AX_18A)
  {
    setAXInfo();
  }
  else if (num == RX_24F || num == RX_28 || num == RX_64)
  {
    setRXInfo();
  }
  else if (num == EX_106)
  {
    setEXInfo();
  }
  else if (num == MX_12W || num == MX_28)
  {
    setMXInfo();
  }
  else if (num == MX_64 || num == MX_106)
  {
    setExtMXInfo();
  }
  else if (num == MX_28_2)
  {
    setMX2Info();
  }
  else if (num == MX_64_2 || num == MX_106_2)
  {
    setExtMX2Info();
  }
  else if (num == XL_320)
  {
    setXL320Info();
  }
  else if (num == XL430_W250)
  {
    setXLInfo();
  }
  else if (num == XM430_W210 || num == XM430_W350)
  {
    setXMInfo();
  }
  else if (num == XM540_W150 || num == XM540_W270)
  {
    setExtXMInfo();
  }
  else if (num == XH430_V210 || num == XH430_V350 || num == XH430_W210 || num == XH430_W350)
  {
    setXHInfo();
  }
  else if (num == PRO_L42_10_S300_R  || num == PRO_L54_30_S400_R || num == PRO_L54_30_S500_R || num == PRO_L54_50_S290_R || num == PRO_L54_50_S500_R  ||
           num == PRO_M42_10_S260_R  || num == PRO_M54_40_S250_R || num == PRO_M54_60_S250_R || num == PRO_H42_20_S300_R || num == PRO_H54_100_S500_R ||
           num == PRO_H54_200_S500_R)
  {
    setPROInfo();
  }
  else
  {
    setXMInfo();
  }

  return &model_info;
}

uint8_t getTheNumberOfControlItem()
{
  return the_number_of_item;
}
