/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

#include "../../include/dynamixel_workbench_toolbox/dynamixel_workbench.h"

// MX
// Wheel Mode	both are 0
// Joint Mode	neither are 0
// Multi-turn Mode	both are 4095

static const uint8_t XL_320_WHEEL_MODE = 1;
static const uint8_t XL_320_JOINT_MODE = 2;

static const uint8_t SERIES_CURRENT_CONTROL_MODE                  = 0;
static const uint8_t SERIES_TORQUE_CONTROL_MODE                 = 0;
static const uint8_t SERIES_VELOCITY_CONTROL_MODE                 = 1;
static const uint8_t SERIES_POSITION_CONTROL_MODE                 = 3;
static const uint8_t SERIES_EXTENDED_POSITION_CONTROL_MODE        = 4;
static const uint8_t SERIES_CURRENT_BASED_POSITION_CONTROL_MODE   = 5;
static const uint8_t SERIES_VOLTAGE_CONTROL_MODE                  = 16;

DynamixelWorkbench::DynamixelWorkbench(){}

DynamixelWorkbench::~DynamixelWorkbench(){}

bool DynamixelWorkbench::torque(uint8_t id, bool onoff, const char **log)
{
  bool result = false;

  result = writeRegister(id, "Torque_Enable", (uint8_t)onoff, log);
  if (result == false)
    return false;

  *log = "[DynamixelWorkbench] Succeeded to change torque status!";
  return true;
}

bool DynamixelWorkbench::torqueOn(uint8_t id, const char **log)
{
  bool result = false;

  result = torque(id, (uint8_t)1, log);

  return result;
}

bool DynamixelWorkbench::torqueOff(uint8_t id, const char **log)
{
  bool result = false;

  result = torque(id, (uint8_t)0, log);

  return result;
}

bool DynamixelWorkbench::setID(uint8_t id, uint8_t new_id, const char **log)
{
  bool result = false;

  result = torqueOff(id, log);
  if (result == false) return false;

  result = writeRegister(id, "ID", new_id, log);
  // millis(1000);
  if (result == false) return false;

  *log = "[DynamixelWorkbench] Succeeded to change ID!";
  return result;
}

bool DynamixelWorkbench::setBaud(uint8_t id, uint32_t new_baudrate, const char **log)
{
  bool result = false;

  result = torqueOff(id, log);
  if (result == false) return false;

  if (getProtocolVersion() == 1.0f)
  {
    switch (new_baudrate)
    {
      case 9600:
        result = writeRegister(id, "Baud_Rate", (uint8_t)207, log);
        if (result == false) return false;
       break;

      case 19200:
        result = writeRegister(id, "Baud_Rate", (uint8_t)103, log);
        if (result == false) return false;
       break;

      case 57600:
        result = writeRegister(id, "Baud_Rate", (uint8_t)34, log);
        if (result == false) return false;
       break;

      case 115200:
        result = writeRegister(id, "Baud_Rate", (uint8_t)16, log);
        if (result == false) return false;
       break;

      case 200000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)9, log);
        if (result == false) return false;    
       break;

      case 250000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)7, log);
        if (result == false) return false;       
       break;
       
      case 400000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)4, log);
        if (result == false) return false;       
       break;

      case 500000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)3, log);
        if (result == false) return false;       
       break;

      case 1000000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)1, log);
        if (result == false) return false;       
       break;

      case 2250000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)250, log);
        if (result == false) return false;       
       break;

      case 2500000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)251, log);
        if (result == false) return false;       
       break;

      case 3000000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)252, log);
        if (result == false) return false;       
       break;
       
      default:
        result = writeRegister(id, "Baud_Rate", (uint8_t)34, log);
        if (result == false) return false;
       break;
    }
  }
  else if (getProtocolVersion() == 2.0f)
  {    
    switch (new_baudrate)
    {
      case 9600:
        result = writeRegister(id, "Baud_Rate", (uint8_t)0, log);
        if (result == false) return false;       
       break;

      case 57600:
        result = writeRegister(id, "Baud_Rate", (uint8_t)1, log);
        if (result == false) return false;       
       break;

      case 115200:
        result = writeRegister(id, "Baud_Rate", (uint8_t)2, log);
        if (result == false) return false;       
       break;

      case 1000000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)3, log);
        if (result == false) return false;       
       break;

      case 2000000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)4, log);
        if (result == false) return false;       
       break;

      case 3000000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)5, log);
        if (result == false) return false;       
       break;
       
      case 4000000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)6, log);
        if (result == false) return false;       
       break;

      case 4500000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)7, log);
        if (result == false) return false;       
       break;

      case 10500000:
        result = writeRegister(id, "Baud_Rate", (uint8_t)8, log);
        if (result == false) return false;       
       break;
       
      default:
        result = writeRegister(id, "Baud_Rate", (uint8_t)1, log);
        if (result == false) return false;       
       break;
    }
  }
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(2000);
#else
    usleep(1000*2000);
#endif

  *log = "[DynamixelWorkbench] Succeeded to change Baud Rate!";
  return result;
}


bool DynamixelWorkbench::led(uint8_t id, bool onoff, const char **log)
{
  bool result = false;

  result = writeRegister(id, "LED", (uint8_t)onoff, log);
  if (result == false)
    return false;

  *log = "[DynamixelWorkbench] Succeeded to change led status!";
  return true;
}

bool DynamixelWorkbench::ledOn(uint8_t id, const char **log)
{
  bool result = false;

  result = led(id, (uint8_t)1, log);

  return result;
}

bool DynamixelWorkbench::ledOff(uint8_t id, const char **log)
{
  bool result = false;

  result = led(id, (uint8_t)0, log);

  return result;
}

bool DynamixelWorkbench::setPositionControlMode(uint8_t id, const char **log)
{
  bool result = false;

  const char* model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  if (getProtocolVersion() == 1.0)
  {
    if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(model_name, "XL430", strlen("XL430"))       ||
        !strncmp(model_name, "XM", strlen("XM"))             ||
        !strncmp(model_name, "XH", strlen("XH"))             ||
        !strncmp(model_name, "PRO", strlen("PRO")))
    {
      result = writeRegister(id, "Operating_Mode", (uint8_t)SERIES_POSITION_CONTROL_MODE, log);
      if (result == false) return false;
    }
    else if (!strncmp(model_name, "AX", 2) || !strncmp(model_name, "RX", 2))
    {
      result = writeRegister(id, "CW_Angle_Limit", (uint16_t)0, log);
      if (result == false) return false;

      result = writeRegister(id, "CCW_Angle_Limit", (uint16_t)1023, log);
      if (result == false) return false;
    }
    else
    {
      result = writeRegister(id, "CW_Angle_Limit", (uint16_t)0, log);
      if (result == false) return false;

      result = writeRegister(id, "CCW_Angle_Limit", (uint16_t)4095, log);
      if (result == false) return false;
    }
  }
  else if (getProtocolVersion() == 2.0)
  {
    if (!strncmp(model_name, "XL-320", 6))
    {
      result = writeRegister(id, "CW_Angle_Limit", (uint16_t)0, log);
      if (result == false) return false;

      result = writeRegister(id, "CCW_Angle_Limit", (uint16_t)1023, log);
      if (result == false) return false;
    }
    else
    {
      result = writeRegister(id, "Operating_Mode", (uint8_t)SERIES_POSITION_CONTROL_MODE, log);
      if (result == false) return false;
    }
  }
  // millis(10);

  return result;
}

bool DynamixelWorkbench::jointMode(uint8_t id, uint32_t velocity, uint32_t acceleration, const char **log)
{
  bool result = false;

  const char* model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  result = torqueOff(id, log);
  if (result == false) return false;

  result = setPositionControlMode(id, log);
  if (result == false) return false;

  if (getProtocolVersion() == 1.0)
  {
    if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(model_name, "XL430", strlen("XL430"))       ||
        !strncmp(model_name, "XM", strlen("XM"))             ||
        !strncmp(model_name, "XH", strlen("XH")))
    {
      result = writeRegister(id, "Profile_Acceleration", acceleration, log);
      if (result == false) return false;

      result = writeRegister(id, "Profile_Velocity", velocity, log);
      if (result == false) return false;
    }
    else
    {
      result = writeRegister(id, "Moving_Speed", velocity, log);
      if (result == false) return false;
    }
  }
  else if (getProtocolVersion() == 2.0)
  {
    if (!strncmp(model_name, "XL-320", strlen("XL-320")) || !strncmp(model_name, "PRO", strlen("PRO")))
    {
      result = writeRegister(id, "Moving_Speed", velocity, log);
      if (result == false) return false;
    }
    else
    {
      result = writeRegister(id, "Profile_Acceleration", acceleration, log);
      if (result == false) return false;

      result = writeRegister(id, "Profile_Velocity", velocity, log);
      if (result == false) return false;
    }
  }

  return result;
}

#if 0
bool DynamixelWorkbench::wheelMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  bool result = false;

  strcpy(dxl_, getModelName(id));

  result = torque(id, false);

  result = setVelocityControlMode(id);

  result = torque(id, true);

  if (getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(dxl_, "XL430", strlen("XL430"))       ||
        !strncmp(dxl_, "XM", strlen("XM"))             ||
        !strncmp(dxl_, "XH", strlen("XH")))
    {
      result = writeRegister(id, "Profile_Acceleration", acc);
      result = writeRegister(id, "Profile_Velocity", vel);
    }
  }
  else if (getProtocolVersion() == 2.0 && (strncmp(dxl_, "PRO", 3) != 0))
  {   
    result = writeRegister(id, "Profile_Acceleration", acc);
    result = writeRegister(id, "Profile_Velocity", vel);
  }

  return result;
}

bool DynamixelWorkbench::currentMode(uint8_t id, uint8_t cur)
{
  bool result = false;

  strcpy(dxl_, getModelName(id));
  
  result = torque(id, false);

  result = setCurrentControlMode(id);

  result = torque(id, true);

  if (!strncmp(dxl_, "X", 1)                         ||
      !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) )
  {   
    result = writeRegister(id, "Goal_Current", cur);
  }

  return result;
}

bool DynamixelWorkbench::goalPosition(uint8_t id, int32_t goal)
{
  bool result = false;
  
  result = writeRegister(id, "Goal_Position", goal);

  return result;
}

bool DynamixelWorkbench::goalSpeed(uint8_t id, int32_t goal)
{
  bool result = false;

  strcpy(dxl_, getModelName(id));

  if (getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(dxl_, "XL430", strlen("XL430"))       ||
        !strncmp(dxl_, "XM", strlen("XM"))             ||
        !strncmp(dxl_, "XH", strlen("XH")))
    {
      result = writeRegister(id, "Goal_Velocity", goal);
    }
    else
    {
      if (goal < 0)
      {
        goal = (-1) * goal;
        goal |= 1024;
      }
      result = writeRegister(id, "Moving_Speed", goal);
    }
  }
  else if (getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      if (goal < 0)
      {
        goal = (-1) * goal;
        goal |= 1024;
      }
      result = writeRegister(id, "Moving_Speed", goal);
    }
    else
      result = writeRegister(id, "Goal_Velocity", goal);
  }

  return result;
}

int32_t DynamixelWorkbench::convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
  return convertRadian2Value(radian, max_position, min_position, max_radian, min_radian);
}

float DynamixelWorkbench::convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
{
  return convertValue2Radian(value, max_position, min_position, max_radian, min_radian);
}

int32_t DynamixelWorkbench::convertVelocity2Value(uint8_t id, float velocity)
{
  return convertVelocity2Value(id, velocity);
}

float DynamixelWorkbench::convertValue2Velocity(uint8_t id, int32_t value)
{
  return convertValue2Velocity(id, value);
}

// int16_t DynamixelWorkbench::convertTorque2Value(uint8_t id, float torque)
// {
//   return convertTorque2Value(id, torque);
// }

// float DynamixelWorkbench::convertValue2Torque(uint8_t id, int16_t value)
// {
//   return convertValue2Torque(id, value);
// }

bool DynamixelWorkbench::setVelocityControlMode(uint8_t id)
{
  bool result = false;

  strcpy(dxl_, getModelName(id));

  if (getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2"))   ||
        !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
        !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) ||
        !strncmp(dxl_, "XL430", strlen("XL430"))       ||
        !strncmp(dxl_, "XM", strlen("XM"))             ||
        !strncmp(dxl_, "XH", strlen("XH"))             ||
        !strncmp(dxl_, "PRO", strlen("PRO")))
    {
      result = writeRegister(id, "Operating_Mode", X_SERIES_VELOCITY_CONTROL_MODE);
    }
    else
    {
      result = writeRegister(id, "CW_Angle_Limit", 0);
      result = writeRegister(id, "CCW_Angle_Limit", 0);
    }
  }
  else if (getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      result = writeRegister(id, "CW_Angle_Limit", 0);
      result = writeRegister(id, "CCW_Angle_Limit", 0);
    }
    else
      result = writeRegister(id, "Operating_Mode", X_SERIES_VELOCITY_CONTROL_MODE);
  } 
  millis(10);

  return result;
}

bool DynamixelWorkbench::setCurrentControlMode(uint8_t id)
{
  bool result = false;

  strcpy(dxl_, getModelName(id));
  
  if (!strncmp(dxl_, "X", 1)                         ||
      !strncmp(dxl_, "MX-64-2", strlen("MX-64-2"))   ||
      !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")) )
  {
    result = writeRegister(id, "Operating_Mode", X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE);
  }   

  millis(10);

  return result;
}

// int32_t DynamixelDriver::convertRadian2Value(uint8_t id, float radian)
// {
//   int32_t value = 0;
//   uint8_t factor = getTool(id);

//   if (radian > 0)
//   {
//     value = (radian * (tools_[factor].getValueOfMaxRadianPosition() - tools_[factor].getValueOfZeroRadianPosition()) / tools_[factor].getMaxRadian()) + tools_[factor].getValueOfZeroRadianPosition();
//   }
//   else if (radian < 0)
//   {
//     value = (radian * (tools_[factor].getValueOfMinRadianPosition() - tools_[factor].getValueOfZeroRadianPosition()) / tools_[factor].getMinRadian()) + tools_[factor].getValueOfZeroRadianPosition();
//   }
//   else
//   {
//     value = tools_[factor].getValueOfZeroRadianPosition();
//   }

//   return value;
// }

// float DynamixelDriver::convertValue2Radian(uint8_t id, int32_t value)
// {
//   float radian = 0.0;
//   uint8_t factor = getTool(id);
//   if (factor == 0) factor = 0;  // just use first one

//   if (value > tools_[factor].getValueOfZeroRadianPosition())
//   {
//     radian = (float)(value - tools_[factor].getValueOfZeroRadianPosition()) * tools_[factor].getMaxRadian() / (float)(tools_[factor].getValueOfMaxRadianPosition() - tools_[factor].getValueOfZeroRadianPosition());
//   }
//   else if (value < tools_[factor].getValueOfZeroRadianPosition())
//   {
//     radian = (float)(value - tools_[factor].getValueOfZeroRadianPosition()) * tools_[factor].getMinRadian() / (float)(tools_[factor].getValueOfMinRadianPosition() - tools_[factor].getValueOfZeroRadianPosition());
//   }

//   return radian;
// }

// int32_t DynamixelDriver::convertRadian2Value(float radian, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
// {
//   int32_t value = 0;
//   int32_t zero_position = (max_position + min_position)/2;

//   if (radian > 0)
//   {
//     value = (radian * (max_position - zero_position) / max_radian) + zero_position;
//   }
//   else if (radian < 0)
//   {
//     value = (radian * (min_position - zero_position) / min_radian) + zero_position;
//   }
//   else
//   {
//     value = zero_position;
//   }

//   return value;
// }

// float DynamixelDriver::convertValue2Radian(int32_t value, int32_t max_position, int32_t min_position, float max_radian, float min_radian)
// {
//   float radian = 0.0;
//   int32_t zero_position = (max_position + min_position)/2;

//   if (value > zero_position)
//   {
//     radian = (float)(value - zero_position) * max_radian / (float)(max_position - zero_position);
//   }
//   else if (value < zero_position)
//   {
//     radian = (float)(value - zero_position) * min_radian / (float)(min_position - zero_position);
//   }

//   return radian;
// }

// int32_t DynamixelDriver::convertVelocity2Value(uint8_t id, float velocity)
// {
//   int32_t value = 0;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) factor = 0; 

//   // value = velocity * tools_[factor].getVelocityToValueRatio();

//   return value;
// }

// float DynamixelDriver::convertValue2Velocity(uint8_t id, int32_t value)
// {
//   float velocity = 0;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) factor = 0; 

//   // velocity = value / tools_[factor].getVelocityToValueRatio();

//   return velocity;
// }

// int16_t DynamixelDriver::convertTorque2Value(uint8_t id, float torque)
// {
//   int16_t value = 0;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) factor = 0; 

//   value = torque * tools_[factor].getTorqueToCurrentValueRatio();

//   return value;
// }

// float DynamixelDriver::convertValue2Torque(uint8_t id, int16_t value)
// {
//   float torque = 0.0;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) factor = 0; 

//   torque = value / tools_[factor].getTorqueToCurrentValueRatio();

//   return torque;
// }


#endif