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

#include "../../include/dynamixel_workbench/dynamixel_workbench.h"

DynamixelWorkbench::DynamixelWorkbench()
{

}

DynamixelWorkbench::~DynamixelWorkbench()
{

}

bool DynamixelWorkbench::begin(char* model_series, char* device_name, uint32_t baud_rate, float protocol_version)
{
  bool error = false;
  
  strcpy(dxl_, model_series);

  if (!strncmp(dxl_, "MX", 2) && (protocol_version == 2.0))
    error = driver_.begin(device_name, baud_rate, protocol_version);
  else
    error = driver_.begin(model_series, device_name, baud_rate);

  return error;
}

uint8_t  DynamixelWorkbench::scan(uint8_t *get_id)
{
  uint8_t id_cnt = 0;

  id_cnt = driver_.scan(get_id, 16);

  return id_cnt;
}

uint16_t DynamixelWorkbench::ping(uint8_t id)
{
  uint16_t model_num = 0;

  model_num = driver_.ping(id);

  return model_num;
}

bool DynamixelWorkbench::reboot(uint8_t id)
{
  bool error = false;

  error = driver_.reboot(id);

  return error;
}

bool DynamixelWorkbench::reset(uint8_t id)
{
  bool error = false;

  error = driver_.reset(id);

  return error;
}

bool DynamixelWorkbench::setID(uint8_t id, uint8_t new_id)
{
  driver_.writeRegister(id, "ID", new_id);
}

bool DynamixelWorkbench::setBaud(uint8_t id, uint32_t new_baud)
{
  driver_.writeRegister(id, "Baud Rate", new_baud);
}

bool DynamixelWorkbench::setProtocolVersion(uint8_t id, uint8_t new_version)
{
  driver_.writeRegister(id, "Protocol Version", new_version);
}

bool DynamixelWorkbench::jointMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  setPositionControlMode(id);

  torque(id, TRUE);

  if (driver_.getProtocolVersion() == 1.0)
  {
    driver_.writeRegister(id, "Moving Speed", vel);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {    
    if (!strncmp(dxl_, "XL320", 5) || !strncmp(dxl_, "PRO", 3))
    {
      driver_.writeRegister(id, "Moving Speed", vel);
    }
    else
    {
      driver_.writeRegister(id, "Profile Acceleration", acc);
      driver_.writeRegister(id, "Profile Velocity", vel);
    }
  }
}

bool DynamixelWorkbench::wheelMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  setVelocityControlMode(id);

  torque(id, TRUE);

  if (driver_.getProtocolVersion() == 2.0 && strncmp(dxl_, "PRO", 3))
  {   
    driver_.writeRegister(id, "Profile Acceleration", acc);
    driver_.writeRegister(id, "Profile Velocity", vel);
  }
}

bool DynamixelWorkbench::goalPosition(uint8_t id, uint16_t goal)
{
  driver_.writeRegister(id, "Goal Position", goal);
}

bool DynamixelWorkbench::goalSpeed(uint8_t id, int32_t goal)
{
  if (driver_.getProtocolVersion() == 1.0)
  {
    driver_.writeRegister(id, "Moving Speed", goal);  
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL320", 5))
      driver_.writeRegister(id, "Moving Speed", goal);
    else
      driver_.writeRegister(id, "Goal Velocity", goal);  
  }
}

bool DynamixelWorkbench::torque(uint8_t id, bool onoff)
{
  if (driver_.getProtocolVersion() == 1.0)
  {
    driver_.writeRegister(id, "Torque ON/OFF", onoff);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL320", 5))
    {
      driver_.writeRegister(id, "Torque ON/OFF", onoff);
    }
    else
    {
      driver_.writeRegister(id, "Torque Enable", onoff);
    }
  }
}

bool DynamixelWorkbench::setPositionControlMode(uint8_t id)
{
  if (driver_.getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "AX", 2) || !strncmp(dxl_, "RX", 2))
    {
      driver_.writeRegister(id, "CW Angle Limit", 0);
      driver_.writeRegister(id, "CCW Angle Limit", 1023);
    }
    else
    {
      driver_.writeRegister(id, "CW Angle Limit", 0);
      driver_.writeRegister(id, "CCW Angle Limit", 4095);
    }
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL320", 5))
    {
      driver_.writeRegister(id, "CW Angle Limit", 0);
      driver_.writeRegister(id, "CCW Angle Limit", 1023);
      driver_.writeRegister(id, "Control Mode", XL320_POSITION_CONTROL_MODE);
    }
    else
      driver_.writeRegister(id, "Operating Mode", X_SERIES_POSITION_CONTROL_MODE);
  }
}

bool DynamixelWorkbench::setVelocityControlMode(uint8_t id)
{
  if (driver_.getProtocolVersion() == 1.0)
  {
    driver_.writeRegister(id, "CW Angle Limit", 0);
    driver_.writeRegister(id, "CCW Angle Limit", 0);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL320", 5))
    {
      driver_.writeRegister(id, "CW Angle Limit", 0);
      driver_.writeRegister(id, "CCW Angle Limit", 0);
      driver_.writeRegister(id, "Control Mode", XL320_VELOCITY_CONTROL_MODE);
    }
    else
      driver_.writeRegister(id, "Operating Mode", X_SERIES_VELOCITY_CONTROL_MODE);
  }   
}