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

bool DynamixelWorkbench::begin(const char* device_name, uint32_t baud_rate)
{
  bool error = false;

  error = driver_.begin(device_name, baud_rate);

  return error;
}

bool DynamixelWorkbench::scan(uint8_t *get_id, uint8_t *get_id_num, float protocol_version)
{
  bool error = false;

  error = driver_.scan(get_id, get_id_num, 16, protocol_version);

  return error;
}

bool DynamixelWorkbench::ping(uint8_t id, uint16_t *get_model_number, float protocol_version)
{
  bool error = false;

  error = driver_.ping(id, get_model_number, protocol_version);

  return error;
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
  bool error = false;

  torque(id, false);

  error = driver_.writeRegister(id, "ID", new_id);

#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(1000);
#else
  usleep(1000*1000);
#endif

  return error;
}

bool DynamixelWorkbench::setBaud(uint8_t id, uint32_t new_baud)
{
  bool error = false;

  torque(id, false);

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (new_baud == 9600)
      error = driver_.writeRegister(id, "Baud Rate", 207);
    else if (new_baud == 57600)
      error = driver_.writeRegister(id, "Baud Rate", 34);
    else if (new_baud == 115200)
      error = driver_.writeRegister(id, "Baud Rate", 16);
    else if (new_baud == 1000000)
      error = driver_.writeRegister(id, "Baud Rate", 1);
    else if (new_baud == 2000000)
      error = driver_.writeRegister(id, "Baud Rate", 9);
    else
      error = driver_.writeRegister(id, "Baud Rate", 1);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {    
    if (new_baud == 9600)
      error = driver_.writeRegister(id, "Baud Rate", 0);
    else if (new_baud == 57600)
      error = driver_.writeRegister(id, "Baud Rate", 1);
    else if (new_baud == 115200)
      error = driver_.writeRegister(id, "Baud Rate", 2);
    else if (new_baud == 1000000)
      error = driver_.writeRegister(id, "Baud Rate", 3);
    else if (new_baud == 2000000)
      error = driver_.writeRegister(id, "Baud Rate", 4);
    else
      error = driver_.writeRegister(id, "Baud Rate", 3);
  }
#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(1000);
#else
  usleep(1000*1000);
#endif

  return error;
}

bool DynamixelWorkbench::setPacketHandler(float protocol_version)
{
  bool error = false;

  driver_.setPacketHandler(protocol_version, &error);

  return error;
}

char* DynamixelWorkbench::getModelName(uint8_t id)
{
  return driver_.getModelName(id);
}

bool DynamixelWorkbench::ledOn(uint8_t id, int32_t data)
{
  bool error = false;

  error = driver_.writeRegister(id, "LED", data);

  return error;
}

bool DynamixelWorkbench::ledOff(uint8_t id)
{
  bool error = false;

  error = driver_.writeRegister(id, "LED", 0);

  return error;
}

bool DynamixelWorkbench::jointMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  bool error = false;

  strcpy(dxl_, driver_.getModelName(id));

  error = torque(id, false);

  error = setPositionControlMode(id);

  error = torque(id, true);

  if (driver_.getProtocolVersion() == 1.0)
  {
    error = driver_.writeRegister(id, "Moving Speed", vel);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {    
    if (!strncmp(dxl_, "XL-320", 6) || !strncmp(dxl_, "PRO", 3))
    {
      error = driver_.writeRegister(id, "Moving Speed", vel);
    }
    else
    {
      error = driver_.writeRegister(id, "Profile Acceleration", acc);
      error = driver_.writeRegister(id, "Profile Velocity", vel);
    }
  }

  return error;
}

bool DynamixelWorkbench::wheelMode(uint8_t id, uint16_t vel, uint16_t acc)
{
  bool error = false;

  strcpy(dxl_, driver_.getModelName(id));

  error = torque(id, false);

  error = setVelocityControlMode(id);

  error = torque(id, true);

  if (driver_.getProtocolVersion() == 2.0 && (strncmp(dxl_, "PRO", 3) != 0))
  {   
    error = driver_.writeRegister(id, "Profile Acceleration", acc);
    error = driver_.writeRegister(id, "Profile Velocity", vel);
  }

  return error;
}

bool DynamixelWorkbench::currentMode(uint8_t id, uint8_t cur)
{
  bool error = false;

  strcpy(dxl_, driver_.getModelName(id));
  
  error = torque(id, false);

  error = setCurrentControlMode(id);

  error = torque(id, true);

  if (!strncmp(dxl_, "X", 1))
  {   
    error = driver_.writeRegister(id, "Goal Current", cur);
  }

  return error;
}

bool DynamixelWorkbench::goalPosition(uint8_t id, uint16_t goal)
{
  bool error = false;
  
  error = driver_.writeRegister(id, "Goal Position", goal);

  return error;
}

bool DynamixelWorkbench::goalSpeed(uint8_t id, int32_t goal)
{
  bool error = false;

  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (goal < 0)
    {
      goal = (-1) * goal;
      goal |= 1024;
    }
    error = driver_.writeRegister(id, "Moving Speed", goal);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      if (goal < 0)
      {
        goal = (-1) * goal;
        goal |= 1024;
      }
      error = driver_.writeRegister(id, "Moving Speed", goal);
    }
    else
      error = driver_.writeRegister(id, "Goal Velocity", goal);
  }

  return error;
}

bool DynamixelWorkbench::itemWrite(uint8_t id, const char* item_name, int32_t value)
{
  bool error = false;

  error = driver_.writeRegister(id, item_name, value);

  return error;
}

bool DynamixelWorkbench::syncWrite(const char *item_name, int32_t* value)
{
  bool error = false;

  error =  driver_.syncWrite(item_name, value);

  return error;
}

bool DynamixelWorkbench::bulkWrite()
{
  bool error = false;

  error = driver_.bulkWrite();

  return error;
}

int32_t DynamixelWorkbench::itemRead(uint8_t id, const char* item_name)
{
  static int32_t data = 0;

  if (driver_.readRegister(id, item_name, &data))
    return data;
}

int32_t* DynamixelWorkbench::syncRead(const char *item_name)
{
  static int32_t data[16];
  if (driver_.syncRead(item_name, data))
    return data;
}

int32_t DynamixelWorkbench::bulkRead(uint8_t id, const char* item_name)
{
  static int32_t data;
  if (driver_.bulkRead(id, item_name, &data))
    return data;
}

void DynamixelWorkbench::addSyncWrite(const char* item_name)
{
  driver_.addSyncWrite(item_name);
}

void DynamixelWorkbench::addSyncRead(const char* item_name)
{
  driver_.addSyncRead(item_name);
}

void DynamixelWorkbench::initBulkWrite()
{
  driver_.initBulkWrite();
}

void DynamixelWorkbench::initBulkRead()
{
  driver_.initBulkRead();
}

bool DynamixelWorkbench::addBulkWriteParam(uint8_t id, const char *item_name, int32_t data)
{
  bool error = false;

  error = driver_.addBulkWriteParam(id, item_name, data);

  return error;
}

bool DynamixelWorkbench::addBulkReadParam(uint8_t id, const char *item_name)
{
  bool error = false;

  error = driver_.addBulkReadParam(id, item_name);

  return error;
}

bool DynamixelWorkbench::setBulkRead()
{
  bool error = false;

  error = driver_.sendBulkReadPacket();

  return error;
}


/*/////////////////////////////////////////////////////////////////////////////
// Private Function
*//////////////////////////////////////////////////////////////////////////////

bool DynamixelWorkbench::torque(uint8_t id, bool onoff)
{
  bool error = false;

  error = driver_.writeRegister(id, "Torque Enable", onoff);

  return error;
}

bool DynamixelWorkbench::setPositionControlMode(uint8_t id)
{
  bool error = false;

  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    if (!strncmp(dxl_, "AX", 2) || !strncmp(dxl_, "RX", 2))
    {
      error = driver_.writeRegister(id, "CW Angle Limit", 0);
      error = driver_.writeRegister(id, "CCW Angle Limit", 1023);
    }
    else
    {
      error = driver_.writeRegister(id, "CW Angle Limit", 0);
      error = driver_.writeRegister(id, "CCW Angle Limit", 4095);
    }
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      error = driver_.writeRegister(id, "CW Angle Limit", 0);
      error = driver_.writeRegister(id, "CCW Angle Limit", 1023);
      error = driver_.writeRegister(id, "Control Mode", XL320_POSITION_CONTROL_MODE);
    }
    else
      error = driver_.writeRegister(id, "Operating Mode", X_SERIES_POSITION_CONTROL_MODE);
  }
#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(10);
#else
  usleep(1000*10);
#endif

  return error;
}

bool DynamixelWorkbench::setVelocityControlMode(uint8_t id)
{
  bool error = false;

  strcpy(dxl_, driver_.getModelName(id));

  if (driver_.getProtocolVersion() == 1.0)
  {
    error = driver_.writeRegister(id, "CW Angle Limit", 0);
    error = driver_.writeRegister(id, "CCW Angle Limit", 0);
  }
  else if (driver_.getProtocolVersion() == 2.0)
  {
    if (!strncmp(dxl_, "XL-320", 6))
    {
      error = driver_.writeRegister(id, "CW Angle Limit", 0);
      error = driver_.writeRegister(id, "CCW Angle Limit", 0);
      error = driver_.writeRegister(id, "Control Mode", XL320_VELOCITY_CONTROL_MODE);
    }
    else
      error = driver_.writeRegister(id, "Operating Mode", X_SERIES_VELOCITY_CONTROL_MODE);
  } 
#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(10);
#else
  usleep(1000*10);
#endif

  return error;
}

bool DynamixelWorkbench::setCurrentControlMode(uint8_t id)
{
  bool error = false;

  strcpy(dxl_, driver_.getModelName(id));
  
  if (!strncmp(dxl_, "X", 1))
  {
    error = driver_.writeRegister(id, "Operating Mode", X_SERIES_CURRENT_BASED_POSITION_CONTROL_MODE);
  }   

#if defined(__OPENCR__) || defined(__OPENCM904__)
  delay(10);
#else
  usleep(1000*10);
#endif

  return error;
}
