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

#include "../../include/dynamixel_workbench_toolbox/dynamixel_driver.h"

static dynamixel::PacketHandler *packetHandler_1 = dynamixel::PacketHandler::getPacketHandler(1.0f);
static dynamixel::PacketHandler *packetHandler_2 = dynamixel::PacketHandler::getPacketHandler(2.0f);

DynamixelDriver::DynamixelDriver() : tools_cnt_(0), sync_write_handler_cnt_(0), sync_read_handler_cnt_(0) {}

DynamixelDriver::~DynamixelDriver()
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      // writeRegister(tools_[i].getID()[j], "Torque_Enable", false);
    }
  }

  portHandler_->closePort();
}

void DynamixelDriver::initTools(void)
{
  tools_cnt_ = 0;

  for (uint8_t num = 0; num < tools_cnt_; num++)
    tools_[num].initTool();
}

bool DynamixelDriver::setTool(uint16_t model_number, uint8_t id, const char *err)
{
  bool result = false;

  // See if we have a matching tool? 
  for (uint8_t num = 0; num < tools_cnt_; num++)
  {
    if (tools_[num].getModelNumber() == model_number)
    {
      if (tools_[num].getDynamixelCount() < tools_[num].getDynamixelBuffer())
      {
        // Found one with the right model number and it is not full
        tools_[num].addDXL(model_number, id);
        return true;
      }
      else
      {
        err = "[DynamixelDriver] Too many Dynamixels are connected (default buffer size is 16, the same series of Dynamixels)";
        return false;
      }
    }
  }
  // We did not find one so lets allocate a new one
  if (tools_cnt_ < MAX_DXL_SERIES_NUM) 
  {
    // only do it if we still have some room...
    result = tools_[tools_cnt_++].addTool(model_number, id, err);
    return result;
  }
  else
  {
    err = "[DynamixelDriver] Too many series are connected (MAX = 5 different series)";
    return false;
  }

  err = "[DynamixelDriver] Failed to set the Tool";
  return false;
}

uint8_t DynamixelDriver::getTool(uint8_t id, const char *err)
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      if (tools_[i].getID()[j] == id)
      {
        return i;
      }
    }
  }

  err = "[DynamixelDriver] Failed to get the Tool";
  return 0xff;
}

bool DynamixelDriver::init(const char *device_name, uint32_t baud_rate, const char* err)
{
  bool result = false;

  result = setPortHandler(device_name, err);
  if (result == false) return false;

  result = setBaudrate(baud_rate, err);
  if (result == false) return false;

  return result;
}

bool DynamixelDriver::setPortHandler(const char *device_name, const char* err)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);

  if (portHandler_->openPort())
  {
    err = "[DynamixelDriver] Succeeded to open the port!";
    return true;
  }

  err = "[DynamixelDriver] Failed to open the port!";
  return false;
}

bool DynamixelDriver::setBaudrate(uint32_t baud_rate, const char* err)
{
  if (portHandler_->setBaudRate(baud_rate))
  {
    err = "[DynamixelDriver] Succeeded to change the baudrate!";
    return true;
  }

  err = "[DynamixelDriver] Failed to change the baudrate!";
  return false;
}

bool DynamixelDriver::setPacketHandler(float protocol_version, const char* err)
{
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  if (packetHandler_->getProtocolVersion() == protocol_version)
  {
    err = "[DynamixelDriver] Succeeded to set the protocol!";
    return true;
  }

  err = "[DynamixelDriver] Failed to set the protocol!";
  return false;
}

float DynamixelDriver::getProtocolVersion(void)
{
  return packetHandler_->getProtocolVersion();
}

uint32_t DynamixelDriver::getBaudrate(void)
{
  return portHandler_->getBaudRate();
}

const char* DynamixelDriver::getModelName(uint8_t id, const char *err)
{
  uint8_t factor = getTool(id, err);

  if (factor == 0xff) 
    return NULL;
  else
    return tools_[factor].getModelName();

  return NULL;
}

uint16_t DynamixelDriver::getModelNumber(uint8_t id, const char *err)
{
  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return 0;

  for (int i = 0; i < tools_[factor].getDynamixelCount(); i++)
  {
    if (tools_[factor].getID()[i] == id)
      return tools_[factor].getModelNumber();
  }

  return 0;
}

const ControlItem* DynamixelDriver::getControlTable(uint8_t id, const char* err)
{
  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return NULL;

  return tools_[factor].getControlTable();
}

uint8_t DynamixelDriver::getTheNumberOfControlItem(uint8_t id, const char *err)
{
  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return 0;

  return tools_[factor].getTheNumberOfControlItem();
}

bool DynamixelDriver::scan(uint8_t *get_id, uint8_t *get_the_number_of_id, uint8_t range, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint8_t id = 0;
  uint8_t id_cnt = 0;

  uint16_t model_number = 0;

  uint8_t get_range = range;

  if (get_range > 253) get_range = 253;

  initTools();

  for (id = 0; id <= get_range; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_1->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      err = packetHandler_1->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      err = packetHandler_1->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      get_id[id_cnt++] = id;
      setTool(model_number, id);
    }    
  }

  if (id_cnt > 0)
  {
    *get_the_number_of_id = id_cnt;
    result = setPacketHandler(1.0f, err);
    return result;
  }

  for (id = 0; id <= get_range; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_2->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      err = packetHandler_2->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      err = packetHandler_2->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      get_id[id_cnt++] = id;
      setTool(model_number, id);
    }   
  }

  if (id_cnt > 0)
  {
    *get_the_number_of_id = id_cnt;
    result = setPacketHandler(2.0f, err);
    return result;
  }

  err = "[DynamixelDriver] Failed to scan!";
  return false;
}

bool DynamixelDriver::ping(uint8_t id, uint16_t *get_model_number, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint16_t model_number = 0;

  sdk_error.dxl_comm_result = packetHandler_1->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
  
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_1->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_1->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    setTool(model_number, id);
    *get_model_number = model_number;
    result = setPacketHandler(1.0f, err);
    return result;
  }

  sdk_error.dxl_comm_result = packetHandler_2->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
  
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_2->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_2->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    setTool(model_number, id);
    *get_model_number = model_number;
    result = setPacketHandler(2.0f, err);
    return result;
  }  

  err = "[DynamixelDriver] Failed to ping!";
  return false;
}

bool DynamixelDriver::reboot(uint8_t id, const char* err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  if (getProtocolVersion() == 1.0)
  {
    err = "[DynamixelDriver] reboot functions is not available with the Dynamixel Protocol 1.0.";
    return false;
  }
  else
  {
    sdk_error.dxl_comm_result = packetHandler_2->reboot(portHandler_, id, &sdk_error.dxl_error);
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      err = packetHandler_2->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      err = packetHandler_2->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      err = "[DynamixelDriver] Succeeded to reboot!";
      return true;
    }
  }

  err = "[DynamixelDriver] Failed to reboot!";
  return false;
}

bool DynamixelDriver::reset(uint8_t id, const char* err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint32_t new_baud_rate = 0;
  uint8_t new_id = 1;

  const char* model_name = getModelName(id, err);
  if (model_name == NULL) return false;

  uint16_t model_number = getModelNumber(id, err);
  if (model_number == 0) return false;

  if (getProtocolVersion() == 1.0)
  {

    if (!strncmp(model_name, "AX", strlen("AX")) ||
        !strncmp(model_name, "MX-12W", strlen("MX-12W")))
      new_baud_rate = 1000000;
    else
      new_baud_rate = 57600;

    sdk_error.dxl_comm_result = packetHandler_1->factoryReset(portHandler_, id, 0x00, &sdk_error.dxl_error);
    wait(2000);

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      err = packetHandler_1->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      err = packetHandler_1->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      result = setBaudrate(new_baud_rate, err);
      if (result == false) 
        return false;
      else
      {
        if (!strncmp(model_name, "MX-28-2", strlen("MX-28-2"))   ||
            !strncmp(model_name, "MX-64-2", strlen("MX-64-2"))   ||
            !strncmp(model_name, "MX-106-2", strlen("MX-106-2")) ||
            !strncmp(model_name, "XL", strlen("XL")) ||
            !strncmp(model_name, "XM", strlen("XM")) ||
            !strncmp(model_name, "XH", strlen("XH")) ||
            !strncmp(model_name, "PRO", strlen("PRO")))
        {
          result = setPacketHandler(2.0f, err);
          if (result == false) return false;
        }          
        else
        {
          result = setPacketHandler(1.0f, err);
          if (result == false) return false; 
        }
      }
    }

    initTools();
    result = setTool(model_number, new_id, err);
    if (result == false) return false; 
    
    err = "[DynamixelDriver] Succeeded to reset!";
    return true;
  }
  else if (getProtocolVersion() == 2.0)
  {
    sdk_error.dxl_comm_result = packetHandler_2->factoryReset(portHandler_, id, 0x0f, &sdk_error.dxl_error);
    wait(2000);

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      err = packetHandler_2->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      err = packetHandler_2->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      if (!strncmp(model_name, "XL-320", strlen("XL-320"))) 
        new_baud_rate = 1000000;
      else 
        new_baud_rate = 57600;

      result = setBaudrate(new_baud_rate, err);
      if (result == false) 
        return false;
      else
      {
        result = setPacketHandler(2.0f, err);
        if (result == false)  return false;
      }
    }

    initTools();
    result = setTool(model_number, new_id, err);
    if (result == false) return false; 
    
    err = "[DynamixelDriver] Succeeded to reset!";
    return true;
  }

  err = "[DynamixelDriver] Failed to reset!";
  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, uint16_t address, uint8_t length, uint8_t* data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        address, 
                                                        length, 
                                                        data, 
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    return true;
  }

  err = "[DynamixelDriver] Failed to write Register!";
  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint8_t data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  uint8_t data_write[1] = { data };
  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        control_item->address, 
                                                        control_item->data_length, 
                                                        data_write, 
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    return true;
  }

  err = "[DynamixelDriver] Failed to write Register!";
  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint16_t data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        control_item->address, 
                                                        control_item->data_length, 
                                                        data_write, 
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    return true;
  }

  err = "[DynamixelDriver] Failed to write Register!";
  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint32_t data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxRx(portHandler_, 
                                                        id, 
                                                        control_item->address, 
                                                        control_item->data_length, 
                                                        data_write, 
                                                        &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    return true;
  }

  err = "[DynamixelDriver] Failed to write Register!";
  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          address, 
                                                          length, 
                                                          data);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    return true;
  }

  err = "[DynamixelDriver] Failed to write Only Register!";
  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint8_t data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    return true;
  }

  err = "[DynamixelDriver] Failed to write Only Register!";
  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint16_t data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    return true;
  }

  err = "[DynamixelDriver] Failed to write Only Register!";
  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint32_t data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    return true;
  }

  err = "[DynamixelDriver] Failed to write Only Register!";
  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[1] = {0};

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       address,
                                                       length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = data_read[0];
    return true;
  }

  err = "[DynamixelDriver] Failed to read Register!";
  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint8_t *data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[1] = {0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = data_read[0];
    return true;
  }

  err = "[DynamixelDriver] Failed to read Register!";
  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint16_t *data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[2] = {0, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = DXL_MAKEWORD(data_read[0], data_read[1]);
    return true;
  }

  err = "[DynamixelDriver] Failed to read Register!";
  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint32_t *data, const char *err)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[4] = {0, 0, 0, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, err);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, err);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    err = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    err = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
    return true;
  }

  err = "[DynamixelDriver] Failed to read Register!";
  return false;
}

// const char *DynamixelDriver::findModelName(uint16_t model_num)
// {
//   uint16_t num = model_num;
//   static const char* model_name = NULL;

//   if (num == AX_12A)
//     model_name = "AX-12A";
//   else if (num == AX_12W)
//     model_name = "AX-12W";
//   else if (num == AX_18A)
//     model_name = "AX-18A";

//   else if (num == RX_24F)
//     model_name = "RX-24F";
//   else if (num == RX_28)
//     model_name = "RX-28";
//   else if (num == RX_64)
//     model_name = "RX-64";

//   else if (num == EX_106)
//     model_name = "EX-106";

//   else if (num == MX_12W)
//     model_name = "MX-12W";
//   else if (num == MX_28)
//     model_name = "MX-28";
//   else if (num == MX_28_2)
//     model_name = "MX-28-2";
//   else if (num == MX_64)
//     model_name = "MX-64";
//   else if (num == MX_64_2)
//     model_name = "MX-64-2";
//   else if (num == MX_106)
//     model_name = "MX-106";
//   else if (num == MX_106_2)
//     model_name = "MX-106-2";

//   else if (num == XL_320)
//     model_name = "XL-320";
//   else if (num == XL430_W250)
//     model_name = "XL430-W250";

//   else if (num == XM430_W210)
//     model_name = "XM430-W210";
//   else if (num == XM430_W350)
//     model_name = "XM430-W350";
//   else if (num == XM540_W150)
//     model_name = "XM540-W150";
//   else if (num == XM540_W270)
//     model_name = "XM540-W270";

//   else if (num == XH430_V210)
//     model_name = "XH430-V210";
//   else if (num == XH430_V350)
//     model_name = "XH430-V350";
//   else if (num == XH430_W210)
//     model_name = "XH430-W210";
//   else if (num == XH430_W350)
//     model_name = "XH430-W350";

//   else if (num == PRO_L42_10_S300_R)
//     model_name = "PRO-L42-10-S300-R";
//   else if (num == PRO_L54_30_S400_R)
//     model_name = "PRO-L54-30-S400-R";
//   else if (num == PRO_L54_30_S500_R)
//     model_name = "PRO-L54-30-S500-R";
//   else if (num == PRO_L54_50_S290_R)
//     model_name = "PRO-L54-50-S290-R";
//   else if (num == PRO_L54_50_S500_R)
//     model_name = "PRO-L54-50-S500-R";

//   else if (num == PRO_M42_10_S260_R)
//     model_name = "PRO-M42-10-S260-R";
//   else if (num == PRO_M54_40_S250_R)
//     model_name = "PRO-M54-40-S250-R";
//   else if (num == PRO_M54_60_S250_R)
//     model_name = "PRO-M54-60-S250-R";

//   else if (num == PRO_H42_20_S300_R)
//     model_name = "PRO-H42-20-S300-R";
//   else if (num == PRO_H54_100_S500_R)
//     model_name = "PRO-H54-100-S500-R";
//   else if (num == PRO_H54_200_S500_R)
//     model_name = "PRO-H54-200-S500-R";

//   return model_name;
// }

// void DynamixelDriver::addSyncWrite(const char *item_name)
// {
//   const ControlItem *control_item;
//   control_item = tools_[0].getControlItem(item_name);

//   if (control_item == NULL)
//   {
//     return;
//   }

//   syncWriteHandler_[sync_write_handler_cnt_].control_item = control_item;

//   syncWriteHandler_[sync_write_handler_cnt_++].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
//                                                                                               packetHandler_[0],
//                                                                                               control_item->address,
//                                                                                               control_item->data_length);
// }

// bool DynamixelDriver::syncWrite(const char *item_name, int32_t *data)
// {
//   bool dxl_addparam_result = false;
//   int dxl_comm_result = COMM_TX_FAIL;

//   uint8_t data_byte[4] = {0, };
//   uint8_t cnt = 0;

//   SyncWriteHandler swh;
//   bool swh_found = false;

//   for (int index = 0; index < sync_write_handler_cnt_; index++)
//   {
//     if (!strncmp(syncWriteHandler_[index].control_item->item_name, item_name, strlen(item_name)))
//     {
//       swh = syncWriteHandler_[index];
//       swh_found = true;
//       break;
//     }
//   }

//   if (!swh_found)
//   {
//     return false;
//   }

//   for (int i = 0; i < tools_cnt_; i++)
//   {
//     for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
//     {
//       data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data[cnt]));
//       data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data[cnt]));
//       data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data[cnt]));
//       data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data[cnt]));

//       dxl_addparam_result = swh.groupSyncWrite->addParam(tools_[i].getID()[j], (uint8_t *)&data_byte);
//       if (dxl_addparam_result != true)
//       {
//         return false;
//       }

//       cnt++;
//     }
//   }

//   dxl_comm_result = swh.groupSyncWrite->txPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }
//   swh.groupSyncWrite->clearParam();
//   return true;
// }

// bool DynamixelDriver::syncWrite(uint8_t *id, uint8_t id_num, const char *item_name, int32_t *data)
// {
//   bool dxl_addparam_result = false;
//   int dxl_comm_result = COMM_TX_FAIL;

//   uint8_t data_byte[4] = {0, };
//   uint8_t cnt = 0;

//   SyncWriteHandler swh;
//   bool swh_found = false;

//   for (int index = 0; index < sync_write_handler_cnt_; index++)
//   {
//     if (!strncmp(syncWriteHandler_[index].control_item->item_name, item_name, strlen(item_name)))
//     {
//       swh = syncWriteHandler_[index];
//       swh_found = true;
//       break;
//     }
//   }

//   if (!swh_found)
//   {
//     return false;
//   }
  
//   for (int i = 0; i < id_num; i++)
//   {
//     data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data[cnt]));
//     data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data[cnt]));
//     data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data[cnt]));
//     data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data[cnt]));

//     dxl_addparam_result = swh.groupSyncWrite->addParam(id[i], (uint8_t *)&data_byte);
//     if (dxl_addparam_result != true)
//     {
//       return false;
//     }
//     cnt++;
//   }

//   dxl_comm_result = swh.groupSyncWrite->txPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }
//   swh.groupSyncWrite->clearParam();
//   return true;
// }

// void DynamixelDriver::addSyncRead(const char *item_name)
// {
//   const ControlItem *control_item;
//   control_item = tools_[0].getControlItem(item_name);

//   if (control_item == NULL)
//   {
//     return;
//   }
//   syncReadHandler_[sync_read_handler_cnt_].control_item = control_item;
  
//   syncReadHandler_[sync_read_handler_cnt_++].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
//                                                                                           packetHandler_[0],
//                                                                                           control_item->address,
//                                                                                           control_item->data_length);
// }

// bool DynamixelDriver::syncRead(const char *item_name, int32_t *data)
// {
//   int dxl_comm_result = COMM_RX_FAIL;
//   bool dxl_addparam_result = false;
//   bool dxl_getdata_result = false;

//   int index = 0;

//   SyncReadHandler srh;
//   bool srh_found = false;
  
//   for (int index = 0; index < sync_read_handler_cnt_; index++)
//   {
//     if (!strncmp(syncReadHandler_[index].control_item->item_name, item_name, strlen(item_name)))
//     {
//       srh = syncReadHandler_[index];
//       srh_found = true;
//       break; // Found it, don't need to continue search
//     }
//   }
//   if (!srh_found)
//   {
//     return false; // did not find item_name in list
//   }
//   for (int i = 0; i < tools_cnt_; i++)
//   {
//     for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
//     {
//       dxl_addparam_result = srh.groupSyncRead->addParam(tools_[i].getID()[j]);
//       if (dxl_addparam_result != true)
//         return false;
//     }
//   }

//   dxl_comm_result = srh.groupSyncRead->txRxPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }

//   for (int i = 0; i < tools_cnt_; i++)
//   {
//     for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
//     {
//       uint8_t id = tools_[i].getID()[j];

//       dxl_getdata_result = srh.groupSyncRead->isAvailable(id, srh.control_item->address, srh.control_item->data_length);
//       if (dxl_getdata_result)
//       {
//         data[index++] = srh.groupSyncRead->getData(id, srh.control_item->address, srh.control_item->data_length);
//       }
//       else
//       {
//         return false;
//       }
//     }
//   }

//   srh.groupSyncRead->clearParam();

//   return true;
// }

// void DynamixelDriver::initBulkWrite()
// {
//   groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, packetHandler_[0]);
// }

// bool DynamixelDriver::addBulkWriteParam(uint8_t id, const char *item_name, int32_t data)
// {
//   bool dxl_addparam_result = false;
//   uint8_t data_byte[4] = {0, };

//   const ControlItem *control_item;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) return false;
//   control_item = tools_[factor].getControlItem(item_name);
//   if (control_item == NULL)
//   {
//     return false;
//   }

//   data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data));
//   data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data));
//   data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data));
//   data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data));

//   dxl_addparam_result = groupBulkWrite_->addParam(id, control_item->address, control_item->data_length, data_byte);
//   if (dxl_addparam_result != true)
//   {
//     return false;
//   }

//   return true;
// }

// bool DynamixelDriver::bulkWrite()
// {
//   int dxl_comm_result = COMM_TX_FAIL;

//   dxl_comm_result = groupBulkWrite_->txPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }

//   groupBulkWrite_->clearParam();

//   return true;
// }

// void DynamixelDriver::initBulkRead()
// {
//   groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_[0]);
// }

// bool DynamixelDriver::addBulkReadParam(uint8_t id, const char *item_name)
// {
//   bool dxl_addparam_result = false;

//   const ControlItem *control_item;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) return false;
//   control_item = tools_[factor].getControlItem(item_name);
//   if (control_item == NULL)
//   {
//     return false;
//   }

//   dxl_addparam_result = groupBulkRead_->addParam(id, control_item->address, control_item->data_length);
//   if (dxl_addparam_result != true)
//   {
//     return false;
//   }

//   return true;
// }

// bool DynamixelDriver::sendBulkReadPacket()
// {
//   int dxl_comm_result = COMM_RX_FAIL;

//   dxl_comm_result = groupBulkRead_->txRxPacket();
//   if (dxl_comm_result != COMM_SUCCESS)
//   {
//     return false;
//   }

//   return true;
// }

// bool DynamixelDriver::bulkRead(uint8_t id, const char *item_name, int32_t *data)
// {
//   bool dxl_getdata_result = false;
//   const ControlItem *control_item;
//   uint8_t factor = getTool(id);
//   if (factor == 0xff) 
//   {
//     return false;
//   }
//   control_item = tools_[factor].getControlItem(item_name);
//   if (control_item == NULL)
//   {
//     return false;
//   }

//   dxl_getdata_result = groupBulkRead_->isAvailable(id, control_item->address, control_item->data_length);
//   if (dxl_getdata_result != true)
//   {
//     return false;
//   }

//   *data = groupBulkRead_->getData(id, control_item->address, control_item->data_length);

//   return true;
// }

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

void DynamixelDriver::wait(uint16_t msec)
{
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(msec);
#else
    usleep(1000*msec);
#endif
}
