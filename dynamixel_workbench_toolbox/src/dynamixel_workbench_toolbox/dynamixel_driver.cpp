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

static dynamixel::PacketHandler *packetHandler_1_0 = dynamixel::PacketHandler::getPacketHandler(1.0f);
static dynamixel::PacketHandler *packetHandler_2_0 = dynamixel::PacketHandler::getPacketHandler(2.0f);

DynamixelDriver::DynamixelDriver() : tools_cnt_(0), 
                                    sync_write_handler_cnt_(0), 
                                    sync_read_handler_cnt_(0),
                                    bulk_parameter_cnt_(0)
{

}

DynamixelDriver::~DynamixelDriver()
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      writeRegister(tools_[i].getID()[j], "Torque_Enable", (uint8_t)0);
    }
  }

  for (int i = 0; i < sync_write_handler_cnt_; i++)
  {
    delete[] syncWriteHandler_[i].groupSyncWrite;
  }  

  for (int i = 0; i < sync_read_handler_cnt_; i++)
  {
    delete[] syncReadHandler_[i].groupSyncRead;
  }  

  portHandler_->closePort();

  delete portHandler_;
  delete packetHandler_;
  delete packetHandler_1_0;
  delete packetHandler_2_0;
}

void DynamixelDriver::initTools(void)
{
  tools_cnt_ = 0;

  for (uint8_t num = 0; num < tools_cnt_; num++)
    tools_[num].initTool();
}

bool DynamixelDriver::setTool(uint16_t model_number, uint8_t id, const char **log)
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
        *log = "[DynamixelDriver] Too many Dynamixels are connected (default buffer size is 16, the same series of Dynamixels)";
        return false;
      }
    }
  }
  // We did not find one so lets allocate a new one
  if (tools_cnt_ < MAX_DXL_SERIES_NUM) 
  {
    // only do it if we still have some room...
    result = tools_[tools_cnt_++].addTool(model_number, id, log);
    return result;
  }
  else
  {
    *log = "[DynamixelDriver] Too many series are connected (MAX = 5 different series)";
    return false;
  }

  *log = "[DynamixelDriver] Failed to set the Tool";
  return false;
}

uint8_t DynamixelDriver::getTool(uint8_t id, const char **log)
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

  *log = "[DynamixelDriver] Failed to get the Tool";
  return 0xff;
}

bool DynamixelDriver::init(const char *device_name, uint32_t baud_rate, const char **log)
{
  bool result = false;

  result = setPortHandler(device_name, log);
  if (result == false) return false;

  result = setBaudrate(baud_rate, log);
  if (result == false) return false;

  return result;
}

bool DynamixelDriver::begin(const char *device_name, uint32_t baud_rate, const char **log)
{
  return init(device_name, baud_rate, log);
}

bool DynamixelDriver::setPortHandler(const char *device_name, const char **log)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);

  if (portHandler_->openPort())
  {
    *log = "[DynamixelDriver] Succeeded to open the port!";
    return true;
  }

  *log = "[DynamixelDriver] Failed to open the port!";
  return false;
}

bool DynamixelDriver::setBaudrate(uint32_t baud_rate, const char **log)
{
  if (portHandler_->setBaudRate(baud_rate))
  {
    *log = "[DynamixelDriver] Succeeded to change the baudrate!";
    return true;
  }

  *log = "[DynamixelDriver] Failed to change the baudrate!";
  return false;
}

bool DynamixelDriver::setPacketHandler(float protocol_version, const char **log)
{
  delete [] packetHandler_;

  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  if (packetHandler_->getProtocolVersion() == protocol_version)
  {
    *log = "[DynamixelDriver] Succeeded to set the protocol!";
    return true;
  }

  *log = "[DynamixelDriver] Failed to set the protocol!";
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

const char* DynamixelDriver::getModelName(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) 
    return NULL;
  else
    return tools_[factor].getModelName();

  return NULL;
}

uint16_t DynamixelDriver::getModelNumber(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false;

  for (int i = 0; i < tools_[factor].getDynamixelCount(); i++)
  {
    if (tools_[factor].getID()[i] == id)
      return tools_[factor].getModelNumber();
  }

  return false;
}

const ControlItem* DynamixelDriver::getControlTable(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return NULL;

  return tools_[factor].getControlTable();
}

uint8_t DynamixelDriver::getTheNumberOfControlItem(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false;

  return tools_[factor].getTheNumberOfControlItem();
}

const ModelInfo* DynamixelDriver::getModelInfo(uint8_t id, const char **log)
{
  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return NULL;

  return tools_[factor].getModelInfo();
}

bool DynamixelDriver::scan(uint8_t *get_id, uint8_t *get_the_number_of_id, uint8_t start_num, uint8_t end_num, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint8_t id = 0;
  uint8_t id_cnt = 0;

  uint16_t model_number = 0;

  uint8_t get_end_num = end_num;

  if (end_num > 253) end_num = 253;

  initTools();

  for (id = start_num; id <= end_num; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_1_0->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_1_0->getTxRxResult(sdk_error.dxl_comm_result);
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_1_0->getRxPacketError(sdk_error.dxl_error);
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
    result = setPacketHandler(1.0f, log);
    return result;
  }

  for (id = start_num; id <= end_num; id++)
  {
    sdk_error.dxl_comm_result = packetHandler_2_0->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
    
    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_2_0->getTxRxResult(sdk_error.dxl_comm_result);
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_2_0->getRxPacketError(sdk_error.dxl_error);
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
    result = setPacketHandler(2.0f, log);
    return result;
  }

  return result;
}

bool DynamixelDriver::scan(uint8_t *get_id, uint8_t *get_the_number_of_id, uint8_t range, const char **log)
{
  return scan(get_id, get_the_number_of_id, 0, range, log);
}

bool DynamixelDriver::ping(uint8_t id, uint16_t *get_model_number, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint16_t model_number = 0;

  sdk_error.dxl_comm_result = packetHandler_1_0->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
  
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_1_0->getTxRxResult(sdk_error.dxl_comm_result);
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_1_0->getRxPacketError(sdk_error.dxl_error);
  }
  else
  {
    setTool(model_number, id);
    *get_model_number = model_number;
    result = setPacketHandler(1.0f, log);
    return result;
  }

  sdk_error.dxl_comm_result = packetHandler_2_0->ping(portHandler_, id, &model_number, &sdk_error.dxl_error);
  
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_2_0->getTxRxResult(sdk_error.dxl_comm_result);
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_2_0->getRxPacketError(sdk_error.dxl_error);
  }
  else
  {
    setTool(model_number, id);
    *get_model_number = model_number;
    result = setPacketHandler(2.0f, log);
    return result;
  }  

  return result;
}

bool DynamixelDriver::reboot(uint8_t id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  if (getProtocolVersion() == 1.0)
  {
    *log = "[DynamixelDriver] reboot functions is not available with the Dynamixel Protocol 1.0.";
    return false;
  }
  else
  {
    sdk_error.dxl_comm_result = packetHandler_2_0->reboot(portHandler_, id, &sdk_error.dxl_error);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(1000);
#else
    usleep(1000*1000);
#endif

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_2_0->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_2_0->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      *log = "[DynamixelDriver] Succeeded to reboot!";
      return true;
    }
  }

  return false;
}

bool DynamixelDriver::reset(uint8_t id, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  bool result = false;

  uint8_t new_baud_rate = 0;
  uint8_t new_id = 1;

  const char* model_name = getModelName(id, log);
  if (model_name == NULL) return false;

  uint16_t model_number = getModelNumber(id, log);
  if (model_number == 0) return false;

  if (getProtocolVersion() == 1.0)
  {
    sdk_error.dxl_comm_result = packetHandler_1_0->factoryReset(portHandler_, id, 0x00, &sdk_error.dxl_error);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(2000);
#else
    usleep(1000*2000);
#endif

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_1_0->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_1_0->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      if (!strncmp(model_name, "AX", strlen("AX")) ||
          !strncmp(model_name, "MX-12W", strlen("MX-12W")))
        new_baud_rate = 1; //1000000
      else
        new_baud_rate = 34; //57600

      result = setBaudrate(new_baud_rate, log);
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
          result = setPacketHandler(2.0f, log);
          if (result == false) return false;
        }          
        else
        {
          result = setPacketHandler(1.0f, log);
          if (result == false) return false; 
        }
      }
    }

    initTools();
    result = setTool(model_number, new_id, log);
    if (result == false) return false; 
    
    *log = "[DynamixelDriver] Succeeded to reset!";
    return true;
  }
  else if (getProtocolVersion() == 2.0)
  {
    sdk_error.dxl_comm_result = packetHandler_2_0->factoryReset(portHandler_, id, 0xff, &sdk_error.dxl_error);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    delay(2000);
#else
    usleep(1000*2000);
#endif

    if (sdk_error.dxl_comm_result != COMM_SUCCESS)
    {
      *log = packetHandler_2_0->getTxRxResult(sdk_error.dxl_comm_result);
      return false;
    }
    else if (sdk_error.dxl_error != 0)
    {
      *log = packetHandler_2_0->getRxPacketError(sdk_error.dxl_error);
      return false;
    }
    else
    {
      if (!strncmp(model_name, "XL-320", strlen("XL-320"))) 
        new_baud_rate = 3; // 1000000
      else 
        new_baud_rate = 1; // 57600

      result = setBaudrate(new_baud_rate, log);
      if (result == false) 
        return false;
      else
      {
        result = setPacketHandler(2.0f, log);
        if (result == false)  return false;
      }
    }

    initTools();
    result = setTool(model_number, new_id, log);
    if (result == false) return false; 
    
    *log = "[DynamixelDriver] Succeeded to reset!";
    return true;
  }

  return result;
}

bool DynamixelDriver::writeRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t* data, const char **log)
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
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint8_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
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
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint16_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
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
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, uint32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
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
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, uint16_t address, uint16_t length, uint8_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          address, 
                                                          length, 
                                                          data);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint8_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[1] = { data };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint16_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[2] = { DXL_LOBYTE(data), DXL_HIBYTE(data) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::writeOnlyRegister(uint8_t id, const char *item_name, uint32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  uint8_t data_write[4] = { DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data)) };
  sdk_error.dxl_comm_result = packetHandler_->writeTxOnly(portHandler_, 
                                                          id, 
                                                          control_item->address, 
                                                          control_item->data_length, 
                                                          data_write);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else
  {
    *log = "[DynamixelDriver] Succeeded to write!";
    return true;
  }

  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, uint16_t address, uint16_t length, uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  
  uint8_t data_read[length];

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                      id, 
                                                      address,
                                                      length, 
                                                      (uint8_t *)&data_read, 
                                                      &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    switch (length)
    {
      case BYTE:
        *data = data_read[0];
       break;

      case WORD:
        *data = DXL_MAKEWORD(data_read[0], data_read[1]);
       break;

      case DWORD:
        *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
       break;

      default:
        for (uint16_t index = 0; index < length; index++)
        {
          data[index] = (uint32_t)data_read[index];
        }
       break;
    }
    *log = "[DynamixelDriver] Succeeded to read!";
    return true;
  }

  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint8_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[1] = {0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = data_read[0];
    *log = "[DynamixelDriver] Succeeded to read!";
    return true;
  }

  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint16_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[2] = {0, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = DXL_MAKEWORD(data_read[0], data_read[1]);
    *log = "[DynamixelDriver] Succeeded to read!";
    return true;
  }

  return false;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t data_read[4] = {0, 0, 0, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  sdk_error.dxl_comm_result = packetHandler_->readTxRx(portHandler_, 
                                                       id, 
                                                       control_item->address,
                                                       control_item->data_length, 
                                                       (uint8_t *)&data_read, 
                                                       &sdk_error.dxl_error);
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }
  else if (sdk_error.dxl_error != 0)
  {
    *log = packetHandler_->getRxPacketError(sdk_error.dxl_error);
    return false;
  }
  else
  {
    *data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]));
    *log = "[DynamixelDriver] Succeeded to read!";
    return true;
  }

  return false;
}

void DynamixelDriver::getParam(uint16_t data, uint8_t *param)
{
  param[0] = DXL_LOWORD(data);
  param[1] = DXL_LOWORD(data);
}

void DynamixelDriver::getParam(uint32_t data, uint8_t *param)
{
  param[0] = DXL_LOBYTE(DXL_LOWORD(data));
  param[1] = DXL_HIBYTE(DXL_LOWORD(data));
  param[2] = DXL_LOBYTE(DXL_HIWORD(data));
  param[3] = DXL_HIBYTE(DXL_HIWORD(data));
}

bool DynamixelDriver::addSyncWriteHandler(uint8_t id, uint16_t address, uint16_t length, const char **log)
{
  if (sync_write_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    *log = "[DynamixelDriver] Too many sync write handler are added (MAX = 5)";
    return false;
  }

  syncWriteHandler_[sync_write_handler_cnt_].control_item = NULL;

  syncWriteHandler_[sync_write_handler_cnt_].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
                                                                                            packetHandler_,
                                                                                            address,
                                                                                            length);

  sync_write_handler_cnt_++;

  *log = "[DynamixelDriver] Succeeded to add sync write handler";
  return true;    
}

bool DynamixelDriver::addSyncWriteHandler(uint8_t id, const char *item_name, const char **log)
{
  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  if (sync_write_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    *log = "[DynamixelDriver] Too many sync write handler are added (MAX = 5)";
    return false;
  }

  syncWriteHandler_[sync_write_handler_cnt_].control_item = control_item;

  syncWriteHandler_[sync_write_handler_cnt_].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
                                                                                            packetHandler_,
                                                                                            control_item->address,
                                                                                            control_item->data_length);

  sync_write_handler_cnt_++;

  *log = "[DynamixelDriver] Succeeded to add sync write handler";
  return true;                                                            
}

bool DynamixelDriver::syncWrite(uint8_t index, uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t dxl_cnt = 0;
  uint8_t parameter[4] = {0, 0, 0, 0};

  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      getParam(data[dxl_cnt], parameter);
      sdk_error.dxl_addparam_result = syncWriteHandler_[index].groupSyncWrite->addParam(tools_[i].getID()[j], (uint8_t *)&parameter);
      if (sdk_error.dxl_addparam_result != true)
      {
        *log = "groupSyncWrite addparam failed";
        return false;
      }
      else
        dxl_cnt++;
    }
  }

  sdk_error.dxl_comm_result = syncWriteHandler_[index].groupSyncWrite->txPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  syncWriteHandler_[index].groupSyncWrite->clearParam();

  *log = "[DynamixelDriver] Succeeded to sync write!";
  return true;
}

bool DynamixelDriver::syncWrite(uint8_t index, uint8_t *id, uint8_t id_num, uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t parameter[4] = {0, 0, 0, 0};

  for (int i = 0; i < id_num; i++)
  {
    getParam(data[i], parameter);
    sdk_error.dxl_addparam_result = syncWriteHandler_[index].groupSyncWrite->addParam(id[i], (uint8_t *)&parameter);
    if (sdk_error.dxl_addparam_result != true)
    {
      *log = "groupSyncWrite addparam failed";
      return false;
    }
  }

  sdk_error.dxl_comm_result = syncWriteHandler_[index].groupSyncWrite->txPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  syncWriteHandler_[index].groupSyncWrite->clearParam();

  *log = "[DynamixelDriver] Succeeded to sync write!";
  return true;
}

bool DynamixelDriver::addSyncReadHandler(uint8_t id, uint16_t address, uint16_t length, const char **log)
{
  if (sync_read_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    *log = "[DynamixelDriver] Too many sync read handler are added (MAX = 5)";
    return false;
  }

  syncReadHandler_[sync_read_handler_cnt_].control_item = NULL;

  syncReadHandler_[sync_read_handler_cnt_++].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
                                                                                          packetHandler_,
                                                                                          address,
                                                                                          length);

  sync_read_handler_cnt_++;

  *log = "[DynamixelDriver] Succeeded to add sync read handler";
  return true;
}

bool DynamixelDriver::addSyncReadHandler(uint8_t id, const char *item_name, const char **log)
{
  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  if (sync_read_handler_cnt_ > (MAX_HANDLER_NUM-1))
  {
    *log = "[DynamixelDriver] Too many sync read handler are added (MAX = 5)";
    return false;
  }

  syncReadHandler_[sync_read_handler_cnt_].control_item = control_item;

  syncReadHandler_[sync_read_handler_cnt_++].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
                                                                                          packetHandler_,
                                                                                          control_item->address,
                                                                                          control_item->data_length);

  sync_read_handler_cnt_++;

  *log = "[DynamixelDriver] Succeeded to add sync read handler";
  return true;       
}

bool DynamixelDriver::syncRead(uint8_t index, uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      sdk_error.dxl_addparam_result = syncReadHandler_[index].groupSyncRead->addParam(tools_[i].getID()[j]);
      if (sdk_error.dxl_addparam_result != true)
      {
        *log = "groupSyncWrite addparam failed";
        return false;
      }
    }
  }

  sdk_error.dxl_comm_result = syncReadHandler_[index].groupSyncRead->txRxPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].getDynamixelCount(); j++)
    {
      sdk_error.dxl_getdata_result = syncReadHandler_[index].groupSyncRead->isAvailable(tools_[i].getID()[j], 
                                                                                        syncReadHandler_[index].control_item->address, 
                                                                                        syncReadHandler_[index].control_item->data_length);
      if (sdk_error.dxl_getdata_result != true)
      {
        *log = "groupSyncRead getdata failed";
        return false;
      }
      else
      {
        data[i+j] = syncReadHandler_[index].groupSyncRead->getData(tools_[i].getID()[j], 
                                                                    syncReadHandler_[index].control_item->address, 
                                                                    syncReadHandler_[index].control_item->data_length);
      }
    }
  }

  syncReadHandler_[index].groupSyncRead->clearParam();

  *log = "[DynamixelDriver] Succeeded to sync read!";
  return true;
}

bool DynamixelDriver::syncRead(uint8_t index, uint8_t *id, uint8_t id_num, uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  for (int i = 0; i < id_num; i++)
  {
    sdk_error.dxl_addparam_result = syncReadHandler_[index].groupSyncRead->addParam(id[i]);
    if (sdk_error.dxl_addparam_result != true)
    {
      *log = "groupSyncWrite addparam failed";
      return false;
    }
  }

  sdk_error.dxl_comm_result = syncReadHandler_[index].groupSyncRead->txRxPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  for (int i = 0; i < id_num; i++)
  {
    sdk_error.dxl_getdata_result = syncReadHandler_[index].groupSyncRead->isAvailable(id[i], 
                                                                                      syncReadHandler_[index].control_item->address, 
                                                                                      syncReadHandler_[index].control_item->data_length);
    if (sdk_error.dxl_getdata_result != true)
    {
      *log = "groupSyncRead getdata failed";
      return false;
    }
    else
    {
      data[i] = syncReadHandler_[index].groupSyncRead->getData(id[i], 
                                                                syncReadHandler_[index].control_item->address, 
                                                                syncReadHandler_[index].control_item->data_length);
    }
  }

  syncReadHandler_[index].groupSyncRead->clearParam();

  *log = "[DynamixelDriver] Succeeded to sync read!";
  return true;
}

bool DynamixelDriver::initBulkWrite(const char **log)
{
  if (portHandler_ == NULL)
    *log = "[DynamixelDriver] Failed to load portHandler!";
  else if (packetHandler_ == NULL)
    *log = "[DynamixelDriver] Failed to load packetHandler!";
  else
  {
    delete groupBulkWrite_;
    groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, packetHandler_);

    *log = "[DynamixelDriver] Succeeded to init groupBulkWrite!";
    return true;
  }

  return false;
}

bool DynamixelDriver::addBulkWriteParam(uint8_t id, const char *item_name, uint32_t data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  uint8_t parameter[4] = {0, 0, 0, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  getParam(data, parameter);
  sdk_error.dxl_addparam_result = groupBulkWrite_->addParam(id, 
                                                            control_item->address, 
                                                            control_item->data_length, 
                                                            parameter);
  if (sdk_error.dxl_addparam_result != true)
  {
    *log = "groupBulkWrite addparam failed";
    return false;
  }

  *log = "[DynamixelDriver] Succeeded to add param for bulk write!";
  return true;
}

bool DynamixelDriver::bulkWrite(const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  sdk_error.dxl_comm_result = groupBulkWrite_->txPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS)
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  groupBulkWrite_->clearParam();

  *log = "[DynamixelDriver] Succeeded to bulk write!";
  return true;
}

bool DynamixelDriver::initBulkRead(const char **log)
{
  if (portHandler_ == NULL)
    *log = "[DynamixelDriver] Failed to load portHandler!";
  else if (packetHandler_ == NULL)
    *log = "[DynamixelDriver] Failed to load packetHandler!";
  else
  {
    delete groupBulkRead_;
    groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);

    *log = "[DynamixelDriver] Succeeded to init groupBulkRead!";
    return true;
  }

  return false;
}

bool DynamixelDriver::addBulkReadParam(uint8_t id, const char *item_name, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};

  const ControlItem *control_item;

  uint8_t factor = getTool(id, log);
  if (factor == 0xff) return false; 

  control_item = tools_[factor].getControlItem(item_name, log);
  if (control_item == NULL) return false;

  sdk_error.dxl_addparam_result = groupBulkRead_->addParam(id, 
                                                          control_item->address, 
                                                          control_item->data_length);
  if (sdk_error.dxl_addparam_result != true)
  {
    *log = "grouBulkRead addparam failed";
    return false;
  }

  if (bulk_parameter_cnt_ < (MAX_BULK_PARAMETER-1))
  {
    bulk_param_[bulk_parameter_cnt_].id = id;
    bulk_param_[bulk_parameter_cnt_].address = control_item->address;
    bulk_param_[bulk_parameter_cnt_].data_length = control_item->data_length;
    bulk_parameter_cnt_++;
  }
  else
  {
    *log = "[DynamixelDriver] Too many bulk parameter are added (default buffer size is 16)";
    return false;
  }

  *log = "[DynamixelDriver] Succeeded to add param for bulk read!";
  return true;

  // bool dxl_addparam_result = false;

  // const ControlItem *control_item;
  // uint8_t factor = getTool(id);
  // if (factor == 0xff) return false;
  // control_item = tools_[factor].getControlItem(item_name);
  // if (control_item == NULL)
  // {
  //   return false;
  // }

  // dxl_addparam_result = groupBulkRead_->addParam(id, control_item->address, control_item->data_length);
  // if (dxl_addparam_result != true)
  // {
  //   return false;
  // }

  // return true;
}

bool DynamixelDriver::bulkRead(uint32_t *data, const char **log)
{
  ErrorFromSDK sdk_error = {0, false, false, 0};
  
  sdk_error.dxl_comm_result = groupBulkRead_->txRxPacket();
  if (sdk_error.dxl_comm_result != COMM_SUCCESS) 
  {
    *log = packetHandler_->getTxRxResult(sdk_error.dxl_comm_result);
    return false;
  }

  const ControlItem *control_item;

  for (int i = 0; i < bulk_parameter_cnt_; i++)
  {

    sdk_error.dxl_getdata_result = groupBulkRead_->isAvailable(bulk_param_[i].id, 
                                                              bulk_param_[i].address, 
                                                              bulk_param_[i].data_length);
    if (sdk_error.dxl_getdata_result != true)
    {
      *log = "groupBulkRead getdata failed";
      return false;
    }
    else
    {
      data[i] = groupBulkRead_->getData(bulk_param_[i].id, 
                                        bulk_param_[i].address, 
                                        bulk_param_[i].data_length);
    }
  }

  groupBulkRead_->clearParam();
  bulk_parameter_cnt_ = 0;

  for (uint8_t i = 0; i < MAX_BULK_PARAMETER; i++)
  {
    bulk_param_[i].id = 0;
    bulk_param_[i].address = 0;
    bulk_param_[i].data_length = 0;
  }

  *log = "[DynamixelDriver] Succeeded to bulk read!";
  return true;
}