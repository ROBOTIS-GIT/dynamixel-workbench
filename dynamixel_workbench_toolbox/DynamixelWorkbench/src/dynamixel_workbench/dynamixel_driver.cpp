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

#include "../../include/dynamixel_workbench/dynamixel_driver.h"

DynamixelDriver::DynamixelDriver()
{
  tools_cnt_ = 0;  
}

DynamixelDriver::~DynamixelDriver()
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    writeRegister(tools_[i].getID(), "Torque Enable", 0);
  }
  portHandler_->closePort();
}

void DynamixelDriver::setTools(uint16_t model_num, uint8_t id)
{
  uint8_t cnt = tools_cnt_;

  tools_[cnt].begin(model_num);
  tools_[cnt].setID(id);

  tools_cnt_++;
}

uint8_t DynamixelDriver::theNumberOfTools()
{
  return tools_cnt_;
}

bool DynamixelDriver::begin(char* model_series, char* device_name, uint32_t baud_rate)
{
  bool error = false;
  char* name = model_series;

  setPortHandler(device_name, &error);
  setBaudrate(baud_rate, &error);

  if (!strncmp(name, "AX", 2) || !strncmp(name, "RX", 2) || !strncmp(name, "MX", 2) || !strncmp(name, "EX", 2))
  {
    setPacketHandler(1.0, &error);
  }
  else
  {
    setPacketHandler(2.0, &error);
  }

  return error;
}

void DynamixelDriver::setPortHandler(char* device_name, bool *error)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);

  if (portHandler_->openPort())
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Succeeded to open the port!");
#else
    printf("\nSucceeded to open the port(%s)!\n", device_name.c_str());
#endif

    *error = false;
  }
  else
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Failed to open the port!");
#else
    printf("Failed to open the port!\n");
#endif

    *error = true;
  }
}

void DynamixelDriver::setPacketHandler(float protocol_version, bool *error)
{
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  if (packetHandler_->getProtocolVersion() == 0)
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Failed to setPacketHandler!");
#else
    printf("Failed to setPacketHandler!\n");
#endif

    *error =  true;
  }
  else
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Succeeded to setPacketHandler!");
#else
    printf("Succeeded to setPacketHandler!\n");
#endif

    *error =  false;
  }
}

void DynamixelDriver::setBaudrate(uint32_t baud_rate, bool *error)
{
  if (portHandler_->setBaudRate(baud_rate))
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Succeeded to change the baudrate!");
#else
    printf("Succeeded to change the baudrate(%d)!\n", portHandler_->getBaudRate());
#endif

    *error =  false;
  }
  else
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Failed to change the baudrate!");
#else
    printf("Failed to change the baudrate!\n");
#endif

    *error =  true;
  }
}

uint8_t DynamixelDriver::scan(uint8_t *get_id, uint8_t num)
{
  uint8_t error      = 0;
  uint8_t id         = 0;
  uint16_t model_num = 0;
  uint8_t id_cnt     = 0;

#if defined(__OPENCR__) || defined(__OPENCM904__)
  Serial.print("...wait for seconds\n");
#else
  printf("...wait for seconds\n");
#endif

  for (id = 1; id < num; id++)
  {
    if (packetHandler_->ping(portHandler_, id, &model_num, &error) == COMM_SUCCESS)
    {
      get_id[id_cnt] = id;
      setTools(model_num, id);
      id_cnt++;
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Succeeded to scan ");
      Serial.print(id);
      Serial.println("");
#else
      printf("Succeeded to scan %d \n", id);
#endif
    }
  }

#if defined(__OPENCR__) || defined(__OPENCM904__)
  Serial.println("Scan END");
#else
  printf("Scan END\n");
#endif

  return id_cnt;
}

uint16_t DynamixelDriver::ping(uint8_t id)
{
  uint8_t  error     = 0;
  uint16_t model_num = 0;

  if (packetHandler_->ping(portHandler_, id, &model_num, &error) == COMM_SUCCESS)
  {
    setTools(model_num, id);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("Succeeded to ping ");
    Serial.print(id);
    Serial.println("");
#else
    printf("Succeeded to ping %d \n", id);
#endif
    return model_num;
  }

  return 0;
}

bool DynamixelDriver::reboot(uint8_t id)
{
  if (packetHandler_->getProtocolVersion() == 1.0)
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("reboot command only can support in protocol version 2.0\n");
#else
    printf("reboot command only can support in protocol version 2.0\n");
#endif    
  }
  else
  {
    uint8_t error = 0;
    uint16_t comm_result = COMM_RX_FAIL;

    comm_result = packetHandler_->reboot(portHandler_, id, &error);

#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("...wait for seconds\n");
    delay(1000);
#else
    printf("...wait for seconds\n");
    sleep(1);
#endif    

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        packetHandler_->printRxPacketError(error);
#if defined(__OPENCR__) || defined(__OPENCM904__)
        Serial.print("Failed to reboot!\n");
#else
        printf("Failed to reboot!\n");
#endif
        return false;
      }
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Succeeded to reboot!\n");
#else
      printf("Succeeded to reboot!\n");
      printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", id, model_name_, portHandler_->getBaudRate());
#endif      
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);

#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Failed to reboot!\n");
#else
      printf("Failed to reboot!\n");
#endif

      return false;
    }
  }
  return true;
}

bool DynamixelDriver::reset(uint8_t id)
{
  uint8_t  error = 0;
  uint16_t comm_result = COMM_RX_FAIL;
  int baud = 0;

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
    // Reset Dynamixel except ID and Baudrate
    comm_result = packetHandler_->factoryReset(portHandler_, id, 0x00, &error);

#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("...wait for seconds\n");
    delay(1000);
#else
    printf("...wait for seconds\n");
    sleep(1);
#endif 

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        packetHandler_->printRxPacketError(error);
      }
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Succeeded to reset!\n");
#else
      printf("Succeeded to reset!\n");
#endif

      for (int i = 0; i < tools_cnt_; i++)
      {
        if (tools_[i].getID() == id)
        {
          if (!strncmp(tools_[i].getModelName(), "AX", 2) || !strncmp(tools_[i].getModelName(), "MX_12W", 6) )
            baud = 1000000;
          else
            baud = 57600;
        }
      }

      if(portHandler_->setBaudRate(baud) == false)
      {
#if defined(__OPENCR__) || defined(__OPENCM904__)
        delay(1000);
        Serial.print("Failed to change baudrate!\n");        
#else
        sleep(1);
        printf("Failed to change baudrate!\n");
#endif 

        return false;
      }
      else
      {
#if defined(__OPENCR__) || defined(__OPENCM904__)
        delay(1000);
        Serial.print("Succeeded to change baudrate!\n");        
#else
        sleep(1);
        printf("Succeeded to change baudrate!\n");
        printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", 1, model_name_, portHandler_->getBaudRate());
#endif 
      }
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Failed to reset!\n");
#else
      printf("Failed to reset!\n");
#endif

      return false;
    }
  }
  else if (packetHandler_->getProtocolVersion() == 2.0)
  {
    comm_result = packetHandler_->factoryReset(portHandler_, id, 0xff, &error);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("...wait for seconds\n");
    delay(1000);
#else
    printf("...wait for seconds\n");
    sleep(1);
#endif 

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        packetHandler_->printRxPacketError(error);
      }
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Succeeded to reset!\n");
#else
      printf("Succeeded to reset!\n");
#endif

      if (portHandler_->setBaudRate(57600) == false)
      {
#if defined(__OPENCR__) || defined(__OPENCM904__)
        delay(1000);
        Serial.print("Failed to change baudrate!\n");        
#else
        sleep(1);
        printf("Failed to change baudrate!\n");
#endif 

        return false;
      }
      else
      {
#if defined(__OPENCR__) || defined(__OPENCM904__)
        delay(1000);
        Serial.print("Succeeded to change baudrate!\n");        
#else
        sleep(1);
        printf("Succeeded to change baudrate!\n");
        printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", 1, model_name_, portHandler_->getBaudRate());
#endif 
       }
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Failed to reset!\n");
#else
      printf("Failed to reset!\n");
#endif

      return false;
    }
  }
  return true;
}

bool DynamixelDriver::writeRegister(uint8_t id, char* item_name, int32_t data)
{
  uint8_t error   = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  ControlTableItem* cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  if (cti->data_length == 1)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, cti->address, (uint8_t)data, &error);
  }
  else if (cti->data_length == 2)
  {
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, cti->address, (uint16_t)data, &error);
  }
  else if (cti->data_length == 4)
  {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, cti->address, (uint32_t)data, &error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      packetHandler_->printRxPacketError(error);
      return false;
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);

#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("Failed to write!\n");        
#else
    printf("[ID] %u, Failed to write!\n", id);
#endif 
    return false;
  }
  return true;
}

bool DynamixelDriver::readRegister(uint8_t id, char* item_name, int32_t* data)
{
  uint8_t error   = 0;
  int dxl_comm_result = COMM_RX_FAIL;

  int8_t  value_8_bit  = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  ControlTableItem* cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  if (cti->data_length == 1)
  {
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, cti->address, (uint8_t*)&value_8_bit, &error);
  }
  else if (cti->data_length == 2)
  {
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, cti->address, (uint16_t*)&value_16_bit, &error);
  }
  else if (cti->data_length == 4)
  {
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, cti->address, (uint32_t*)&value_32_bit, &error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      packetHandler_->printRxPacketError(error);
    }

    if (cti->data_length == 1)
    {
      *data = value_8_bit;
    }
    else if (cti->data_length == 2)
    {
      *data = value_16_bit;
    }
    else if (cti->data_length == 4)
    {
      *data = value_32_bit;
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("Failed to read!\n");        
#else
    printf("[ID] %u, Failed to read!\n", id);
#endif 
    return false;
  }
  return true;
}

uint8_t DynamixelDriver::findTools(uint8_t id)
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    if (tools_[i].getID() == id)
    {
      return i;
    }
  }
}

void DynamixelDriver::initSyncWrite(uint8_t id, char* item_name)
{
  ControlTableItem* cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  groupSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, 
                                                  packetHandler_, 
                                                  cti->address, 
                                                  cti->data_length);
}

bool DynamixelDriver::syncWrite(int32_t *data)
{
  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;

  uint8_t data_byte[4] = {0, };

  uint8_t cnt = tools_cnt_;

  for (int num = 0; num < cnt; ++num)
  {
    data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data[num]));
    data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data[num]));
    data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data[num]));
    data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data[num]));

    dxl_addparam_result = groupSyncWrite_->addParam(tools_[num].getID(), (uint8_t*)&data_byte);
    if (dxl_addparam_result != true)
    {
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("groupSyncWrite addparam failed\n");        
#else
      printf("[ID:%03d] groupSyncWrite addparam failed", tools_[num].getID());
#endif 
      return false;
    }
  }

  dxl_comm_result = groupSyncWrite_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupSyncWrite tx failed\n");        
#else
    packetHandler_->printTxRxResult(dxl_comm_result);
#endif 
    return false;
  }
  groupSyncWrite_->clearParam();
  return true;
}

void DynamixelDriver::initSyncRead(uint8_t id, char* item_name)
{
  ControlTableItem* cti;
  cti = tools_[findTools(id)].getControlItem(item_name);
  
  groupSyncRead_ = new dynamixel::GroupSyncRead(portHandler_, 
                                                packetHandler_, 
                                                cti->address, 
                                                cti->data_length);
}

bool DynamixelDriver::syncRead(char* item_name, int32_t *data)
{
  int  dxl_comm_result     = COMM_RX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result  = false;

  uint8_t cnt = tools_cnt_;

  for (int num = 0; num < cnt; ++num)
  {
    dxl_addparam_result = groupSyncRead_->addParam(tools_[num].getID());
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncRead_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupSyncRead rx failed\n");        
#else
    packetHandler_->printTxRxResult(dxl_comm_result);
#endif 
    return false;
  }

  for (int num = 0; num < cnt; ++num)
  {
    uint8_t id  = tools_[num].getID();
    ControlTableItem* cti;
    cti = tools_[findTools(id)].getControlItem(item_name);

    dxl_getdata_result = groupSyncRead_->isAvailable(id, cti->address, cti->data_length);

    if (dxl_getdata_result)
    {
      data[num] = groupSyncRead_->getData(id, cti->address, cti->data_length);
    }
    else
    {
      return false;
    }
  }

  groupSyncRead_->clearParam();

  return true;
}

void DynamixelDriver::initBulkWrite()
{  
  groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, 
                                                  packetHandler_);
}

bool DynamixelDriver::addBulkWriteParam(uint8_t id, char* item_name, int32_t data)
{
  bool dxl_addparam_result = false;
  uint8_t data_byte[4] = {0, };

  ControlTableItem* cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data));
  data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data));
  data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data));
  data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data));

  dxl_addparam_result = groupBulkWrite_->addParam(id, cti->address, cti->data_length, data_byte);
  if (dxl_addparam_result != true)
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupSyncWrite addparam failed\n");        
#else
    printf("[ID:%03d] groupSyncWrite addparam failed", id);
#endif 
    return false;
  }
}

void DynamixelDriver::bulkWrite()
{
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = groupBulkWrite_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS) 
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupBulkWrite tx failed\n");        
#else
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
#endif     
  }

  groupBulkWrite_->clearParam();
}

void DynamixelDriver::initBulkRead()
{
  groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, 
                                                packetHandler_);
}

void DynamixelDriver::addBulkReadParam(uint8_t id, char* item_name)
{
  bool dxl_addparam_result = false;

  ControlTableItem* cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  dxl_addparam_result = groupBulkRead_->addParam(id, cti->address, cti->data_length);
  if (dxl_addparam_result != true)
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupBulkRead addparam failed\n");        
#else
    printf("[ID:%03d] groupBulkRead addparam failed", id);
#endif 
  }
}

void DynamixelDriver::sendBulkReadPacket()
{
  int dxl_comm_result = COMM_RX_FAIL;

  dxl_comm_result = groupBulkRead_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupBulkRead tx failed\n");        
#else
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
#endif  
  }
}

bool DynamixelDriver::bulkRead(uint8_t id, char* item_name, int32_t *data)
{
  bool dxl_getdata_result = false;
  ControlTableItem* cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  dxl_getdata_result = groupBulkRead_->isAvailable(id, cti->address, cti->data_length);
  if (dxl_getdata_result != true)
  {
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupBulkRead getdata failed\n");        
#else
    fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", id);
#endif  
    
    return 0;
  }

  *data = groupBulkRead_->getData(id, cti->address, cti->data_length);
}
int32_t DynamixelDriver::convertRadian2Value(int8_t id, float radian)
{
  int32_t value = 0;
  int8_t num = 0;
  
  num = findTools(id);

  if (radian > 0)
  {
    if (tools_[num].getValueOfMaxRadianPosition() <= tools_[num].getValueOfZeroRadianPosition())
      return tools_[num].getValueOfMaxRadianPosition();

    value = (radian * (tools_[num].getValueOfMaxRadianPosition() - tools_[num].getValueOfZeroRadianPosition()) / tools_[num].getMaxRadian())
                + tools_[num].getValueOfZeroRadianPosition();
  }
  else if (radian < 0)
  {
    if (tools_[num].getValueOfMinRadianPosition() >= tools_[num].getValueOfZeroRadianPosition())
      return tools_[num].getValueOfMinRadianPosition();

    value = (radian * (tools_[num].getValueOfMinRadianPosition() - tools_[num].getValueOfZeroRadianPosition()) / tools_[num].getMinRadian())
                + tools_[num].getValueOfZeroRadianPosition();
  }
  else
  {
    value = tools_[num].getValueOfZeroRadianPosition();
  }
  // if (value[id-1] > tools_[num].getValueOfMaxRadianPosition())
  //   value[id-1] =  tools_[num].getValueOfMaxRadianPosition();
  // else if (value[id-1] < tools_[num].getValueOfMinRadianPosition())
  //   value[id-1] =  tools_[num].getValueOfMinRadianPosition();

  return value;
}

float DynamixelDriver::convertValue2Radian(int8_t id, int32_t value)
{
  float radian = 0.0;
  int8_t num = 0;
  
  num = findTools(id);

  if (value > tools_[num].getValueOfZeroRadianPosition())
  {
    if (tools_[num].getMaxRadian() <= 0)
      return tools_[num].getMaxRadian();

    radian = (float) (value - tools_[num].getValueOfZeroRadianPosition()) * tools_[num].getMaxRadian()
               / (float) (tools_[num].getValueOfMaxRadianPosition() - tools_[num].getValueOfZeroRadianPosition());
  }
  else if (value < tools_[num].getValueOfZeroRadianPosition())
  {
    if (tools_[num].getMinRadian() >= 0)
      return tools_[num].getMinRadian();

    radian = (float) (value - tools_[num].getValueOfZeroRadianPosition()) * tools_[num].getMinRadian()
               / (float) (tools_[num].getValueOfMinRadianPosition() - tools_[num].getValueOfZeroRadianPosition());
  }
  //  if (radian[id-1] > tools_[num].getMaxRadian())
  //    radian[id-1] =  tools_[num].getMaxRadian();
  //  else if (radian[id-1] < tools_[num].min_radian_)
  //    radian[id-1] =  tools_[num].min_radian_;

  return radian;
}