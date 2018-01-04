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

#include "../../include/dynamixel_workbench/dynamixel_driver.h"

DynamixelDriver::DynamixelDriver() : tools_cnt_(0), sync_write_handler_cnt_(0), sync_read_handler_cnt_(0) {}

DynamixelDriver::~DynamixelDriver()
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      writeRegister(tools_[i].dxl_info_[j].id, "Torque Enable", false);
    }
  }

  portHandler_->closePort();
}

void DynamixelDriver::setTools(uint16_t model_num, uint8_t id)
{
  if (tools_cnt_ == 0)
  {
    tools_[tools_cnt_].begin(model_num, id);
  }
  else
  {
    if (!strncmp(tools_[tools_cnt_-1].dxl_info_[0].model_name, findModelName(model_num), strlen(findModelName(model_num))))
    {
      tools_[--tools_cnt_].addDXL(model_num, id);
    }
    else
    {
      tools_[tools_cnt_].begin(model_num, id);
    }
  }

  tools_cnt_++;
}

bool DynamixelDriver::begin(const char *device_name, uint32_t baud_rate)
{
  bool error = false;

  setPortHandler(device_name, &error);
  setBaudrate(baud_rate, &error);
  setPacketHandler(&error);

  return error;
}

void DynamixelDriver::setPortHandler(const char *device_name, bool *error)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name);

  if (portHandler_->openPort())
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Succeeded to open the port!");
#else
    printf("\nSucceeded to open the port(%s)!\n", device_name);
#endif
#endif

    *error = false;
  }
  else
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Failed to open the port!");
#else
    printf("Failed to open the port!\n");
#endif
#endif

    *error = true;
  }
}

void DynamixelDriver::setPacketHandler(bool *error)
{
  packetHandler_1 = dynamixel::PacketHandler::getPacketHandler(1.0);
  packetHandler_2 = dynamixel::PacketHandler::getPacketHandler(2.0);

  if (packetHandler_1->getProtocolVersion() == 0)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Failed to setPacketHandler_1!");
#else
    printf("Failed to setPacketHandler_1!\n");
#endif
#endif

    *error = true;
  }
  else if (packetHandler_2->getProtocolVersion() == 0)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Failed to setPacketHandler_2!");
#else
    printf("Failed to setPacketHandler_2!\n");
#endif
#endif

    *error = true;
  }
  else
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Succeeded to setPacketHandler_1, setPacketHandler_2!");
#else
    printf("Succeeded to setPacketHandler_1, setPacketHandler_2!\n");
#endif
#endif

    *error = false;
  }
}

void DynamixelDriver::setPacketHandler(float protocol_version)
{
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);
}

void DynamixelDriver::setBaudrate(uint32_t baud_rate, bool *error)
{
  if (portHandler_->setBaudRate(baud_rate))
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("Succeeded to change the baudrate!(");
    Serial.print(baud_rate);
    Serial.println(")");
#else
    printf("Succeeded to change the baudrate(%d)!\n", portHandler_->getBaudRate());
#endif
#endif

    *error = false;
  }
  else
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Failed to change the baudrate!");
#else
    printf("Failed to change the baudrate!\n");
#endif
#endif

    *error = true;
  }
}

float DynamixelDriver::getProtocolVersion()
{
  return packetHandler_->getProtocolVersion();
}

char *DynamixelDriver::getModelName(uint8_t id)
{
  uint8_t cnt = findTools(id);

  for (int i = 0; i < tools_[cnt].dxl_info_cnt_; i++)
  {
    if (tools_[cnt].dxl_info_[i].id == id)
      return tools_[cnt].dxl_info_[i].model_name;
  }
}

uint16_t DynamixelDriver::getModelNum(uint8_t id)
{
  uint8_t cnt = findTools(id);

  for (int i = 0; i < tools_[cnt].dxl_info_cnt_; i++)
  {
    if (tools_[cnt].dxl_info_[i].id == id)
      return tools_[cnt].dxl_info_[i].model_num;
  }
}

ControlTableItem* DynamixelDriver::getControlItemPtr(uint8_t id)
{
  uint8_t cnt = findTools(id);

  return tools_[cnt].getControlItemPtr();
}

uint8_t DynamixelDriver::getControlTableSize(uint8_t id)
{
  uint8_t cnt = findTools(id);

  return tools_[cnt].getControlTableSize();
}

uint8_t DynamixelDriver::scan(uint8_t *get_id, uint8_t num, float protocol_version)
{
  uint8_t error = 0;
  uint8_t id = 0;
  uint16_t model_num = 0;
  uint8_t id_cnt = 0;

  tools_cnt_ = 0;

#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
  Serial.print("...wait for seconds\n");
#else
  printf("...wait for seconds\n");
#endif
#endif

  for (id = 1; id <= num; id++)
  {
    if (packetHandler_1->ping(portHandler_, id, &model_num, &error) == COMM_SUCCESS)
    {
      get_id[id_cnt] = id;
      setTools(model_num, id);
      id_cnt++;
    }
  }

  for (id = 1; id <= num; id++)
  {
    if (packetHandler_2->ping(portHandler_, id, &model_num, &error) == COMM_SUCCESS)
    {
      get_id[id_cnt] = id;
      setTools(model_num, id);
      id_cnt++;
    }
  }

  if (id_cnt == 0)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Can't find Dynamixel");
#else
    printf("Can't find Dynamixel\n");
#endif
#endif

    return false;
  }
  else
  {
    for (int num = 0; num < id_cnt; num++)
    {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("ID : ");
    Serial.print(get_id[num]);
    Serial.println(" ");
#else
    printf("ID : %d\n", get_id[num]);
#endif
#endif
    }
  }

  strcpy(dxl_, getModelName(get_id[0]));
  if (protocol_version == 2.0)
  {
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
  }
  else if (protocol_version == 1.0)
  {
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
  }
  else
  {
    if (!strncmp(dxl_, "AX", 2) || !strncmp(dxl_, "RX", 2) || !strncmp(dxl_, "EX", 2))
    {
      packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
    }
    else if (!strncmp(dxl_, "MX", 2))
    {
      if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2")) || !strncmp(dxl_, "MX-64-2", strlen("MX-64-2")) || !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")))
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
      else
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
    }
    else
    {
      packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    }
  }


#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
  Serial.println("Scan END");
  Serial.println(" ");
#else
  printf("Scan END\n");
#endif
#endif

  return id_cnt;
}

uint16_t DynamixelDriver::ping(uint8_t id, float protocol_version)
{
  uint8_t error = 0;
  uint16_t model_num = 0;

  tools_cnt_ = 0;

  if (packetHandler_1->ping(portHandler_, id, &model_num, &error) == COMM_SUCCESS)
    setTools(model_num, id);
  else if (packetHandler_2->ping(portHandler_, id, &model_num, &error) == COMM_SUCCESS)
    setTools(model_num, id);
  else
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println("Can't find Dynamixel");
#else
    printf("Can't find Dynamixel\n");
#endif
#endif
    return false;
  } 

#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("ID : ");
    Serial.println(id);
#else
    printf("ID : %d\n", id);
#endif
#endif

  strcpy(dxl_, getModelName(id));
  if (protocol_version == 2.0)
  {
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
  }
  else if (protocol_version == 1.0)
  {
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
  }
  else
  {
    if (!strncmp(dxl_, "AX", 2) || !strncmp(dxl_, "RX", 2) || !strncmp(dxl_, "EX", 2))
    {
      packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
    }
    else if (!strncmp(dxl_, "MX", 2))
    {
      if (!strncmp(dxl_, "MX-28-2", strlen("MX-28-2")) || !strncmp(dxl_, "MX-64-2", strlen("MX-64-2")) || !strncmp(dxl_, "MX-106-2", strlen("MX-106-2")))
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
      else
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(1.0);
    }
    else
    {
      packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);
    }
  }

  return model_num;
}

bool DynamixelDriver::reboot(uint8_t id)
{
  if (packetHandler_->getProtocolVersion() == 1.0)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("reboot command only can support in protocol version 2.0\n");
#else
    printf("reboot command only can support in protocol version 2.0\n");
#endif
#endif
  }
  else
  {
    uint8_t error = 0;
    uint16_t comm_result = COMM_RX_FAIL;

    comm_result = packetHandler_->reboot(portHandler_, id, &error);

#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("...wait for seconds\n");
    delay(1000);
#else
    printf("...wait for seconds\n");
    usleep(1000*1000);
#endif
#endif

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
        Serial.println(packetHandler_->getRxPacketError(error));
#else
        printf("%s\n", packetHandler_->getRxPacketError(error));
#endif
#endif    

#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
        Serial.print("Failed to reboot!\n");
#else
        printf("Failed to reboot!\n");
#endif
#endif
        return false;
      }
#if DEBUG
  #if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Succeeded to reboot!\n");
#else
      printf("Succeeded to reboot!\n");
      printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", id, getModelName(id), portHandler_->getBaudRate());
#endif
#endif
    }
    else
    {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.println(packetHandler_->getTxRxResult(comm_result));
      Serial.print("Failed to reboot!\n");
#else
      printf("%s\n", packetHandler_->getTxRxResult(comm_result));
      printf("Failed to reboot!\n");
#endif
#endif

      return false;
    }
  }
  return true;
}

bool DynamixelDriver::reset(uint8_t id)
{
  uint8_t error = 0;
  uint16_t comm_result = COMM_RX_FAIL;
  int baud = 0;

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
    // Reset Dynamixel except ID and Baudrate
    comm_result = packetHandler_->factoryReset(portHandler_, id, 0x00, &error);

#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("...wait for seconds\n");
    delay(1000);
#else
    printf("...wait for seconds\n");
    usleep(1000*1000);
#endif
#endif

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
#if DEBUG
    #if defined(__OPENCR__) || defined(__OPENCM904__)
        Serial.println(packetHandler_->getRxPacketError(error));
#else
        printf("%s\n", packetHandler_->getRxPacketError(error));
#endif
#endif    
      }
#if DEBUG
  #if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Succeeded to reset!\n");
#else
      printf("Succeeded to reset!\n");
#endif
#endif

      for (int i = 0; i < tools_cnt_; i++)
      {
//        if (tools_[i].getID() == id)
//        {
          if (!strncmp(getModelName(id), "AX", strlen("AX")) || !strncmp(getModelName(id), "MX-12W", strlen("MX-12W")))
            baud = 1000000;
          else
            baud = 57600;
//        }
      }

      if (portHandler_->setBaudRate(baud) == false)
      {
#if DEBUG
    #if defined(__OPENCR__) || defined(__OPENCM904__)
        delay(1000);
        Serial.print("Failed to change baudrate!\n");
#else
        sleep(1);
        printf("Failed to change baudrate!\n");
#endif
#endif

        return false;
      }
      else
      {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
        delay(1000);
        Serial.print("Succeeded to change baudrate!\n");
#else
        sleep(1);
        printf("Succeeded to change baudrate!\n");
        printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", 1, getModelName(id), portHandler_->getBaudRate());
#endif
#endif
      }
    }
    else
    {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.println(packetHandler_->getTxRxResult(comm_result));
      Serial.print("Failed to reset!\n");
#else
      printf("%s\n", packetHandler_->getTxRxResult(comm_result));
      printf("Failed to reset!\n");
#endif
#endif

      return false;
    }
  }
  else if (packetHandler_->getProtocolVersion() == 2.0)
  {
    comm_result = packetHandler_->factoryReset(portHandler_, id, 0xff, &error);
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("...wait for seconds\n");
    delay(1000);
#else
    printf("...wait for seconds\n");
    usleep(1000*1000);
#endif
#endif

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
        Serial.println(packetHandler_->getRxPacketError(error));
#else
        printf("%s\n", packetHandler_->getRxPacketError(error));
#endif
#endif    
      }
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("Succeeded to reset!\n");
#else
      printf("Succeeded to reset!\n");
#endif
#endif

      if (portHandler_->setBaudRate(57600) == false)
      {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
        delay(1000);
        Serial.print("Failed to change baudrate!\n");
#else
        usleep(1000*1000);
        printf("Failed to change baudrate!\n");
#endif
#endif

        return false;
      }
      else
      {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
        delay(1000);
        Serial.print("Succeeded to change baudrate!\n");
#else
        usleep(1000*1000);
        printf("Succeeded to change baudrate!\n");
        printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", 1, getModelName(id), portHandler_->getBaudRate());
#endif
#endif
      }
    }
    else
    {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.println(packetHandler_->getTxRxResult(comm_result));
      Serial.print("Failed to reset!\n");
#else
      printf("%s\n", packetHandler_->getTxRxResult(comm_result));
      printf("Failed to reset!\n");
#endif
#endif

      return false;
    }
  }
  return true;
}

bool DynamixelDriver::writeRegister(uint8_t id, const char *item_name, int32_t data)
{
  uint8_t error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  ControlTableItem *cti;
  cti = tools_[findTools(id)].getControlItem(item_name);  

  if (cti->data_length == BYTE)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, cti->address, (uint8_t)data, &error);
  }
  else if (cti->data_length == WORD)
  {
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, cti->address, (uint16_t)data, &error);
  }
  else if (cti->data_length == DWORD)
  {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, cti->address, (uint32_t)data, &error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.println(packetHandler_->getRxPacketError(error));
#else
      printf("%s\n", packetHandler_->getRxPacketError(error));
#endif
#endif    

      return false;
    }
  }
  else
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
#else
    printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
#endif
#endif

    return false;
  }

  return true;
}

bool DynamixelDriver::readRegister(uint8_t id, const char *item_name, int32_t *data)
{
  uint8_t error = 0;
  int dxl_comm_result = COMM_RX_FAIL;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  ControlTableItem *cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  if (cti->data_length == BYTE)
  {
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, cti->address, (uint8_t *)&value_8_bit, &error);
  }
  else if (cti->data_length == WORD)
  {
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, cti->address, (uint16_t *)&value_16_bit, &error);
  }
  else if (cti->data_length == DWORD)
  {
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, cti->address, (uint32_t *)&value_32_bit, &error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print(packetHandler_->getRxPacketError(error));
#else
      printf("%s\n", packetHandler_->getRxPacketError(error));
#endif
#endif
    }

    if (cti->data_length == BYTE)
    {
      *data = value_8_bit;
    }
    else if (cti->data_length == WORD)
    {
      *data = value_16_bit;
    }
    else if (cti->data_length == DWORD)
    {
      *data = value_32_bit;
    }
  }
  else
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
#else
    printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
#endif
#endif
    return false;
  }

  return true;
}

uint8_t DynamixelDriver::findTools(uint8_t id)
{
  for (int i = 0; i < tools_cnt_; i++)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      if (tools_[i].dxl_info_[j].id == id)
      {
        return i;
      }
    }
  }
}

const char *DynamixelDriver::findModelName(uint16_t model_num)
{
  uint16_t num = model_num;
  const char* model_name = NULL;

  if (num == AX_12A)
    model_name = "AX-12A";
  else if (num == AX_12W)
    model_name = "AX-12W";
  else if (num == AX_18A)
    model_name = "AX-18A";

  else if (num == RX_24F)
    model_name = "RX-24F";
  else if (num == RX_28)
    model_name = "RX-28";
  else if (num == RX_64)
    model_name = "RX-64";

  else if (num == EX_106)
    model_name = "EX-106";

  else if (num == MX_12W)
    model_name = "MX-12W";
  else if (num == MX_28)
    model_name = "MX-28";
  else if (num == MX_28_2)
    model_name = "MX-28-2";
  else if (num == MX_64)
    model_name = "MX-64";
  else if (num == MX_64_2)
    model_name = "MX-64-2";
  else if (num == MX_106)
    model_name = "MX-106";
  else if (num == MX_106_2)
    model_name = "MX-106-2";

  else if (num == XL_320)
    model_name = "XL-320";
  else if (num == XL430_W250)
    model_name = "XL430-W250";

  else if (num == XM430_W210)
    model_name = "XM430-W210";
  else if (num == XM430_W350)
    model_name = "XM430-W350";
  else if (num == XM540_W150)
    model_name = "XM540-W150";
  else if (num == XM540_W270)
    model_name = "XM540-W270";

  else if (num == XH430_V210)
    model_name = "XH430-V210";
  else if (num == XH430_V350)
    model_name = "XH430-V350";
  else if (num == XH430_W210)
    model_name = "XH430-W210";
  else if (num == XH430_W350)
    model_name = "XH430-W350";

  else if (num == PRO_L42_10_S300_R)
    model_name = "PRO-L42-10-S300-R";
  else if (num == PRO_L54_30_S400_R)
    model_name = "PRO-L54-30-S400-R";
  else if (num == PRO_L54_30_S500_R)
    model_name = "PRO-L54-30-S500-R";
  else if (num == PRO_L54_50_S290_R)
    model_name = "PRO-L54-50-S290-R";
  else if (num == PRO_L54_50_S500_R)
    model_name = "PRO-L54-50-S500-R";

  else if (num == PRO_M42_10_S260_R)
    model_name = "PRO-M42-10-S260-R";
  else if (num == PRO_M54_40_S250_R)
    model_name = "PRO-M54-40-S250-R";
  else if (num == PRO_M54_60_S250_R)
    model_name = "PRO-M54-60-S250-R";

  else if (num == PRO_H42_20_S300_R)
    model_name = "PRO-H42-20-S300-R";
  else if (num == PRO_H54_100_S500_R)
    model_name = "PRO-H54-100-S500-R";
  else if (num == PRO_H54_200_S500_R)
    model_name = "PRO-H54-200-S500-R";

  return model_name;
}

void DynamixelDriver::addSyncWrite(const char *item_name)
{
  ControlTableItem *cti;
  cti = tools_[0].getControlItem(item_name);

  syncWriteHandler_[sync_write_handler_cnt_].cti = cti;

  syncWriteHandler_[sync_write_handler_cnt_].groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_,
                                                                                            packetHandler_,
                                                                                            cti->address,
                                                                                            cti->data_length);

  sync_write_handler_cnt_++;
}

bool DynamixelDriver::syncWrite(const char *item_name, int32_t *data)
{
  bool dxl_addparam_result = false;
  int dxl_comm_result = COMM_TX_FAIL;

  uint8_t data_byte[4] = {0, };

  SyncWriteHandler swh;

  for (int index = 0; index < sync_write_handler_cnt_; index++)
  {
    if (!strncmp(syncWriteHandler_[index].cti->item_name, item_name, strlen(item_name)))
    {
      swh.groupSyncWrite = syncWriteHandler_[index].groupSyncWrite;
      swh.cti = syncWriteHandler_[index].cti;
    }
  }

  for (int i = 0; i < tools_cnt_; ++i)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data[i]));
      data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data[i]));
      data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data[i]));
      data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data[i]));

      dxl_addparam_result = swh.groupSyncWrite->addParam(tools_[i].dxl_info_[j].id, (uint8_t *)&data_byte);
      if (dxl_addparam_result != true)
      {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
      Serial.print("groupSyncWrite addparam failed\n");
#else
      printf("[ID:%03d] groupSyncWrite addparam failed", tools_[i].dxl_info_[j].id);
#endif
#endif
        return false;
      }
    }
  }

  dxl_comm_result = swh.groupSyncWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
#else
    printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
#endif
#endif
    return false;
  }
  swh.groupSyncWrite->clearParam();
  return true;
}

void DynamixelDriver::addSyncRead(const char *item_name)
{
  ControlTableItem *cti;
  cti = tools_[0].getControlItem(item_name);

  syncReadHandler_[sync_read_handler_cnt_].cti = cti;
  
  syncReadHandler_[sync_read_handler_cnt_].groupSyncRead = new dynamixel::GroupSyncRead(portHandler_,
                                                                                        packetHandler_,
                                                                                        cti->address,
                                                                                        cti->data_length);

  sync_read_handler_cnt_++;
}

bool DynamixelDriver::syncRead(const char *item_name, int32_t *data)
{
  int dxl_comm_result = COMM_RX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  int data_num = 0;

  SyncReadHandler srh;
  
  for (int index = 0; index < sync_read_handler_cnt_; index++)
  {
    if (!strncmp(syncReadHandler_[index].cti->item_name, item_name, strlen(item_name)))
    {
      srh.groupSyncRead = syncReadHandler_[index].groupSyncRead;
      srh.cti = syncReadHandler_[index].cti;
    }
  }

  for (int i = 0; i < tools_cnt_; ++i)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      dxl_addparam_result = srh.groupSyncRead->addParam(tools_[i].dxl_info_[j].id);
      if (dxl_addparam_result != true)
        return false;
    }
  }
  
  dxl_comm_result = srh.groupSyncRead->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
#else
    printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
#endif
#endif
    return false;
  }

  for (int i = 0; i < tools_cnt_; ++i)
  {
    for (int j = 0; j < tools_[i].dxl_info_cnt_; j++)
    {
      uint8_t id = tools_[i].dxl_info_[j].id;

      dxl_getdata_result = srh.groupSyncRead->isAvailable(id, srh.cti->address, srh.cti->data_length);

      if (dxl_getdata_result)
      {
        data[data_num++] = srh.groupSyncRead->getData(id, srh.cti->address, srh.cti->data_length);
      }
      else
      {
        return false;
      }
    }
  }

  srh.groupSyncRead->clearParam();

  return true;
}

void DynamixelDriver::initBulkWrite()
{
  groupBulkWrite_ = new dynamixel::GroupBulkWrite(portHandler_, packetHandler_);
}

bool DynamixelDriver::addBulkWriteParam(uint8_t id, const char *item_name, int32_t data)
{
  bool dxl_addparam_result = false;
  uint8_t data_byte[4] = {0, };

  ControlTableItem *cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  data_byte[0] = DXL_LOBYTE(DXL_LOWORD(data));
  data_byte[1] = DXL_HIBYTE(DXL_LOWORD(data));
  data_byte[2] = DXL_LOBYTE(DXL_HIWORD(data));
  data_byte[3] = DXL_HIBYTE(DXL_HIWORD(data));

  dxl_addparam_result = groupBulkWrite_->addParam(id, cti->address, cti->data_length, data_byte);
  if (dxl_addparam_result != true)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupSyncWrite addparam failed\n");
#else
    printf("[ID:%03d] groupSyncWrite addparam failed", id);
#endif
#endif
    return false;
  }

  return true;
}

bool DynamixelDriver::bulkWrite()
{
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = groupBulkWrite_->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
#else
    printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
#endif
#endif

    return false;
  }

  groupBulkWrite_->clearParam();

  return true;
}

void DynamixelDriver::initBulkRead()
{
  groupBulkRead_ = new dynamixel::GroupBulkRead(portHandler_, packetHandler_);
}

bool DynamixelDriver::addBulkReadParam(uint8_t id, const char *item_name)
{
  bool dxl_addparam_result = false;

  ControlTableItem *cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  dxl_addparam_result = groupBulkRead_->addParam(id, cti->address, cti->data_length);
  if (dxl_addparam_result != true)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupBulkRead addparam failed\n");
#else
    printf("[ID:%03d] groupBulkRead addparam failed", id);
#endif
#endif

    return false;
  }

  return true;
}

bool DynamixelDriver::sendBulkReadPacket()
{
  int dxl_comm_result = COMM_RX_FAIL;

  dxl_comm_result = groupBulkRead_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
#else
    printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
#endif
#endif

    return false;
  }

  return true;
}

bool DynamixelDriver::bulkRead(uint8_t id, const char *item_name, int32_t *data)
{
  bool dxl_getdata_result = false;
  ControlTableItem *cti;
  cti = tools_[findTools(id)].getControlItem(item_name);

  dxl_getdata_result = groupBulkRead_->isAvailable(id, cti->address, cti->data_length);
  if (dxl_getdata_result != true)
  {
#if DEBUG
#if defined(__OPENCR__) || defined(__OPENCM904__)
    Serial.print("groupBulkRead getdata failed\n");
#else
    fprintf(stderr, "[ID:%03d] groupBulkRead getdata failed", id);
#endif
#endif

    return false;
  }

  *data = groupBulkRead_->getData(id, cti->address, cti->data_length);

  return true;
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

    value = (radian * (tools_[num].getValueOfMaxRadianPosition() - tools_[num].getValueOfZeroRadianPosition()) / tools_[num].getMaxRadian()) + tools_[num].getValueOfZeroRadianPosition();
  }
  else if (radian < 0)
  {
    if (tools_[num].getValueOfMinRadianPosition() >= tools_[num].getValueOfZeroRadianPosition())
      return tools_[num].getValueOfMinRadianPosition();

    value = (radian * (tools_[num].getValueOfMinRadianPosition() - tools_[num].getValueOfZeroRadianPosition()) / tools_[num].getMinRadian()) + tools_[num].getValueOfZeroRadianPosition();
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

    radian = (float)(value - tools_[num].getValueOfZeroRadianPosition()) * tools_[num].getMaxRadian() / (float)(tools_[num].getValueOfMaxRadianPosition() - tools_[num].getValueOfZeroRadianPosition());
  }
  else if (value < tools_[num].getValueOfZeroRadianPosition())
  {
    if (tools_[num].getMinRadian() >= 0)
      return tools_[num].getMinRadian();

    radian = (float)(value - tools_[num].getValueOfZeroRadianPosition()) * tools_[num].getMinRadian() / (float)(tools_[num].getValueOfMinRadianPosition() - tools_[num].getValueOfZeroRadianPosition());
  }
  //  if (radian[id-1] > tools_[num].getMaxRadian())
  //    radian[id-1] =  tools_[num].getMaxRadian();
  //  else if (radian[id-1] < tools_[num].min_radian_)
  //    radian[id-1] =  tools_[num].min_radian_;

  return radian;
}
