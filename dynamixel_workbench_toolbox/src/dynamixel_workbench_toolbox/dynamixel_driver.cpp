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

#include "../../include/dynamixel_workbench_toolbox/dynamixel_driver.h"

using namespace dynamixel_driver;

DynamixelDriver::DynamixelDriver(std::string device_name, int baud_rate, float protocol_version)
{
  setPacketHandler(protocol_version);
  setPortHandler(device_name);
  setBaudrate(baud_rate);
}

DynamixelDriver::~DynamixelDriver()
{
  portHandler_->closePort();
}

bool DynamixelDriver::scan()
{
  uint8_t error      = 0;
  uint8_t id         = 0;
  uint16_t model_num = 0;


  printf("\n...wait for seconds\n\n");
  for (id = 1; id < 254; id++)
  {
    if (packetHandler_->ping(portHandler_, id, &model_num, &error) == COMM_SUCCESS)
    {
      dynamixel_ = new dynamixel_tool::DynamixelTool(id, model_num);
      return true;
    }
  }

  return false;
}

bool DynamixelDriver::ping(uint8_t id)
{
  uint8_t error      = 0;
  uint16_t model_num = 0;

  if (packetHandler_->ping(portHandler_, id, &model_num, &error) == COMM_SUCCESS)
  {
    dynamixel_ = new dynamixel_tool::DynamixelTool(id, model_num);
    return true;
  }

  return false;
}

bool DynamixelDriver::setPortHandler(std::string device_name)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name.c_str());

  if (portHandler_->openPort())
  {
    printf("\nSucceeded to open the port(%s)!\n", device_name.c_str());
    return true;
  }
  else
  {
    printf("Failed to open the port!\n");
    return false;
  }
}

bool DynamixelDriver::setPacketHandler(float protocol_version)
{
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version);

  return true;
}

bool DynamixelDriver::setBaudrate(uint32_t baud_rate)
{
  if (portHandler_->setBaudRate(baud_rate))
  {
    printf("Succeeded to change the baudrate(%d)!\n", portHandler_->getBaudRate());
    return true;
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return false;
  }
}

char* DynamixelDriver::getPortName()
{
  return portHandler_->getPortName();
}

float DynamixelDriver::getProtocolVersion()
{
  return packetHandler_->getProtocolVersion();
}

uint32_t DynamixelDriver::getBaudrate()
{
  return portHandler_->getBaudRate();
}

bool DynamixelDriver::writeRegister(std::string addr_name, uint32_t value)
{
  uint8_t error = 0;
  int comm_result = COMM_TX_FAIL;

  dynamixel_->item_ = dynamixel_->ctrl_table_[addr_name];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  if (addr_item->data_length      == 1)
  {
    comm_result = packetHandler_->write1ByteTxRx(portHandler_, dynamixel_->id_, addr_item->address, (uint8_t)value, &error);
  }
  else if (addr_item->data_length == 2)
  {
    comm_result = packetHandler_->write2ByteTxRx(portHandler_, dynamixel_->id_, addr_item->address, (uint16_t)value, &error);
  }
  else if (addr_item->data_length == 4)
  {
    comm_result = packetHandler_->write4ByteTxRx(portHandler_, dynamixel_->id_, addr_item->address, (uint32_t)value, &error);
  }

  if (comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      packetHandler_->printRxPacketError(error);
      return false;
    }
  }
  else
  {
    packetHandler_->printTxRxResult(comm_result);

    printf("[ID] %u, Fail to write!\n", dynamixel_->id_);
    return false;
  }
  return true;
}

bool DynamixelDriver::readRegister(std::string addr_name, int32_t *value)
{
  uint8_t error = 0;
  int comm_result = COMM_RX_FAIL;

  dynamixel_->item_ = dynamixel_->ctrl_table_[addr_name];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (addr_item->data_length == 1)
  {
    comm_result = packetHandler_->read1ByteTxRx(portHandler_, dynamixel_->id_, addr_item->address, (uint8_t*)&value_8_bit, &error);
  }
  else if (addr_item->data_length == 2)
  {
    comm_result = packetHandler_->read2ByteTxRx(portHandler_, dynamixel_->id_, addr_item->address, (uint16_t*)&value_16_bit, &error);
  }
  else if (addr_item->data_length == 4)
  {
    comm_result = packetHandler_->read4ByteTxRx(portHandler_, dynamixel_->id_, addr_item->address, (uint32_t*)&value_32_bit, &error);
  }

  if (comm_result == COMM_SUCCESS)
  {
    if (error != 0)
    {
      packetHandler_->printRxPacketError(error);
    }

    if (addr_item->data_length == 1)
    {
      *value = value_8_bit;
    }
    else if (addr_item->data_length == 2)
    {
      *value = value_16_bit;
    }
    else if (addr_item->data_length == 4)
    {
      *value = value_32_bit;
    }
  }
  else
  {
    packetHandler_->printTxRxResult(comm_result);
    printf("[ID] %u, Fail to read!(%s)\n", dynamixel_->id_, addr_item->item_name.c_str());

    return false;
  }
  return true;
}

bool DynamixelDriver::reboot()
{
  if (getProtocolVersion() != 2.0)
  {
    printf("reboot command only can support in protocol version 2.0\n");
  }
  else
  {
    uint8_t error = 0;
    uint16_t comm_result = COMM_RX_FAIL;

    comm_result = packetHandler_->reboot(portHandler_, dynamixel_->id_, &error);

    printf("...wait for a second\n");
    sleep(1);

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        packetHandler_->printRxPacketError(error);
        printf("Fail to reboot!\n");
        return false;
      }
      printf("Success to reboot!\n");
      printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);
      printf("Fail to reboot!\n");

      return false;
    }
  }
  return true;
}

bool DynamixelDriver::reset()
{
  uint8_t  error = 0;
  uint16_t comm_result = COMM_RX_FAIL;
  int baud = 0;

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
    // Reset Dynamixel except ID and Baudrate
    comm_result = packetHandler_->factoryReset(portHandler_, dynamixel_->id_, 0x00, &error);

    printf("...wait for a second\n");
    sleep(1);

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        packetHandler_->printRxPacketError(error);
      }
      printf("Success to reset!\n");

      if (dynamixel_->model_name_.find("AX") != std::string::npos ||
          dynamixel_->model_name_.find("MX_12W") != std::string::npos)
        baud = 1000000;
      else
        baud = 57600;

      if(portHandler_->setBaudRate(baud) == false)
      {
        sleep(1);
        printf(" Failed to change baudrate!\n");

        return false;
      }
      else
      {
        sleep(1);
        dynamixel_ = new dynamixel_tool::DynamixelTool(1, dynamixel_->model_number_);
        printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
      }
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);
      printf("Fail to reset!");

      return false;
    }
  }
  else if (packetHandler_->getProtocolVersion() == 2.0)
  {
    comm_result = packetHandler_->factoryReset(portHandler_, dynamixel_->id_, 0xff, &error);
    sleep(1);

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        packetHandler_->printRxPacketError(error);
      }
      printf("Success to reset!\n");

      if (portHandler_->setBaudRate(57600) == false)
      {
        sleep(1);
        printf(" Failed to change baudrate!\n");

        return false;
      }
      else
      {
        sleep(1);
        dynamixel_ = new dynamixel_tool::DynamixelTool(1, dynamixel_->model_number_);
        printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d\n", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
      }
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);
      printf("Fail to reset!\n");

      return false;
    }
  }
  return true;
}
