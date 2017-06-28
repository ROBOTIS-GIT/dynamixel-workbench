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

#include "dynamixel_workbench_toolbox/dynamixel_driver.h"

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

  ROS_INFO("...wait for seconds");
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
    ROS_INFO("Succeeded to open the port(%s)!", device_name.c_str());
    return true;
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
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
    ROS_INFO("Succeeded to change the baudrate(%d)!", portHandler_->getBaudRate());
    return true;
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
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
    }
    return true;
  }
  else
  {
    packetHandler_->printTxRxResult(comm_result);

    ROS_ERROR("[ID] %u, Fail to write!", dynamixel_->id_);
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
      return true;

    }
    else if (addr_item->data_length == 2)
    {
      *value = value_16_bit;
      return true;
    }
    else if (addr_item->data_length == 4)
    {
      *value = value_32_bit;
      return true;
    }
  }
  else
  {
    packetHandler_->printTxRxResult(comm_result);

    ROS_ERROR("[ID] %u, Fail to read!(%s)", dynamixel_->id_, addr_item->item_name.c_str());
    return false;
  }
}

bool DynamixelDriver::reboot()
{
  if (getProtocolVersion() != 2.0)
  {
    ROS_WARN("reboot command only can support in protocol version 2.0");
  }
  else
  {
    uint8_t error = 0;
    uint16_t comm_result = COMM_RX_FAIL;

    comm_result = packetHandler_->reboot(portHandler_, dynamixel_->id_, &error);

    ROS_INFO("...wait for a second");
    sleep(1);

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        packetHandler_->printRxPacketError(error);
      }
      ROS_INFO("Success to reboot!");
      ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());

      return true;
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);
      ROS_ERROR("Fail to reboot!");

      return false;
    }
  }
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

    ROS_INFO("...wait for a second");
    sleep(1);

    if (comm_result == COMM_SUCCESS)
    {
      if (error != 0)
      {
        packetHandler_->printRxPacketError(error);
      }
      ROS_INFO("Success to reset!");

      if (dynamixel_->model_name_.find("AX") != std::string::npos ||
          dynamixel_->model_name_.find("MX_12W") != std::string::npos)
        baud = 1000000;
      else
        baud = 57600;

      if(portHandler_->setBaudRate(baud) == false)
      {
        sleep(1);
        ROS_ERROR(" Failed to change baudrate!");

        return false;
      }
      else
      {
        sleep(1);
        dynamixel_ = new dynamixel_tool::DynamixelTool(1, dynamixel_->model_number_);
        ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());

        return true;
      }
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);
      ROS_ERROR("Fail to reset!");

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
      ROS_INFO("Success to reset!");

      if (portHandler_->setBaudRate(57600) == false)
      {
        sleep(1);
        ROS_ERROR(" Failed to change baudrate!");

        return false;
      }
      else
      {
        sleep(1);
        dynamixel_ = new dynamixel_tool::DynamixelTool(1, dynamixel_->model_number_);
        ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());

        return true;
      }
    }
    else
    {
      packetHandler_->printTxRxResult(comm_result);
      ROS_ERROR("Fail to reset!");

      return false;
    }
  }
}
