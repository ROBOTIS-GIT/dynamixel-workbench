/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Taehoon Lim (Darby) */

#include "dynamixel_workbench_toolbox//dynamixel_driver.h"

using namespace dynamixel_driver;

DynamixelDriver::DynamixelDriver(std::string device_name, int baud_rate, float protocol_version)
{
  device_name_      = device_name;
  protocol_version_ = protocol_version;
  baud_rate_        = baud_rate;

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol 1.0 PacketHandler and Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the port(%s)!", device_name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate_))
  {
    ROS_INFO("Succeeded to change the baudrate(%d)!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
  }
}

DynamixelDriver::~DynamixelDriver()
{
  portHandler_->closePort();
}

bool DynamixelDriver::scan()
{
  uint8_t error = 0;

  ROS_INFO("...wait for seconds");
  for (id_ = 1; id_ < 254; id_++)
  {
    if (packetHandler_->ping(portHandler_, id_, &model_num_, &error) == COMM_SUCCESS)
    {
      dynamixel_ = new dynamixel_tool::DynamixelTool(id_, model_num_, protocol_version_);
      return true;
    }
  }

  return false;
}

bool DynamixelDriver::ping(uint8_t id)
{
  uint8_t error = 0;

  if (packetHandler_->ping(portHandler_, id, &model_num_, &error) == COMM_SUCCESS)
  {
    dynamixel_ = new dynamixel_tool::DynamixelTool(id, model_num_, protocol_version_);
    return true;
  }

  return false;
}

bool DynamixelDriver::setPortHandler(std::string device_name)
{
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name.c_str());

  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the port(%s)!", device_name_.c_str());
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

  if (addr_item->data_length == 1)
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
    return true;
  }
  else
  {
    packetHandler_->printTxRxResult(comm_result);

    ROS_ERROR("[ID] %u, Fail to write!", dynamixel_->id_);
    return false;
  }

  return true;
}

bool DynamixelDriver::readRegister(std::string addr_name, uint32_t *value)
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

    ROS_ERROR("[ID] %u, Fail to read!", dynamixel_->id_);
    return false;
  }
}

bool DynamixelDriver::reboot()
{
  if (protocol_version_ != 2.0)
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

      //dynamixel_ = new dynamixel_tool::DynamixelTool(dynamixel_->id_, dynamixel_->model_number_, packetHandler_->getProtocolVersion());
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
  uint8_t error = 0;
  uint16_t comm_result = COMM_RX_FAIL;

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

      if (portHandler_->setBaudRate(1000000) == false)
      {
        sleep(1);
        ROS_ERROR(" Failed to change baudrate!");

        return false;
      }
      else
      {
        sleep(1);
        dynamixel_ = new dynamixel_tool::DynamixelTool(1, dynamixel_->model_number_, packetHandler_->getProtocolVersion());
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
        dynamixel_ = new dynamixel_tool::DynamixelTool(1, dynamixel_->model_number_, packetHandler_->getProtocolVersion());
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
