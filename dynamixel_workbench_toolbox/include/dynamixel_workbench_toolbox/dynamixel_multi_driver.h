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
#ifndef DYNAMIXEL_WORKBENCH_DYNAMIXEL_MULTI_DRIVER_H
#define DYNAMIXEL_WORKBENCH_DYNAMIXEL_MULTI_DRIVER_H

#include "dynamixel_driver.h"

namespace dynamixel_multi_driver
{
class DynamixelMultiDriver : public dynamixel_driver::DynamixelDriver
{
 public:
  std::vector<dynamixel_tool::DynamixelTool *> multi_dynamixel_;
  std::map<std::string, std::vector<int64_t> *> read_value_;

 private:
  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

  dynamixel::GroupSyncWrite *groupSyncWritePosition_;
  dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
  dynamixel::GroupSyncWrite *groupSyncWriteMovingSpeed_;
  dynamixel::GroupSyncWrite *groupSyncWriteCurrent_;
  dynamixel::GroupSyncWrite *groupSyncWriteTorque_;
  dynamixel::GroupSyncWrite *groupSyncWriteProfileVelocity_;
  dynamixel::GroupSyncWrite *groupSyncWriteProfileAcceleration_;

  dynamixel::GroupSyncRead  *groupSyncReadPosition_;

 public:
  DynamixelMultiDriver(std::string device_name, int baud_rate, float protocol_version);
  ~DynamixelMultiDriver();

  bool loadDynamixel(std::vector<dynamixel_driver::DynamixelInfo *> dynamixel_info);
  bool initSyncWrite();
  bool initSyncRead();
  dynamixel::GroupSyncWrite* setSyncWrite(std::string addr_name);
  dynamixel::GroupSyncRead*  setSyncRead(std::string addr_name);

  bool readMultiRegister(std::string addr_name);

  bool syncWriteTorque(std::vector<uint8_t> &onoff);
  bool syncWritePosition(std::vector<uint32_t> pos);
  bool syncWriteVelocity(std::vector<int32_t> vel);
  bool syncWriteMovingSpeed(std::vector<uint16_t> spd);
  bool syncWriteCurrent(std::vector<int16_t> cur);
  bool syncWriteProfileVelocity(std::vector<uint32_t> vel);
  bool syncWriteProfileAcceleration(std::vector<uint32_t> acc);

  bool syncReadPosition(std::vector<uint32_t> &pos);
};
}

#endif //DYNAMIXEL_WORKBENCH_DYNAMIXEL_MULTI_DRIVER_H
