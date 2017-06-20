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
