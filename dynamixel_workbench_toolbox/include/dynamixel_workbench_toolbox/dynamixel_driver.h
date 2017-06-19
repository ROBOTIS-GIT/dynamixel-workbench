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

#ifndef DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H

#include "dynamixel_tool.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel_driver
{

typedef struct
{
  std::string device_name;
  int baud_rate;
  float protocol_version;
}DynamixelLoadInfo;

typedef struct
{
  int16_t model_number;
  int8_t model_id;
  std::string model_name;
  DynamixelLoadInfo lode_info;
}DynamixelInfo;

class DynamixelDriver
{
 protected:
  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

 public:
  dynamixel_tool::DynamixelTool *dynamixel_;

 public:
  DynamixelDriver(std::string device_name, int baud_rate, float protocol_version);
  ~DynamixelDriver();

  bool setPortHandler(std::string device_name);
  bool setPacketHandler(float protocol_version);

  bool scan();
  bool ping(uint8_t id);

  bool reboot();
  bool reset();

  bool setBaudrate(uint32_t baud_rate);

  char* getPortName();
  float getProtocolVersion();
  uint32_t getBaudrate();

  bool writeRegister(std::string addr_name, uint32_t value);
  bool readRegister(std::string addr_name, int32_t *value);
};
}


#endif //DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H
