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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_MONITOR_H
#define DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_MONITOR_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>

#include "dynamixel_workbench_toolbox/dynamixel_driver.h"
#include "dynamixel_workbench_single_manager/message_header.h"

namespace single_dynamixel_monitor
{

struct DynamixelLoad
{
  std::string device_name;
  int baud_rate;
  float protocol_version;
};

struct DyanmixelInfo
{
  int16_t mode_number;
  int8_t model_id;
  int8_t torque_status;
};

struct DyanmixelAddrValue
{
  std::string addr_name;
  int8_t addr_length;
  int8_t value_8_bit;
  int16_t value_16_bit;
  int32_t value_32_bit;
};

class SingleDynamixelMonitor
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_pub_;

  // ROS Topic Subscriber

  // Dynamixel Monitor variable
  bool use_ping_;
  int ping_id_;

  DynamixelLoad *dynamixel_load_;
  DyanmixelInfo *dynamixel_info_;
  DyanmixelAddrValue *dynamixel_addr_value_;

  dynamixel_driver::DynamixelDriver *dynamixel_driver_;

 public:
  SingleDynamixelMonitor();
  ~SingleDynamixelMonitor();
  bool controlLoop(void);

 private:
  bool initSingleDynamixelMonitor();
  bool shutdownSingleDynamixelMonitor();

  bool initDynamixelStatePublisher();
  bool dynamixelStatePublish();

  bool AX();

};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_MONITOR_H
