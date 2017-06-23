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

#include <ros/ros.h>

#include "dynamixel_workbench_toolbox/dynamixel_driver.h"

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "dynamixel_workbench_toolbox//message_header.h"

namespace single_dynamixel_monitor
{
class SingleDynamixelMonitor
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_status_pub_;

  // ROS Topic Subscriber

  // ROS Service Server
  ros::ServiceServer dynamixel_info_server_;
  ros::ServiceServer dynamixel_command_server_;

  // Dynamixel Monitor variable
  bool use_ping_;
  int ping_id_;

  dynamixel_driver::DynamixelInfo *dynamixel_info_;
  dynamixel_driver::DynamixelDriver *dynamixel_driver_;

 public:
  SingleDynamixelMonitor();
  ~SingleDynamixelMonitor();
  bool controlLoop();

 private:
  bool initSingleDynamixelMonitor();
  bool shutdownSingleDynamixelMonitor();

  // TODO : Add new Dynamixel
  bool initDynamixelStatePublisher();
  bool initDynamixelInfoServer();
  bool initDynamixelCommandServer();
  // TODO : Add new Dynamixel
  bool dynamixelStatePublish();

  bool showDynamixelControlTable();
  bool checkValidationCommand(std::string cmd);
  bool checkValidAccess(std::string cmd);
  bool changeId(uint8_t id);
  bool changeBaudrate(uint64_t baud_rate);
  bool changeProtocolVersion(float ver);

  bool dynamixelInfoMsgCallback(dynamixel_workbench_msgs::GetDynamixelInfo::Request &req,
                                dynamixel_workbench_msgs::GetDynamixelInfo::Response &res);

  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);

  bool AX();
  bool RX();
  bool EX();
  bool MX();
  bool XL320();
  bool XL();
  bool XM();
  bool XH();
  bool PRO();
};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_MONITOR_H
