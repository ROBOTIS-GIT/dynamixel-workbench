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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_MONITOR_H
#define DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_MONITOR_H

#include <ros/ros.h>

#include "dynamixel_driver.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "message_header.h"

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

  DynamixelDriver *dynamixel_driver_;

  std::string device_name_;
  uint32_t dxl_baud_rate_;
  uint8_t dxl_id_;  

 public:
  SingleDynamixelMonitor();
  ~SingleDynamixelMonitor();
//  bool controlLoop();

 private:
  void initSingleDynamixelMonitor();
  void shutdownSingleDynamixelMonitor();

  // TODO : Add new Dynamixel
  void initDynamixelStatePublisher();
  void initDynamixelInfoServer();
  void initDynamixelCommandServer();
//  // TODO : Add new Dynamixel
//  bool dynamixelStatePublish();

  bool showDynamixelControlTable();
  bool checkValidationCommand(std::string cmd);
//  bool checkValidAccess(std::string cmd);
  bool changeId(uint8_t new_id);
  bool changeBaudrate(uint32_t new_baud_rate);
//  bool changeProtocolVersion(float ver);

  bool dynamixelInfoMsgCallback(dynamixel_workbench_msgs::GetDynamixelInfo::Request &req,
                                dynamixel_workbench_msgs::GetDynamixelInfo::Response &res);

  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);

//  bool AX();
//  bool RX();
//  bool EX();
//  bool MX();
//  bool XL320();
//  bool XL();
//  bool XM();
//  bool XH();
//  bool PRO();
};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_MONITOR_H
