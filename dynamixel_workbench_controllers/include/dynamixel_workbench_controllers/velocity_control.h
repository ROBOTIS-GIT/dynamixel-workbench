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

/*
 *
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *    O-------------O
 *  left          right
 *   1              2
 *
 *
 * */

#ifndef DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
#define DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>

#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/WheelCommand.h>

namespace velocity_control
{
#define MOTOR  0
#define LEFT   0
#define RIGHT  1

typedef struct
{
  std::vector<uint8_t> torque;
  std::vector<int32_t> vel;
  std::vector<uint16_t> spd;
}WriteValue;

class VelocityControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle node_handle_priv_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  // ROS Topic Subscriber

  // ROS Service Server
  ros::ServiceServer wheel_command_server;
  // ROS Service Client

  // ROS Topic Publisher

  // ROS Service Server

  // Dynamixel Workbench Parameters
  std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_;
  dynamixel_multi_driver::DynamixelMultiDriver *multi_driver_;

  WriteValue *writeValue_;

 public:
  VelocityControl();
  ~VelocityControl();
  bool controlLoop(void);

 private:
  bool loadDynamixel();
  bool checkLoadDynamixel();
  bool initDynamixelStatePublisher();
  bool initDynamixelInfoServer();

  bool setTorque(bool onoff);
  bool setVelocity(int32_t left_vel, int32_t right_vel);
  bool setMovingSpeed(uint16_t left_spd, uint16_t right_spd);

  bool readDynamixelState();
  bool dynamixelStatePublish();

  int32_t convertVelocity2Value(float velocity);

  bool wheelCommandMsgCallback(dynamixel_workbench_msgs::WheelCommand::Request &req,
                               dynamixel_workbench_msgs::WheelCommand::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
