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

#ifndef DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
#define DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>

#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

namespace position_control
{
#define MOTOR 0
#define PAN   0
#define TILT  1

typedef struct
{
  std::vector<uint8_t>  torque;
  std::vector<uint32_t> pos;
  std::vector<uint32_t> prof_vel;
  std::vector<uint32_t> prof_acc;
}WriteValue;

class PositionControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle node_handle_priv_;

  // ROS Parameters
  int profile_velocity_;
  int profile_acceleration_;
  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  // ROS Topic Subscriber

  // ROS Service Server
  ros::ServiceServer joint_command_server;
  // ROS Service Client

  // ROS Topic Publisher

  // ROS Service Server

  // Dynamixel Workbench Parameters
  std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info_;
  dynamixel_multi_driver::DynamixelMultiDriver *multi_driver_;

  WriteValue *writeValue_;

 public:
  PositionControl();
  ~PositionControl();
  bool controlLoop(void);

 private:
  bool loadDynamixel();
  bool checkLoadDynamixel();
  bool initDynamixelStatePublisher();
  bool initDynamixelInfoServer();

  bool setTorque(bool onoff);
  bool setProfileValue(uint32_t prof_vel, uint32_t prof_acc);
  bool setPosition(uint32_t pan_pos, uint32_t tilt_pos);

  bool readDynamixelState();
  bool dynamixelStatePublish();

  uint32_t convertRadian2Value(float radian);

  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
