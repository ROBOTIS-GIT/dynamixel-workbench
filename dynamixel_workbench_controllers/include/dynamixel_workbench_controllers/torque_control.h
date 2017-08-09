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

#ifndef DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H
#define DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_multi_driver.h>

#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

namespace torque_control
{
#define MOTOR 0
#define PAN   0
#define TILT  1

typedef struct
{
  std::vector<uint8_t>  torque;
  std::vector<int16_t>  current;
}WriteValue;

typedef struct
{
  std::vector<uint32_t> cur_pos;
  std::vector<uint32_t> des_pos;
}MotorPos;

class TorqueControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle node_handle_priv_;

  // ROS Parameters
  float p_gain_;
  float d_gain_;

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
  MotorPos   *motorPos_;

 public:
  TorqueControl();
  ~TorqueControl();
  bool controlLoop(void);

 private:
  bool loadDynamixel();
  bool checkLoadDynamixel();
  bool initDynamixelStatePublisher();
  bool initDynamixelInfoServer();

  bool setTorque(bool onoff);
  bool setCurrent(int16_t pan_cur, int16_t tilt_cur);

  bool readDynamixelState();
  bool dynamixelStatePublish();

  float  convertValue2Torque(int16_t value);
  int16_t convertTorque2Value(float torque);

  uint32_t convertRadian2Value(float radian);
  float convertValue2Radian(int32_t value);

  bool gravityCompensation();

  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H
