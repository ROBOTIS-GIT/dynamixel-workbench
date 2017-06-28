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

#ifndef DYNAMIXEL_WORKBENCH_MULTI_PORT_H
#define DYNAMIXEL_WORKBENCH_MULTI_PORT_H

#include <ros/ros.h>

#include <dynamixel_workbench_toolbox/dynamixel_driver.h>

#include <dynamixel_workbench_msgs/DynamixelState.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

namespace multi_port
{
#define MOTOR 0
#define PAN   0
#define TILT  1

typedef struct
{
  std::vector<uint8_t>  torque;
  std::vector<uint32_t> pos;
}WriteValue;

class MultiPort
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle node_handle_priv_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher pan_state_pub_;
  ros::Publisher tilt_state_pub_;
  // ROS Topic Subscriber

  // ROS Service Server
  ros::ServiceServer joint_command_server;
  // ROS Service Client

  // ROS Topic Publisher

  // ROS Service Server

  // Dynamixel Workbench Parameters
  std::vector<dynamixel_driver::DynamixelInfo* > dynamixel_info_;

  dynamixel_driver::DynamixelDriver* pan_driver_;
  dynamixel_driver::DynamixelDriver* tilt_driver_;

  WriteValue *writeValue_;

  std::map<std::string, int32_t> pan_data_;
  std::map<std::string, int32_t> tilt_data_;

 public:
  MultiPort();
  ~MultiPort();
  bool controlLoop(void);

 private:
  bool loadDynamixel();
  bool checkLoadDynamixel();
  bool initDynamixelStatePublisher();
  bool initDynamixelInfoServer();

  bool setTorque(bool onoff);
  bool setPosition(uint32_t pan_pos, uint32_t tilt_pos);

  bool readValue(uint8_t motor, std::string addr_name);
  bool readDynamixelState(uint8_t motor);
  bool dynamixelStatePublish(uint8_t motor);

  uint32_t convertRadian2Value(uint8_t motor, float radian);

  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_MULTI_PORT_H
