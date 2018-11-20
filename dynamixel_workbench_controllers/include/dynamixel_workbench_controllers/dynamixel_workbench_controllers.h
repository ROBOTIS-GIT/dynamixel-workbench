/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_CONTROLLERS_H
#define DYNAMIXEL_WORKBENCH_CONTROLLERS_H

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// Protocol 2.0
#define ADDR_PRESENT_CURRENT_2 126
#define ADDR_PRESENT_VELOCITY_2 128
#define ADDR_PRESENT_POSITION_2 132

#define LENGTH_PRESENT_CURRENT_2 2
#define LENGTH_PRESENT_VELOCITY_2 4
#define LENGTH_PRESENT_POSITION_2 4

// Protocol 1.0
#define ADDR_PRESENT_CURRENT_1 = 40;
#define ADDR_PRESENT_VELOCITY_1 = 38;
#define ADDR_PRESENT_POSITION_1 = 36;

#define LENGTH_PRESENT_CURRENT_1 = 2;
#define LENGTH_PRESENT_VELOCITY_1 = 2;
#define LENGTH_PRESENT_POSITION_1 = 2;

class DynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher joint_states_pub_;

  // ROS Topic Subscriber
  ros::Subscriber cmd_vel_sub_;

  // ROS Service Server
  ros::ServiceServer dynamixel_command_server_;
  ros::ServiceServer joint_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  std::map<std::string, uint32_t> dynamixel_;
  std::vector<std::pair<std::string, uint32_t>> dynamixel_info_;

  bool joint_state_topic_;
  bool cmd_vel_topic_;

 public:
  DynamixelController();
  ~DynamixelController();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(const std::string yaml_file);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initHandlers(void);

//  void initMsg();

  void initPublisher(void);
  void initSubscriber(void);
//  void dynamixelStatePublish();
//  void jointStatePublish();

  void initServer();

  void readCallback(const ros::TimerEvent&);
  void writeCallback(const ros::TimerEvent&);
  void publishCallback(const ros::TimerEvent&);

  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
  bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                   dynamixel_workbench_msgs::DynamixelCommand::Response &res);
  bool jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                               dynamixel_workbench_msgs::JointCommand::Response &res);
  void goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg);
};

#endif //DYNAMIXEL_WORKBENCH_CONTROLLERS_H
