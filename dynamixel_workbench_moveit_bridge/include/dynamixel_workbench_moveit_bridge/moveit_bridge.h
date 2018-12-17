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

/* Authors: Taehun Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_MOVEIT_BRIDGE_H
#define DYNAMIXEL_WORKBENCH_MOVEIT_BRIDGE_H

#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <vector>

#include <std_msgs/String.h>

#include <geometry_msgs/PoseStamped.h>

#include "open_manipulator_msgs/OpenManipulatorState.h"

#include "open_manipulator_msgs/GetJointPosition.h"
#include "open_manipulator_msgs/GetKinematicsPose.h"

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"

class MoveItBridge
{
 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  // ROS Parameters
  std::string planning_group_;

  // ROS Publisher
  ros::Publisher dynamixel_workbench_pub_;

  // ROS Subscribers
  ros::Subscriber display_planned_path_sub_;

  // ROS Service Server
  ros::ServiceServer get_joint_position_server_;
  ros::ServiceServer get_kinematics_pose_server_;
  ros::ServiceServer set_joint_position_server_;
  ros::ServiceServer set_kinematics_pose_server_;

  // ROS Service Client

  // MoveIt! interface
  moveit::planning_interface::MoveGroupInterface* move_group_;

 public:
  MoveItBridge();
  virtual ~MoveItBridge();

  void controlCallback(const ros::TimerEvent&);

  void initPublisher();
  void initSubscriber();

  void initServer();

  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::JointPosition msg);
  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::KinematicsPose msg);

  void displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg);

  bool setJointPositionMsgCallback(open_manipulator_msgs::SetJointPosition::Request &req,
                                   open_manipulator_msgs::SetJointPosition::Response &res);

  bool setKinematicsPoseMsgCallback(open_manipulator_msgs::SetKinematicsPose::Request &req,
                                    open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool getJointPositionMsgCallback(open_manipulator_msgs::GetJointPosition::Request &req,
                                   open_manipulator_msgs::GetJointPosition::Response &res);

  bool getKinematicsPoseMsgCallback(open_manipulator_msgs::GetKinematicsPose::Request &req,
                                    open_manipulator_msgs::GetKinematicsPose::Response &res);
};

#endif /*DYNAMIXEL_WORKBENCH_MOVEIT_BRIDGE_H*/
