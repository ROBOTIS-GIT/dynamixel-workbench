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

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "dynamixel_workbench_moveit_bridge/moveit_bridge.h"

MoveItBridge::MoveItBridge()
    :nh_(""),
     priv_nh_("~")
{
  // Init parameter
  planning_group_ = priv_nh_.param<std::string>("planning_group", "manipulator");

  move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_);
}

MoveItBridge::~MoveItBridge()
{
  ros::shutdown();
  return;
}

void MoveItBridge::initPublisher()
{
  dynamixel_workbench_pub_ = priv_nh_.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);
}

void MoveItBridge::initSubscriber()
{
  display_planned_path_sub_ = nh_.subscribe("/move_group/display_planned_path", 100,
                                            &MoveItBridge::displayPlannedPathMsgCallback, this);
}

void MoveItBridge::initServer()
{
  get_joint_position_server_  = priv_nh_.advertiseService("moveit/get_joint_position", &MoveItBridge::getJointPositionMsgCallback, this);
  get_kinematics_pose_server_ = priv_nh_.advertiseService("moveit/get_kinematics_pose", &MoveItBridge::getKinematicsPoseMsgCallback, this);
  set_joint_position_server_  = priv_nh_.advertiseService("moveit/set_joint_position", &MoveItBridge::setJointPositionMsgCallback, this);
  set_kinematics_pose_server_ = priv_nh_.advertiseService("moveit/set_kinematics_pose", &MoveItBridge::setKinematicsPoseMsgCallback, this);
}

bool MoveItBridge::getJointPositionMsgCallback(open_manipulator_msgs::GetJointPosition::Request &req,
                                                open_manipulator_msgs::GetJointPosition::Response &res)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  const std::vector<std::string> &joint_names = move_group_->getJointNames();
  std::vector<double> joint_values = move_group_->getCurrentJointValues();

  for (std::size_t i = 0; i < joint_names.size(); i++)
  {
    res.joint_position.joint_name.push_back(joint_names[i]);
    res.joint_position.position.push_back(joint_values[i]);
  }

  spinner.stop();
  return true;
}

bool MoveItBridge::getKinematicsPoseMsgCallback(open_manipulator_msgs::GetKinematicsPose::Request &req,
                                                 open_manipulator_msgs::GetKinematicsPose::Response &res)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  geometry_msgs::PoseStamped current_pose = move_group_->getCurrentPose();

  res.header                     = current_pose.header;
  res.kinematics_pose.pose       = current_pose.pose;

  spinner.stop();
  return true;
}

bool MoveItBridge::setJointPositionMsgCallback(open_manipulator_msgs::SetJointPosition::Request &req,
                                                open_manipulator_msgs::SetJointPosition::Response &res)
{
  open_manipulator_msgs::JointPosition msg = req.joint_position;
  res.is_planned = calcPlannedPath(req.planning_group, msg);

  return true;
}

bool MoveItBridge::setKinematicsPoseMsgCallback(open_manipulator_msgs::SetKinematicsPose::Request &req,
                                                 open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  open_manipulator_msgs::KinematicsPose msg = req.kinematics_pose;
  res.is_planned = calcPlannedPath(req.planning_group, msg);

  return true;
}

bool MoveItBridge::calcPlannedPath(const std::string planning_group, open_manipulator_msgs::KinematicsPose msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool is_planned = false;
  geometry_msgs::Pose target_pose = msg.pose;

  move_group_->setPoseTarget(target_pose);

  move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  move_group_->setGoalTolerance(msg.tolerance);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    is_planned = true;
  }
  else
  {
    ROS_WARN("Planning (task space goal) is FAILED");
    is_planned = false;
  }

  spinner.stop();

  return is_planned;
}

bool MoveItBridge::calcPlannedPath(const std::string planning_group, open_manipulator_msgs::JointPosition msg)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

  bool is_planned = false;

  const robot_state::JointModelGroup *joint_model_group = move_group_->getCurrentState()->getJointModelGroup(planning_group);

  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  uint8_t joint_num = msg.position.size();
  for (uint8_t index = 0; index < joint_num; index++)
  {
    joint_group_positions[index] = msg.position[index];
  }

  move_group_->setJointValueTarget(joint_group_positions);

  move_group_->setMaxVelocityScalingFactor(msg.max_velocity_scaling_factor);
  move_group_->setMaxAccelerationScalingFactor(msg.max_accelerations_scaling_factor);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    is_planned = true;
  }
  else
  {
    ROS_WARN("Planning (joint space goal) is FAILED");
    is_planned = false;
  }

  spinner.stop();

  return is_planned;
}

void MoveItBridge::displayPlannedPathMsgCallback(const moveit_msgs::DisplayTrajectory::ConstPtr &msg)
{
  ROS_INFO("Get Planned Path");

  trajectory_msgs::JointTrajectory jnt_tra = msg->trajectory[0].joint_trajectory;
  dynamixel_workbench_pub_.publish(jnt_tra);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_bridge");

  MoveItBridge bridge;

  bridge.initPublisher();
  bridge.initSubscriber();

  bridge.initServer();

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
