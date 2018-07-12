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

#include "message_header.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <cmath>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/WheelCommand.h>
#define WHEEL_NUM                        2
#define LEFT                             0
#define RIGHT                            1

#define LINEAR                           0
#define ANGULAR                          1

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
class VelocityControl
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_list_pub_;
  ros::Publisher odom_pub_;

  // ROS Topic Subscriber
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber reset_odom_sub_;

  // ROS Service Server
  ros::ServiceServer wheel_command_server_;

  // ROS Service Client

  // Dynamixel Workbench Parameters
  DynamixelWorkbench *dxl_wb_;
  uint8_t dxl_id_[2];
  uint8_t dxl_cnt_;

  // Other Parameters
  float wheel_separation_;
  float wheel_radius_;

  // Calculation for odometry
  bool init_encoder = true;
  double  last_diff_tick[WHEEL_NUM] = {0.0, 0.0};
  double  last_rad[WHEEL_NUM]       = {0.0, 0.0};
  double  last_velocity[WHEEL_NUM]  = {0.0, 0.0};
  /// Current timestamp:
  ros::Time timestamp_;
  float odom_pose[3];
  double odom_vel[3];
  nav_msgs::Odometry odom;
  geometry_msgs::TransformStamped odom_tf;
  tf::TransformBroadcaster tf_broadcaster;

 public:
  VelocityControl();
  ~VelocityControl();
  void controlLoop(void);

 private:
  void initMsg();

  void initPublisher();
  void initSubscriber();
  void dynamixelStatePublish();
  void initOdom(void);
  void odomPublish();
  void updateMotorInfo(double left_tick, double right_tick);
  bool calcOdometry(const ros::Time &time);

  void initServer();
  bool wheelCommandMsgCallback(dynamixel_workbench_msgs::WheelCommand::Request &req,
                               dynamixel_workbench_msgs::WheelCommand::Response &res);
  void commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void subscribeResetOdometry(const std_msgs::EmptyConstPtr msg);
};

#endif //DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
