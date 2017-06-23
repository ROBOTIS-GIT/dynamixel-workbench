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
