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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H
#define DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H

#include <unistd.h>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>

#include "dynamixel_workbench_single_manager/message_header.h"
#include "dynamixel_workbench_toolbox//dynamixel_driver.h"

namespace single_manager
{
#define ESC_ASCII_VALUE             0x1b
#define SPACEBAR_ASCII_VALUE        0x20

struct DynamixelLoad
{
  std::string device_name;
  int baud_rate;
  float protocol_version;
};

struct DyanmixelInfo
{
  int16_t mode_number;
  int8_t model_id;
  int8_t torque_status;
};

struct DyanmixelAddrValue
{
  std::string addr_name;
  int8_t addr_length;
  int8_t value_8_bit;
  int16_t value_16_bit;
  int32_t value_32_bit;
};

class SingleManager
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher
  ros::Publisher dynamixel_state_pub_;

  // ROS Topic Subscriber
  ros::Subscriber dynamixel_command_sub_;

  // ROS Server
  ros::ServiceServer workbench_param_server_;

  // Dynamixel Workbench Parameters
  bool use_ping_;
  int ping_id_;

  DynamixelLoad *dynamixel_load_;
  DyanmixelInfo *dynamixel_info_;
  DyanmixelAddrValue *dynamixel_addr_value_;

  dynamixel_driver::DynamixelDriver *dynamixel_driver_;

 public:
  SingleManager();
  ~SingleManager();
  void viewManagerMenu(void);
  bool controlLoop(void);

 private:
  bool initSingleManager(void);
  bool shutdownSingleManager(void);

  int getch(void);
  int kbhit(void);

  void showControlTable(void);
  bool rebootDynamixel();
  bool resetDynamixel();
  void checkValidationCommand(bool *valid_cmd, char *cmd);

  void setPublisher(void);
  void setSubscriber(void);
  void setPublishedMsg(void);
  void setServer(void);

  void dynamixelCommandMsgCallback(const dynamixel_workbench_msgs::DynamixelCommand::ConstPtr &msg);
  bool getWorkbenchParamCallback(dynamixel_workbench_msgs::GetWorkbenchParam::Request &req, dynamixel_workbench_msgs::GetWorkbenchParam::Response &res);

  bool writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t value);
  bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value);

  void axMotorMessage(void);
  void rxMotorMessage(void);
  void mxMotorMessage(void);
  void mx64MotorMessage(void);
  void mx106MotorMessage(void);
  void exMotorMessage(void);
  void xl320MotorMessage(void);
  void xlMotorMessage(void);
  void xmMotorMessage(void);
  void xhMotorMessage(void);
  void proMotorMessage(void);
  void proL42MotorMessage(void);
};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H
