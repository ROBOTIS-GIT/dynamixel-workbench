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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H
#define DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H

#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <ros/ros.h>

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"

#include "dynamixel_workbench_toolbox/dynamixel_tool.h"

namespace single_dynamixel_controller
{
#define ESC_ASCII_VALUE             0x1b
#define SPACEBAR_ASCII_VALUE        0x20
#define ENTER_ASCII_VALUE           0x0a

class SingleDynamixelController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;

  // ROS Parameters

  // ROS Topic Publisher

  // ROS Topic Subscriber

  // ROS Service Server

  // ROS Service Client
  ros::ServiceClient dynamixel_info_client_;
  ros::ServiceClient dynamixel_command_client_;

  // Dynamixel Workbench Parameters

 public:
  SingleDynamixelController();
  ~SingleDynamixelController();
  void viewManagerMenu(void);
  bool controlLoop(void);

 private:
  bool initSingleDynamixelController();
  bool shutdownSingleDynamixelController();

  int getch(void);
  int kbhit(void);

  bool sendCommandMsg(std::string cmd, std::string addr = "", int64_t value = 0);

};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_DYNAMIXEL_CONTROLLER_H
