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

#ifndef DYNAMIXEL_WORKBENCH_WHEEL_H
#define DYNAMIXEL_WORKBENCH_WHEEL_H

#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <stdio.h>
#include <ros/ros.h>

#include <dynamixel_workbench_msgs/SetDirection.h>

#define ESC_ASCII_VALUE             0x1b
#define FORWARD                     0x77
#define BACKWARD                    0x73
#define LEFT                        0x61
#define RIGHT                       0x64

namespace dynamixel_workbench_wheel
{
class DynamixelWorkbenchWheel
{
 public:
  // ROS Server Client
  ros::ServiceClient wheel_control_client_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // Parameters

 public:
  DynamixelWorkbenchWheel();
  ~DynamixelWorkbenchWheel();
  int getch(void);
  int kbhit(void);

 private:
  bool initDynamixelWorkbenchWheel(void);
  bool shutdownDynamixelWorkbenchWheel(void);
};
}

#endif //DYNAMIXEL_WORKBENCH_WHEEL_H
