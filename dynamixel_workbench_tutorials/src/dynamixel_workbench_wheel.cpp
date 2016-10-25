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

#include "dynamixel_workbench_tutorials/dynamixel_workbench_wheel.h"

using namespace dynamixel_workbench_wheel;

DynamixelWorkbenchWheel::DynamixelWorkbenchWheel()
    :nh_priv_("~"),
     is_debug_(false)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchWheel());

  // init ROS Client
  wheel_control_client_ = nh_.serviceClient<dynamixel_workbench_msgs::SetDirection>("/dynamixel_workbench_tutorials/wheel");
}

DynamixelWorkbenchWheel::~DynamixelWorkbenchWheel()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchWheel());
}

bool DynamixelWorkbenchWheel::initDynamixelWorkbenchWheel(void)
{
  ROS_INFO("dynamixel_workbench_wheel : Init OK!");
  return true;
}

bool DynamixelWorkbenchWheel::shutdownDynamixelWorkbenchWheel(void)
{
  ros::shutdown();
  return true;
}

int DynamixelWorkbenchWheel::getch(void)
{
  struct termios oldt, newt;
  int ch;

  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 1;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

int DynamixelWorkbenchWheel::kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_wheel");
  ROS_INFO("Set angular velocity(+-0.2 rad/sec) to your motor!! by using keyboard");
  ROS_INFO("w : Forward");
  ROS_INFO("s : Backward");
  ROS_INFO("a : Left");
  ROS_INFO("d : Right\n");
  ROS_INFO("ESC : exit");

  DynamixelWorkbenchWheel dynamixel_wheel;
  dynamixel_workbench_msgs::SetDirection srv;
  ros::Rate loop_rate(10);

  while(1)
  {
    if (dynamixel_wheel.kbhit())
    {
      char c = dynamixel_wheel.getch();

      if (c == ESC_ASCII_VALUE)
      {
        break;
      }

      if (c == FORWARD)
      {
        srv.request.right_wheel_velocity = 0.2;
        srv.request.left_wheel_velocity = 0.2;
      }
      else if (c == BACKWARD)
      {
        srv.request.right_wheel_velocity = -0.2;
        srv.request.left_wheel_velocity = -0.2;
      }
      else if (c == LEFT)
      {
        srv.request.right_wheel_velocity = 0.2;
        srv.request.left_wheel_velocity = -0.2;
      }
      else if (c == RIGHT)
      {
        srv.request.right_wheel_velocity = -0.2;
        srv.request.left_wheel_velocity = 0.2;
      }

      if (dynamixel_wheel.wheel_control_client_.call(srv))
      {
        sleep(0.8);
        ROS_INFO("[LEFT_WHEEL_VELOCITY]: %.2f, [RIGHT_WHEEL_VELOCITY]: %.2f", srv.response.left_wheel_velocity, srv.response.right_wheel_velocity);
      }
      else
      {
        sleep(0.8);
        ROS_ERROR("Failed to call service /wheel");
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
