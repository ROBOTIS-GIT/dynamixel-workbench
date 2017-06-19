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

#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO

#include <ros/ros.h>
#include <dynamixel_workbench_msgs/WheelCommand.h>

#define ESC_ASCII_VALUE             0x1b
#define FORWARD                     0x77
#define BACKWARD                    0x78
#define LEFT                        0x61
#define RIGHT                       0x64
#define STOPS                       0x73

int getch(void)
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

int kbhit(void)
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
  ros::init(argc, argv, "wheel_operator");

  ROS_INFO("Set angular velocity(+-0.2 rad/sec) to your Dynamixel!! by using keyboard");
  ROS_INFO("w : Forward");
  ROS_INFO("x : Backward");
  ROS_INFO("a : Left");
  ROS_INFO("d : Right");
  ROS_INFO("s : STOPS\n");
  ROS_INFO("ESC : exit");

  dynamixel_workbench_msgs::WheelCommand wheel_command;
  ros::NodeHandle node_handle;

  ros::ServiceClient wheel_command_client =
                node_handle.serviceClient<dynamixel_workbench_msgs::WheelCommand>("/wheel_command");
  ros::Rate loop_rate(250);

  while(ros::ok())
  {
    if (kbhit())
    {
      char c = getch();

      if (c == ESC_ASCII_VALUE)
      {
        break;
      }

      if (c == FORWARD)
      {
        wheel_command.request.right_wheel_velocity = 0.2;
        wheel_command.request.left_wheel_velocity = 0.2;
      }
      else if (c == BACKWARD)
      {
        wheel_command.request.right_wheel_velocity = -0.2;
        wheel_command.request.left_wheel_velocity = -0.2;
      }
      else if (c == LEFT)
      {
        wheel_command.request.right_wheel_velocity = 0.2;
        wheel_command.request.left_wheel_velocity = -0.2;
      }
      else if (c == RIGHT)
      {
        wheel_command.request.right_wheel_velocity = -0.2;
        wheel_command.request.left_wheel_velocity = 0.2;
      }
      else if (c == STOPS)
      {
        wheel_command.request.right_wheel_velocity = 0.0;
        wheel_command.request.left_wheel_velocity = 0.0;
      }
      else
      {
        wheel_command.request.right_wheel_velocity = 0.2;
        wheel_command.request.left_wheel_velocity = 0.2;
      }

      if (wheel_command_client.call(wheel_command))
      {
        ROS_INFO("[LEFT_WHEEL_VELOCITY]: %.2f, [RIGHT_WHEEL_VELOCITY]: %.2f", wheel_command.response.left_wheel_velocity, wheel_command.response.right_wheel_velocity);
      }
      else
      {
        ROS_ERROR("Failed to call service /wheel_command");
      }
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
