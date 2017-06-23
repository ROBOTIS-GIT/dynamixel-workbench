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

#include <ros/ros.h>
#include <dynamixel_workbench_msgs/JointCommand.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_operator");

  dynamixel_workbench_msgs::JointCommand joint_command;
  ros::NodeHandle node_handle;

  ros::ServiceClient joint_command_client =
                node_handle.serviceClient<dynamixel_workbench_msgs::JointCommand>("/joint_command");

  if (argc != 4)
  {
    ROS_ERROR("rosrun dynamixel_workbench_operator joint_operator [mode] [pan_pos] [tilt_pos]");
    return 1;
  }

  joint_command.request.unit = argv[1];
  joint_command.request.pan_pos = atof(argv[2]);
  joint_command.request.tilt_pos = atof(argv[3]);

  if (joint_command_client.call(joint_command))
  {
    ROS_INFO("[pan_pos: %.2f (value)] [tilt_pos: %.2f (value)]", joint_command.response.pan_pos, joint_command.response.tilt_pos);
  }
  else
  {
    ROS_ERROR("Failed to call service /joint_command");
  }

  return 0;
}
