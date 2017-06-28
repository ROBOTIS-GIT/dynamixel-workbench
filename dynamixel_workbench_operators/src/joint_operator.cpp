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

/* Authors: Taehoon Lim (Darby) */

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
