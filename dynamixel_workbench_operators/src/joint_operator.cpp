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

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

bool getTrajectoryInfo(const std::string yaml_file, trajectory_msgs::JointTrajectory *jnt_tra_msg)
{
  YAML::Node file;
  file = YAML::LoadFile(yaml_file.c_str());

  if (file == NULL)
    return false;

  YAML::Node joint = file["joint"];
  uint8_t joint_size = joint["names"].size();

  for (uint8_t index = 0; index < joint_size; index++)
  {
    std::string joint_name = joint["names"][index].as<std::string>();
    jnt_tra_msg->joint_names.push_back(joint["names"][index].as<std::string>());
  }

  YAML::Node motion = file["motion"];
  uint8_t motion_size = motion["names"].size();

  for (uint8_t index = 0; index < motion_size; index++)
  {
    trajectory_msgs::JointTrajectoryPoint jnt_tra_point;

    std::string name = motion["names"][index].as<std::string>();
    YAML::Node motion_name = motion[name];
    for (uint8_t size = 0; size < joint_size; size++)
    {
      if (joint_size != motion_name["step"].size())
      {
        ROS_ERROR("Please check motion step size. It must be equal to joint size");
        return 0;
      }

      jnt_tra_point.positions.push_back(motion_name["step"][size].as<double>());

      ROS_INFO("motion_name : %s, step : %f", name.c_str(), motion_name["step"][size].as<double>());
    }

    if (motion_name["time_from_start"] == NULL)
    {
      ROS_ERROR("Please check time_from_start. It must be set time_from_start each step");
      return 0;
    }

    jnt_tra_point.time_from_start.fromSec(motion_name["time_from_start"].as<double>());

    ROS_INFO("time_from_start : %f", motion_name["time_from_start"].as<double>());

    jnt_tra_msg->points.push_back(jnt_tra_point);
  }

  return true;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "joint_operator");
  ros::NodeHandle node_handle("");
  ros::NodeHandle priv_node_handle("~");

  bool is_loop = priv_node_handle.param<bool>("loop", false);

  std::string yaml_file = node_handle.param<std::string>("trajectory_info", "");
  trajectory_msgs::JointTrajectory *jnt_tra_msg = new trajectory_msgs::JointTrajectory;

  ros::Publisher joint_trajectory_pub = node_handle.advertise<trajectory_msgs::JointTrajectory>("joint_trajectory", 100);

  bool result = getTrajectoryInfo(yaml_file, jnt_tra_msg);
  if (result == false)
  {
    ROS_ERROR("Please check YAML file");
    return 0;
  }

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    if (!is_loop)
    {
      while (!joint_trajectory_pub.getNumSubscribers())

      joint_trajectory_pub.publish(*jnt_tra_msg);

      ros::spin();
    }
    else
    {
      joint_trajectory_pub.publish(*jnt_tra_msg);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  return 0;
}
