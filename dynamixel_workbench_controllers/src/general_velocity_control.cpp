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

/* Authors: Taehun Lim (Darby), Achille Verheye */

#include "dynamixel_workbench_controllers/general_velocity_control.h"
#include <vector>
#include <sstream>

VelocityControl::VelocityControl()
    :node_handle_(""),
    dxl_cnt_(0)
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  std::string str           = node_handle_.param<std::string>("id_list", "1");
  ROS_INFO("IDs provided: %s", str.c_str());

  std::vector<int> vect;
  std::stringstream ss(str);
  int i;
  while (ss >> i)
  {
    vect.push_back(i);
    if (ss.peek() == ',' || ss.peek() == ' ')
      ss.ignore();
  }

  dxl_cnt_ = vect.size();
  ROS_INFO("number of dxls provided: %d", dxl_cnt_);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  dxl_id_ = new uint8_t[dxl_cnt_];
  for (int id_index=0; id_index < dxl_cnt_; id_index++)
  {
    dxl_id_[id_index] = vect[id_index];
  }

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

  for (int index = 0; index < dxl_cnt_; index++)
  {
    uint16_t get_model_number;
    if (dxl_wb_->ping(dxl_id_[index], &get_model_number) != true)
    {
      ROS_ERROR("Could not find motors, please check id and baud rate");

      ros::shutdown();
      return;
    }
  }

  initMsg();

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->wheelMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Velocity");

  initPublisher();
  initSubscriber();
}

VelocityControl::~VelocityControl()
{
  for (int index = 0; index < 2; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void VelocityControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("        dynamixel_workbench controller; velocity control               \n");
  printf("              -This code supports MX2.0 and X Series-                  \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

void VelocityControl::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
}

void VelocityControl::initSubscriber()
{
  cmd_vel_sub_ = node_handle_.subscribe("cmd_joint_vel", 10, &VelocityControl::commandVelocityCallback, this);
}

void VelocityControl::dynamixelStatePublish()
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
    dynamixel_state[index].id                  = dxl_id_[index];
    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);
}

void VelocityControl::controlLoop()
{
  dynamixelStatePublish();
}

void VelocityControl::commandVelocityCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
  bool dxl_comm_result = false;

  int32_t dynamixel_velocity[dxl_cnt_];

  const float RPM_OF_DXL = 0.229;
  const float VELOCITY_CONSTANT_VALUE = 1 / (RPM_OF_DXL * 0.10472); // convert to rad/s

  for (int id_index=0; id_index < dxl_cnt_; id_index++)
  {
    float vel_ind = msg->data[id_index];
    dynamixel_velocity[id_index] = vel_ind * VELOCITY_CONSTANT_VALUE;
  }

  dxl_wb_->syncWrite("Goal_Velocity", dynamixel_velocity);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "general_velocity_control");
  VelocityControl vel_ctrl;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    vel_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
