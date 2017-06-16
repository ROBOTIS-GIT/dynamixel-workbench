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

#include "dynamixel_workbench_controllers/position_control.h"

using namespace position_control;

PositionControl::PositionControl()
    :profile_velocity_(0),
     profile_acceleration_(0)
{
  // Load Paramameter For Connection and Information about Dynamixel
  dynamixel_driver::DynamixelInfo *pan_info = new dynamixel_driver::DynamixelInfo;

  pan_info->lode_info.device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  pan_info->lode_info.baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  pan_info->lode_info.protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  pan_info->model_name                 = node_handle_.param<std::string>("model_name", "XM430_W210");

  pan_info->model_id                   = node_handle_.param<int>("pan_id", 1);

  dynamixel_info_.push_back(pan_info);

  dynamixel_driver::DynamixelInfo *tilt_info = new dynamixel_driver::DynamixelInfo;

  tilt_info->lode_info.device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  tilt_info->lode_info.baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  tilt_info->lode_info.protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  tilt_info->model_name                 = node_handle_.param<std::string>("model_name", "XM430_W210");

  tilt_info->model_id                   = node_handle_.param<int>("tilt_id", 1);

  dynamixel_info_.push_back(tilt_info);

  node_handle_.getParam("profile_velocity", profile_velocity_);
  node_handle_.getParam("profile_acceleration", profile_acceleration_);

  multi_driver_ = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info_[MOTOR]->lode_info.device_name,
                                                                   dynamixel_info_[MOTOR]->lode_info.baud_rate,
                                                                   dynamixel_info_[MOTOR]->lode_info.protocol_version);

  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("  dynamixel_workbench controller; position control example(Pan&Tilt)  ");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("PAN MOTOR INFO");
  ROS_INFO("ID    : %d", dynamixel_info_[PAN]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[PAN]->model_name.c_str());
  ROS_INFO(" ");
  ROS_INFO("TILT MOTOR INFO");
  ROS_INFO("ID    : %d", dynamixel_info_[TILT]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[TILT]->model_name.c_str());
  ROS_INFO("----------------------------------------------------------------------");

  multi_driver_->loadDynamixel(dynamixel_info_);

  multi_driver_->initSyncWrite();

  uint8_t tq = 1;
  torque_.push_back(tq);
  torque_.push_back(tq);
  multi_driver_->syncWriteTorque(torque_);

  //  writeTorque(true);
  //  writeProfile(profile_velocity_, profile_acceleration_);

  initDynamixelStatePublisher();
  initDynamixelInfoServer();
}

PositionControl::~PositionControl()
{
  ros::shutdown();
}

bool PositionControl::initDynamixelStatePublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("/position_control/dynamixel_state", 10);
}

bool PositionControl::initDynamixelInfoServer()
{
  joint_command_server = node_handle_.advertiseService("/joint_command", &PositionControl::jointCommandMsgCallback, this);

}

//bool DynamixelWorkbenchPositionControl::writeTorque(bool onoff)
//{
//  dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["torque_enable"];
//  if (onoff == true)
//  {
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, true, true);
//  }
//  else
//  {
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, false, false);
//  }
//}

//bool DynamixelWorkbenchPositionControl::writeProfile(int velocity, int acceleration)
//{
//  if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
//  {
//    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["moving_speed"];
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, velocity, velocity);
//  }
//  else if (!strncmp(motor_model_.c_str(), "MX", 2))
//  {
//    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["moving_speed"];
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, velocity, velocity);
//    sleep(1);
//    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_acceleration"];
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, acceleration, acceleration);
//    sleep(1);
//  }
//  else if (!strncmp(motor_model_.c_str(), "PRO", 3))
//  {
//    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_velocity"];
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, velocity, velocity);
//    sleep(1);
//    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_acceleration"];
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, acceleration, acceleration);
//    sleep(1);
//  }
//  else
//  {
//    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["profile_velocity"];
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, velocity, velocity);
//    sleep(1);
//    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["profile_acceleration"];
//    syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, acceleration, acceleration);
//    sleep(1);
//  }
//}

//bool DynamixelWorkbenchPositionControl::writePosition(int64_t pan_pos, int64_t tilt_pos)
//{
//  dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_position"];
//  syncWriteDynamixels(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, pan_pos, tilt_pos);
//}

//bool DynamixelWorkbenchPositionControl::readMotorState(std::string addr_name)
//{
//  std::vector<int64_t> *read_data = new std::vector<int64_t>;
//  int64_t read_value;

//  dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_[addr_name];
//  dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_[addr_name];

//  readDynamixelRegister(dynamixel_[PAN_MOTOR]->id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, &read_value);
//  read_data->push_back(read_value);

//  readDynamixelRegister(dynamixel_[TILT_MOTOR]->id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, &read_value);
//  read_data->push_back(read_value);

//  read_data_[addr_name] = read_data;
//  return true;
//}

//bool DynamixelWorkbenchPositionControl::getPublishedMsg(void)
//{
//  readMotorState("torque_enable");
//  readMotorState("moving");
//  readMotorState("goal_position");
//  readMotorState("present_position");
//  readMotorState("present_velocity");

//  if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
//  {
//    readMotorState("moving_speed");
//  }
//  else
//  {
//    readMotorState("goal_velocity");
//  }

//  if (!strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "PRO", 3))
//  {
//    readMotorState("goal_acceleration");
//  }

//  if (!strncmp(motor_model_.c_str(), "XL", 2) || !strncmp(motor_model_.c_str(), "XM", 2) || !strncmp(motor_model_.c_str(), "XH", 2))
//  {
//    readMotorState("profile_velocity");
//    readMotorState("profile_acceleration");
//  }

//  if (!strncmp(motor_model_.c_str(), "XL", 2) || !strncmp(motor_model_.c_str(), "XM", 2)  || !strncmp(motor_model_.c_str(), "XH", 2) || !strncmp(motor_model_.c_str(), "PRO", 3))
//  {
//    readMotorState("min_position_limit");
//    readMotorState("max_position_limit");
//  }
//  else
//  {
//    readMotorState("cw_angle_limit");
//    readMotorState("ccw_angle_limit");
//  }
//}

//int64_t DynamixelWorkbenchPositionControl::convertRadian2Value(double radian)
//{
//  int64_t value = 0;
//  if (radian > 0)
//  {
//    if (dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_ <= dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_)
//      return dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_;

//    value = (radian * (dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_ - dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_) / dynamixel_[PAN_TILT_MOTOR]->max_radian_)
//                + dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_;
//  }
//  else if (radian < 0)
//  {
//    if (dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_ >= dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_)
//      return dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_;

//    value = (radian * (dynamixel_[PAN_MOTOR]->value_of_min_radian_position_ - dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_) / dynamixel_[PAN_TILT_MOTOR]->min_radian_)
//                + dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_;
//  }
//  else
//    value = dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_;

//  if (value > dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_)
//    return dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_;
//  else if (value < dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_)
//    return dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_;

//  return value;
//}

bool PositionControl::controlLoop()
{
//  getPublishedMsg();

//  dynamixel_workbench_msgs::MotorState dynamixel_response[dynamixel_.size()];
//  dynamixel_workbench_msgs::MotorStateList dynamixel_response_list;

//  for (int i = 0; i < dynamixel_.size(); i++)
//  {
//    dynamixel_response[i].motor_model = dynamixel_[i]->model_name_;
//    dynamixel_response[i].id = dynamixel_[i]->id_;
//    dynamixel_response[i].torque_enable = read_data_["torque_enable"]->at(i);
//    dynamixel_response[i].present_position = read_data_["present_position"]->at(i);
//    dynamixel_response[i].present_velocity = read_data_["present_velocity"]->at(i);
//    dynamixel_response[i].goal_position = read_data_["goal_position"]->at(i);
//    dynamixel_response[i].moving = read_data_["moving"]->at(i);

//    if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
//    {
//      dynamixel_response[i].moving_speed = read_data_["moving_speed"]->at(i);
//    }
//    else
//    {
//      dynamixel_response[i].goal_velocity = read_data_["goal_velocity"]->at(i);
//    }

//    if (!strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "PRO", 3))
//    {
//      dynamixel_response[i].goal_acceleration = read_data_["goal_acceleration"]->at(i);
//    }

//    if (!strncmp(motor_model_.c_str(), "XL", 2) || !strncmp(motor_model_.c_str(), "XM", 2)  || !strncmp(motor_model_.c_str(), "XH", 2))
//    {
//      dynamixel_response[i].profile_velocity = read_data_["profile_velocity"]->at(i);
//      dynamixel_response[i].profile_acceleration = read_data_["profile_acceleration"]->at(i);
//    }

//    if (!strncmp(motor_model_.c_str(), "XL", 2) || !strncmp(motor_model_.c_str(), "XM", 2)  || !strncmp(motor_model_.c_str(), "XH", 2) || !strncmp(motor_model_.c_str(), "PRO", 3))
//    {
//      dynamixel_response[i].max_position_limit = read_data_["max_position_limit"]->at(i);
//      dynamixel_response[i].min_position_limit = read_data_["min_position_limit"]->at(i);
//    }
//    else
//    {
//      dynamixel_response[i].cw_angle_limit = read_data_["cw_angle_limit"]->at(i);
//      dynamixel_response[i].ccw_angle_limit = read_data_["ccw_angle_limit"]->at(i);
//    }

//    dynamixel_response_list.motor_states.push_back(dynamixel_response[i]);
//  }
//  dynamixel_state_pub_.publish(dynamixel_response_list);
}

bool PositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
//  int64_t pan_pos = 0;
//  int64_t tilt_pos = 0;

//  if (req.unit == "rad")
//  {
//    pan_pos = convertRadian2Value(req.pan_pos);
//    tilt_pos = convertRadian2Value(req.tilt_pos);
//  }
//  else if (req.unit == "raw")
//  {
//    pan_pos = req.pan_pos;
//    tilt_pos = req.tilt_pos;
//  }
//  else
//  {
//    pan_pos = req.pan_pos;
//    tilt_pos = req.tilt_pos;
//  }

//  writePosition(pan_pos, tilt_pos);

//  res.pan_pos = pan_pos;
//  res.tilt_pos = tilt_pos;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "position_control");
  PositionControl pos_ctrl;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    pos_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
