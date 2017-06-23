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

#include "dynamixel_workbench_controllers/multi_port.h"

using namespace multi_port;

MultiPort::MultiPort()
    :node_handle_(""),
     node_handle_priv_("~")
{
  if (loadDynamixel())
  {
    checkLoadDynamixel();
  }
  else
  {
    ROS_ERROR("Cant' Load Dynamixel, Please check Parameter");
  }

  writeValue_ = new WriteValue;

  setTorque(true);

  initDynamixelStatePublisher();
  initDynamixelInfoServer();
}

MultiPort::~MultiPort()
{
  setTorque(false);

  ros::shutdown();
}

bool MultiPort::loadDynamixel()
{
  bool ret = false;

  dynamixel_driver::DynamixelInfo *pan_info = new dynamixel_driver::DynamixelInfo;

  pan_info->lode_info.device_name      = node_handle_.param<std::string>("pan/device_name", "/dev/ttyUSB0");
  pan_info->lode_info.baud_rate        = node_handle_.param<int>("pan/baud_rate", 57600);
  pan_info->lode_info.protocol_version = node_handle_.param<float>("pan/protocol_version", 2.0);

  pan_info->model_id                   = node_handle_.param<int>("pan/id", 1);

  dynamixel_info_.push_back(pan_info);

  dynamixel_driver::DynamixelInfo *tilt_info = new dynamixel_driver::DynamixelInfo;

  tilt_info->lode_info.device_name      = node_handle_.param<std::string>("tilt/device_name", "/dev/ttyUSB1");
  tilt_info->lode_info.baud_rate        = node_handle_.param<int>("tilt/baud_rate", 57600);
  tilt_info->lode_info.protocol_version = node_handle_.param<float>("tilt/protocol_version", 1.0);

  tilt_info->model_id                   = node_handle_.param<int>("tilt/id", 2);

  dynamixel_info_.push_back(tilt_info);

  pan_driver_  = new dynamixel_driver::DynamixelDriver(dynamixel_info_[PAN]->lode_info.device_name,
                                                       dynamixel_info_[PAN]->lode_info.baud_rate,
                                                       dynamixel_info_[PAN]->lode_info.protocol_version);

  tilt_driver_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_[TILT]->lode_info.device_name,
                                                       dynamixel_info_[TILT]->lode_info.baud_rate,
                                                       dynamixel_info_[TILT]->lode_info.protocol_version);

  ret = pan_driver_ ->ping(dynamixel_info_[PAN]->model_id);
  ret = tilt_driver_->ping(dynamixel_info_[TILT]->model_id);

  if (ret)
  {
    dynamixel_info_[PAN] ->model_name  = pan_driver_->dynamixel_->model_name_.c_str();
    dynamixel_info_[TILT]->model_name  = tilt_driver_->dynamixel_->model_name_.c_str();
  }

 return ret;
}

bool MultiPort::setTorque(bool onoff)
{
  writeValue_->torque.clear();
  writeValue_->torque.push_back(onoff);
  writeValue_->torque.push_back(onoff);

  if (!pan_driver_->writeRegister("torque_enable", writeValue_->torque.at(PAN)))
  {
    ROS_ERROR("Write Pan Torque Failed!");
    return false;
  }

  if (!tilt_driver_->writeRegister("torque_enable", writeValue_->torque.at(TILT)))
  {
    ROS_ERROR("Write Tilt Torque Failed!");
    return false;
  }

  return true;
}

bool MultiPort::setPosition(uint32_t pan_pos, uint32_t tilt_pos)
{
  writeValue_->pos.clear();
  writeValue_->pos.push_back(pan_pos);
  writeValue_->pos.push_back(tilt_pos);

  if (!pan_driver_->writeRegister("goal_position", writeValue_->pos.at(PAN)))
  {
    ROS_ERROR("Write Pan Position Failed!");
    return false;
  }

  if (!tilt_driver_->writeRegister("goal_position", writeValue_->pos.at(TILT)))
  {
    ROS_ERROR("Write Tilt Position Failed!");
    return false;
  }

  return true;
}

bool MultiPort::checkLoadDynamixel()
{
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("  dynamixel_workbench controller; multi port example(Pan & Tilt)       ");
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("PAN MOTOR INFO");
  ROS_INFO("Device Name    : %s", dynamixel_info_[PAN]->lode_info.device_name.c_str());
  ROS_INFO("ID             : %d", dynamixel_info_[PAN]->model_id);
  ROS_INFO("MODEL          : %s", dynamixel_info_[PAN]->model_name.c_str());
  ROS_INFO(" ");
  ROS_INFO("TILT MOTOR INFO");
  ROS_INFO("Device Name    : %s", dynamixel_info_[TILT]->lode_info.device_name.c_str());
  ROS_INFO("ID             : %d", dynamixel_info_[TILT]->model_id);
  ROS_INFO("MODEL          : %s", dynamixel_info_[TILT]->model_name.c_str());
  ROS_INFO("-----------------------------------------------------------------------");
}

bool MultiPort::initDynamixelStatePublisher()
{
  pan_state_pub_  = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelState>("/multi_port/pan_state", 10);
  tilt_state_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelState>("/multi_port/tilt_state", 10);
}

bool MultiPort::initDynamixelInfoServer()
{
  joint_command_server = node_handle_.advertiseService("/joint_command", &MultiPort::jointCommandMsgCallback, this);
}

bool MultiPort::readValue(uint8_t motor, std::string addr_name)
{
  int32_t read_value;

  if (motor == PAN)
  {
    pan_driver_->readRegister(addr_name, &read_value);
    pan_data_[addr_name] = read_value;
  }
  else if (motor == TILT)
  {
    tilt_driver_->readRegister(addr_name, &read_value);
    tilt_data_[addr_name] = read_value;
  }
}

bool MultiPort::readDynamixelState(uint8_t motor)
{
  dynamixel_driver::DynamixelDriver* dynamixel_driver;

  if (motor == PAN)
    dynamixel_driver = pan_driver_;
  else if (motor == TILT)
    dynamixel_driver = tilt_driver_;

  readValue(motor, "torque_enable");

  readValue(motor, "present_position");

  readValue(motor, "goal_position");
  readValue(motor, "moving");

  if (dynamixel_driver->getProtocolVersion() == 2.0)
  {
    if (dynamixel_driver->dynamixel_->model_name_.find("XM") != std::string::npos)
    {
      readValue(motor, "goal_current");
      readValue(motor, "present_current");
    }

    readValue(motor, "goal_velocity");
    readValue(motor, "present_velocity");
  }
  else
  {
    readValue(motor, "moving_speed");
    readValue(motor, "present_speed");
  }
}

bool MultiPort::dynamixelStatePublish(uint8_t motor)
{
  readDynamixelState(PAN);
  readDynamixelState(TILT);

  dynamixel_driver::DynamixelDriver*       dynamixel_driver;
  dynamixel_workbench_msgs::DynamixelState dynamixel_state;

  std::map<std::string, int32_t> read_data;

  if (motor == PAN)
  {
    dynamixel_driver = pan_driver_;
    read_data        = pan_data_;
  }
  else if (motor == TILT)
  {
    dynamixel_driver = tilt_driver_;
    read_data        = tilt_data_;
  }

  dynamixel_state.model_name          = dynamixel_driver->dynamixel_->model_name_;
  dynamixel_state.id                  = dynamixel_driver->dynamixel_->id_;
  dynamixel_state.torque_enable       = read_data["torque_enable"];
  dynamixel_state.present_position    = read_data["present_position"];
  dynamixel_state.goal_position       = read_data["goal_position"];
  dynamixel_state.moving              = read_data["moving"];

  if (dynamixel_driver->getProtocolVersion() == 2.0)
  {
    if (dynamixel_driver->dynamixel_->model_name_.find("XM") != std::string::npos)
    {
      dynamixel_state.goal_current    = read_data["goal_current"];
      dynamixel_state.present_current = read_data["present_current"];
    }

    dynamixel_state.goal_velocity    = read_data["goal_velocity"];
    dynamixel_state.present_velocity = read_data["present_velocity"];
  }
  else
  {
    dynamixel_state.goal_velocity    = read_data["moving_speed"];
    dynamixel_state.present_velocity = read_data["present_speed"];
  }

  if (motor == PAN)
  {
    pan_state_pub_.publish(dynamixel_state);
  }
  else if (motor == TILT)
  {
    tilt_state_pub_.publish(dynamixel_state);
  }
}

uint32_t MultiPort::convertRadian2Value(uint8_t motor, float radian)
{
  uint32_t value = 0;
  dynamixel_driver::DynamixelDriver* dynamixel_driver;

  if (motor == PAN)
    dynamixel_driver = pan_driver_;
  else if (motor == TILT)
    dynamixel_driver = tilt_driver_;

  if (radian > 0)
  {
    if (dynamixel_driver->dynamixel_->value_of_max_radian_position_ <= dynamixel_driver->dynamixel_->value_of_0_radian_position_)
      return dynamixel_driver->dynamixel_->value_of_max_radian_position_;

    value = (radian * (dynamixel_driver->dynamixel_->value_of_max_radian_position_ - dynamixel_driver->dynamixel_->value_of_0_radian_position_) / dynamixel_driver->dynamixel_->max_radian_)
                + dynamixel_driver->dynamixel_->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (dynamixel_driver->dynamixel_->value_of_min_radian_position_ >= dynamixel_driver->dynamixel_->value_of_0_radian_position_)
      return dynamixel_driver->dynamixel_->value_of_min_radian_position_;

    value = (radian * (dynamixel_driver->dynamixel_->value_of_min_radian_position_ - dynamixel_driver->dynamixel_->value_of_0_radian_position_) / dynamixel_driver->dynamixel_->min_radian_)
                + dynamixel_driver->dynamixel_->value_of_0_radian_position_;
  }
  else
    value = dynamixel_driver->dynamixel_->value_of_0_radian_position_;

//  if (value > multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_)
//    return multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_;
//  else if (value < multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_)
//    return multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_;

  return value;
}

bool MultiPort::controlLoop()
{
  dynamixelStatePublish(PAN);
  dynamixelStatePublish(TILT);
}

bool MultiPort::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
  uint32_t pan_pos = 0;
  uint32_t tilt_pos = 0;

  if (req.unit == "rad")
  {
    pan_pos = convertRadian2Value(PAN, req.pan_pos);
    tilt_pos = convertRadian2Value(TILT, req.tilt_pos);
  }
  else if (req.unit == "raw")
  {
    pan_pos = req.pan_pos;
    tilt_pos = req.tilt_pos;
  }
  else
  {
    pan_pos = req.pan_pos;
    tilt_pos = req.tilt_pos;
  }

  setPosition(pan_pos, tilt_pos);

  res.pan_pos = pan_pos;
  res.tilt_pos = tilt_pos;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "multi_port");
  MultiPort multi;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    multi.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
