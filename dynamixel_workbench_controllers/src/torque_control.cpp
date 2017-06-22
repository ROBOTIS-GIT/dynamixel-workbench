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

#include "dynamixel_workbench_controllers/torque_control.h"

using namespace torque_control;

TorqueControl::TorqueControl()
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

  if (!multi_driver_->initSyncWrite())
    ROS_ERROR("Init SyncWrite Failed!");

  if (!multi_driver_->initSyncRead())
    ROS_ERROR("Init SyncRead Failed!");

  writeValue_ = new WriteValue;
  motorPos_   = new MotorPos;

  motorPos_->des_pos.clear();
  motorPos_->des_pos.push_back(2048);
  motorPos_->des_pos.push_back(2048);

  setTorque(true);

  initDynamixelStatePublisher();
  initDynamixelInfoServer();
}

TorqueControl::~TorqueControl()
{
  setTorque(false);

  ros::shutdown();
}

bool TorqueControl::loadDynamixel()
{
  bool ret = false;

  dynamixel_driver::DynamixelInfo *pan_info = new dynamixel_driver::DynamixelInfo;

  pan_info->lode_info.device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  pan_info->lode_info.baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  pan_info->lode_info.protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  pan_info->model_id                   = node_handle_.param<int>("pan_id", 1);

  dynamixel_info_.push_back(pan_info);

  dynamixel_driver::DynamixelInfo *tilt_info = new dynamixel_driver::DynamixelInfo;

  tilt_info->lode_info.device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  tilt_info->lode_info.baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  tilt_info->lode_info.protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  tilt_info->model_id                   = node_handle_.param<int>("tilt_id", 1);

  dynamixel_info_.push_back(tilt_info);

  node_handle_priv_.getParam("p_gain", p_gain_);
  node_handle_priv_.getParam("d_gain", d_gain_);

  multi_driver_ = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info_[MOTOR]->lode_info.device_name,
                                                                   dynamixel_info_[MOTOR]->lode_info.baud_rate,
                                                                   dynamixel_info_[MOTOR]->lode_info.protocol_version);

 ret =  multi_driver_->loadDynamixel(dynamixel_info_);

 return ret;
}

bool TorqueControl::setTorque(bool onoff)
{
  writeValue_->torque.clear();
  writeValue_->torque.push_back(onoff);
  writeValue_->torque.push_back(onoff);

  if (!multi_driver_->syncWriteTorque(writeValue_->torque))
  {
    ROS_ERROR("SyncWrite Torque Failed!");
    return false;
  }

  return true;
}

bool TorqueControl::setCurrent(int16_t pan_cur, int16_t tilt_cur)
{
  writeValue_->current.clear();
  writeValue_->current.push_back(pan_cur);
  writeValue_->current.push_back(tilt_cur);

  if (!multi_driver_->syncWriteCurrent(writeValue_->current))
  {
    ROS_ERROR("SyncWrite Current Failed!");
    return false;
  }

  return true;
}

bool TorqueControl::checkLoadDynamixel()
{
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("  dynamixel_workbench controller; torque control example(Pan & Tilt)   ");
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("PAN MOTOR INFO");
  ROS_INFO("ID    : %d", dynamixel_info_[PAN]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[PAN]->model_name.c_str());
  ROS_INFO(" ");
  ROS_INFO("TILT MOTOR INFO");
  ROS_INFO("ID    : %d", dynamixel_info_[TILT]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[TILT]->model_name.c_str());
  ROS_INFO("-----------------------------------------------------------------------");
}

bool TorqueControl::initDynamixelStatePublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("/torque_control/dynamixel_state", 10);
}

bool TorqueControl::initDynamixelInfoServer()
{
  joint_command_server = node_handle_.advertiseService("/joint_command", &TorqueControl::jointCommandMsgCallback, this);

}

bool TorqueControl::readDynamixelState()
{
  multi_driver_->readMultiRegister("torque_enable");

  multi_driver_->readMultiRegister("present_position");

  multi_driver_->readMultiRegister("goal_position");
  multi_driver_->readMultiRegister("moving");

  if (multi_driver_->getProtocolVersion() == 2.0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->model_name_.find("XM") != std::string::npos)
    {
      multi_driver_->readMultiRegister("goal_current");

      multi_driver_->readMultiRegister("present_current");
    }
    multi_driver_->readMultiRegister("goal_velocity");
    multi_driver_->readMultiRegister("present_velocity");
  }
  else
  {
    multi_driver_->readMultiRegister("moving_speed");
    multi_driver_->readMultiRegister("present_speed");
  }
}

bool TorqueControl::dynamixelStatePublish()
{
  readDynamixelState();

  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[multi_driver_->multi_dynamixel_.size()];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_driver_->multi_dynamixel_.size(); ++num)
  {
    dynamixel_state[num].model_name          = multi_driver_->multi_dynamixel_[num]->model_name_;
    dynamixel_state[num].id                  = multi_driver_->multi_dynamixel_[num]->id_;
    dynamixel_state[num].torque_enable       = multi_driver_->read_value_["torque_enable"]      ->at(num);
    dynamixel_state[num].present_position    = multi_driver_->read_value_["present_position"]   ->at(num);
    dynamixel_state[num].goal_position       = multi_driver_->read_value_["goal_position"]      ->at(num);
    dynamixel_state[num].moving              = multi_driver_->read_value_["moving"]             ->at(num);

    if (multi_driver_->getProtocolVersion() == 2.0)
    {
      if (multi_driver_->multi_dynamixel_[MOTOR]->model_name_.find("XM") != std::string::npos)
      {
        dynamixel_state[num].goal_current    = multi_driver_->read_value_["goal_current"]   ->at(num);
        dynamixel_state[num].present_current = multi_driver_->read_value_["present_current"]->at(num);
      }

      dynamixel_state[num].goal_velocity    = multi_driver_->read_value_["goal_velocity"]->at(num);
      dynamixel_state[num].present_velocity = multi_driver_->read_value_["present_velocity"]->at(num);
    }
    else
    {
      dynamixel_state[num].goal_velocity    = multi_driver_->read_value_["moving_speed"]->at(num);
      dynamixel_state[num].present_velocity = multi_driver_->read_value_["present_speed"]->at(num);
    }

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[num]);
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);

}

float TorqueControl::convertValue2Torque(int16_t value)
{
  return (float) value / multi_driver_->multi_dynamixel_[MOTOR]->torque_to_current_value_ratio_;
}

int16_t TorqueControl::convertTorque2Value(float torque)
{
  return (int16_t) (torque * multi_driver_->multi_dynamixel_[MOTOR]->torque_to_current_value_ratio_);
}

uint32_t TorqueControl::convertRadian2Value(float radian)
{
  uint32_t value = 0;

  if (radian > 0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ <= multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
      return multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_;

    value = (radian * (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) / multi_driver_->multi_dynamixel_[MOTOR]->max_radian_)
                + multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ >= multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
      return multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_;

    value = (radian * (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) / multi_driver_->multi_dynamixel_[MOTOR]->min_radian_)
                + multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;
  }
  else
    value = multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;

//  if (value > multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_)
//    return multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_;
//  else if (value < multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_)
//    return multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_;

  return value;
}

float TorqueControl::convertValue2Radian(uint32_t value)
{
  float radian = 0.0;

  if (value > multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->max_radian_ <= 0)
      return multi_driver_->multi_dynamixel_[MOTOR]->max_radian_;

    radian = (float) (value - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) * multi_driver_->multi_dynamixel_[MOTOR]->max_radian_
               / (float) (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_);
  }
  else if (value < multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->min_radian_ >= 0)
      return multi_driver_->multi_dynamixel_[MOTOR]->min_radian_;

    radian = (float) (value - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) * multi_driver_->multi_dynamixel_[MOTOR]->min_radian_
               / (float) (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_);
  }

//  if (radian > dynamixel_[PAN_TILT_MOTOR]->max_radian_)
//    return dynamixel_[PAN_TILT_MOTOR]->max_radian_;
//  else if (radian < dynamixel_[PAN_TILT_MOTOR]->min_radian_)
//    return dynamixel_[PAN_TILT_MOTOR]->min_radian_;

  return radian;
}

bool TorqueControl::controlLoop()
{
  dynamixelStatePublish();
  gravityCompensation();
}

bool TorqueControl::gravityCompensation()
{
  const float tilt_motor_mass = 0.082;
  const float gravity         = 9.8;
  const float link_length     = 0.018;

  int32_t error[2] = {0, 0};
  static int32_t pre_error[2] = {0, 0};
  float torque[2] = {0.0, 0.0};

  node_handle_priv_.getParam("p_gain", p_gain_);
  node_handle_priv_.getParam("d_gain", d_gain_);

  motorPos_->cur_pos.clear();
  if (!multi_driver_->syncReadPosition(motorPos_->cur_pos))
    ROS_ERROR("Sync Read Failed!");

  error[PAN]  = motorPos_->des_pos.at(PAN)  - motorPos_->cur_pos.at(PAN);
  error[TILT] = motorPos_->des_pos.at(TILT) - motorPos_->cur_pos.at(TILT);

  torque[PAN]  = p_gain_ * error[PAN] +
                 d_gain_ * ((error[PAN] - pre_error[PAN]) / 0.004);
  torque[TILT] = p_gain_ * error[TILT] +
                 d_gain_ * ((error[TILT] - pre_error[TILT]) / 0.004) +
                 tilt_motor_mass * gravity * link_length * cos(convertValue2Radian(motorPos_->cur_pos.at(TILT)));

  setCurrent(convertTorque2Value(torque[PAN]), convertTorque2Value(torque[TILT]));

  pre_error[PAN]  = error[PAN];
  pre_error[TILT] = error[TILT];
}

bool TorqueControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
  motorPos_->des_pos.clear();

  if (req.unit == "rad")
  {
    motorPos_->des_pos.push_back(convertRadian2Value(req.pan_pos));
    motorPos_->des_pos.push_back(convertRadian2Value(req.tilt_pos));
  }
  else if (req.unit == "raw")
  {
    motorPos_->des_pos.push_back(req.pan_pos);
    motorPos_->des_pos.push_back(req.tilt_pos);
  }
  else
  {
    motorPos_->des_pos.push_back(req.pan_pos);
    motorPos_->des_pos.push_back(req.tilt_pos);
  }

  res.pan_pos  = motorPos_->des_pos.at(PAN);
  res.tilt_pos = motorPos_->des_pos.at(TILT);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "torque_control");
  TorqueControl torque_ctrl;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    torque_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
