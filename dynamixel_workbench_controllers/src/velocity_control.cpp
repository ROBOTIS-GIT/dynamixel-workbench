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

#include "dynamixel_workbench_controllers/velocity_control.h"

using namespace velocity_control;

VelocityControl::VelocityControl()
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

  multi_driver_->initSyncWrite();

  writeValue_ = new WriteValue;

  writeValue_->torque.push_back(true);
  writeValue_->torque.push_back(true);

  multi_driver_->syncWriteTorque(writeValue_->torque);

  initDynamixelStatePublisher();
  initDynamixelInfoServer();
}

VelocityControl::~VelocityControl()
{
  writeValue_->torque.clear();
  writeValue_->torque.push_back(false);
  writeValue_->torque.push_back(false);
  multi_driver_->syncWriteTorque(writeValue_->torque);

  ros::shutdown();
}

bool VelocityControl::loadDynamixel()
{
  bool ret = false;

  dynamixel_driver::DynamixelInfo *left_info = new dynamixel_driver::DynamixelInfo;

  left_info->lode_info.device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  left_info->lode_info.baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  left_info->lode_info.protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  left_info->model_name                 = node_handle_.param<std::string>("model_name", "XM430_W210");

  left_info->model_id                   = node_handle_.param<int>("left_id", 1);

  dynamixel_info_.push_back(left_info);

  dynamixel_driver::DynamixelInfo *right_info = new dynamixel_driver::DynamixelInfo;

  right_info->lode_info.device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  right_info->lode_info.baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  right_info->lode_info.protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  right_info->model_name                 = node_handle_.param<std::string>("model_name", "XM430_W210");

  right_info->model_id                   = node_handle_.param<int>("right_id", 1);

  dynamixel_info_.push_back(right_info);

  multi_driver_ = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info_[MOTOR]->lode_info.device_name,
                                                                   dynamixel_info_[MOTOR]->lode_info.baud_rate,
                                                                   dynamixel_info_[MOTOR]->lode_info.protocol_version);

 ret =  multi_driver_->loadDynamixel(dynamixel_info_);

 return ret;
}

bool VelocityControl::setTorque(bool onoff)
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

bool VelocityControl::setVelocity(int32_t left_vel, int32_t right_vel)
{
  writeValue_->vel.clear();
  writeValue_->vel.push_back(left_vel);
  writeValue_->vel.push_back(right_vel);

  multi_driver_->syncWriteVelocity(writeValue_->vel);
}

bool VelocityControl::setMovingSpeed(uint16_t left_spd, uint16_t right_spd)
{
  writeValue_->spd.clear();
  writeValue_->spd.push_back(left_spd);
  writeValue_->spd.push_back(right_spd);

  multi_driver_->syncWriteMovingSpeed(writeValue_->spd);
}

bool VelocityControl::checkLoadDynamixel()
{
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("  dynamixel_workbench controller; velocity control example             ");
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("PAN MOTOR INFO");
  ROS_INFO("ID    : %d", dynamixel_info_[LEFT]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[LEFT]->model_name.c_str());
  ROS_INFO(" ");
  ROS_INFO("TILT MOTOR INFO");
  ROS_INFO("ID    : %d", dynamixel_info_[RIGHT]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[RIGHT]->model_name.c_str());
  ROS_INFO("-----------------------------------------------------------------------");
}

bool VelocityControl::initDynamixelStatePublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("/velocity_control/dynamixel_state", 10);
}

bool VelocityControl::initDynamixelInfoServer()
{
  wheel_command_server = node_handle_.advertiseService("/wheel_command", &VelocityControl::wheelCommandMsgCallback, this);
}

bool VelocityControl::readDynamixelState()
{
  multi_driver_->readMultiRegister("torque_enable");

  multi_driver_->readMultiRegister("present_position");
  multi_driver_->readMultiRegister("goal_position");

  multi_driver_->readMultiRegister("present_velocity");

  multi_driver_->readMultiRegister("moving");

  if (multi_driver_->getProtocolVersion() == 2.0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->model_name_.find("XM") != std::string::npos)
    {
      multi_driver_->readMultiRegister("goal_current");

      multi_driver_->readMultiRegister("present_current");
    }
    else
    {
      multi_driver_->readMultiRegister("goal_velocity");
    }
  }
  else
  {
    multi_driver_->readMultiRegister("moving_speed");
  }
}

bool VelocityControl::dynamixelStatePublish()
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
    dynamixel_state[num].present_velocity    = multi_driver_->read_value_["present_velocity"]   ->at(num);
    dynamixel_state[num].goal_position       = multi_driver_->read_value_["goal_position"]      ->at(num);
    dynamixel_state[num].moving              = multi_driver_->read_value_["moving"]             ->at(num);

    if (multi_driver_->getProtocolVersion() == 2.0)
    {
      if (multi_driver_->multi_dynamixel_[MOTOR]->model_name_.find("XM") != std::string::npos)
      {
        dynamixel_state[num].goal_current    = multi_driver_->read_value_["goal_current"]   ->at(num);
        dynamixel_state[num].present_current = multi_driver_->read_value_["present_current"]->at(num);
      }
      else
      {
        dynamixel_state[num].goal_velocity = multi_driver_->read_value_["goal_velocity"]->at(num);
      }
    }
    else
    {
      dynamixel_state[num].goal_velocity = multi_driver_->read_value_["moving_speed"]->at(num);
    }

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[num]);
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);

}

int32_t VelocityControl::convertVelocity2Value(float velocity)
{
  return (int32_t) (velocity * multi_driver_->multi_dynamixel_[MOTOR]->velocity_to_value_ratio_);
}

bool VelocityControl::controlLoop()
{
  dynamixelStatePublish();
}

bool VelocityControl::wheelCommandMsgCallback(dynamixel_workbench_msgs::WheelCommand::Request &req,
                                              dynamixel_workbench_msgs::WheelCommand::Response &res)
{
  sleep(0.01);

  static float right_vel = 0.0;
  static float left_vel  = 0.0;

  if (req.right_vel == 0.0 && req.left_vel == 0.0)
  {
    right_vel = 0.0;
    left_vel  = 0.0;
  }
  else
  {
    right_vel += req.right_vel;
    left_vel  += req.left_vel;
  }

  if (multi_driver_->getProtocolVersion() == 2.0)
  {
    setVelocity(convertVelocity2Value(left_vel), convertVelocity2Value(right_vel));
  }
  else
  {
    if (right_vel < 0.0 && left_vel < 0.0)
    {
      setMovingSpeed((uint16_t)(convertVelocity2Value(left_vel * (-1))) + 1024, (uint16_t)(convertVelocity2Value(right_vel * (-1))));
    }
    else if (right_vel < 0.0 && left_vel > 0.0)
    {
      setMovingSpeed((uint16_t)(convertVelocity2Value(left_vel)), (uint16_t)(convertVelocity2Value(right_vel)) * (-1));
    }
    else if (right_vel > 0.0 && left_vel < 0.0)
    {
      setMovingSpeed((uint16_t)(convertVelocity2Value(left_vel * (-1))) + 1024, (uint16_t)(convertVelocity2Value(right_vel)) + 1024);
    }
    else
    {
      setMovingSpeed((uint16_t)(convertVelocity2Value(left_vel)), (uint16_t)(convertVelocity2Value(right_vel)) + 1024);
    }
  }

  res.right_vel = right_vel;
  res.left_vel  = left_vel;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "velocity_control");
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
