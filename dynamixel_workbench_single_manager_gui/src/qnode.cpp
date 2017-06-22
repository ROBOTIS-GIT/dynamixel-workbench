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
#include <ros/network.h>

#include <string>
#include <sstream>
#include <std_msgs/String.h>
#include "../include/dynamixel_workbench_single_manager_gui/qnode.hpp"

using namespace qnode;

QNode::QNode(int argc, char** argv )
    :init_argc(argc),
     init_argv(argv),
     row_count_(0)
{}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"single_manager_gui");

  if (!ros::master::check())
      return false;

  ros::start();
  ros::NodeHandle node_handle;

  // Init Service Client
  dynamixel_info_client_    = node_handle.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("dynamixel/info");
  dynamixel_command_client_ = node_handle.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel/command");
  getDynamixelInfo();

  // Init Message Subscriber
  initDynamixelStateSubscriber();

  start();
  return true;
}

bool QNode::sendCommandMsg(std::string cmd, std::string addr, int64_t value)
{
  dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

  set_dynamixel_command.request.command   = cmd;
  set_dynamixel_command.request.addr_name = addr;
  set_dynamixel_command.request.value     = value;

  if (dynamixel_command_client_.call(set_dynamixel_command))
  {
    if (!set_dynamixel_command.response.comm_result)
      return false;
    else
      return true;
  }
}

bool QNode::sendSetIdMsg(uint8_t set_id)
{
  if (sendCommandMsg("addr", "id", set_id))
    return true;
  else
    return false;
}

bool QNode::sendSetBaudrateMsg(int64_t baud_rate)
{
  if (sendCommandMsg("addr", "baud_rate", baud_rate))
    return true;
  else
    return false;
}

bool QNode::sendSetOperatingModeMsg(std::string index, float protocol_version, std::string model_name, int32_t value_of_max_radian_position)
{
  dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

  if (protocol_version == 1.0)
  {
    if (index == "position_control")
    {
      if (sendCommandMsg("addr", "cw_angle_limit", 0) && sendCommandMsg("addr", "ccw_angle_limit", value_of_max_radian_position-1))
        return true;
      else if (!sendCommandMsg("addr", "cw_angle_limit", 0))
        return false;
      else if (!sendCommandMsg("addr", "ccw_angle_limit", value_of_max_radian_position-1))
        return false;
    }
    else if (index == "velocity_control")
    {
      if (sendCommandMsg("addr", "cw_angle_limit", 0) && sendCommandMsg("addr", "ccw_angle_limit", 0))
        return true;
      else if (!sendCommandMsg("addr", "cw_angle_limit", 0))
        return false;
      else if (!sendCommandMsg("addr", "ccw_angle_limit", 0))
        return false;
    }
    else if (index == "extended_position_control")
    {
      if (sendCommandMsg("addr", "cw_angle_limit", value_of_max_radian_position) && sendCommandMsg("addr", "ccw_angle_limit", value_of_max_radian_position))
        return true;
      else if (!sendCommandMsg("addr", "cw_angle_limit", value_of_max_radian_position))
        return false;
      else if (!sendCommandMsg("addr", "ccw_angle_limit", value_of_max_radian_position))
        return false;
    }
  }
  else
  {
    if (model_name.find("XM") != std::string::npos ||
        model_name.find("XH") != std::string::npos)
    {
      if (index == "current_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 0))
          return true;
        else
          return false;
      }
      else if (index == "velocity_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 1))
          return true;
        else
          return false;
      }
      else if (index == "position_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 3))
          return true;
        else
          return false;
      }
      else if (index == "extended_position_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 4))
          return true;
        else
          return false;
      }
      else if (index == "position_control_based_on_current")
      {
        if (sendCommandMsg("addr", "operating_mode", 5))
          return true;
        else
          return false;
      }
      else if (index == "pwm_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 16))
          return true;
        else
          return false;
      }
    }
    else if (model_name.find("XL") != std::string::npos)
    {
      if (index == "velocity_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 1))
          return true;
        else
          return false;
      }
      else if (index == "position_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 3))
          return true;
        else
          return false;
      }
      else if (index == "extended_position_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 4))
          return true;
        else
          return false;
      }
      else if (index == "pwm_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 16))
          return true;
        else
          return false;
      }
    }
    else if (model_name.find("PRO") != std::string::npos)
    {
      if (index == "torque_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 0))
          return true;
        else
          return false;
      }
      else if (index == "velocity_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 1))
          return true;
        else
          return false;
      }
      else if (index == "position_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 3))
          return true;
        else
          return false;
      }
      else if (index == "extended_position_control")
      {
        if (sendCommandMsg("addr", "operating_mode", 4))
          return true;
        else
          return false;
      }
    }
  }
}

bool QNode::sendTorqueMsg(int64_t onoff)
{
  if (sendCommandMsg("addr", "torque_enable", onoff))
    return true;
  else
    return false;
}

bool QNode::sendRebootMsg(void)
{
  if (sendCommandMsg("reboot"))
    return true;
  else
    return false;
}

bool QNode::sendResetMsg(void)
{
  if (sendCommandMsg("factory_reset"))
  {
    getDynamixelInfo();
    return true;
  }
  else
  {
    return false;
  }
}

bool QNode::setPositionZeroMsg(int32_t zero_position)
{
  if (sendCommandMsg("addr", "goal_position", zero_position))
    return true;
  else
    return false;
}

bool QNode::sendAddressValueMsg(std::string addr_name, int64_t value)
{
  if (sendCommandMsg("addr", addr_name, value))
    return true;
  else
    return false;
}

void QNode::getDynamixelInfo()
{
  dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;

  if (dynamixel_info_client_.call(get_dynamixel_info))
  {
    dynamixel_info_.load_info.device_name      = get_dynamixel_info.response.dynamixel_info.load_info.device_name;
    dynamixel_info_.load_info.baud_rate        = get_dynamixel_info.response.dynamixel_info.load_info.baud_rate;
    dynamixel_info_.load_info.protocol_version = get_dynamixel_info.response.dynamixel_info.load_info.protocol_version;

    dynamixel_info_.model_id         = get_dynamixel_info.response.dynamixel_info.model_id;
    dynamixel_info_.model_name       = get_dynamixel_info.response.dynamixel_info.model_name;
    dynamixel_info_.model_number     = get_dynamixel_info.response.dynamixel_info.model_number;

    Q_EMIT updateDynamixelInfo(dynamixel_info_);
  }
}

void QNode::initDynamixelStateSubscriber()
{
  ros::NodeHandle node_handle;

  if (dynamixel_info_.model_name.find("AX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + dynamixel_info_.model_name + "_state", 10, &QNode::AXStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("RX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + dynamixel_info_.model_name + "_state", 10, &QNode::RXStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("MX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + dynamixel_info_.model_name + "_state", 10, &QNode::MXStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("EX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + dynamixel_info_.model_name + "_state", 10, &QNode::EXStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("XL") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + dynamixel_info_.model_name + "_state", 10, &QNode::XLStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("XM") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + dynamixel_info_.model_name + "_state", 10, &QNode::XMStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("XH") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + dynamixel_info_.model_name + "_state", 10, &QNode::XHStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("PRO") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + dynamixel_info_.model_name + "_state", 10, &QNode::PROStatusMsgCallback, this);
  }
}

void QNode::run()
{
  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "ROS shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();
}

void QNode::log(const std::string &msg, int64_t sender)
{
    if(logging_model_.rowCount() == row_count_)
        logging_model_.insertRows(row_count_, 1);

    std::stringstream logging_model_msg;

    logging_model_msg << msg << sender;

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model_.setData(logging_model_.index(row_count_), new_row);

    row_count_++;
}

void QNode::log(const std::string &msg)
{
    if(logging_model_.rowCount() == row_count_)
        logging_model_.insertRows(row_count_, 1);

    std::stringstream logging_model_msg;

    logging_model_msg << msg;

    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model_.setData(logging_model_.index(row_count_), new_row);

    row_count_++;
}


void QNode::AXStatusMsgCallback(const dynamixel_workbench_msgs::AX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_time: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("the_highest_limit_temperature: "), msg->the_highest_limit_temperature);
  log(std::string("the_lowest_limit_voltage: "), msg->the_lowest_limit_voltage);
  log(std::string("the_highest_limit_voltage: "), msg->the_highest_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("alarm_led: "), msg->alarm_led);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("cw_compliance_margin: "), msg->cw_compliance_margin);
  log(std::string("ccw_compliance_margin: "), msg->ccw_compliance_margin);
  log(std::string("cw_compliance_slope: "), msg->cw_compliance_slope);
  log(std::string("ccw_compliance_slope: "), msg->ccw_compliance_slope);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("moving_speed: "), msg->moving_speed);
  log(std::string("torque_limit: "), msg->torque_limit);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("present_speed: "), msg->present_speed);
  log(std::string("present_load: "), msg->present_load);
  log(std::string("present_voltage: "), msg->present_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);
  log(std::string("registered: "), msg->registered);
  log(std::string("moving: "), msg->moving);
  log(std::string("lock: "), msg->lock);
  log(std::string("punch: "), msg->punch);

  row_count_ = 0;
}

void QNode::RXStatusMsgCallback(const dynamixel_workbench_msgs::RX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_time: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("the_highest_limit_temperature: "), msg->the_highest_limit_temperature);
  log(std::string("the_lowest_limit_voltage: "), msg->the_lowest_limit_voltage);
  log(std::string("the_highest_limit_voltage: "), msg->the_highest_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("alarm_led: "), msg->alarm_led);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("cw_compliance_margin: "), msg->cw_compliance_margin);
  log(std::string("ccw_compliance_margin: "), msg->ccw_compliance_margin);
  log(std::string("cw_compliance_slope: "), msg->cw_compliance_slope);
  log(std::string("ccw_compliance_slope: "), msg->ccw_compliance_slope);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("moving_speed: "), msg->moving_speed);
  log(std::string("torque_limit: "), msg->torque_limit);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("present_speed: "), msg->present_speed);
  log(std::string("present_load: "), msg->present_load);
  log(std::string("present_voltage: "), msg->present_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);
  log(std::string("registered: "), msg->registered);
  log(std::string("moving: "), msg->moving);
  log(std::string("lock: "), msg->lock);
  log(std::string("punch: "), msg->punch);

  row_count_ = 0;
}
void QNode::MXStatusMsgCallback(const dynamixel_workbench_msgs::MX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_time: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("drive_mode: "), msg->drive_mode);
  log(std::string("the_highest_limit_temperature: "), msg->the_highest_limit_temperature);
  log(std::string("the_lowest_limit_voltage: "), msg->the_lowest_limit_voltage);
  log(std::string("the_highest_limit_voltage: "), msg->the_highest_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("alarm_led: "), msg->alarm_led);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);
  log(std::string("multi_turn_offset: "), msg->multi_turn_offset);
  log(std::string("resolution_divider: "), msg->resolution_divider);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("d_gain: "), msg->d_gain);
  log(std::string("i_gain: "), msg->i_gain);
  log(std::string("p_gain: "), msg->p_gain);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("moving_speed: "), msg->moving_speed);
  log(std::string("torque_limit: "), msg->torque_limit);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("present_speed: "), msg->present_speed);
  log(std::string("present_load: "), msg->present_load);
  log(std::string("present_voltage: "), msg->present_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);
  log(std::string("registered: "), msg->registered);
  log(std::string("moving: "), msg->moving);
  log(std::string("lock: "), msg->lock);
  log(std::string("punch: "), msg->punch);
  log(std::string("current: "), msg->current);
  log(std::string("torque_control_mode_enable: "), msg->torque_control_mode_enable);
  log(std::string("goal_torque: "), msg->goal_torque);
  log(std::string("goal_acceleration: "), msg->goal_acceleration);

  row_count_ = 0;
}

void QNode::EXStatusMsgCallback(const dynamixel_workbench_msgs::EX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_time: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("drive_mode: "), msg->drive_mode);
  log(std::string("the_highest_limit_temperature: "), msg->the_highest_limit_temperature);
  log(std::string("the_lowest_limit_voltage: "), msg->the_lowest_limit_voltage);
  log(std::string("the_highest_limit_voltage: "), msg->the_highest_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("alarm_led: "), msg->alarm_led);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("cw_compliance_margin: "), msg->cw_compliance_margin);
  log(std::string("ccw_compliance_margin: "), msg->ccw_compliance_margin);
  log(std::string("cw_compliance_slope: "), msg->cw_compliance_slope);
  log(std::string("ccw_compliance_slope: "), msg->ccw_compliance_slope);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("moving_speed: "), msg->moving_speed);
  log(std::string("torque_limit: "), msg->torque_limit);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("present_velocity: "), msg->present_velocity);
  log(std::string("present_load: "), msg->present_load);
  log(std::string("present_voltage: "), msg->present_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);
  log(std::string("registered: "), msg->registered);
  log(std::string("moving: "), msg->moving);
  log(std::string("lock: "), msg->lock);
  log(std::string("punch: "), msg->punch);
  log(std::string("sensed_current: "), msg->sensed_current);

  row_count_ = 0;
}

void QNode::XLStatusMsgCallback(const dynamixel_workbench_msgs::XL::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_time: "), msg->return_delay_time);
  log(std::string("drive_mode: "), msg->drive_mode);
  log(std::string("operating_mode: "), msg->operating_mode);
  log(std::string("secondary_id: "), msg->secondary_id);
  log(std::string("protocol_version: "), msg->protocol_version);
  log(std::string("homing_offset: "), msg->homing_offset);
  log(std::string("moving_threshold: "), msg->moving_threshold);
  log(std::string("temperature_limit: "), msg->temperature_limit);
  log(std::string("max_voltage_limit: "), msg->max_voltage_limit);
  log(std::string("min_voltage_limit: "), msg->min_voltage_limit);
  log(std::string("pwm_limit: "), msg->pwm_limit);
  log(std::string("acceleration_limit: "), msg->acceleration_limit);
  log(std::string("velocity_limit: "), msg->velocity_limit);
  log(std::string("max_position_limit: "), msg->max_position_limit);
  log(std::string("min_position_limit: "), msg->min_position_limit);
  log(std::string("shutdown: "), msg->shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("registered_instruction: "), msg->registered_instruction);
  log(std::string("hardware_error_status: "), msg->hardware_error_status);
  log(std::string("velocity_i_gain: "), msg->velocity_i_gain);
  log(std::string("velocity_p_gain: "), msg->velocity_p_gain);
  log(std::string("position_d_gain: "), msg->position_d_gain);
  log(std::string("position_i_gain: "), msg->position_i_gain);
  log(std::string("position_p_gain: "), msg->position_p_gain);
  log(std::string("feedforward_2nd_gain: "), msg->feedforward_2nd_gain);
  log(std::string("feedforward_1st_gain: "), msg->feedforward_1st_gain);
  log(std::string("bus_watchdog: "), msg->bus_watchdog);
  log(std::string("goal_pwm: "), msg->goal_pwm);
  log(std::string("goal_velocity: "), msg->goal_velocity);
  log(std::string("profile_acceleration: "), msg->profile_acceleration);
  log(std::string("profile_velocity: "), msg->profile_velocity);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("realtime_tick: "), msg->realtime_tick);
  log(std::string("moving: "), msg->moving);
  log(std::string("moving_status: "), msg->moving_status);
  log(std::string("present_pwm: "), msg->present_pwm);
  log(std::string("present_load: "), msg->present_load);
  log(std::string("present_velocity: "), msg->present_velocity);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("velocity_trajectory: "), msg->velocity_trajectory);
  log(std::string("position_trajectory: "), msg->position_trajectory);
  log(std::string("present_input_voltage: "), msg->present_input_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);

  row_count_ = 0;
}

void QNode::XMStatusMsgCallback(const dynamixel_workbench_msgs::XM::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_time: "), msg->return_delay_time);
  log(std::string("drive_mode: "), msg->drive_mode);
  log(std::string("operating_mode: "), msg->operating_mode);
  log(std::string("protocol_version: "), msg->protocol_version);
  log(std::string("homing_offset: "), msg->homing_offset);
  log(std::string("moving_threshold: "), msg->moving_threshold);
  log(std::string("temperature_limit: "), msg->temperature_limit);
  log(std::string("max_voltage_limit: "), msg->max_voltage_limit);
  log(std::string("min_voltage_limit: "), msg->min_voltage_limit);
  log(std::string("pwm_limit: "), msg->pwm_limit);
  log(std::string("current_limit: "), msg->current_limit);
  log(std::string("acceleration_limit: "), msg->acceleration_limit);
  log(std::string("velocity_limit: "), msg->velocity_limit);
  log(std::string("max_position_limit: "), msg->max_position_limit);
  log(std::string("min_position_limit: "), msg->min_position_limit);
  log(std::string("shutdown: "), msg->shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("registered_instruction: "), msg->registered_instruction);
  log(std::string("hardware_error_status: "), msg->hardware_error_status);
  log(std::string("velocity_i_gain: "), msg->velocity_i_gain);
  log(std::string("velocity_p_gain: "), msg->velocity_p_gain);
  log(std::string("position_d_gain: "), msg->position_d_gain);
  log(std::string("position_i_gain: "), msg->position_i_gain);
  log(std::string("position_p_gain: "), msg->position_p_gain);
  log(std::string("feedforward_2nd_gain: "), msg->feedforward_2nd_gain);
  log(std::string("feedforward_1st_gain: "), msg->feedforward_1st_gain);
  log(std::string("goal_pwm: "), msg->goal_pwm);
  log(std::string("goal_current: "), msg->goal_current);
  log(std::string("goal_velocity: "), msg->goal_velocity);
  log(std::string("profile_acceleration: "), msg->profile_acceleration);
  log(std::string("profile_velocity: "), msg->profile_velocity);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("realtime_tick: "), msg->realtime_tick);
  log(std::string("moving: "), msg->moving);
  log(std::string("moving_status: "), msg->moving_status);
  log(std::string("present_pwm: "), msg->present_pwm);
  log(std::string("present_current: "), msg->present_current);
  log(std::string("present_velocity: "), msg->present_velocity);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("velocity_trajectory: "), msg->velocity_trajectory);
  log(std::string("position_trajectory: "), msg->position_trajectory);
  log(std::string("present_input_voltage: "), msg->present_input_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);

  row_count_ = 0;
}

void QNode::XHStatusMsgCallback(const dynamixel_workbench_msgs::XH::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_time: "), msg->return_delay_time);
  log(std::string("drive_mode: "), msg->drive_mode);
  log(std::string("operating_mode: "), msg->operating_mode);
  log(std::string("protocol_version: "), msg->protocol_version);
  log(std::string("homing_offset: "), msg->homing_offset);
  log(std::string("moving_threshold: "), msg->moving_threshold);
  log(std::string("temperature_limit: "), msg->temperature_limit);
  log(std::string("max_voltage_limit: "), msg->max_voltage_limit);
  log(std::string("min_voltage_limit: "), msg->min_voltage_limit);
  log(std::string("pwm_limit: "), msg->pwm_limit);
  log(std::string("current_limit: "), msg->current_limit);
  log(std::string("acceleration_limit: "), msg->acceleration_limit);
  log(std::string("velocity_limit: "), msg->velocity_limit);
  log(std::string("max_position_limit: "), msg->max_position_limit);
  log(std::string("min_position_limit: "), msg->min_position_limit);
  log(std::string("shutdown: "), msg->shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("registered_instruction: "), msg->registered_instruction);
  log(std::string("hardware_error_status: "), msg->hardware_error_status);
  log(std::string("velocity_i_gain: "), msg->velocity_i_gain);
  log(std::string("velocity_p_gain: "), msg->velocity_p_gain);
  log(std::string("position_d_gain: "), msg->position_d_gain);
  log(std::string("position_i_gain: "), msg->position_i_gain);
  log(std::string("position_p_gain: "), msg->position_p_gain);
  log(std::string("feedforward_2nd_gain: "), msg->feedforward_2nd_gain);
  log(std::string("feedforward_1st_gain: "), msg->feedforward_1st_gain);
  log(std::string("goal_pwm: "), msg->goal_pwm);
  log(std::string("goal_current: "), msg->goal_current);
  log(std::string("goal_velocity: "), msg->goal_velocity);
  log(std::string("profile_acceleration: "), msg->profile_acceleration);
  log(std::string("profile_velocity: "), msg->profile_velocity);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("realtime_tick: "), msg->realtime_tick);
  log(std::string("moving: "), msg->moving);
  log(std::string("moving_status: "), msg->moving_status);
  log(std::string("present_pwm: "), msg->present_pwm);
  log(std::string("present_current: "), msg->present_current);
  log(std::string("present_velocity: "), msg->present_velocity);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("velocity_trajectory: "), msg->velocity_trajectory);
  log(std::string("position_trajectory: "), msg->position_trajectory);
  log(std::string("present_input_voltage: "), msg->present_input_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);

  row_count_ = 0;
}

void QNode::PROStatusMsgCallback(const dynamixel_workbench_msgs::PRO::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_time: "), msg->return_delay_time);
  log(std::string("operating_mode: "), msg->operating_mode);
  log(std::string("homing_offset: "), msg->homing_offset);
  log(std::string("moving_threshold: "), msg->moving_threshold);
  log(std::string("temperature_limit: "), msg->temperature_limit);
  log(std::string("max_voltage_limit: "), msg->max_voltage_limit);
  log(std::string("min_voltage_limit: "), msg->min_voltage_limit);
  log(std::string("acceleration_limit: "), msg->acceleration_limit);
  log(std::string("torque_limit: "), msg->torque_limit);
  log(std::string("velocity_limit: "), msg->velocity_limit);
  log(std::string("max_position_limit: "), msg->max_position_limit);
  log(std::string("min_position_limit: "), msg->min_position_limit);
  log(std::string("external_port_mod_1: "), msg->external_port_mod_1);
  log(std::string("external_port_mod_2: "), msg->external_port_mod_2);
  log(std::string("external_port_mod_3: "), msg->external_port_mod_3);
  log(std::string("external_port_mod_4: "), msg->external_port_mod_4);
  log(std::string("shutdown: "), msg->shutdown);
  log(std::string("indirect_address_1: "), msg->indirect_address_1);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led_red: "), msg->led_red);
  log(std::string("led_green: "), msg->led_green);
  log(std::string("led_blue: "), msg->led_blue);
  log(std::string("velocity_i_gain: "), msg->velocity_i_gain);
  log(std::string("velocity_p_gain: "), msg->velocity_p_gain);
  log(std::string("position_p_gain: "), msg->position_p_gain);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("goal_velocity: "), msg->goal_velocity);
  log(std::string("goal_torque: "), msg->goal_torque);
  log(std::string("goal_acceleration: "), msg->goal_acceleration);
  log(std::string("moving: "), msg->moving);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("present_velocity: "), msg->present_velocity);
  log(std::string("present_current: "), msg->present_current);
  log(std::string("present_input_voltage: "), msg->present_input_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);
  log(std::string("external_port_data_1: "), msg->external_port_data_1);
  log(std::string("external_port_data_2: "), msg->external_port_data_2);
  log(std::string("external_port_data_3: "), msg->external_port_data_3);
  log(std::string("external_port_data_4: "), msg->external_port_data_4);
  log(std::string("indirect_data_1: "), msg->indirect_data_1);
  log(std::string("registered_instruction: "), msg->registered_instruction);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("hardware_error_status: "), msg->hardware_error_status);

  row_count_ = 0;
}
