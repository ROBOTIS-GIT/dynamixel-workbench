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
  if (sendCommandMsg("addr", "ID", set_id))
    return true;
  else
    return false;
}

bool QNode::sendSetBaudrateMsg(int64_t baud_rate)
{
  if (sendCommandMsg("addr", "Baud_Rate", baud_rate))
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
      if (sendCommandMsg("addr", "CW_Angle_Limit", 0) && sendCommandMsg("addr", "CCW_Angle_Limit", value_of_max_radian_position))
        return true;
      else
        return false;
    }
    else if (index == "velocity_control")
    {
      if (sendCommandMsg("addr", "CW_Angle_Limit", 0) && sendCommandMsg("addr", "CCW_Angle_Limit", 0))
        return true;
      else
        return false;
    }
    else if (index == "extended_position_control")
    {
      if (sendCommandMsg("addr", "CW_Angle_Limit", value_of_max_radian_position) && sendCommandMsg("addr", "CCW_Angle_Limit", value_of_max_radian_position))
        return true;
      else
        return false;
    }
  }
  else
  {
    if (model_name.find("MX") != std::string::npos)
    {
      if (model_name.find("MX-28-2") != std::string::npos)
      {
        if (index == "velocity_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 1))
            return true;
          else
            return false;
        }
        else if (index == "position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 3))
            return true;
          else
            return false;
        }
        else if (index == "extended_position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 4))
            return true;
          else
            return false;
        }
        else if (index == "pwm_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 16))
            return true;
          else
            return false;
        }
      }
      else if (model_name.find("MX-64-2") != std::string::npos ||
               model_name.find("MX-106-2") != std::string::npos )
      {
        if (index == "current_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 0))
            return true;
          else
            return false;
        }
        else if (index == "velocity_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 1))
            return true;
          else
            return false;
        }
        else if (index == "position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 3))
            return true;
          else
            return false;
        }
        else if (index == "extended_position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 4))
            return true;
          else
            return false;
        }
        else if (index == "current_based_position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 5))
            return true;
          else
            return false;
        }
        else if (index == "pwm_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 16))
            return true;
          else
            return false;
        }
      }
    }
    if (model_name.find("XL") != std::string::npos)
    {
      if (model_name.find("XL-320") != std::string::npos)
      {
        if (index == "position_control")
        {
          if (sendCommandMsg("addr", "CW_Angle_Limit", 0) && sendCommandMsg("addr", "CCW_Angle_Limit", value_of_max_radian_position))
            return true;
          else
            return false;
        }
        else if (index == "velocity_control")
        {
          if (sendCommandMsg("addr", "CW_Angle_Limit", 0) && sendCommandMsg("addr", "CCW_Angle_Limit", 0))
            return true;
          else
            return false;
        }
      }
      else if (model_name.find("XL430-W250") != std::string::npos)
      {
        if (index == "velocity_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 1))
            return true;
          else
            return false;
        }
        else if (index == "position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 3))
            return true;
          else
            return false;
        }
        else if (index == "extended_position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 4))
            return true;
          else
            return false;
        }
        else if (index == "pwm_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 16))
            return true;
          else
            return false;
        }
      }
    }
    if (model_name.find("XM") != std::string::npos  ||
        model_name.find("XH") != std::string::npos   )
    {
      if (index == "current_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 0))
          return true;
        else
          return false;
      }
      else if (index == "velocity_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 1))
          return true;
        else
          return false;
      }
      else if (index == "position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 3))
          return true;
        else
          return false;
      }
      else if (index == "extended_position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 4))
          return true;
        else
          return false;
      }
      else if (index == "current_based_position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 5))
          return true;
        else
          return false;
      }
      else if (index == "pwm_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 16))
          return true;
        else
          return false;
      }
    }
    else if (model_name.find("PRO") != std::string::npos)
    {
      if (index == "torque_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 0))
          return true;
        else
          return false;
      }
      else if (index == "velocity_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 1))
          return true;
        else
          return false;
      }
      else if (index == "position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 3))
          return true;
        else
          return false;
      }
      else if (index == "extended_position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 4))
          return true;
        else
          return false;
      }
    }
  }
}

bool QNode::sendTorqueMsg(int64_t onoff)
{
  if (sendCommandMsg("addr", "Torque_Enable", onoff))
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
  if (sendCommandMsg("addr", "Goal_Position", zero_position))
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
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("AX"), 10, &QNode::AXStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("RX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("RX"), 10, &QNode::RXStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("MX") != std::string::npos)
  {
    if (dynamixel_info_.model_name.find("MX-28-2") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("MX"), 10, &QNode::MX2StatusMsgCallback, this);
    }
    else if (dynamixel_info_.model_name.find("MX-64-2") != std::string::npos ||
             dynamixel_info_.model_name.find("MX-106-2") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("MX"), 10, &QNode::MX2ExtStatusMsgCallback, this);
    }
    else if (dynamixel_info_.model_name.find("MX-12W") != std::string::npos ||
        dynamixel_info_.model_name.find("MX-28") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("MX"), 10, &QNode::MXStatusMsgCallback, this);
    }
    else if (dynamixel_info_.model_name.find("MX-64") != std::string::npos ||
             dynamixel_info_.model_name.find("MX-106") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("MX"), 10, &QNode::MXExtStatusMsgCallback, this);
    }
  }
  else if (dynamixel_info_.model_name.find("EX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("EX"), 10, &QNode::EXStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("XL") != std::string::npos)
  {
    if (dynamixel_info_.model_name.find("XL-320") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XL"), 10, &QNode::XL320StatusMsgCallback, this);
    }
    else if (dynamixel_info_.model_name.find("XL430-W250") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XL"), 10, &QNode::XLStatusMsgCallback, this);
    }
  }
  else if (dynamixel_info_.model_name.find("XM") != std::string::npos)
  {
    if (dynamixel_info_.model_name.find("XM430-W210") != std::string::npos ||
        dynamixel_info_.model_name.find("XM430-W350") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XM"), 10, &QNode::XMStatusMsgCallback, this);
    }
    else if (dynamixel_info_.model_name.find("XM540-W150") != std::string::npos ||
             dynamixel_info_.model_name.find("XM540-W270") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XM"), 10, &QNode::XMExtStatusMsgCallback, this);
    }
  }
  else if (dynamixel_info_.model_name.find("XH") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XH"), 10, &QNode::XHStatusMsgCallback, this);
  }
  else if (dynamixel_info_.model_name.find("PRO") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("PRO"), 10, &QNode::PROStatusMsgCallback, this);
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
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), msg->CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), msg->CCW_Angle_Limit);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Max_Torque: "), msg->Max_Torque);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Alarm_LED: "), msg->Alarm_LED);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("CW_Compliance_Margin: "), msg->CW_Compliance_Margin);
  log(std::string("CCW_Compliance_Margin: "), msg->CCW_Compliance_Margin);
  log(std::string("CW_Compliance_Slope: "), msg->CW_Compliance_Slope);
  log(std::string("CCW_Compliance_Slope: "), msg->CCW_Compliance_Slope);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Moving_Speed: "), msg->Moving_Speed);
  log(std::string("Torque_Limit: "), msg->Torque_Limit);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Present_Speed: "), msg->Present_Speed);
  log(std::string("Present_Load: "), msg->Present_Load);
  log(std::string("Present_Voltage: "), msg->Present_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);
  log(std::string("Registered: "), msg->Registered);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Lock: "), msg->Lock);
  log(std::string("Punch: "), msg->Punch);

  row_count_ = 0;
}

void QNode::RXStatusMsgCallback(const dynamixel_workbench_msgs::RX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), msg->CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), msg->CCW_Angle_Limit);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Max_Torque: "), msg->Max_Torque);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Alarm_LED: "), msg->Alarm_LED);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("CW_Compliance_Margin: "), msg->CW_Compliance_Margin);
  log(std::string("CCW_Compliance_Margin: "), msg->CCW_Compliance_Margin);
  log(std::string("CW_Compliance_Slope: "), msg->CW_Compliance_Slope);
  log(std::string("CCW_Compliance_Slope: "), msg->CCW_Compliance_Slope);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Moving_Speed: "), msg->Moving_Speed);
  log(std::string("Torque_Limit: "), msg->Torque_Limit);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Present_Speed: "), msg->Present_Speed);
  log(std::string("Present_Load: "), msg->Present_Load);
  log(std::string("Present_Voltage: "), msg->Present_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);
  log(std::string("Registered: "), msg->Registered);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Lock: "), msg->Lock);
  log(std::string("Punch: "), msg->Punch);

  row_count_ = 0;
}

void QNode::MXStatusMsgCallback(const dynamixel_workbench_msgs::MX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), msg->CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), msg->CCW_Angle_Limit);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Max_Torque: "), msg->Max_Torque);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Alarm_LED: "), msg->Alarm_LED);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string("Multi_Turn_Offset: "), msg->Multi_Turn_Offset);
  log(std::string("Resolution_Divider: "), msg->Resolution_Divider);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("D_gain: "), msg->D_gain);
  log(std::string("I_gain: "), msg->I_gain);
  log(std::string("P_gain: "), msg->P_gain);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Moving_Speed: "), msg->Moving_Speed);
  log(std::string("Torque_Limit: "), msg->Torque_Limit);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Present_Speed: "), msg->Present_Speed);
  log(std::string("Present_Load: "), msg->Present_Load);
  log(std::string("Present_Voltage: "), msg->Present_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);
  log(std::string("Registered: "), msg->Registered);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Lock: "), msg->Lock);
  log(std::string("Punch: "), msg->Punch);
  log(std::string("Goal_Acceleration: "), msg->Goal_Acceleration);

  row_count_ = 0;
}

void QNode::MXExtStatusMsgCallback(const dynamixel_workbench_msgs::MXExt::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), msg->CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), msg->CCW_Angle_Limit);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Max_Torque: "), msg->Max_Torque);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Alarm_LED: "), msg->Alarm_LED);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string("Multi_Turn_Offset: "), msg->Multi_Turn_Offset);
  log(std::string("Resolution_Divider: "), msg->Resolution_Divider);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("D_gain: "), msg->D_gain);
  log(std::string("I_gain: "), msg->I_gain);
  log(std::string("P_gain: "), msg->P_gain);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Moving_Speed: "), msg->Moving_Speed);
  log(std::string("Torque_Limit: "), msg->Torque_Limit);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Present_Speed: "), msg->Present_Speed);
  log(std::string("Present_Load: "), msg->Present_Load);
  log(std::string("Present_Voltage: "), msg->Present_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);
  log(std::string("Registered: "), msg->Registered);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Lock: "), msg->Lock);
  log(std::string("Punch: "), msg->Punch);
  log(std::string("Current: "), msg->Current);
  log(std::string("Torque_Control_Mode_Enable: "), msg->Torque_Control_Mode_Enable);
  log(std::string("Goal_Torque: "), msg->Goal_Torque);
  log(std::string("Goal_Acceleration: "), msg->Goal_Acceleration);

  row_count_ = 0;
}

void QNode::MX2StatusMsgCallback(const dynamixel_workbench_msgs::MX2::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("Drive_Mode: "), msg->Drive_Mode);
  log(std::string("Operating_Mode: "), msg->Operating_Mode);
  log(std::string("Secondary_ID: "), msg->Secondary_ID);
  log(std::string("Protocol_Version: "), msg->Protocol_Version);
  log(std::string("Homing_Offset: "), msg->Homing_Offset);
  log(std::string("Moving_Threshold: "), msg->Moving_Threshold);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), msg->PWM_Limit);
  log(std::string("Acceleration_Limit: "), msg->Acceleration_Limit);
  log(std::string("Velocity_Limit: "), msg->Velocity_Limit);
  log(std::string("Max_Position_Limit: "), msg->Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), msg->Min_Position_Limit);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Registered_Instruction: "), msg->Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), msg->Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), msg->Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), msg->Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), msg->Position_D_Gain);
  log(std::string("Position_I_Gain: "), msg->Position_I_Gain);
  log(std::string("Position_P_Gain: "), msg->Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), msg->Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), msg->Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), msg->Bus_Watchdog);
  log(std::string("Goal_PWM: "), msg->Goal_PWM);
  log(std::string("Goal_Velocity: "), msg->Goal_Velocity);
  log(std::string("Profile_Acceleration: "), msg->Profile_Acceleration);
  log(std::string("Profile_Velocity: "), msg->Profile_Velocity);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Realtime_Tick: "), msg->Realtime_Tick);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Moving_Status: "), msg->Moving_Status);
  log(std::string("Present_PWM: "), msg->Present_PWM);
  log(std::string("Present_Load: "), msg->Present_Load);
  log(std::string("Present_Velocity: "), msg->Present_Velocity);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Velocity_Trajectory: "), msg->Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), msg->Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), msg->Present_Input_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);

  row_count_ = 0;
}

void QNode::MX2ExtStatusMsgCallback(const dynamixel_workbench_msgs::MX2Ext::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("Drive_Mode: "), msg->Drive_Mode);
  log(std::string("Operating_Mode: "), msg->Operating_Mode);
  log(std::string("Secondary_ID: "), msg->Secondary_ID);
  log(std::string("Protocol_Version: "), msg->Protocol_Version);
  log(std::string("Homing_Offset: "), msg->Homing_Offset);
  log(std::string("Moving_Threshold: "), msg->Moving_Threshold);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), msg->PWM_Limit);
  log(std::string("Current_Limit: "), msg->Current_Limit);
  log(std::string("Acceleration_Limit: "), msg->Acceleration_Limit);
  log(std::string("Velocity_Limit: "), msg->Velocity_Limit);
  log(std::string("Max_Position_Limit: "), msg->Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), msg->Min_Position_Limit);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Registered_Instruction: "), msg->Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), msg->Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), msg->Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), msg->Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), msg->Position_D_Gain);
  log(std::string("Position_I_Gain: "), msg->Position_I_Gain);
  log(std::string("Position_P_Gain: "), msg->Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), msg->Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), msg->Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), msg->Bus_Watchdog);
  log(std::string("Goal_PWM: "), msg->Goal_PWM);
  log(std::string("Goal_Velocity: "), msg->Goal_Velocity);
  log(std::string("Profile_Acceleration: "), msg->Profile_Acceleration);
  log(std::string("Profile_Velocity: "), msg->Profile_Velocity);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Realtime_Tick: "), msg->Realtime_Tick);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Moving_Status: "), msg->Moving_Status);
  log(std::string("Present_PWM: "), msg->Present_PWM);
  log(std::string("Present_Current: "), msg->Present_Current);
  log(std::string("Present_Velocity: "), msg->Present_Velocity);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Velocity_Trajectory: "), msg->Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), msg->Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), msg->Present_Input_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);

  row_count_ = 0;
}

void QNode::EXStatusMsgCallback(const dynamixel_workbench_msgs::EX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), msg->CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), msg->CCW_Angle_Limit);
  log(std::string("Drive_Mode: "), msg->Drive_Mode);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Max_Torque: "), msg->Max_Torque);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Alarm_LED: "), msg->Alarm_LED);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("CW_Compliance_Margin: "), msg->CW_Compliance_Margin);
  log(std::string("CCW_Compliance_Margin: "), msg->CCW_Compliance_Margin);
  log(std::string("CW_Compliance_Slope: "), msg->CW_Compliance_Slope);
  log(std::string("CCW_Compliance_Slope: "), msg->CCW_Compliance_Slope);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Moving_Speed: "), msg->Moving_Speed);
  log(std::string("Torque_Limit: "), msg->Torque_Limit);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Present_Speed: "), msg->Present_Speed);
  log(std::string("Present_Load: "), msg->Present_Load);
  log(std::string("Present_Voltage: "), msg->Present_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);
  log(std::string("Registered: "), msg->Registered);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Lock: "), msg->Lock);
  log(std::string("Punch: "), msg->Punch);
  log(std::string("Sensored_Current: "), msg->Sensored_Current);

  row_count_ = 0;
}

void QNode::XL320StatusMsgCallback(const dynamixel_workbench_msgs::XL320::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), msg->CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), msg->CCW_Angle_Limit);
  log(std::string("Control_Mode: "), msg->Control_Mode);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Max_Torque: "), msg->Max_Torque);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("D_gain: "), msg->D_gain);
  log(std::string("I_gain: "), msg->I_gain);
  log(std::string("P_gain: "), msg->P_gain);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Moving_Speed: "), msg->Moving_Speed);
  log(std::string("Torque_Limit: "), msg->Torque_Limit);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Present_Speed: "), msg->Present_Speed);
  log(std::string("Present_Load: "), msg->Present_Load);
  log(std::string("Present_Voltage: "), msg->Present_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);
  log(std::string("Registered: "), msg->Registered);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Hardware_Error_Status: "), msg->Hardware_Error_Status);
  log(std::string("Punch: "), msg->Punch);

  row_count_ = 0;
}

void QNode::XLStatusMsgCallback(const dynamixel_workbench_msgs::XL::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("Drive_Mode: "), msg->Drive_Mode);
  log(std::string("Operating_Mode: "), msg->Operating_Mode);
  log(std::string("Secondary_ID: "), msg->Secondary_ID);
  log(std::string("Protocol_Version: "), msg->Protocol_Version);
  log(std::string("Homing_Offset: "), msg->Homing_Offset);
  log(std::string("Moving_Threshold: "), msg->Moving_Threshold);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), msg->PWM_Limit);
  log(std::string("Acceleration_Limit: "), msg->Acceleration_Limit);
  log(std::string("Velocity_Limit: "), msg->Velocity_Limit);
  log(std::string("Max_Position_Limit: "), msg->Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), msg->Min_Position_Limit);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Registered_Instruction: "), msg->Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), msg->Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), msg->Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), msg->Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), msg->Position_D_Gain);
  log(std::string("Position_I_Gain: "), msg->Position_I_Gain);
  log(std::string("Position_P_Gain: "), msg->Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), msg->Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), msg->Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), msg->Bus_Watchdog);
  log(std::string("Goal_PWM: "), msg->Goal_PWM);
  log(std::string("Goal_Velocity: "), msg->Goal_Velocity);
  log(std::string("Profile_Acceleration: "), msg->Profile_Acceleration);
  log(std::string("Profile_Velocity: "), msg->Profile_Velocity);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Realtime_Tick: "), msg->Realtime_Tick);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Moving_Status: "), msg->Moving_Status);
  log(std::string("Present_PWM: "), msg->Present_PWM);
  log(std::string("Present_Load: "), msg->Present_Load);
  log(std::string("Present_Velocity: "), msg->Present_Velocity);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Velocity_Trajectory: "), msg->Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), msg->Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), msg->Present_Input_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);

  row_count_ = 0;
}

void QNode::XMStatusMsgCallback(const dynamixel_workbench_msgs::XM::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("Drive_Mode: "), msg->Drive_Mode);
  log(std::string("Operating_Mode: "), msg->Operating_Mode);
  log(std::string("Secondary_ID: "), msg->Secondary_ID);
  log(std::string("Protocol_Version: "), msg->Protocol_Version);
  log(std::string("Homing_Offset: "), msg->Homing_Offset);
  log(std::string("Moving_Threshold: "), msg->Moving_Threshold);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), msg->PWM_Limit);
  log(std::string("Current_Limit: "), msg->Current_Limit);
  log(std::string("Acceleration_Limit: "), msg->Acceleration_Limit);
  log(std::string("Velocity_Limit: "), msg->Velocity_Limit);
  log(std::string("Max_Position_Limit: "), msg->Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), msg->Min_Position_Limit);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Registered_Instruction: "), msg->Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), msg->Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), msg->Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), msg->Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), msg->Position_D_Gain);
  log(std::string("Position_I_Gain: "), msg->Position_I_Gain);
  log(std::string("Position_P_Gain: "), msg->Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), msg->Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), msg->Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), msg->Bus_Watchdog);
  log(std::string("Goal_PWM: "), msg->Goal_PWM);
  log(std::string("Goal_Current: "), msg->Goal_Current);
  log(std::string("Goal_Velocity: "), msg->Goal_Velocity);
  log(std::string("Profile_Acceleration: "), msg->Profile_Acceleration);
  log(std::string("Profile_Velocity: "), msg->Profile_Velocity);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Realtime_Tick: "), msg->Realtime_Tick);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Moving_Status: "), msg->Moving_Status);
  log(std::string("Present_PWM: "), msg->Present_PWM);
  log(std::string("Present_Current: "), msg->Present_Current);
  log(std::string("Present_Velocity: "), msg->Present_Velocity);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Velocity_Trajectory: "), msg->Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), msg->Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), msg->Present_Input_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);

  row_count_ = 0;
}

void QNode::XMExtStatusMsgCallback(const dynamixel_workbench_msgs::XMExt::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("Drive_Mode: "), msg->Drive_Mode);
  log(std::string("Operating_Mode: "), msg->Operating_Mode);
  log(std::string("Secondary_ID: "), msg->Secondary_ID);
  log(std::string("Protocol_Version: "), msg->Protocol_Version);
  log(std::string("Homing_Offset: "), msg->Homing_Offset);
  log(std::string("Moving_Threshold: "), msg->Moving_Threshold);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), msg->PWM_Limit);
  log(std::string("Current_Limit: "), msg->Current_Limit);
  log(std::string("Acceleration_Limit: "), msg->Acceleration_Limit);
  log(std::string("Velocity_Limit: "), msg->Velocity_Limit);
  log(std::string("Max_Position_Limit: "), msg->Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), msg->Min_Position_Limit);
  log(std::string("External_Port_Mode_1: "), msg->External_Port_Mode_1);
  log(std::string("External_Port_Mode_2: "), msg->External_Port_Mode_2);
  log(std::string("External_Port_Mode_3: "), msg->External_Port_Mode_3);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Registered_Instruction: "), msg->Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), msg->Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), msg->Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), msg->Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), msg->Position_D_Gain);
  log(std::string("Position_I_Gain: "), msg->Position_I_Gain);
  log(std::string("Position_P_Gain: "), msg->Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), msg->Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), msg->Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), msg->Bus_Watchdog);
  log(std::string("Goal_PWM: "), msg->Goal_PWM);
  log(std::string("Goal_Current: "), msg->Goal_Current);
  log(std::string("Goal_Velocity: "), msg->Goal_Velocity);
  log(std::string("Profile_Acceleration: "), msg->Profile_Acceleration);
  log(std::string("Profile_Velocity: "), msg->Profile_Velocity);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Realtime_Tick: "), msg->Realtime_Tick);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Moving_Status: "), msg->Moving_Status);
  log(std::string("Present_PWM: "), msg->Present_PWM);
  log(std::string("Present_Current: "), msg->Present_Current);
  log(std::string("Present_Velocity: "), msg->Present_Velocity);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Velocity_Trajectory: "), msg->Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), msg->Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), msg->Present_Input_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);

  row_count_ = 0;
}

void QNode::XHStatusMsgCallback(const dynamixel_workbench_msgs::XH::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("Drive_Mode: "), msg->Drive_Mode);
  log(std::string("Operating_Mode: "), msg->Operating_Mode);
  log(std::string("Secondary_ID: "), msg->Secondary_ID);
  log(std::string("Protocol_Version: "), msg->Protocol_Version);
  log(std::string("Homing_Offset: "), msg->Homing_Offset);
  log(std::string("Moving_Threshold: "), msg->Moving_Threshold);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), msg->PWM_Limit);
  log(std::string("Current_Limit: "), msg->Current_Limit);
  log(std::string("Acceleration_Limit: "), msg->Acceleration_Limit);
  log(std::string("Velocity_Limit: "), msg->Velocity_Limit);
  log(std::string("Max_Position_Limit: "), msg->Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), msg->Min_Position_Limit);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED: "), msg->LED);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Registered_Instruction: "), msg->Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), msg->Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), msg->Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), msg->Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), msg->Position_D_Gain);
  log(std::string("Position_I_Gain: "), msg->Position_I_Gain);
  log(std::string("Position_P_Gain: "), msg->Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), msg->Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), msg->Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), msg->Bus_Watchdog);
  log(std::string("Goal_PWM: "), msg->Goal_PWM);
  log(std::string("Goal_Current: "), msg->Goal_Current);
  log(std::string("Goal_Velocity: "), msg->Goal_Velocity);
  log(std::string("Profile_Acceleration: "), msg->Profile_Acceleration);
  log(std::string("Profile_Velocity: "), msg->Profile_Velocity);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Realtime_Tick: "), msg->Realtime_Tick);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Moving_Status: "), msg->Moving_Status);
  log(std::string("Present_PWM: "), msg->Present_PWM);
  log(std::string("Present_Current: "), msg->Present_Current);
  log(std::string("Present_Velocity: "), msg->Present_Velocity);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Velocity_Trajectory: "), msg->Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), msg->Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), msg->Present_Input_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);

  row_count_ = 0;
}

void QNode::PROStatusMsgCallback(const dynamixel_workbench_msgs::PRO::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), msg->Model_Number);
  log(std::string("Firmware_Version: "), msg->Firmware_Version);
  log(std::string("ID: "), msg->ID);
  log(std::string("Baud_Rate: "), msg->Baud_Rate);
  log(std::string("Return_Delay_Time: "), msg->Return_Delay_Time);
  log(std::string("Operating_Mode: "), msg->Operating_Mode);
  log(std::string("Homing_Offset: "), msg->Homing_Offset);
  log(std::string("Moving_Threshold: "), msg->Moving_Threshold);
  log(std::string("Temperature_Limit: "), msg->Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), msg->Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), msg->Min_Voltage_Limit);
  log(std::string("Acceleration_Limit: "), msg->Acceleration_Limit);
  log(std::string("Torque_Limit: "), msg->Torque_Limit);
  log(std::string("Velocity_Limit: "), msg->Velocity_Limit);
  log(std::string("Max_Position_Limit: "), msg->Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), msg->Min_Position_Limit);
  log(std::string("External_Port_Mode_1: "), msg->External_Port_Mode_1);
  log(std::string("External_Port_Mode_2: "), msg->External_Port_Mode_2);
  log(std::string("External_Port_Mode_3: "), msg->External_Port_Mode_3);
  log(std::string("External_Port_Mode_4: "), msg->External_Port_Mode_4);
  log(std::string("Shutdown: "), msg->Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), msg->Torque_Enable);
  log(std::string("LED_RED: "), msg->LED_RED);
  log(std::string("LED_GREEN: "), msg->LED_GREEN);
  log(std::string("LED_BLUE: "), msg->LED_BLUE);
  log(std::string("Velocity_I_Gain: "), msg->Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), msg->Velocity_P_Gain);
  log(std::string("Position_P_Gain: "), msg->Position_P_Gain);
  log(std::string("Goal_Position: "), msg->Goal_Position);
  log(std::string("Goal_Velocity: "), msg->Goal_Velocity);
  log(std::string("Goal_Torque: "), msg->Goal_Torque);
  log(std::string("Moving: "), msg->Moving);
  log(std::string("Present_Position: "), msg->Present_Position);
  log(std::string("Present_Velocity: "), msg->Present_Velocity);
  log(std::string("Present_Current: "), msg->Present_Current);
  log(std::string("Present_Input_Voltage: "), msg->Present_Input_Voltage);
  log(std::string("Present_Temperature: "), msg->Present_Temperature);
  log(std::string("Registered_Instruction: "), msg->Registered_Instruction);
  log(std::string("Status_Return_Level: "), msg->Status_Return_Level);
  log(std::string("Hardware_Error_Status: "), msg->Hardware_Error_Status);

  row_count_ = 0;
}
