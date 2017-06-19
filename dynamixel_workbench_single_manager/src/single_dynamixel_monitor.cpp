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

#include "dynamixel_workbench_single_manager/single_dynamixel_monitor.h"

using namespace single_dynamixel_monitor;

SingleDynamixelMonitor::SingleDynamixelMonitor()
{
  // Check Dynamixel Ping or Scan (default : Scan (1~253))
  use_ping_ = node_handle_.param<bool>("ping", false);
  ping_id_  = node_handle_.param<int>("ping_id", 1);

  // Load Paramameter For Connection
  dynamixel_info_ = new dynamixel_driver::DynamixelInfo;

  dynamixel_info_->lode_info.device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  dynamixel_info_->lode_info.baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  dynamixel_info_->lode_info.protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  dynamixel_driver_ = new dynamixel_driver::DynamixelDriver(dynamixel_info_->lode_info.device_name,
                                                            dynamixel_info_->lode_info.baud_rate,
                                                            dynamixel_info_->lode_info.protocol_version);

  if(!use_ping_)
  {
    // Get Connected Single Dynamixel State
    if (dynamixel_info_->lode_info.protocol_version == 1.0)
      ROS_INFO("Scan Dynamixel(ID: 1~253) Using Protocol 1.0\n");
    else if (dynamixel_info_->lode_info.protocol_version == 2.0)
      ROS_INFO("Scan Dynamixel(ID: 1~253) Using Protocol 2.0\n");

    if (dynamixel_driver_->scan())
    {
      ROS_INFO("...Succeeded to find dynamixel\n");
      ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d",
               dynamixel_driver_->dynamixel_->id_, dynamixel_driver_->dynamixel_->model_name_.c_str(), dynamixel_info_->lode_info.baud_rate);
    }
    else
    {
      ROS_WARN("Please Check USB Port authorization and");
      ROS_WARN("Baudrate [ex : 57600, 115200, 1000000, 3000000]");
      ROS_ERROR("...Failed to find dynamixel!");      
      shutdownSingleDynamixelMonitor();
    }
  }
  else
  {
    // Ping Connected Single Dynamixel State
    if (dynamixel_info_->lode_info.protocol_version == 1.0)
      ROS_INFO("Ping(ID: %d) Dynamixel Using Protocol 1.0\n", ping_id_);
    else if (dynamixel_info_->lode_info.protocol_version == 2.0)
      ROS_INFO("Ping(ID: %d) Dynamixel Using Protocol 2.0\n", ping_id_);

    if (dynamixel_driver_->ping(ping_id_))
    {
      ROS_INFO("...Succeeded to ping dynamixel\n");
      ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d",
               dynamixel_driver_->dynamixel_->id_, dynamixel_driver_->dynamixel_->model_name_.c_str(), dynamixel_info_->lode_info.baud_rate);
    }
    else
    {
      ROS_WARN("Please Check USB Port authorization and");
      ROS_WARN("Baudrate [ex : 57600, 115200, 1000000, 3000000]");
      ROS_ERROR("...Failed to find dynamixel!");
      shutdownSingleDynamixelMonitor();
    }
  }

  initDynamixelStatePublisher();
  initDynamixelInfoServer();
  initDynamixelCommandServer();

  ROS_INFO("dynamixel_workbench_single_manager : Init Success!");
}

SingleDynamixelMonitor::~SingleDynamixelMonitor()
{

}

bool SingleDynamixelMonitor::initSingleDynamixelMonitor()
{

}

bool SingleDynamixelMonitor::shutdownSingleDynamixelMonitor()
{
  dynamixel_driver_->writeRegister("torque_enable", 0);

  ros::shutdown();
  return true;
}

bool SingleDynamixelMonitor::initDynamixelStatePublisher()
{
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

  if (dynamixel->model_name_.find("AX") != std::string::npos)
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::AX>("dynamixel/" + dynamixel->model_name_ + "_state", 10);
  }
  else if (dynamixel->model_name_.find("XM") != std::string::npos)
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XM>("dynamixel/" + dynamixel->model_name_ + "_state", 10);
  }

  return true;
}

bool SingleDynamixelMonitor::initDynamixelInfoServer()
{
  dynamixel_info_server_ = node_handle_.advertiseService("dynamixel/info", &SingleDynamixelMonitor::dynamixelInfoMsgCallback, this);

  return true;
}

bool SingleDynamixelMonitor::initDynamixelCommandServer()
{
  dynamixel_command_server_ = node_handle_.advertiseService("dynamixel/command", &SingleDynamixelMonitor::dynamixelCommandMsgCallback, this);

  return true;
}

bool SingleDynamixelMonitor::showDynamixelControlTable()
{
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;
  int32_t torque_status = 0;

  dynamixel_driver_->readRegister("torque_enable", &torque_status);

  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
       dynamixel->it_ctrl_++)
  {
    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];

    if (torque_status)
    {
      if ((dynamixel->item_->access_type == dynamixel_tool::READ_WRITE) && (dynamixel->item_->memory_type == dynamixel_tool::RAM))
      {
        ROS_INFO("%s", dynamixel->item_->item_name.c_str());
      }
    }
    else
    {
      if (dynamixel->item_->access_type == dynamixel_tool::READ_WRITE)
      {
        ROS_INFO("%s", dynamixel->item_->item_name.c_str());
      }
    }
  }

  return true;
}

bool SingleDynamixelMonitor::checkValidationCommand(std::string cmd)
{
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
       dynamixel->it_ctrl_++)
  {
    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];

    if (cmd == dynamixel->item_->item_name)
      return true;
  }

  ROS_WARN("Please Check DYNAMXEL Address Name('table')");
  return false;
}

bool SingleDynamixelMonitor::checkValidAccess(std::string cmd)
{
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;
  int32_t torque_status = 0;

  dynamixel_driver_->readRegister("torque_enable", &torque_status);

  dynamixel->item_ = dynamixel->ctrl_table_[cmd];
  if (dynamixel->item_->access_type == dynamixel_tool::READ_WRITE)
  {
    if ((torque_status == true) && (dynamixel->item_->memory_type == dynamixel_tool::EEPROM))
    {
      ROS_WARN("address in EEPROM can't be accessed when torque is on");
      ROS_WARN("Check a 'table'");

      return false;
    }
    else if ((torque_status == false) && (dynamixel->item_->item_name != "torque_enable") && dynamixel->item_->memory_type == dynamixel_tool::RAM)
    {
      ROS_WARN("address in RAM can't be accessed when torque is off");
      ROS_WARN("Check a 'table'");

      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    return false;
  }
}

bool SingleDynamixelMonitor::changeId(uint8_t id)
{
  if (id > 0 && id < 254)
  {
    dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;
    dynamixel->item_ = dynamixel->ctrl_table_["id"];

    dynamixel_driver_->writeRegister("id", id);
    usleep(dynamixel->item_->data_length * 55 * 1000 * 10);

    dynamixel_driver_->ping(id);

    ROS_INFO("...Succeeded to set dynamixel id [%u]", dynamixel_driver_->dynamixel_->id_);
    return true;
  }
  else
  {
    ROS_WARN("Dynamixel ID can be set 1~253");
    return false;
  }
}

bool SingleDynamixelMonitor::changeBaudrate(uint64_t baud_rate)
{
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

  if (dynamixel->baud_rate_table_.find(baud_rate)->second == dynamixel->baud_rate_table_.end()->second)
  {
    ROS_ERROR(" Failed to change [ BAUD RATE: %ld ]", baud_rate);
    ROS_WARN(" Please check a valid baud rate at E-MANUAL");

    if (dynamixel_driver_->getProtocolVersion() == 2.0)
    {
      dynamixel_driver_->writeRegister("baud_rate", dynamixel->baud_rate_table_.find(57600)->second);
      usleep(dynamixel->item_->data_length* 55 * 1000 *10);

      if (dynamixel_driver_->setBaudrate(57600) == false)
      {
        ROS_ERROR(" Failed to change default baudrate(57600)!");
      }
      else
      {
        ROS_INFO(" Success to change default baudrate! [ BAUD RATE: 57600 ]");
      }
    }
    else if (dynamixel_driver_->getProtocolVersion() == 1.0)
    {
      dynamixel_driver_->writeRegister("baud_rate", dynamixel->baud_rate_table_.find(1000000)->second);
      usleep(dynamixel->item_->data_length* 55 * 1000 *10);

      if (dynamixel_driver_->setBaudrate(1000000) == false)
      {
        ROS_ERROR(" Failed to change default baudrate(1000000)!");
      }
      else
      {
        ROS_INFO(" Success to change default baudrate! [ BAUD RATE: 1000000 ]");
      }
    }

    return false;
  }
  else
  {
    dynamixel_driver_->writeRegister("baud_rate", dynamixel->baud_rate_table_.find(baud_rate)->second);
    usleep(dynamixel->item_->data_length* 55 * 1000 *10);

    if (dynamixel_driver_->setBaudrate(baud_rate) == false)
    {
      ROS_INFO(" Failed to change baudrate!");
      return false;
    }
    else
    {
      ROS_INFO(" Success to change baudrate! [ BAUD RATE: %d ]", dynamixel->baud_rate_table_.find(baud_rate)->first);
      return true;
    }
  }
}

bool SingleDynamixelMonitor::changeProtocolVersion(float ver)
{
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

  if (ver == 1.0 || ver == 2.0)
  {
    // TODO
    dynamixel_driver_->writeRegister("protocol_version", ver);
    usleep(dynamixel->item_->data_length* 55 * 1000 *10);

    dynamixel_driver_->setPacketHandler(ver);

    ROS_INFO(" Success to change protocol version [ PROTOCOL VERSION: %.2f]", dynamixel_driver_->getProtocolVersion());
    return true;
  }
  else
  {
    ROS_ERROR(" Dynamixel has '1.0' or '2.0' protocol version");
    return false;
  }
}

bool SingleDynamixelMonitor::controlLoop()
{
  dynamixelStatePublish();

  return true;
}

bool SingleDynamixelMonitor::dynamixelInfoMsgCallback(dynamixel_workbench_msgs::GetDynamixelInfo::Request &req,
                                                   dynamixel_workbench_msgs::GetDynamixelInfo::Response &res)
{
  const char *portName = dynamixel_driver_->getPortName();
  std::string get_port_name(portName);

  res.dynamixel_info.load_info.device_name      = get_port_name;
  res.dynamixel_info.load_info.baud_rate        = dynamixel_driver_->getBaudrate();
  res.dynamixel_info.load_info.protocol_version = dynamixel_driver_->getProtocolVersion();

  res.dynamixel_info.model_id         = dynamixel_driver_->dynamixel_->id_;
  res.dynamixel_info.model_name       = dynamixel_driver_->dynamixel_->model_name_;
  res.dynamixel_info.model_number     = dynamixel_driver_->dynamixel_->model_number_;

  return true;
}

bool SingleDynamixelMonitor::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                         dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
  if (req.command == "table")
  {
    if (showDynamixelControlTable())
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "reboot")
  {
    if (dynamixel_driver_->reboot())
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "factory_reset")
  {
    if (dynamixel_driver_->reset())
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "torque_on")
  {
    std::string addr = req.addr_name;
    int64_t value    = req.value;

    if (dynamixel_driver_->writeRegister(addr, value))
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "torque_off")
  {
    std::string addr = req.addr_name;
    int64_t value    = req.value;

    if (dynamixel_driver_->writeRegister(addr, value))
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "exit")
  {
    if (shutdownSingleDynamixelMonitor())
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "addr")
  {
    std::string addr = req.addr_name;
    int64_t value    = req.value;

    if (checkValidationCommand(addr))
    {
      res.comm_result = true;
    }
    else
    {
      res.comm_result = false;
      return true;
    }

    if (checkValidAccess(addr))
    {
      res.comm_result = true;
    }
    else
    {
      res.comm_result = false;
      return true;
    }

    if (addr == "id")
    {
      if (changeId(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
    else if (addr == "baud_rate")
    {
      if (changeBaudrate(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
    else if (addr == "protocol_version")
    {
      if (changeProtocolVersion(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
    else
    {
      if (dynamixel_driver_->writeRegister(addr, value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
  }
  else
  {    
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "single_dynamixel_monitor");

  SingleDynamixelMonitor single_dynamixel_monitor;
  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    single_dynamixel_monitor.controlLoop();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


bool SingleDynamixelMonitor::dynamixelStatePublish()
{
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

  if (dynamixel->model_name_.find("AX") != std::string::npos)
  {
    AX();
  }
  else if (dynamixel->model_name_.find("XM") != std::string::npos)
  {
    XM();
  }

  return true;
}

bool SingleDynamixelMonitor::AX()
{
  int32_t read_value = 0;

  dynamixel_workbench_msgs::AX ax_state;
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
       dynamixel->it_ctrl_++)
  {
    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

    if ("model_number" == dynamixel->item_->item_name)
      ax_state.model_number = read_value;
    else if ("version_of_firmware" == dynamixel->item_->item_name)
      ax_state.version_of_firmware = read_value;
    else if ("id" == dynamixel->item_->item_name)
      ax_state.id = read_value;
    else if ("baud_rate" == dynamixel->item_->item_name)
      ax_state.baud_rate = read_value;
    else if ("return_delay_time" == dynamixel->item_->item_name)
      ax_state.return_delay_time = read_value;
    else if ("cw_angle_limit" == dynamixel->item_->item_name)
      ax_state.cw_angle_limit = read_value;
    else if ("ccw_angle_limit" == dynamixel->item_->item_name)
      ax_state.ccw_angle_limit = read_value;
    else if ("the_highest_limit_temperature" == dynamixel->item_->item_name)
      ax_state.the_highest_limit_temperature = read_value;
    else if ("the_lowest_limit_voltage" == dynamixel->item_->item_name)
      ax_state.the_lowest_limit_voltage = read_value;
    else if ("the_highest_limit_voltage" == dynamixel->item_->item_name)
      ax_state.the_highest_limit_voltage = read_value;
    else if ("max_torque" == dynamixel->item_->item_name)
      ax_state.max_torque = read_value;
    else if ("status_return_level" == dynamixel->item_->item_name)
      ax_state.status_return_level = read_value;
    else if ("alarm_led" == dynamixel->item_->item_name)
      ax_state.alarm_led = read_value;
    else if ("alarm_shutdown" == dynamixel->item_->item_name)
      ax_state.alarm_shutdown = read_value;
    else if ("torque_enable" == dynamixel->item_->item_name)
      ax_state.torque_enable = read_value;
    else if ("led" == dynamixel->item_->item_name)
      ax_state.led = read_value;
    else if ("cw_compliance_margin" == dynamixel->item_->item_name)
      ax_state.cw_compliance_margin = read_value;
    else if ("ccw_compliance_margin" == dynamixel->item_->item_name)
      ax_state.ccw_compliance_margin = read_value;
    else if ("cw_compliance_slope" == dynamixel->item_->item_name)
      ax_state.cw_compliance_slope = read_value;
    else if ("ccw_compliance_margin" == dynamixel->item_->item_name)
      ax_state.ccw_compliance_margin = read_value;
    else if ("goal_position" == dynamixel->item_->item_name)
      ax_state.goal_position = read_value;
    else if ("moving_speed" == dynamixel->item_->item_name)
      ax_state.moving_speed = read_value;
    else if ("torque_limit" == dynamixel->item_->item_name)
      ax_state.torque_limit = read_value;
    else if ("present_position" == dynamixel->item_->item_name)
      ax_state.present_position = read_value;
    else if ("present_velocity" == dynamixel->item_->item_name)
      ax_state.present_velocity = read_value;
    else if ("present_load" == dynamixel->item_->item_name)
      ax_state.present_load = read_value;
    else if ("present_voltage" == dynamixel->item_->item_name)
      ax_state.present_voltage = read_value;
    else if ("present_temperature" == dynamixel->item_->item_name)
      ax_state.present_temperature = read_value;
    else if ("registered" == dynamixel->item_->item_name)
      ax_state.registered = read_value;
    else if ("moving" == dynamixel->item_->item_name)
      ax_state.moving = read_value;
    else if ("lock" == dynamixel->item_->item_name)
      ax_state.lock = read_value;
    else if ("punch" == dynamixel->item_->item_name)
      ax_state.punch = read_value;
  }

  dynamixel_status_pub_.publish(ax_state);

  return true;
}

bool SingleDynamixelMonitor::XM()
{
  int32_t read_value = 0;

  dynamixel_workbench_msgs::XM xm_state;
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
       dynamixel->it_ctrl_++)
  {
    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

    if ("model_number" == dynamixel->item_->item_name)
      xm_state.model_number = read_value;
    else if ("version_of_firmware" == dynamixel->item_->item_name)
      xm_state.version_of_firmware = read_value;
    else if ("id" == dynamixel->item_->item_name)
      xm_state.id = read_value;
    else if ("baud_rate" == dynamixel->item_->item_name)
      xm_state.baud_rate = read_value;
    else if ("return_delay_time" == dynamixel->item_->item_name)
      xm_state.return_delay_time = read_value;
    else if ("drive_mode" == dynamixel->item_->item_name)
      xm_state.drive_mode = read_value;
    else if ("operating_mode" == dynamixel->item_->item_name)
      xm_state.operating_mode = read_value;
    else if ("protocol_version" == dynamixel->item_->item_name)
      xm_state.protocol_version = read_value;
    else if ("homing_offset" == dynamixel->item_->item_name)
      xm_state.homing_offset = read_value;
    else if ("moving_threshold" == dynamixel->item_->item_name)
      xm_state.moving_threshold = read_value;
    else if ("max_temperature_limit" == dynamixel->item_->item_name)
      xm_state.temperature_limit = read_value;
    else if ("max_voltage_limit" == dynamixel->item_->item_name)
      xm_state.max_voltage_limit = read_value;
    else if ("min_voltage_limit" == dynamixel->item_->item_name)
      xm_state.min_voltage_limit = read_value;
    else if ("pwm_limit" == dynamixel->item_->item_name)
      xm_state.pwm_limit = read_value;
    else if ("current_limit" == dynamixel->item_->item_name)
      xm_state.current_limit = read_value;
    else if ("acceleration_limit" == dynamixel->item_->item_name)
      xm_state.acceleration_limit = read_value;
    else if ("velocity_limit" == dynamixel->item_->item_name)
      xm_state.velocity_limit = read_value;
    else if ("max_position_limit" == dynamixel->item_->item_name)
      xm_state.max_position_limit = read_value;
    else if ("min_position_limit" == dynamixel->item_->item_name)
      xm_state.min_position_limit = read_value;
    else if ("shutdown" == dynamixel->item_->item_name)
      xm_state.shutdown = read_value;
    else if ("torque_enable" == dynamixel->item_->item_name)
      xm_state.torque_enable = read_value;
    else if ("led" == dynamixel->item_->item_name)
      xm_state.led = read_value;
    else if ("status_return_level" == dynamixel->item_->item_name)
      xm_state.status_return_level = read_value;
    else if ("registered_instruction" == dynamixel->item_->item_name)
      xm_state.registered_instruction = read_value;
    else if ("hardware_error_status" == dynamixel->item_->item_name)
      xm_state.hardware_error_status = read_value;
    else if ("velocity_i_gain" == dynamixel->item_->item_name)
      xm_state.velocity_i_gain = read_value;
    else if ("velocity_p_gain" == dynamixel->item_->item_name)
      xm_state.velocity_p_gain = read_value;
    else if ("position_d_gain" == dynamixel->item_->item_name)
      xm_state.position_d_gain = read_value;
    else if ("position_i_gain" == dynamixel->item_->item_name)
      xm_state.position_i_gain = read_value;
    else if ("position_p_gain" == dynamixel->item_->item_name)
      xm_state.position_p_gain = read_value;
    else if ("feedforward_2nd_gain" == dynamixel->item_->item_name)
      xm_state.feedforward_2nd_gain = read_value;
    else if ("feedforward_1st_gain" == dynamixel->item_->item_name)
      xm_state.feedforward_1st_gain = read_value;
    else if ("goal_pwm" == dynamixel->item_->item_name)
      xm_state.goal_pwm = read_value;
    else if ("goal_current" == dynamixel->item_->item_name)
      xm_state.goal_current = read_value;
    else if ("goal_velocity" == dynamixel->item_->item_name)
      xm_state.goal_velocity = read_value;
    else if ("profile_acceleration" == dynamixel->item_->item_name)
      xm_state.profile_acceleration = read_value;
    else if ("profile_velocity" == dynamixel->item_->item_name)
      xm_state.profile_velocity = read_value;
    else if ("goal_position" == dynamixel->item_->item_name)
      xm_state.goal_position = read_value;
    else if ("realtime_tick" == dynamixel->item_->item_name)
      xm_state.realtime_tick = read_value;
    else if ("moving" == dynamixel->item_->item_name)
      xm_state.moving = read_value;
    else if ("moving_status" == dynamixel->item_->item_name)
      xm_state.moving_status = read_value;
    else if ("present_pwm" == dynamixel->item_->item_name)
      xm_state.present_pwm = read_value;
    else if ("present_current" == dynamixel->item_->item_name)
      xm_state.present_current = read_value;
    else if ("present_velocity" == dynamixel->item_->item_name)
      xm_state.present_velocity = read_value;
    else if ("present_position" == dynamixel->item_->item_name)
      xm_state.present_position = read_value;
    else if ("velocity_trajectory" == dynamixel->item_->item_name)
      xm_state.velocity_trajectory = read_value;
    else if ("position_trajectory" == dynamixel->item_->item_name)
      xm_state.position_trajectory = read_value;
    else if ("present_input_voltage" == dynamixel->item_->item_name)
      xm_state.present_input_voltage = read_value;
    else if ("present_temperature" == dynamixel->item_->item_name)
      xm_state.present_temperature = read_value;
  }

  dynamixel_status_pub_.publish(xm_state);
}
