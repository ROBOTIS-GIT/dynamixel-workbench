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
  dynamixel_load_ = new DynamixelLoad;

  dynamixel_load_->device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  dynamixel_load_->baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  dynamixel_load_->protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  dynamixel_driver_ = new dynamixel_driver::DynamixelDriver(dynamixel_load_->device_name,
                                                            dynamixel_load_->baud_rate,
                                                            dynamixel_load_->protocol_version);

  if(!use_ping_)
  {
    // Get Connected Single Dynamixel State
    if (dynamixel_load_->protocol_version == 1.0)
      ROS_INFO("Scan Dynamixel(ID: 1~253) Using Protocol 1.0\n");
    else if (dynamixel_load_->protocol_version == 2.0)
      ROS_INFO("Scan Dynamixel(ID: 1~253) Using Protocol 2.0\n");

    if (dynamixel_driver_->scan())
    {
      ROS_INFO("...Succeeded to find dynamixel\n");
      ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d",
               dynamixel_driver_->dynamixel_->id_, dynamixel_driver_->dynamixel_->model_name_.c_str(), dynamixel_load_->baud_rate);
    }
    else
    {
      ROS_ERROR("...Failed to find dynamixel!");
      shutdownSingleDynamixelMonitor();
    }
  }
  else
  {
    // Ping Connected Single Dynamixel State
    if (dynamixel_load_->protocol_version == 1.0)
      ROS_INFO("Ping(ID: %d) Dynamixel Using Protocol 1.0\n", ping_id_);
    else if (dynamixel_load_->protocol_version == 2.0)
      ROS_INFO("Ping(ID: %d) Dynamixel Using Protocol 2.0\n", ping_id_);

    if (dynamixel_driver_->ping(ping_id_))
    {
      ROS_INFO("...Succeeded to ping dynamixel\n");
      ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d",
               dynamixel_driver_->dynamixel_->id_, dynamixel_driver_->dynamixel_->model_name_.c_str(), dynamixel_load_->baud_rate);
    }
    else
    {
      ROS_ERROR("...Failed to find dynamixel!");
      shutdownSingleDynamixelMonitor();
    }
  }

  initDynamixelStatePublisher();

  ROS_INFO("dynamixel_workbench_single_manager : Init Success!");
}

SingleDynamixelMonitor::~SingleDynamixelMonitor()
{
  ROS_ASSERT(shutdownSingleDynamixelMonitor());
}

bool SingleDynamixelMonitor::initSingleDynamixelMonitor()
{

}

bool SingleDynamixelMonitor::shutdownSingleDynamixelMonitor()
{
  ros::shutdown();
  return true;
}

bool SingleDynamixelMonitor::initDynamixelStatePublisher()
{
  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

  if (dynamixel->model_name_.find("AX") != std::string::npos)
  {
    dynamixel_state_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelAX>("dynamixel/" + dynamixel->model_name_ + "_state", 10);
  }
  else if (dynamixel->model_name_.find("XM") != std::string::npos)
  {
    dynamixel_state_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelXM>("dynamixel/" + dynamixel->model_name_ + "_state", 10);
  }
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
    //XM();
  }
}

bool SingleDynamixelMonitor::AX()
{
  uint32_t read_value = 0;

  dynamixel_workbench_msgs::DynamixelAX ax_state;
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

  dynamixel_state_pub_.publish(ax_state);
}

bool SingleDynamixelMonitor::controlLoop()
{
  dynamixelStatePublish();
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "single_dynamixel_monitor");

  SingleDynamixelMonitor single_dynamixel_monitor;
  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    single_dynamixel_monitor.controlLoop();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


