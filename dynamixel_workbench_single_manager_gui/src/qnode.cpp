/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/dynamixel_workbench_single_manager_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dynamixel_workbench_single_manager_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv )
    :init_argc(argc),
     init_argv(argv),
     row_count_(0),
     dxl_model_name_(""),
     dxl_model_number_(0)
{}

QNode::~QNode()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"dynamixel_workbench_single_manager_gui");
  if ( ! ros::master::check() ) {
      return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle nh;

  // Add your ros communications here.
  set_workbench_param_msg_pub_ = nh.advertise<dynamixel_workbench_msgs::WorkbenchParam>("/dynamixel_workbench_single_manager/set_workbench_parameter", 10);
  get_workbench_param_client_ = nh.serviceClient<dynamixel_workbench_msgs::GetWorkbenchParam>("/dynamixel_workbench_single_manager/get_workbench_parameter", 10);
  getWorkbenchParam();

  setSubscriber(nh);
  dxl_command_msg_pub_ = nh.advertise<dynamixel_workbench_msgs::DynamixelCommand>("/dynamixel_workbench_single_manager/motor_command", 10);

  start();
  return true;

}

void QNode::sendTorqueMsg(std::string addr_name, int64_t onoff)
{
  dynamixel_workbench_msgs::DynamixelCommand msg;

  msg.addr_name = addr_name;
  msg.value = onoff;

  dxl_command_msg_pub_.publish(msg);
}

void QNode::sendRebootMsg(void)
{
  dynamixel_workbench_msgs::DynamixelCommand msg;

  msg.addr_name = std::string("reboot");

  dxl_command_msg_pub_.publish(msg);
}

void QNode::sendResetMsg(void)
{
  dynamixel_workbench_msgs::DynamixelCommand msg;

  msg.addr_name = std::string("factory_reset");

  dxl_command_msg_pub_.publish(msg);
}

void QNode::sendSetIdMsg(int64_t id)
{
  dynamixel_workbench_msgs::DynamixelCommand msg;

  msg.addr_name = std::string("id");
  msg.value = id;

  dxl_command_msg_pub_.publish(msg);
}

void QNode::sendSetOperatingModeMsg(std::string index)
{
  dynamixel_workbench_msgs::DynamixelCommand msg;

  if (index == "position_control")
  {
    if((!strncmp(dxl_model_name_.c_str(), "XM", 2)) || (!strncmp(dxl_model_name_.c_str(), "PRO", 3)))
    {
      msg.addr_name = std::string("operating_mode");
      msg.value = 3;

      dxl_command_msg_pub_.publish(msg);
    }
    else if(!strncmp(dxl_model_name_.c_str(), "XL", 2))
    {
      msg.addr_name = std::string("control_mode");
      msg.value = 2;

      dxl_command_msg_pub_.publish(msg);
    }
    else
    {
      msg.addr_name = std::string("cw_angle_limit");
      msg.value = 1;

      dxl_command_msg_pub_.publish(msg);

      msg.addr_name = std::string("ccw_angle_limit");
      msg.value = 1;

      dxl_command_msg_pub_.publish(msg);
    }
  }
  else if (index == "velocity_control")
  {
    if((!strncmp(dxl_model_name_.c_str(), "XM", 2)) || (!strncmp(dxl_model_name_.c_str(), "PRO", 3)))
    {
      msg.addr_name = std::string("operating_mode");
      msg.value = 1;

      dxl_command_msg_pub_.publish(msg);
    }
    else if(!strncmp(dxl_model_name_.c_str(), "XL", 2))
    {
      msg.addr_name = std::string("control_mode");
      msg.value = 1;

      dxl_command_msg_pub_.publish(msg);
    }
    else
    {
      msg.addr_name = std::string("cw_angle_limit");
      msg.value = 0;

      dxl_command_msg_pub_.publish(msg);

      msg.addr_name = std::string("ccw_angle_limit");
      msg.value = 0;

      dxl_command_msg_pub_.publish(msg);
    }
  }
}

void QNode::sendSetBaudrateMsg(float baud_rate)
{
  dynamixel_workbench_msgs::DynamixelCommand msg;

  msg.addr_name = std::string("baud_rate");
  msg.value = baud_rate;

  dxl_command_msg_pub_.publish(msg);
}

void QNode::setPositionZeroMsg(int32_t zero_position)
{
  dynamixel_workbench_msgs::DynamixelCommand msg;

  msg.addr_name = std::string("goal_position");
  msg.value = zero_position;

  dxl_command_msg_pub_.publish(msg);
}

void QNode::sendControlTableValueMsg(QString table_item, int64_t value)
{
  dynamixel_workbench_msgs::DynamixelCommand msg;

  msg.addr_name = table_item.toStdString();
  msg.value = value;

  dxl_command_msg_pub_.publish(msg);
}

void QNode::getWorkbenchParam(void)
{
  dynamixel_workbench_msgs::GetWorkbenchParam srv;

  if (get_workbench_param_client_.call(srv))
  {
    dynamixel_workbench_msgs::WorkbenchParam msg;

    msg.device_name = srv.response.workbench_parameter.device_name;
    msg.protocol_version = srv.response.workbench_parameter.protocol_version;
    msg.baud_rate = srv.response.workbench_parameter.baud_rate;
    msg.model_name = srv.response.workbench_parameter.model_name;
    msg.model_id = srv.response.workbench_parameter.model_id;
    msg.model_number = srv.response.workbench_parameter.model_number;

    dxl_model_name_ = srv.response.workbench_parameter.model_name;
    dxl_model_number_ = srv.response.workbench_parameter.model_number;

    Q_EMIT updateWorkbenchParam(msg);
  }
}

void QNode::dynamixelAXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelAX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("the_highest_limit_temperature: "), msg->the_highest_limit_temperature);
  log(std::string("the_lowest_limit_voltage: "), msg->the_lowest_limit_voltage);
  log(std::string("the_highest_limit_voltage: "), msg->the_highest_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("alarm_led: "), msg->alarm_led);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);

  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("cw_compliance_margin: "), msg->cw_compliance_margin);
  log(std::string("ccw_compliance_margin: "), msg->ccw_compliance_margin);
  log(std::string("cw_compliance_slope: "), msg->cw_compliance_slope);
  log(std::string("ccw_compliance_slope: "), msg->ccw_compliance_slope);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("goal_velocity: "), msg->goal_velocity);
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

  row_count_ = 0;
}

void QNode::dynamixelRXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelRX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("the_highest_limit_temperature: "), msg->the_highest_limit_temperature);
  log(std::string("the_lowest_limit_voltage: "), msg->the_lowest_limit_voltage);
  log(std::string("the_highest_limit_voltage: "), msg->the_highest_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("alarm_led: "), msg->alarm_led);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);

  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("cw_compliance_margin: "), msg->cw_compliance_margin);
  log(std::string("ccw_compliance_margin: "), msg->ccw_compliance_margin);
  log(std::string("cw_compliance_slope: "), msg->cw_compliance_slope);
  log(std::string("ccw_compliance_slope: "), msg->ccw_compliance_slope);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("goal_velocity: "), msg->goal_velocity);
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

  row_count_ = 0;
}

void QNode::dynamixelMXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("the_highest_limit_temperature: "), msg->the_highest_limit_temperature);
  log(std::string("the_lowest_limit_voltage: "), msg->the_lowest_limit_voltage);
  log(std::string("the_highest_limit_voltage: "), msg->the_highest_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("alarm_led: "), msg->alarm_led);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);
  log(std::string("multi_turn_offset: "), msg->multi_turn_offset);
  log(std::string("resolution_divider: "), msg->resolution_divider);

  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("d_gain: "), msg->d_gain);
  log(std::string("i_gain: "), msg->i_gain);
  log(std::string("p_gain: "), msg->p_gain);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("goal_velocity: "), msg->goal_velocity);
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
  log(std::string("goal_acceleration: "), msg->goal_acceleration);

  row_count_ = 0;
}

void QNode::dynamixelMX64StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX64::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("the_highest_limit_temperature: "), msg->the_highest_limit_temperature);
  log(std::string("the_lowest_limit_voltage: "), msg->the_lowest_limit_voltage);
  log(std::string("the_highest_limit_voltage: "), msg->the_highest_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("alarm_led: "), msg->alarm_led);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);
  log(std::string("multi_turn_offset: "), msg->multi_turn_offset);
  log(std::string("resolution_divider: "), msg->resolution_divider);

  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("d_gain: "), msg->d_gain);
  log(std::string("i_gain: "), msg->i_gain);
  log(std::string("p_gain: "), msg->p_gain);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("goal_velocity: "), msg->goal_velocity);
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
  log(std::string("current: "), msg->current);
  log(std::string("torque_control_mode_enable: "), msg->torque_control_mode_enable);
  log(std::string("goal_torque: "), msg->goal_torque);
  log(std::string("goal_acceleration: "), msg->goal_acceleration);

  row_count_ = 0;
}

void QNode::dynamixelMX106StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX106::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
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

  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("d_gain: "), msg->d_gain);
  log(std::string("i_gain: "), msg->i_gain);
  log(std::string("p_gain: "), msg->p_gain);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("goal_velocity: "), msg->goal_velocity);
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
  log(std::string("current: "), msg->current);
  log(std::string("torque_control_mode_enable: "), msg->torque_control_mode_enable);
  log(std::string("goal_torque: "), msg->goal_torque);
  log(std::string("goal_acceleration: "), msg->goal_acceleration);

  row_count_ = 0;
}

void QNode::dynamixelEXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelEX::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
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

  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("cw_compliance_margin: "), msg->cw_compliance_margin);
  log(std::string("ccw_compliance_margin: "), msg->ccw_compliance_margin);
  log(std::string("cw_compliance_slope: "), msg->cw_compliance_slope);
  log(std::string("ccw_compliance_slope: "), msg->ccw_compliance_slope);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("goal_velocity: "), msg->goal_velocity);
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

void QNode::dynamixelXLStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXL::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
  log(std::string("cw_angle_limit: "), msg->cw_angle_limit);
  log(std::string("ccw_angle_limit: "), msg->ccw_angle_limit);
  log(std::string("control_mode: "), msg->control_mode);
  log(std::string("limit_temperature: "), msg->limit_temperature);
  log(std::string("down_limit_voltage: "), msg->down_limit_voltage);
  log(std::string("up_limit_voltage: "), msg->up_limit_voltage);
  log(std::string("max_torque: "), msg->max_torque);
  log(std::string("return_level: "), msg->return_level);
  log(std::string("alarm_shutdown: "), msg->alarm_shutdown);

  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("d_gain: "), msg->d_gain);
  log(std::string("i_gain: "), msg->i_gain);
  log(std::string("p_gain: "), msg->p_gain);
  log(std::string("goal_position: "), msg->goal_position);
  log(std::string("goal_velocity: "), msg->goal_velocity);
  log(std::string("goal_torque: "), msg->goal_torque);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("present_velocity: "), msg->present_velocity);
  log(std::string("present_load: "), msg->present_load);
  log(std::string("present_voltage: "), msg->present_voltage);
  log(std::string("present_temperature: "), msg->present_temperature);
  log(std::string("registered_instruction: "), msg->registered_instruction);
  log(std::string("moving: "), msg->moving);
  log(std::string("hardware_error_status: "), msg->hardware_error_status);
  log(std::string("punch: "), msg->punch);

  row_count_ = 0;
}

void QNode::dynamixelXMStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXM::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
  log(std::string("operating_mode: "), msg->operating_mode);
  log(std::string("protocol_version: "), msg->protocol_version);
  log(std::string("homing_offset: "), msg->homing_offset);
  log(std::string("moving_threshold: "), msg->moving_threshold);
  log(std::string("max_temperature_limit: "), msg->max_temperature_limit);
  log(std::string("max_voltage_limit: "), msg->max_voltage_limit);
  log(std::string("min_voltage_limit: "), msg->min_voltage_limit);
  log(std::string("pwm_limit: "), msg->pwm_limit);
  log(std::string("current_limit: "), msg->current_limit);
  log(std::string("acceleration_limit: "), msg->acceleration_limit);
  log(std::string("velocity_limit: "), msg->velocity_limit);
  log(std::string("max_position_limit: "), msg->max_position_limit);
  log(std::string("min_position_limit: "), msg->min_position_limit);
  log(std::string("shutdown: "), msg->shutdown);

  log(std::string("< RAM >"));
  log(std::string("torque_enable: "), msg->torque_enable);
  log(std::string("led: "), msg->led);
  log(std::string("status_return_level: "), msg->status_return_level);
  log(std::string("registered_instruction: "), msg->registered_instruction);
  log(std::string("hardware_error_status: "), msg->hardware_error_status);
  log(std::string("velocity_i_gain: "), msg->velocity_i_gain);
  log(std::string("velocity_p_gain: "), msg->velocity_p_gain);
  log(std::string("velocity_d_gain: "), msg->velocity_d_gain);
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
  log(std::string("indirect_address_1: "), msg->indirect_address_1);
  log(std::string("indirect_data_1: "), msg->indirect_data_1);
  log(std::string("indirect_address_29: "), msg->indirect_address_29);
  log(std::string("indirect_data_29: "), msg->indirect_data_29);

  row_count_ = 0;
}

void QNode::dynamixelProStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelPro::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
  log(std::string("operating_mode: "), msg->operating_mode);
  log(std::string("homing_offset: "), msg->homing_offset);
  log(std::string("moving_threshold: "), msg->moving_threshold);
  log(std::string("max_temperature_limit: "), msg->max_temperature_limit);
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
  log(std::string("is_moving: "), msg->is_moving);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("present_velocity: "), msg->present_velocity);
  log(std::string("present_current: "), msg->present_current);
  log(std::string("present_voltage: "), msg->present_voltage);
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

void QNode::dynamixelProL42StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelProL42::ConstPtr &msg)
{
  log(std::string("< EEPROM >"));
  log(std::string("model_number: "), msg->model_number);
  log(std::string("version_of_firmware: "), msg->version_of_firmware);
  log(std::string("id: "), msg->id);
  log(std::string("baud_rate: "), msg->baud_rate);
  log(std::string("return_delay_tiem: "), msg->return_delay_time);
  log(std::string("operating_mode: "), msg->operating_mode);
  log(std::string("moving_threshold: "), msg->moving_threshold);
  log(std::string("max_temperature_limit: "), msg->max_temperature_limit);
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
  log(std::string("is_moving: "), msg->is_moving);
  log(std::string("present_position: "), msg->present_position);
  log(std::string("present_velocity: "), msg->present_velocity);
  log(std::string("present_current: "), msg->present_current);
  log(std::string("present_voltage: "), msg->present_voltage);
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

void QNode::setSubscriber(ros::NodeHandle nh)
{
  // Init ROS subscribe
  if(!strncmp(dxl_model_name_.c_str(), "AX", 2))
  {
    dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelAXStatusMsgCallback, this);
  }
  else if (!strncmp(dxl_model_name_.c_str(), "MX", 2))
  {
    if (dxl_model_number_ == 310) // MX-64
    {
      dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelMX64StatusMsgCallback, this);
    }
    else if (dxl_model_number_ == 320) // MX-106
    {
      dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelMX106StatusMsgCallback, this);
    }
    else
    {
      dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelMXStatusMsgCallback, this);
    }
  }
  else if (!strncmp(dxl_model_name_.c_str(), "RX", 2))
  {
    dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelRXStatusMsgCallback, this);
  }
  else if (!strncmp(dxl_model_name_.c_str(), "EX", 2))
  {
    dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelEXStatusMsgCallback, this);
  }
  else if (!strncmp(dxl_model_name_.c_str(), "XL", 2))
  {
    dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelXLStatusMsgCallback, this);
  }
  else if (!strncmp(dxl_model_name_.c_str(), "XM", 2))
  {
    dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelXMStatusMsgCallback, this);
  }
  else if (!strncmp(dxl_model_name_.c_str(), "PRO", 3))
  {
    if (dxl_model_number_ == 35072) // PRO_L42_10_S300_R
    {
      dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelProL42StatusMsgCallback, this);
    }
    else
    {
      dxl_status_msg_sub_ = nh.subscribe("/dynamixel_workbench_single_manager/motor_state", 10, &QNode::dynamixelProStatusMsgCallback, this);
    }
  }
}

void QNode::run()
{
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "ROS shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
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

}  // namespace dynamixel_workbench_single_manager_gui
