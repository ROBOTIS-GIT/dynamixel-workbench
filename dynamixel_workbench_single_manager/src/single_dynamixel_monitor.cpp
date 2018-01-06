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

#include "dynamixel_workbench_single_manager/single_dynamixel_monitor.h"

using namespace single_dynamixel_monitor;

SingleDynamixelMonitor::SingleDynamixelMonitor()
  :dxl_baud_rate_(0),
   dxl_id_(0)
{
  // Check Dynamixel Ping or Scan (default : Scan (1~253))

  // Dynamixel Monitor variable
  bool use_ping = node_handle_.param<bool>("ping", false);
  int ping_id  = node_handle_.param<int>("ping_id", 1);

  device_name_   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  dxl_baud_rate_ = node_handle_.param<int>("baud_rate", 57600);

  // Init Dynamixel Driver
  dynamixel_driver_ = new DynamixelDriver;

  dynamixel_driver_->begin(device_name_.c_str(), dxl_baud_rate_);


  if(use_ping == true)
  {
    uint16_t model_number = 0;
    if (dynamixel_driver_->ping(ping_id, &model_number))
    {
      dxl_id_ = ping_id;
      printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d [VERSION] %.1f\n",
               dxl_id_, dynamixel_driver_->getModelName(dxl_id_), dxl_baud_rate_, dynamixel_driver_->getProtocolVersion());
    }
    else
    {
      printf("Please Check USB Port authorization and\n");
      printf("Baudrate [ex : 57600, 115200, 1000000, 3000000]\n");
      printf("...Failed to find dynamixel!\n");

      ros::shutdown();
      exit(1);
    }
  }
  else
  {
    uint8_t id_cnt = 0;
    if (dynamixel_driver_->scan(&dxl_id_, &id_cnt))
    {
      printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d [VERSION] %.1f\n",
               dxl_id_, dynamixel_driver_->getModelName(dxl_id_), dxl_baud_rate_, dynamixel_driver_->getProtocolVersion());
    }
    else
    {
      printf("Please Check USB Port authorization and\n");
      printf("Baudrate [ex : 57600, 115200, 1000000, 3000000]\n");
      printf("...Failed to find dynamixel!\n");

      ros::shutdown();
      exit(1);
    }
  }

  initDynamixelStatePublisher();
  initDynamixelInfoServer();
  initDynamixelCommandServer();

  printf("dynamixel_workbench_single_manager : Init Success!\n");
}

SingleDynamixelMonitor::~SingleDynamixelMonitor()
{

}

void SingleDynamixelMonitor::initSingleDynamixelMonitor()
{

}

void SingleDynamixelMonitor::shutdownSingleDynamixelMonitor()
{
  dynamixel_driver_->writeRegister(dxl_id_, "Torque Enable", false);

  ros::shutdown();
}

void SingleDynamixelMonitor::initDynamixelStatePublisher()
{
  char* model_name = dynamixel_driver_->getModelName(dxl_id_);

  if (!strncmp(model_name, "AX", strlen("AX")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::AX>("dynamixel/" + std::string("AX"), 10);
  }
  else if (!strncmp(model_name, "RX", strlen("RX")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::RX>("dynamixel/" + std::string("RX"), 10);
  }
  else if (!strncmp(model_name, "MX", strlen("MX")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MX>("dynamixel/" + std::string("MX"), 10);
  }
  else if (!strncmp(model_name, "EX", strlen("EX")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::EX>("dynamixel/" + std::string("EX"), 10);
  }
  else if (!strncmp(model_name, "XL", strlen("XL")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XL>("dynamixel/" + std::string("XL"), 10);
  }
  else if (!strncmp(model_name, "XM", strlen("XM")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XM>("dynamixel/" + std::string("XM"), 10);
  }
  else if (!strncmp(model_name, "XH", strlen("XH")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XH>("dynamixel/" + std::string("XH"), 10);
  }
  else if (!strncmp(model_name, "PRO", strlen("PRO")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::PRO>("dynamixel/" + std::string("PRO"), 10);
  }
}

void SingleDynamixelMonitor::initDynamixelInfoServer()
{
  dynamixel_info_server_ = node_handle_.advertiseService("dynamixel/info", &SingleDynamixelMonitor::dynamixelInfoMsgCallback, this);
}

void SingleDynamixelMonitor::initDynamixelCommandServer()
{
  dynamixel_command_server_ = node_handle_.advertiseService("dynamixel/command", &SingleDynamixelMonitor::dynamixelCommandMsgCallback, this);
}

bool SingleDynamixelMonitor::showDynamixelControlTable()
{
  int32_t torque_status = 0;
  uint16_t torque_enable_address = 0;
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);

  for (int item_num = 0; item_num < dynamixel_driver_->getTheNumberOfItem(dxl_id_); item_num++)
  {
    if (!strncmp(item_ptr[item_num].item_name, "Torque Enable", strlen(item_ptr[item_num].item_name)))
    {
      torque_enable_address = item_num;
    }
  }

  dynamixel_driver_->readRegister(dxl_id_, "Torque Enable", &torque_status);

  for (int item_num = 0; item_num < dynamixel_driver_->getTheNumberOfItem(dxl_id_); item_num++)
  {
    if (torque_status == false)
    {
      printf("%s\n", item_ptr[item_num].item_name);
    }
    else
    {
      if (item_num >= torque_enable_address)
        printf("%s\n", item_ptr[item_num].item_name);
    }
  }

  return true;
}

bool SingleDynamixelMonitor::checkValidationCommand(std::string cmd)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);

  for (int item_num = 0; item_num < dynamixel_driver_->getTheNumberOfItem(dxl_id_); item_num++)
  {
    if (!strncmp(item_ptr[item_num].item_name, cmd.c_str(), strlen(item_ptr[item_num].item_name)))
      return true;
  }

  printf("Please Check DYNAMXEL Address Name('table')\n");
  return false;
}

//bool SingleDynamixelMonitor::checkValidAccess(std::string cmd)
//{
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;
//  int32_t torque_status = 0;

//  dynamixel_driver_->readRegister("torque_enable", &torque_status);

//  dynamixel->item_ = dynamixel->ctrl_table_[cmd];
//  if (dynamixel->item_->access_type == dynamixel_tool::READ_WRITE)
//  {
//    if ((torque_status == true) && (dynamixel->item_->memory_type == dynamixel_tool::EEPROM))
//    {
//      printf("address in EEPROM can't be accessed when torque is on\n");
//      printf("Check a 'table'");

//      return false;
//    }
//    return true;
//  }
//  else
//  {
//    return false;
//  }
//}

bool SingleDynamixelMonitor::changeId(uint8_t new_id)
{
  if (new_id > 0 && new_id < 254)
  {
    dynamixel_driver_->writeRegister(dxl_id_, "Torque Enable", false);

    dynamixel_driver_->writeRegister(dxl_id_, "ID", new_id);

    uint16_t model_number = 0;
    if (dynamixel_driver_->ping(new_id, &model_number))
      dxl_id_ = new_id;

    printf("...Succeeded to set dynamixel id [%u]\n", dxl_id_);
    return true;
  }
  else
  {
    printf("Dynamixel ID can be set 1~253\n");
    return false;
  }
}

//bool SingleDynamixelMonitor::changeBaudrate(uint64_t baud_rate)
//{
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  if (dynamixel->baud_rate_table_.find(baud_rate)->second == dynamixel->baud_rate_table_.end()->second)
//  {
//    printf(" Failed to change [ BAUD RATE: %ld ]\n", baud_rate);
//    printf(" Please check a valid baud rate at E-MANUAL\n");

//    if (dynamixel_driver_->getProtocolVersion() == 2.0)
//    {
//      dynamixel_driver_->writeRegister("baud_rate", dynamixel->baud_rate_table_.find(57600)->second);
//      usleep(dynamixel->item_->data_length* 55 * 1000 *10);

//      if (dynamixel_driver_->setBaudrate(57600) == false)
//      {
//        printf(" Failed to change default baudrate(57600)!\n");
//      }
//      else
//      {
//        printf(" Success to change default baudrate! [ BAUD RATE: 57600 ]\n");
//      }
//    }
//    else if (dynamixel_driver_->getProtocolVersion() == 1.0)
//    {
//      dynamixel_driver_->writeRegister("baud_rate", dynamixel->baud_rate_table_.find(1000000)->second);
//      usleep(dynamixel->item_->data_length* 55 * 1000 *10);

//      if (dynamixel_driver_->setBaudrate(1000000) == false)
//      {
//        printf(" Failed to change default baudrate(1000000)!\n");
//      }
//      else
//      {
//        printf(" Success to change default baudrate! [ BAUD RATE: 1000000 ]\n");
//      }
//    }

//    return false;
//  }
//  else
//  {
//    dynamixel_driver_->writeRegister("baud_rate", dynamixel->baud_rate_table_.find(baud_rate)->second);
//    usleep(dynamixel->item_->data_length* 55 * 1000 *10);

//    if (dynamixel_driver_->setBaudrate(baud_rate) == false)
//    {
//      printf(" Failed to change baudrate!\n");
//      return false;
//    }
//    else
//    {
//      printf(" Success to change baudrate! [ BAUD RATE: %d ]\n", dynamixel->baud_rate_table_.find(baud_rate)->first);
//      return true;
//    }
//  }
//}

//bool SingleDynamixelMonitor::changeProtocolVersion(float ver)
//{
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  if (ver == 1.0 || ver == 2.0)
//  {
//    // TODO
//    dynamixel_driver_->writeRegister("protocol_version", (int)(ver));
//    usleep(dynamixel->item_->data_length* 55 * 1000 *10);

//    dynamixel_driver_->setPacketHandler(ver);

//    printf(" Success to change protocol version [ PROTOCOL VERSION: %.1f]\n", dynamixel_driver_->getProtocolVersion());
//    return true;
//  }
//  else
//  {
//    printf(" Dynamixel has '1.0' or '2.0' protocol version\n");
//    return false;
//  }
//}

//bool SingleDynamixelMonitor::controlLoop()
//{
//  dynamixelStatePublish();

//  return true;
//}

bool SingleDynamixelMonitor::dynamixelInfoMsgCallback(dynamixel_workbench_msgs::GetDynamixelInfo::Request &req,
                                                   dynamixel_workbench_msgs::GetDynamixelInfo::Response &res)
{
  res.dynamixel_info.load_info.device_name      = device_name_;
  res.dynamixel_info.load_info.baud_rate        = dxl_baud_rate_;
  res.dynamixel_info.load_info.protocol_version = dynamixel_driver_->getProtocolVersion();

  res.dynamixel_info.model_id         = dxl_id_;
  res.dynamixel_info.model_name       = dynamixel_driver_->getModelName(dxl_id_);
  res.dynamixel_info.model_number     = dynamixel_driver_->getModelNum(dxl_id_);

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
    if (dynamixel_driver_->reboot(dxl_id_))
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "factory_reset")
  {
    if (dynamixel_driver_->reset(dxl_id_))
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "torque")
  {
    int32_t value = req.value;

    if (dynamixel_driver_->writeRegister(dxl_id_, "Torque Enable", value))
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "exit")
  {
    shutdownSingleDynamixelMonitor();
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

//    if (checkValidAccess(addr))
//    {
//      res.comm_result = true;
//    }
//    else
//    {
//      res.comm_result = false;
//      return true;
//    }

    if (addr == "ID")
    {
      if (changeId(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
//    else if (addr == "baud_rate")
//    {
//      if (changeBaudrate(value))
//        res.comm_result = true;
//      else
//        res.comm_result = false;
//    }
//    else if (addr == "protocol_version")
//    {
//      if (changeProtocolVersion(value))
//        res.comm_result = true;
//      else
//        res.comm_result = false;
//    }
    else
    {
      if (dynamixel_driver_->writeRegister(dxl_id_, addr.c_str(), value))
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
//    single_dynamixel_monitor.controlLoop();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


//bool SingleDynamixelMonitor::dynamixelStatePublish()
//{
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  if (dynamixel->model_name_.find("AX") != std::string::npos)
//  {
//    AX();
//  }
//  else if (dynamixel->model_name_.find("RX") != std::string::npos)
//  {
//    RX();
//  }
//  else if (dynamixel->model_name_.find("MX") != std::string::npos)
//  {
//    MX();
//  }
//  else if (dynamixel->model_name_.find("EX") != std::string::npos)
//  {
//    EX();
//  }
//  else if (dynamixel->model_name_.find("XL") != std::string::npos)
//  {
//    XL();
//  }
//  else if (dynamixel->model_name_.find("XM") != std::string::npos)
//  {
//    XM();
//  }
//  else if (dynamixel->model_name_.find("XH") != std::string::npos)
//  {
//    XH();
//  }
//  else if (dynamixel->model_name_.find("PRO") != std::string::npos)
//  {
//    PRO();
//  }

//  return true;
//}

//bool SingleDynamixelMonitor::AX()
//{
//  int32_t read_value = 0;

//  dynamixel_workbench_msgs::AX ax_state;
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
//       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
//       dynamixel->it_ctrl_++)
//  {
//    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
//    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

//    if ("model_number" == dynamixel->item_->item_name)
//      ax_state.model_number = read_value;
//    else if ("version_of_firmware" == dynamixel->item_->item_name)
//      ax_state.version_of_firmware = read_value;
//    else if ("id" == dynamixel->item_->item_name)
//      ax_state.id = read_value;
//    else if ("baud_rate" == dynamixel->item_->item_name)
//      ax_state.baud_rate = read_value;
//    else if ("return_delay_time" == dynamixel->item_->item_name)
//      ax_state.return_delay_time = read_value;
//    else if ("cw_angle_limit" == dynamixel->item_->item_name)
//      ax_state.cw_angle_limit = read_value;
//    else if ("ccw_angle_limit" == dynamixel->item_->item_name)
//      ax_state.ccw_angle_limit = read_value;
//    else if ("the_highest_limit_temperature" == dynamixel->item_->item_name)
//      ax_state.the_highest_limit_temperature = read_value;
//    else if ("the_lowest_limit_voltage" == dynamixel->item_->item_name)
//      ax_state.the_lowest_limit_voltage = read_value;
//    else if ("the_highest_limit_voltage" == dynamixel->item_->item_name)
//      ax_state.the_highest_limit_voltage = read_value;
//    else if ("max_torque" == dynamixel->item_->item_name)
//      ax_state.max_torque = read_value;
//    else if ("status_return_level" == dynamixel->item_->item_name)
//      ax_state.status_return_level = read_value;
//    else if ("alarm_led" == dynamixel->item_->item_name)
//      ax_state.alarm_led = read_value;
//    else if ("alarm_shutdown" == dynamixel->item_->item_name)
//      ax_state.alarm_shutdown = read_value;
//    else if ("torque_enable" == dynamixel->item_->item_name)
//      ax_state.torque_enable = read_value;
//    else if ("led" == dynamixel->item_->item_name)
//      ax_state.led = read_value;
//    else if ("cw_compliance_margin" == dynamixel->item_->item_name)
//      ax_state.cw_compliance_margin = read_value;
//    else if ("ccw_compliance_margin" == dynamixel->item_->item_name)
//      ax_state.ccw_compliance_margin = read_value;
//    else if ("cw_compliance_slope" == dynamixel->item_->item_name)
//      ax_state.cw_compliance_slope = read_value;
//    else if ("ccw_compliance_margin" == dynamixel->item_->item_name)
//      ax_state.ccw_compliance_margin = read_value;
//    else if ("goal_position" == dynamixel->item_->item_name)
//      ax_state.goal_position = read_value;
//    else if ("moving_speed" == dynamixel->item_->item_name)
//      ax_state.moving_speed = read_value;
//    else if ("torque_limit" == dynamixel->item_->item_name)
//      ax_state.torque_limit = read_value;
//    else if ("present_position" == dynamixel->item_->item_name)
//      ax_state.present_position = read_value;
//    else if ("present_speed" == dynamixel->item_->item_name)
//      ax_state.present_speed = read_value;
//    else if ("present_load" == dynamixel->item_->item_name)
//      ax_state.present_load = read_value;
//    else if ("present_voltage" == dynamixel->item_->item_name)
//      ax_state.present_voltage = read_value;
//    else if ("present_temperature" == dynamixel->item_->item_name)
//      ax_state.present_temperature = read_value;
//    else if ("registered" == dynamixel->item_->item_name)
//      ax_state.registered = read_value;
//    else if ("moving" == dynamixel->item_->item_name)
//      ax_state.moving = read_value;
//    else if ("lock" == dynamixel->item_->item_name)
//      ax_state.lock = read_value;
//    else if ("punch" == dynamixel->item_->item_name)
//      ax_state.punch = read_value;
//  }

//  dynamixel_status_pub_.publish(ax_state);

//  return true;
//}

//bool SingleDynamixelMonitor::RX()
//{
//  int32_t read_value = 0;

//  dynamixel_workbench_msgs::RX rx_state;
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
//       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
//       dynamixel->it_ctrl_++)
//  {
//    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
//    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

//    if ("model_number" == dynamixel->item_->item_name)
//      rx_state.model_number = read_value;
//    else if ("version_of_firmware" == dynamixel->item_->item_name)
//      rx_state.version_of_firmware = read_value;
//    else if ("id" == dynamixel->item_->item_name)
//      rx_state.id = read_value;
//    else if ("baud_rate" == dynamixel->item_->item_name)
//      rx_state.baud_rate = read_value;
//    else if ("return_delay_time" == dynamixel->item_->item_name)
//      rx_state.return_delay_time = read_value;
//    else if ("cw_angle_limit" == dynamixel->item_->item_name)
//      rx_state.cw_angle_limit = read_value;
//    else if ("ccw_angle_limit" == dynamixel->item_->item_name)
//      rx_state.ccw_angle_limit = read_value;
//    else if ("the_highest_limit_temperature" == dynamixel->item_->item_name)
//      rx_state.the_highest_limit_temperature = read_value;
//    else if ("the_lowest_limit_voltage" == dynamixel->item_->item_name)
//      rx_state.the_lowest_limit_voltage = read_value;
//    else if ("the_highest_limit_voltage" == dynamixel->item_->item_name)
//      rx_state.the_highest_limit_voltage = read_value;
//    else if ("max_torque" == dynamixel->item_->item_name)
//      rx_state.max_torque = read_value;
//    else if ("status_return_level" == dynamixel->item_->item_name)
//      rx_state.status_return_level = read_value;
//    else if ("alarm_led" == dynamixel->item_->item_name)
//      rx_state.alarm_led = read_value;
//    else if ("alarm_shutdown" == dynamixel->item_->item_name)
//      rx_state.alarm_shutdown = read_value;
//    else if ("torque_enable" == dynamixel->item_->item_name)
//      rx_state.torque_enable = read_value;
//    else if ("led" == dynamixel->item_->item_name)
//      rx_state.led = read_value;
//    else if ("cw_compliance_margin" == dynamixel->item_->item_name)
//      rx_state.cw_compliance_margin = read_value;
//    else if ("ccw_compliance_margin" == dynamixel->item_->item_name)
//      rx_state.ccw_compliance_margin = read_value;
//    else if ("cw_compliance_slope" == dynamixel->item_->item_name)
//      rx_state.cw_compliance_slope = read_value;
//    else if ("ccw_compliance_margin" == dynamixel->item_->item_name)
//      rx_state.ccw_compliance_margin = read_value;
//    else if ("goal_position" == dynamixel->item_->item_name)
//      rx_state.goal_position = read_value;
//    else if ("moving_speed" == dynamixel->item_->item_name)
//      rx_state.moving_speed = read_value;
//    else if ("torque_limit" == dynamixel->item_->item_name)
//      rx_state.torque_limit = read_value;
//    else if ("present_position" == dynamixel->item_->item_name)
//      rx_state.present_position = read_value;
//    else if ("present_speed" == dynamixel->item_->item_name)
//      rx_state.present_speed = read_value;
//    else if ("present_load" == dynamixel->item_->item_name)
//      rx_state.present_load = read_value;
//    else if ("present_voltage" == dynamixel->item_->item_name)
//      rx_state.present_voltage = read_value;
//    else if ("present_temperature" == dynamixel->item_->item_name)
//      rx_state.present_temperature = read_value;
//    else if ("registered" == dynamixel->item_->item_name)
//      rx_state.registered = read_value;
//    else if ("moving" == dynamixel->item_->item_name)
//      rx_state.moving = read_value;
//    else if ("lock" == dynamixel->item_->item_name)
//      rx_state.lock = read_value;
//    else if ("punch" == dynamixel->item_->item_name)
//      rx_state.punch = read_value;
//  }

//  dynamixel_status_pub_.publish(rx_state);
//}

//bool SingleDynamixelMonitor::MX()
//{
//  int32_t read_value = 0;

//  dynamixel_workbench_msgs::MX mx_state;
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
//       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
//       dynamixel->it_ctrl_++)
//  {
//    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
//    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

//    if ("model_number" == dynamixel->item_->item_name)
//      mx_state.model_number = read_value;
//    else if ("version_of_firmware" == dynamixel->item_->item_name)
//      mx_state.version_of_firmware = read_value;
//    else if ("id" == dynamixel->item_->item_name)
//      mx_state.id = read_value;
//    else if ("baud_rate" == dynamixel->item_->item_name)
//      mx_state.baud_rate = read_value;
//    else if ("return_delay_time" == dynamixel->item_->item_name)
//      mx_state.return_delay_time = read_value;
//    else if ("cw_angle_limit" == dynamixel->item_->item_name)
//      mx_state.cw_angle_limit = read_value;
//    else if ("ccw_angle_limit" == dynamixel->item_->item_name)
//      mx_state.ccw_angle_limit = read_value;
//    else if ("drive_mode" == dynamixel->item_->item_name)
//      mx_state.drive_mode = read_value;
//    else if ("the_highest_limit_temperature" == dynamixel->item_->item_name)
//      mx_state.the_highest_limit_temperature = read_value;
//    else if ("the_lowest_limit_voltage" == dynamixel->item_->item_name)
//      mx_state.the_lowest_limit_voltage = read_value;
//    else if ("the_highest_limit_voltage" == dynamixel->item_->item_name)
//      mx_state.the_highest_limit_voltage = read_value;
//    else if ("max_torque" == dynamixel->item_->item_name)
//      mx_state.max_torque = read_value;
//    else if ("status_return_level" == dynamixel->item_->item_name)
//      mx_state.status_return_level = read_value;
//    else if ("alarm_led" == dynamixel->item_->item_name)
//      mx_state.alarm_led = read_value;
//    else if ("alarm_shutdown" == dynamixel->item_->item_name)
//      mx_state.alarm_shutdown = read_value;
//    else if ("multi_turn_offset" == dynamixel->item_->item_name)
//      mx_state.multi_turn_offset = read_value;
//    else if ("resolution_divider" == dynamixel->item_->item_name)
//      mx_state.resolution_divider = read_value;
//    else if ("torque_enable" == dynamixel->item_->item_name)
//      mx_state.torque_enable = read_value;
//    else if ("led" == dynamixel->item_->item_name)
//      mx_state.led = read_value;
//    else if ("d_gain" == dynamixel->item_->item_name)
//      mx_state.d_gain = read_value;
//    else if ("i_gain" == dynamixel->item_->item_name)
//      mx_state.i_gain = read_value;
//    else if ("p_gain" == dynamixel->item_->item_name)
//      mx_state.p_gain = read_value;
//    else if ("goal_position" == dynamixel->item_->item_name)
//      mx_state.goal_position = read_value;
//    else if ("moving_speed" == dynamixel->item_->item_name)
//      mx_state.moving_speed = read_value;
//    else if ("torque_limit" == dynamixel->item_->item_name)
//      mx_state.torque_limit = read_value;
//    else if ("present_position" == dynamixel->item_->item_name)
//      mx_state.present_position = read_value;
//    else if ("present_speed" == dynamixel->item_->item_name)
//      mx_state.present_speed = read_value;
//    else if ("present_load" == dynamixel->item_->item_name)
//      mx_state.present_load = read_value;
//    else if ("present_voltage" == dynamixel->item_->item_name)
//      mx_state.present_voltage = read_value;
//    else if ("present_temperature" == dynamixel->item_->item_name)
//      mx_state.present_temperature = read_value;
//    else if ("registered" == dynamixel->item_->item_name)
//      mx_state.registered = read_value;
//    else if ("moving" == dynamixel->item_->item_name)
//      mx_state.moving = read_value;
//    else if ("lock" == dynamixel->item_->item_name)
//      mx_state.lock = read_value;
//    else if ("punch" == dynamixel->item_->item_name)
//      mx_state.punch = read_value;
//    else if ("current" == dynamixel->item_->item_name)
//      mx_state.current = read_value;
//    else if ("torque_control_mode_enable" == dynamixel->item_->item_name)
//      mx_state.torque_control_mode_enable = read_value;
//    else if ("goal_torque" == dynamixel->item_->item_name)
//      mx_state.goal_torque = read_value;
//    else if ("goal_acceleration" == dynamixel->item_->item_name)
//      mx_state.goal_acceleration = read_value;
//}

//  dynamixel_status_pub_.publish(mx_state);
//}

//bool SingleDynamixelMonitor::EX()
//{
//  int32_t read_value = 0;

//  dynamixel_workbench_msgs::EX ex_state;
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
//       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
//       dynamixel->it_ctrl_++)
//  {
//    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
//    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

//    if ("model_number" == dynamixel->item_->item_name)
//      ex_state.model_number = read_value;
//    else if ("version_of_firmware" == dynamixel->item_->item_name)
//      ex_state.version_of_firmware = read_value;
//    else if ("id" == dynamixel->item_->item_name)
//      ex_state.id = read_value;
//    else if ("baud_rate" == dynamixel->item_->item_name)
//      ex_state.baud_rate = read_value;
//    else if ("return_delay_time" == dynamixel->item_->item_name)
//      ex_state.return_delay_time = read_value;
//    else if ("cw_angle_limit" == dynamixel->item_->item_name)
//      ex_state.cw_angle_limit = read_value;
//    else if ("ccw_angle_limit" == dynamixel->item_->item_name)
//      ex_state.ccw_angle_limit = read_value;
//    else if ("drive_mode" == dynamixel->item_->item_name)
//      ex_state.drive_mode = read_value;
//    else if ("the_highest_limit_temperature" == dynamixel->item_->item_name)
//      ex_state.the_highest_limit_temperature = read_value;
//    else if ("the_lowest_limit_voltage" == dynamixel->item_->item_name)
//      ex_state.the_lowest_limit_voltage = read_value;
//    else if ("the_highest_limit_voltage" == dynamixel->item_->item_name)
//      ex_state.the_highest_limit_voltage = read_value;
//    else if ("max_torque" == dynamixel->item_->item_name)
//      ex_state.max_torque = read_value;
//    else if ("status_return_level" == dynamixel->item_->item_name)
//      ex_state.status_return_level = read_value;
//    else if ("alarm_led" == dynamixel->item_->item_name)
//      ex_state.alarm_led = read_value;
//    else if ("alarm_shutdown" == dynamixel->item_->item_name)
//      ex_state.alarm_shutdown = read_value;
//    else if ("torque_enable" == dynamixel->item_->item_name)
//      ex_state.torque_enable = read_value;
//    else if ("led" == dynamixel->item_->item_name)
//      ex_state.led = read_value;
//    else if ("cw_compliance_margin" == dynamixel->item_->item_name)
//      ex_state.cw_compliance_margin = read_value;
//    else if ("ccw_compliance_margin" == dynamixel->item_->item_name)
//      ex_state.ccw_compliance_margin = read_value;
//    else if ("cw_compliance_slope" == dynamixel->item_->item_name)
//      ex_state.cw_compliance_slope = read_value;
//    else if ("ccw_compliance_slope" == dynamixel->item_->item_name)
//      ex_state.ccw_compliance_slope = read_value;
//    else if ("goal_position" == dynamixel->item_->item_name)
//      ex_state.goal_position = read_value;
//    else if ("moving_speed" == dynamixel->item_->item_name)
//      ex_state.moving_speed = read_value;
//    else if ("torque_limit" == dynamixel->item_->item_name)
//      ex_state.torque_limit = read_value;
//    else if ("present_position" == dynamixel->item_->item_name)
//      ex_state.present_position = read_value;
//    else if ("present_velocity" == dynamixel->item_->item_name)
//      ex_state.present_velocity = read_value;
//    else if ("present_load" == dynamixel->item_->item_name)
//      ex_state.present_load = read_value;
//    else if ("present_voltage" == dynamixel->item_->item_name)
//      ex_state.present_voltage = read_value;
//    else if ("present_temperature" == dynamixel->item_->item_name)
//      ex_state.present_temperature = read_value;
//    else if ("registered" == dynamixel->item_->item_name)
//      ex_state.registered = read_value;
//    else if ("moving" == dynamixel->item_->item_name)
//      ex_state.moving = read_value;
//    else if ("lock" == dynamixel->item_->item_name)
//      ex_state.lock = read_value;
//    else if ("punch" == dynamixel->item_->item_name)
//      ex_state.punch = read_value;
//    else if ("sensed_current" == dynamixel->item_->item_name)
//      ex_state.sensed_current = read_value;
//}

//  dynamixel_status_pub_.publish(ex_state);
//}

//bool SingleDynamixelMonitor::XL()
//{
//  int32_t read_value = 0;

//  dynamixel_workbench_msgs::XL xl_state;
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
//       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
//       dynamixel->it_ctrl_++)
//  {
//    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
//    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

//    if ("model_number" == dynamixel->item_->item_name)
//      xl_state.model_number = read_value;
//    else if ("version_of_firmware" == dynamixel->item_->item_name)
//      xl_state.version_of_firmware = read_value;
//    else if ("id" == dynamixel->item_->item_name)
//      xl_state.id = read_value;
//    else if ("baud_rate" == dynamixel->item_->item_name)
//      xl_state.baud_rate = read_value;
//    else if ("return_delay_time" == dynamixel->item_->item_name)
//      xl_state.return_delay_time = read_value;
//    else if ("drive_mode" == dynamixel->item_->item_name)
//      xl_state.drive_mode = read_value;
//    else if ("operating_mode" == dynamixel->item_->item_name)
//      xl_state.operating_mode = read_value;
//    else if ("secondary_id" == dynamixel->item_->item_name)
//      xl_state.secondary_id = read_value;
//    else if ("protocol_version" == dynamixel->item_->item_name)
//      xl_state.protocol_version = read_value;
//    else if ("homing_offset" == dynamixel->item_->item_name)
//      xl_state.homing_offset = read_value;
//    else if ("moving_threshold" == dynamixel->item_->item_name)
//      xl_state.moving_threshold = read_value;
//    else if ("max_temperature_limit" == dynamixel->item_->item_name)
//      xl_state.temperature_limit = read_value;
//    else if ("max_voltage_limit" == dynamixel->item_->item_name)
//      xl_state.max_voltage_limit = read_value;
//    else if ("min_voltage_limit" == dynamixel->item_->item_name)
//      xl_state.min_voltage_limit = read_value;
//    else if ("pwm_limit" == dynamixel->item_->item_name)
//      xl_state.pwm_limit = read_value;
//    else if ("acceleration_limit" == dynamixel->item_->item_name)
//      xl_state.acceleration_limit = read_value;
//    else if ("velocity_limit" == dynamixel->item_->item_name)
//      xl_state.velocity_limit = read_value;
//    else if ("max_position_limit" == dynamixel->item_->item_name)
//      xl_state.max_position_limit = read_value;
//    else if ("min_position_limit" == dynamixel->item_->item_name)
//      xl_state.min_position_limit = read_value;
//    else if ("shutdown" == dynamixel->item_->item_name)
//      xl_state.shutdown = read_value;
//    else if ("torque_enable" == dynamixel->item_->item_name)
//      xl_state.torque_enable = read_value;
//    else if ("led" == dynamixel->item_->item_name)
//      xl_state.led = read_value;
//    else if ("status_return_level" == dynamixel->item_->item_name)
//      xl_state.status_return_level = read_value;
//    else if ("registered_instruction" == dynamixel->item_->item_name)
//      xl_state.registered_instruction = read_value;
//    else if ("hardware_error_status" == dynamixel->item_->item_name)
//      xl_state.hardware_error_status = read_value;
//    else if ("velocity_i_gain" == dynamixel->item_->item_name)
//      xl_state.velocity_i_gain = read_value;
//    else if ("velocity_p_gain" == dynamixel->item_->item_name)
//      xl_state.velocity_p_gain = read_value;
//    else if ("position_d_gain" == dynamixel->item_->item_name)
//      xl_state.position_d_gain = read_value;
//    else if ("position_i_gain" == dynamixel->item_->item_name)
//      xl_state.position_i_gain = read_value;
//    else if ("position_p_gain" == dynamixel->item_->item_name)
//      xl_state.position_p_gain = read_value;
//    else if ("feedforward_2nd_gain" == dynamixel->item_->item_name)
//      xl_state.feedforward_2nd_gain = read_value;
//    else if ("feedforward_1st_gain" == dynamixel->item_->item_name)
//      xl_state.feedforward_1st_gain = read_value;
//    else if ("bus_watchdog" == dynamixel->item_->item_name)
//      xl_state.bus_watchdog = read_value;
//    else if ("goal_pwm" == dynamixel->item_->item_name)
//      xl_state.goal_pwm = read_value;
//    else if ("goal_velocity" == dynamixel->item_->item_name)
//      xl_state.goal_velocity = read_value;
//    else if ("profile_acceleration" == dynamixel->item_->item_name)
//      xl_state.profile_acceleration = read_value;
//    else if ("profile_velocity" == dynamixel->item_->item_name)
//      xl_state.profile_velocity = read_value;
//    else if ("goal_position" == dynamixel->item_->item_name)
//      xl_state.goal_position = read_value;
//    else if ("realtime_tick" == dynamixel->item_->item_name)
//      xl_state.realtime_tick = read_value;
//    else if ("moving" == dynamixel->item_->item_name)
//      xl_state.moving = read_value;
//    else if ("moving_status" == dynamixel->item_->item_name)
//      xl_state.moving_status = read_value;
//    else if ("present_pwm" == dynamixel->item_->item_name)
//      xl_state.present_pwm = read_value;
//    else if ("present_load" == dynamixel->item_->item_name)
//      xl_state.present_load = read_value;
//    else if ("present_current" == dynamixel->item_->item_name)
//      xl_state.present_current = read_value;
//    else if ("present_velocity" == dynamixel->item_->item_name)
//      xl_state.present_velocity = read_value;
//    else if ("present_position" == dynamixel->item_->item_name)
//      xl_state.present_position = read_value;
//    else if ("velocity_trajectory" == dynamixel->item_->item_name)
//      xl_state.velocity_trajectory = read_value;
//    else if ("position_trajectory" == dynamixel->item_->item_name)
//      xl_state.position_trajectory = read_value;
//    else if ("present_input_voltage" == dynamixel->item_->item_name)
//      xl_state.present_input_voltage = read_value;
//    else if ("present_temperature" == dynamixel->item_->item_name)
//      xl_state.present_temperature = read_value;
//  }

//  dynamixel_status_pub_.publish(xl_state);
//}

//bool SingleDynamixelMonitor::XM()
//{
//  int32_t read_value = 0;

//  dynamixel_workbench_msgs::XM xm_state;
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
//       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
//       dynamixel->it_ctrl_++)
//  {
//    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
//    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

//    if ("model_number" == dynamixel->item_->item_name)
//      xm_state.model_number = read_value;
//    else if ("version_of_firmware" == dynamixel->item_->item_name)
//      xm_state.version_of_firmware = read_value;
//    else if ("id" == dynamixel->item_->item_name)
//      xm_state.id = read_value;
//    else if ("baud_rate" == dynamixel->item_->item_name)
//      xm_state.baud_rate = read_value;
//    else if ("return_delay_time" == dynamixel->item_->item_name)
//      xm_state.return_delay_time = read_value;
//    else if ("drive_mode" == dynamixel->item_->item_name)
//      xm_state.drive_mode = read_value;
//    else if ("operating_mode" == dynamixel->item_->item_name)
//      xm_state.operating_mode = read_value;
//    else if ("secondary_id" == dynamixel->item_->item_name)
//      xm_state.secondary_id = read_value;
//    else if ("protocol_version" == dynamixel->item_->item_name)
//      xm_state.protocol_version = read_value;
//    else if ("homing_offset" == dynamixel->item_->item_name)
//      xm_state.homing_offset = read_value;
//    else if ("moving_threshold" == dynamixel->item_->item_name)
//      xm_state.moving_threshold = read_value;
//    else if ("temperature_limit" == dynamixel->item_->item_name)
//      xm_state.temperature_limit = read_value;
//    else if ("max_voltage_limit" == dynamixel->item_->item_name)
//      xm_state.max_voltage_limit = read_value;
//    else if ("min_voltage_limit" == dynamixel->item_->item_name)
//      xm_state.min_voltage_limit = read_value;
//    else if ("pwm_limit" == dynamixel->item_->item_name)
//      xm_state.pwm_limit = read_value;
//    else if ("current_limit" == dynamixel->item_->item_name)
//      xm_state.current_limit = read_value;
//    else if ("acceleration_limit" == dynamixel->item_->item_name)
//      xm_state.acceleration_limit = read_value;
//    else if ("velocity_limit" == dynamixel->item_->item_name)
//      xm_state.velocity_limit = read_value;
//    else if ("max_position_limit" == dynamixel->item_->item_name)
//      xm_state.max_position_limit = read_value;
//    else if ("min_position_limit" == dynamixel->item_->item_name)
//      xm_state.min_position_limit = read_value;
//    else if ("shutdown" == dynamixel->item_->item_name)
//      xm_state.shutdown = read_value;
//    else if ("torque_enable" == dynamixel->item_->item_name)
//      xm_state.torque_enable = read_value;
//    else if ("led" == dynamixel->item_->item_name)
//      xm_state.led = read_value;
//    else if ("status_return_level" == dynamixel->item_->item_name)
//      xm_state.status_return_level = read_value;
//    else if ("registered_instruction" == dynamixel->item_->item_name)
//      xm_state.registered_instruction = read_value;
//    else if ("hardware_error_status" == dynamixel->item_->item_name)
//      xm_state.hardware_error_status = read_value;
//    else if ("velocity_i_gain" == dynamixel->item_->item_name)
//      xm_state.velocity_i_gain = read_value;
//    else if ("velocity_p_gain" == dynamixel->item_->item_name)
//      xm_state.velocity_p_gain = read_value;
//    else if ("position_d_gain" == dynamixel->item_->item_name)
//      xm_state.position_d_gain = read_value;
//    else if ("position_i_gain" == dynamixel->item_->item_name)
//      xm_state.position_i_gain = read_value;
//    else if ("position_p_gain" == dynamixel->item_->item_name)
//      xm_state.position_p_gain = read_value;
//    else if ("feedforward_2nd_gain" == dynamixel->item_->item_name)
//      xm_state.feedforward_2nd_gain = read_value;
//    else if ("feedforward_1st_gain" == dynamixel->item_->item_name)
//      xm_state.feedforward_1st_gain = read_value;
//    else if ("bus_watchdog" == dynamixel->item_->item_name)
//      xm_state.bus_watchdog = read_value;
//    else if ("goal_pwm" == dynamixel->item_->item_name)
//      xm_state.goal_pwm = read_value;
//    else if ("goal_current" == dynamixel->item_->item_name)
//      xm_state.goal_current = read_value;
//    else if ("goal_velocity" == dynamixel->item_->item_name)
//      xm_state.goal_velocity = read_value;
//    else if ("profile_acceleration" == dynamixel->item_->item_name)
//      xm_state.profile_acceleration = read_value;
//    else if ("profile_velocity" == dynamixel->item_->item_name)
//      xm_state.profile_velocity = read_value;
//    else if ("goal_position" == dynamixel->item_->item_name)
//      xm_state.goal_position = read_value;
//    else if ("realtime_tick" == dynamixel->item_->item_name)
//      xm_state.realtime_tick = read_value;
//    else if ("moving" == dynamixel->item_->item_name)
//      xm_state.moving = read_value;
//    else if ("moving_status" == dynamixel->item_->item_name)
//      xm_state.moving_status = read_value;
//    else if ("present_pwm" == dynamixel->item_->item_name)
//      xm_state.present_pwm = read_value;
//    else if ("present_current" == dynamixel->item_->item_name)
//      xm_state.present_current = read_value;
//    else if ("present_velocity" == dynamixel->item_->item_name)
//      xm_state.present_velocity = read_value;
//    else if ("present_position" == dynamixel->item_->item_name)
//      xm_state.present_position = read_value;
//    else if ("velocity_trajectory" == dynamixel->item_->item_name)
//      xm_state.velocity_trajectory = read_value;
//    else if ("position_trajectory" == dynamixel->item_->item_name)
//      xm_state.position_trajectory = read_value;
//    else if ("present_input_voltage" == dynamixel->item_->item_name)
//      xm_state.present_input_voltage = read_value;
//    else if ("present_temperature" == dynamixel->item_->item_name)
//      xm_state.present_temperature = read_value;
//  }

//  dynamixel_status_pub_.publish(xm_state);
//}

//bool SingleDynamixelMonitor::XH()
//{
//  int32_t read_value = 0;

//  dynamixel_workbench_msgs::XH xh_state;
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
//       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
//       dynamixel->it_ctrl_++)
//  {
//    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
//    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

//    if ("model_number" == dynamixel->item_->item_name)
//      xh_state.model_number = read_value;
//    else if ("version_of_firmware" == dynamixel->item_->item_name)
//      xh_state.version_of_firmware = read_value;
//    else if ("id" == dynamixel->item_->item_name)
//      xh_state.id = read_value;
//    else if ("baud_rate" == dynamixel->item_->item_name)
//      xh_state.baud_rate = read_value;
//    else if ("return_delay_time" == dynamixel->item_->item_name)
//      xh_state.return_delay_time = read_value;
//    else if ("drive_mode" == dynamixel->item_->item_name)
//      xh_state.drive_mode = read_value;
//    else if ("operating_mode" == dynamixel->item_->item_name)
//      xh_state.operating_mode = read_value;
//    else if ("secondary_id" == dynamixel->item_->item_name)
//      xh_state.secondary_id = read_value;
//    else if ("protocol_version" == dynamixel->item_->item_name)
//      xh_state.protocol_version = read_value;
//    else if ("homing_offset" == dynamixel->item_->item_name)
//      xh_state.homing_offset = read_value;
//    else if ("moving_threshold" == dynamixel->item_->item_name)
//      xh_state.moving_threshold = read_value;
//    else if ("temperature_limit" == dynamixel->item_->item_name)
//      xh_state.temperature_limit = read_value;
//    else if ("max_voltage_limit" == dynamixel->item_->item_name)
//      xh_state.max_voltage_limit = read_value;
//    else if ("min_voltage_limit" == dynamixel->item_->item_name)
//      xh_state.min_voltage_limit = read_value;
//    else if ("pwm_limit" == dynamixel->item_->item_name)
//      xh_state.pwm_limit = read_value;
//    else if ("current_limit" == dynamixel->item_->item_name)
//      xh_state.current_limit = read_value;
//    else if ("acceleration_limit" == dynamixel->item_->item_name)
//      xh_state.acceleration_limit = read_value;
//    else if ("velocity_limit" == dynamixel->item_->item_name)
//      xh_state.velocity_limit = read_value;
//    else if ("max_position_limit" == dynamixel->item_->item_name)
//      xh_state.max_position_limit = read_value;
//    else if ("min_position_limit" == dynamixel->item_->item_name)
//      xh_state.min_position_limit = read_value;
//    else if ("shutdown" == dynamixel->item_->item_name)
//      xh_state.shutdown = read_value;
//    else if ("torque_enable" == dynamixel->item_->item_name)
//      xh_state.torque_enable = read_value;
//    else if ("led" == dynamixel->item_->item_name)
//      xh_state.led = read_value;
//    else if ("status_return_level" == dynamixel->item_->item_name)
//      xh_state.status_return_level = read_value;
//    else if ("registered_instruction" == dynamixel->item_->item_name)
//      xh_state.registered_instruction = read_value;
//    else if ("hardware_error_status" == dynamixel->item_->item_name)
//      xh_state.hardware_error_status = read_value;
//    else if ("velocity_i_gain" == dynamixel->item_->item_name)
//      xh_state.velocity_i_gain = read_value;
//    else if ("velocity_p_gain" == dynamixel->item_->item_name)
//      xh_state.velocity_p_gain = read_value;
//    else if ("position_d_gain" == dynamixel->item_->item_name)
//      xh_state.position_d_gain = read_value;
//    else if ("position_i_gain" == dynamixel->item_->item_name)
//      xh_state.position_i_gain = read_value;
//    else if ("position_p_gain" == dynamixel->item_->item_name)
//      xh_state.position_p_gain = read_value;
//    else if ("feedforward_2nd_gain" == dynamixel->item_->item_name)
//      xh_state.feedforward_2nd_gain = read_value;
//    else if ("feedforward_1st_gain" == dynamixel->item_->item_name)
//      xh_state.feedforward_1st_gain = read_value;
//    else if ("bus_watchdog" == dynamixel->item_->item_name)
//      xh_state.bus_watchdog = read_value;
//    else if ("goal_pwm" == dynamixel->item_->item_name)
//      xh_state.goal_pwm = read_value;
//    else if ("goal_current" == dynamixel->item_->item_name)
//      xh_state.goal_current = read_value;
//    else if ("goal_velocity" == dynamixel->item_->item_name)
//      xh_state.goal_velocity = read_value;
//    else if ("profile_acceleration" == dynamixel->item_->item_name)
//      xh_state.profile_acceleration = read_value;
//    else if ("profile_velocity" == dynamixel->item_->item_name)
//      xh_state.profile_velocity = read_value;
//    else if ("goal_position" == dynamixel->item_->item_name)
//      xh_state.goal_position = read_value;
//    else if ("realtime_tick" == dynamixel->item_->item_name)
//      xh_state.realtime_tick = read_value;
//    else if ("moving" == dynamixel->item_->item_name)
//      xh_state.moving = read_value;
//    else if ("moving_status" == dynamixel->item_->item_name)
//      xh_state.moving_status = read_value;
//    else if ("present_pwm" == dynamixel->item_->item_name)
//      xh_state.present_pwm = read_value;
//    else if ("present_current" == dynamixel->item_->item_name)
//      xh_state.present_current = read_value;
//    else if ("present_velocity" == dynamixel->item_->item_name)
//      xh_state.present_velocity = read_value;
//    else if ("present_position" == dynamixel->item_->item_name)
//      xh_state.present_position = read_value;
//    else if ("velocity_trajectory" == dynamixel->item_->item_name)
//      xh_state.velocity_trajectory = read_value;
//    else if ("position_trajectory" == dynamixel->item_->item_name)
//      xh_state.position_trajectory = read_value;
//    else if ("present_input_voltage" == dynamixel->item_->item_name)
//      xh_state.present_input_voltage = read_value;
//    else if ("present_temperature" == dynamixel->item_->item_name)
//      xh_state.present_temperature = read_value;
//  }

//  dynamixel_status_pub_.publish(xh_state);
//}

//bool SingleDynamixelMonitor::PRO()
//{
//  int32_t read_value = 0;

//  dynamixel_workbench_msgs::PRO pro_state;
//  dynamixel_tool::DynamixelTool *dynamixel = dynamixel_driver_->dynamixel_;

//  for (dynamixel->it_ctrl_ = dynamixel->ctrl_table_.begin();
//       dynamixel->it_ctrl_ != dynamixel->ctrl_table_.end();
//       dynamixel->it_ctrl_++)
//  {
//    dynamixel->item_ = dynamixel->ctrl_table_[dynamixel->it_ctrl_->first.c_str()];
//    dynamixel_driver_->readRegister(dynamixel->item_->item_name ,&read_value);

//    if ("model_number" == dynamixel->item_->item_name)
//      pro_state.model_number = read_value;
//    else if ("version_of_firmware" == dynamixel->item_->item_name)
//      pro_state.version_of_firmware = read_value;
//    else if ("id" == dynamixel->item_->item_name)
//      pro_state.id = read_value;
//    else if ("baud_rate" == dynamixel->item_->item_name)
//      pro_state.baud_rate = read_value;
//    else if ("return_delay_time" == dynamixel->item_->item_name)
//      pro_state.return_delay_time = read_value;
//    else if ("operating_mode" == dynamixel->item_->item_name)
//      pro_state.operating_mode = read_value;
//    else if ("homing_offset" == dynamixel->item_->item_name)
//      pro_state.homing_offset = read_value;
//    else if ("moving_threshold" == dynamixel->item_->item_name)
//      pro_state.moving_threshold = read_value;
//    else if ("temperature_limit" == dynamixel->item_->item_name)
//      pro_state.temperature_limit = read_value;
//    else if ("max_voltage_limit" == dynamixel->item_->item_name)
//      pro_state.max_voltage_limit = read_value;
//    else if ("min_voltage_limit" == dynamixel->item_->item_name)
//      pro_state.min_voltage_limit = read_value;
//    else if ("acceleration_limit" == dynamixel->item_->item_name)
//      pro_state.acceleration_limit = read_value;
//    else if ("torque_limit" == dynamixel->item_->item_name)
//      pro_state.torque_limit = read_value;
//    else if ("velocity_limit" == dynamixel->item_->item_name)
//      pro_state.velocity_limit = read_value;
//    else if ("max_position_limit" == dynamixel->item_->item_name)
//      pro_state.max_position_limit = read_value;
//    else if ("min_position_limit" == dynamixel->item_->item_name)
//      pro_state.min_position_limit = read_value;
//    else if ("external_port_mod_1" == dynamixel->item_->item_name)
//      pro_state.external_port_mod_1 = read_value;
//    else if ("external_port_mod_2" == dynamixel->item_->item_name)
//      pro_state.external_port_mod_2 = read_value;
//    else if ("external_port_mod_3" == dynamixel->item_->item_name)
//      pro_state.external_port_mod_3 = read_value;
//    else if ("external_port_mod_4" == dynamixel->item_->item_name)
//      pro_state.external_port_mod_4 = read_value;
//    else if ("shutdown" == dynamixel->item_->item_name)
//      pro_state.shutdown = read_value;
//    else if ("indirect_address_1" == dynamixel->item_->item_name)
//      pro_state.indirect_address_1 = read_value;
//    else if ("torque_enable" == dynamixel->item_->item_name)
//      pro_state.torque_enable = read_value;
//    else if ("led_red" == dynamixel->item_->item_name)
//      pro_state.led_red = read_value;
//    else if ("led_green" == dynamixel->item_->item_name)
//      pro_state.led_green = read_value;
//    else if ("led_blue" == dynamixel->item_->item_name)
//      pro_state.led_blue = read_value;
//    else if ("velocity_i_gain" == dynamixel->item_->item_name)
//      pro_state.velocity_i_gain = read_value;
//    else if ("velocity_p_gain" == dynamixel->item_->item_name)
//      pro_state.velocity_p_gain = read_value;
//    else if ("position_p_gain" == dynamixel->item_->item_name)
//      pro_state.position_p_gain = read_value;
//    else if ("goal_position" == dynamixel->item_->item_name)
//      pro_state.goal_position = read_value;
//    else if ("goal_velocity" == dynamixel->item_->item_name)
//      pro_state.goal_velocity = read_value;
//    else if ("goal_torque" == dynamixel->item_->item_name)
//      pro_state.goal_torque = read_value;
//    else if ("goal_acceleration" == dynamixel->item_->item_name)
//      pro_state.goal_acceleration = read_value;
//    else if ("moving" == dynamixel->item_->item_name)
//      pro_state.moving = read_value;
//    else if ("present_position" == dynamixel->item_->item_name)
//      pro_state.present_position = read_value;
//    else if ("present_velocity" == dynamixel->item_->item_name)
//      pro_state.present_velocity = read_value;
//    else if ("present_current" == dynamixel->item_->item_name)
//      pro_state.present_current = read_value;
//    else if ("present_input_voltage" == dynamixel->item_->item_name)
//      pro_state.present_input_voltage = read_value;
//    else if ("present_temperature" == dynamixel->item_->item_name)
//      pro_state.present_temperature = read_value;
//    else if ("external_port_data_1" == dynamixel->item_->item_name)
//      pro_state.external_port_data_1 = read_value;
//    else if ("external_port_data_2" == dynamixel->item_->item_name)
//      pro_state.external_port_data_2 = read_value;
//    else if ("external_port_data_3" == dynamixel->item_->item_name)
//      pro_state.external_port_data_3 = read_value;
//    else if ("external_port_data_4" == dynamixel->item_->item_name)
//      pro_state.external_port_data_4 = read_value;
//    else if ("indirect_data_1" == dynamixel->item_->item_name)
//      pro_state.indirect_data_1 = read_value;
//    else if ("registered_instruction" == dynamixel->item_->item_name)
//      pro_state.registered_instruction = read_value;
//    else if ("status_return_level" == dynamixel->item_->item_name)
//      pro_state.status_return_level = read_value;
//    else if ("hardware_error_status" == dynamixel->item_->item_name)
//      pro_state.hardware_error_status = read_value;
//  }

//  dynamixel_status_pub_.publish(pro_state);
//}
