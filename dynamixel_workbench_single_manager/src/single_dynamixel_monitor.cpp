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

SingleDynamixelMonitor::SingleDynamixelMonitor(void)
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
      printf("Baudrate [ex : 9600, 57600, 115200, 1000000, 2000000]\n");
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
      printf("Baudrate [ex : 9600, 57600, 115200, 1000000, 2000000]\n");
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

SingleDynamixelMonitor::~SingleDynamixelMonitor(void)
{

}

void SingleDynamixelMonitor::initSingleDynamixelMonitor(void)
{

}

void SingleDynamixelMonitor::shutdownSingleDynamixelMonitor(void)
{
  dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", false);

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

void SingleDynamixelMonitor::initDynamixelInfoServer(void)
{
  dynamixel_info_server_ = node_handle_.advertiseService("dynamixel/info", &SingleDynamixelMonitor::dynamixelInfoMsgCallback, this);
}

void SingleDynamixelMonitor::initDynamixelCommandServer(void)
{
  dynamixel_command_server_ = node_handle_.advertiseService("dynamixel/command", &SingleDynamixelMonitor::dynamixelCommandMsgCallback, this);
}

bool SingleDynamixelMonitor::showDynamixelControlTable(void)
{
  bool isOK = false;
  int32_t torque_status = 0;
  uint16_t torque_enable_address = 0;
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);

  for (int item_num = 0; item_num < dynamixel_driver_->getTheNumberOfItem(dxl_id_); item_num++)
  {
    if (!strncmp(item_ptr[item_num].item_name, "Torque_Enable", strlen("Torque_Enable")))
    {
      torque_enable_address = item_num;
    }
  }

  isOK = dynamixel_driver_->readRegister(dxl_id_, "Torque_Enable", &torque_status);

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

  return isOK;
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

bool SingleDynamixelMonitor::changeId(uint8_t new_id)
{
  bool isOK = false;

  if (new_id > 0 && new_id < 254)
  {
    isOK = dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", false);

    isOK = dynamixel_driver_->writeRegister(dxl_id_, "ID", new_id);

    uint16_t model_number = 0;
    if (dynamixel_driver_->ping(new_id, &model_number))
      dxl_id_ = new_id;

    printf("...Succeeded to set dynamixel id [%u]\n", dxl_id_);
    return isOK;
  }
  else
  {
    printf("Dynamixel ID can be set 1~253\n");
    return isOK;
  }
}

bool SingleDynamixelMonitor::changeBaudrate(uint32_t new_baud_rate)
{
  bool isOK = false;
  bool check_baud_rate = false;

  uint64_t baud_rate_list[5] = {9600, 57600, 115200, 1000000, 2000000};

  for (int i = 0; i < 5; i++)
  {
    if (baud_rate_list[i] == new_baud_rate)
      check_baud_rate = true;
  }

  if (check_baud_rate == false)
  {
    printf("Failed to change [ BAUD RATE: %d ]\n", new_baud_rate);
    printf("Valid baud rate is [9600, 57600, 115200, 1000000, 2000000]\n");
    printf("You can choose other baud rate in GUI,\n");

    return false;
  }
  else
  {
    isOK = dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", false);

    if (dynamixel_driver_->getProtocolVersion() == 1.0)
    {
      if (new_baud_rate == 9600)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 207);
      else if (new_baud_rate == 57600)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 34);
      else if (new_baud_rate == 115200)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 16);
      else if (new_baud_rate == 1000000)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 1);
      else if (new_baud_rate == 2000000)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 9);
      else
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 1);
    }
    else if (dynamixel_driver_->getProtocolVersion() == 2.0)
    {
      if (new_baud_rate == 9600)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 0);
      else if (new_baud_rate == 57600)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 1);
      else if (new_baud_rate == 115200)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 2);
      else if (new_baud_rate == 1000000)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 3);
      else if (new_baud_rate == 2000000)
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 4);
      else
        isOK = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 3);
    }

    usleep(1000*1000);

    if (isOK)
    {
      bool error = false;
      dynamixel_driver_->setBaudrate(new_baud_rate, &error);

      if(error == false)
      {
        printf("Success to change baudrate! [ BAUD RATE: %d ]\n", new_baud_rate);
        return true;
      }
      else
      {
        printf("Failed to change baudrate!\n");
        return false;
      }
    }
    else
    {
      printf("Failed to change baudrate!\n");
      return false;
    }
  }
}

bool SingleDynamixelMonitor::changeProtocolVersion(float ver)
{
  bool error = false;
  bool isOK = false;

  if (ver == 1.0 || ver == 2.0)
  {
    isOK = dynamixel_driver_->writeRegister(dxl_id_, "Protocol_Version", (int)(ver));
    usleep(1000*1000);

    if (isOK)
    {
      bool error = false;
      dynamixel_driver_->setPacketHandler(ver, &error);

      if (error == false)
      {
        printf("Success to change protocol version [ PROTOCOL VERSION: %.1f]\n", dynamixel_driver_->getProtocolVersion());
        return true;
      }
      else
        return false;
    }
    else
    {
      return false;
    }
  }
  else
  {
    printf("Dynamixel supports protocol version [1.0] or [2.0]\n");
    return false;
  }
}

bool SingleDynamixelMonitor::controlLoop(void)
{
  dynamixelStatePublish();

  return true;
}

bool SingleDynamixelMonitor::dynamixelInfoMsgCallback(dynamixel_workbench_msgs::GetDynamixelInfo::Request &req,
                                                   dynamixel_workbench_msgs::GetDynamixelInfo::Response &res)
{
  res.dynamixel_info.load_info.device_name      = device_name_;
  res.dynamixel_info.load_info.baud_rate        = dynamixel_driver_->getBaudrate();
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
    {
      dxl_id_ = 1;
      res.comm_result = true;
    }
    else
      res.comm_result = false;
  }
  else if (req.command == "torque")
  {
    int32_t value = req.value;

    if (dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", value))
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
      return false;
    }

    if (addr == "ID")
    {
      if (changeId(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
    else if (addr == "Baud_Rate")
    {
      if (changeBaudrate(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
    else if (addr == "Protocol_Version")
    {
      if (changeProtocolVersion(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
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
    single_dynamixel_monitor.controlLoop();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


bool SingleDynamixelMonitor::dynamixelStatePublish(void)
{
  char* model_name = dynamixel_driver_->getModelName(dxl_id_);

  if (!strncmp(model_name, "AX", strlen("AX")))
  {
    AX();
  }
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

  return true;
}

void SingleDynamixelMonitor::AX(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::AX ax_state;

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    dynamixel_driver_->readRegister(dxl_id_, item_ptr[index].item_name, &read_value);

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      ax_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      ax_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      ax_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      ax_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      ax_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      ax_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      ax_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      ax_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      ax_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      ax_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      ax_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      ax_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      ax_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      ax_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      ax_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      ax_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Margin", strlen("CW_Compliance_Margin")))
      ax_state.CW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Margin", strlen("CCW_Compliance_Margin")))
      ax_state.CCW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Slope", strlen("CW_Compliance_Slope")))
      ax_state.CW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Slope", strlen("CCW_Compliance_Slope")))
      ax_state.CCW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      ax_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      ax_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      ax_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      ax_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      ax_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      ax_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      ax_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      ax_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      ax_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      ax_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      ax_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      ax_state.Punch = read_value;
  }

  dynamixel_status_pub_.publish(ax_state);
}

//bool SingleDynamixelMonitor::RX(void)
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

//bool SingleDynamixelMonitor::MX(void)
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

//bool SingleDynamixelMonitor::EX(void)
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

//bool SingleDynamixelMonitor::XL(void)
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

//bool SingleDynamixelMonitor::XM(void)
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

//bool SingleDynamixelMonitor::XH(void)
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

//bool SingleDynamixelMonitor::PRO(void)
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
