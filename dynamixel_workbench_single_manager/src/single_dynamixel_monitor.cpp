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

inline void millis(uint16_t msec)
{
  usleep(1000*msec);
}

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
    if (!strncmp(model_name, "MX-12W", strlen(model_name)) ||
        !strncmp(model_name, "MX-28" , strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MX>("dynamixel/" + std::string("MX"), 10);
    }
    else if (!strncmp(model_name, "MX-64", strlen(model_name)) ||
             !strncmp(model_name, "MX-106" , strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MXExt>("dynamixel/" + std::string("MX"), 10);
    }
    else if (!strncmp(model_name, "MX-28-2", strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MX2>("dynamixel/" + std::string("MX"), 10);
    }
    else if (!strncmp(model_name, "MX-64-2", strlen(model_name)) ||
             !strncmp(model_name, "MX-106-2" , strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MX2Ext>("dynamixel/" + std::string("MX"), 10);
    }
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
    millis(1000);

    uint16_t model_number = 0;
    if (isOK && dynamixel_driver_->ping(new_id, &model_number))
    {
      dxl_id_ = new_id;

      printf("...Succeeded to set Dynamixel ID [%u]\n", dxl_id_);
      return true;
    }
    else
    {
      printf("...Failed to change ID!\n");
      return false;
    }
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

    millis(2000);

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
    isOK = dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", false);

    isOK = dynamixel_driver_->writeRegister(dxl_id_, "Protocol_Version", (int)(ver));
    millis(2000);

    if (isOK)
    {
      bool error = false;
      dynamixel_driver_->setPacketHandler(ver, &error);

      if (error == false)
      {
        printf("Success to change protocol version [PROTOCOL VERSION: %.1f]\n", dynamixel_driver_->getProtocolVersion());
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

  if (!strncmp(model_name, "AX", strlen(model_name)))
  {
    AX();
  }
  else if (!strncmp(model_name, "RX", strlen(model_name)))
  {
    RX();
  }
  else if (!strncmp(model_name, "MX-12W", strlen(model_name)) ||
           !strncmp(model_name, "MX-28" , strlen(model_name)))
  {
    MX();
  }
  else if (!strncmp(model_name, "MX-64", strlen(model_name)) ||
           !strncmp(model_name, "MX-106" , strlen(model_name)))
  {
    MXExt();
  }
  else if (!strncmp(model_name, "MX-28-2", strlen(model_name)))
  {
    MX2();
  }
  else if (!strncmp(model_name, "MX-64-2", strlen(model_name)) ||
           !strncmp(model_name, "MX-106-2" , strlen(model_name)))
  {
    MX2Ext();
  }
//  else if (dynamixel->model_name_.find("EX") != std::string::npos)
//  {
//    EX();
//  }
  else if (!strncmp(model_name, "XL-320", strlen(model_name)))
  {
    XL320();
  }
  else if (!strncmp(model_name, "XL430-W250", strlen(model_name)))
  {
    XL();
  }
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

void SingleDynamixelMonitor::RX(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::RX rx_state;

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    dynamixel_driver_->readRegister(dxl_id_, item_ptr[index].item_name, &read_value);

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      rx_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      rx_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      rx_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      rx_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      rx_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      rx_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      rx_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      rx_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      rx_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      rx_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      rx_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      rx_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      rx_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      rx_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      rx_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      rx_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Margin", strlen("CW_Compliance_Margin")))
      rx_state.CW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Margin", strlen("CCW_Compliance_Margin")))
      rx_state.CCW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Slope", strlen("CW_Compliance_Slope")))
      rx_state.CW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Slope", strlen("CCW_Compliance_Slope")))
      rx_state.CCW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      rx_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      rx_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      rx_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      rx_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      rx_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      rx_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      rx_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      rx_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      rx_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      rx_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      rx_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      rx_state.Punch = read_value;
  }

  dynamixel_status_pub_.publish(rx_state);
}

void SingleDynamixelMonitor::MX(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::MX mx_state;

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    dynamixel_driver_->readRegister(dxl_id_, item_ptr[index].item_name, &read_value);

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      mx_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      mx_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      mx_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      mx_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      mx_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      mx_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      mx_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      mx_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      mx_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      mx_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      mx_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      mx_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      mx_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      mx_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Multi_Turn_Offset", strlen("Multi_Turn_Offset")))
      mx_state.Multi_Turn_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Resolution_Divider", strlen("Resolution_Divider")))
      mx_state.Resolution_Divider = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      mx_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      mx_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "D_gain", strlen("D_gain")))
      mx_state.D_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "I_gain", strlen("I_gain")))
      mx_state.I_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "P_gain", strlen("P_gain")))
      mx_state.P_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      mx_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      mx_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      mx_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      mx_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      mx_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      mx_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      mx_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      mx_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      mx_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      mx_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      mx_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      mx_state.Punch = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Acceleration", strlen("Goal_Acceleration")))
      mx_state.Goal_Acceleration = read_value;
  }

  dynamixel_status_pub_.publish(mx_state);
}

void SingleDynamixelMonitor::MXExt(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::MXExt mxext_state;

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    dynamixel_driver_->readRegister(dxl_id_, item_ptr[index].item_name, &read_value);

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      mxext_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      mxext_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      mxext_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      mxext_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      mxext_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      mxext_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      mxext_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      mxext_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      mxext_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      mxext_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      mxext_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      mxext_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      mxext_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      mxext_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Multi_Turn_Offset", strlen("Multi_Turn_Offset")))
      mxext_state.Multi_Turn_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Resolution_Divider", strlen("Resolution_Divider")))
      mxext_state.Resolution_Divider = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      mxext_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      mxext_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "D_gain", strlen("D_gain")))
      mxext_state.D_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "I_gain", strlen("I_gain")))
      mxext_state.I_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "P_gain", strlen("P_gain")))
      mxext_state.P_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      mxext_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      mxext_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      mxext_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      mxext_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      mxext_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      mxext_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      mxext_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      mxext_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      mxext_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      mxext_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      mxext_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      mxext_state.Punch = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Current", strlen("Current")))
      mxext_state.Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Control_Mode_Enable", strlen("Torque_Control_Mode_Enable")))
      mxext_state.Torque_Control_Mode_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Torque", strlen("Goal_Torque")))
      mxext_state.Goal_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Acceleration", strlen("Goal_Acceleration")))
      mxext_state.Goal_Acceleration = read_value;
  }

  dynamixel_status_pub_.publish(mxext_state);
}

void SingleDynamixelMonitor::MX2(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::MX2 mx2_state;

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    dynamixel_driver_->readRegister(dxl_id_, item_ptr[index].item_name, &read_value);

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      mx2_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      mx2_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      mx2_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      mx2_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      mx2_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      mx2_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      mx2_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      mx2_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      mx2_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      mx2_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      mx2_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      mx2_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      mx2_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      mx2_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      mx2_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      mx2_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      mx2_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      mx2_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      mx2_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      mx2_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      mx2_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      mx2_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      mx2_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      mx2_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      mx2_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      mx2_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      mx2_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      mx2_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      mx2_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      mx2_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      mx2_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      mx2_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      mx2_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      mx2_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      mx2_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      mx2_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      mx2_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      mx2_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      mx2_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      mx2_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      mx2_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      mx2_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      mx2_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      mx2_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      mx2_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      mx2_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      mx2_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      mx2_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      mx2_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(mx2_state);
}

void SingleDynamixelMonitor::MX2Ext(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::MX2Ext mx2ext_state;

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    dynamixel_driver_->readRegister(dxl_id_, item_ptr[index].item_name, &read_value);

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      mx2ext_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      mx2ext_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      mx2ext_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      mx2ext_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      mx2ext_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      mx2ext_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      mx2ext_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      mx2ext_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      mx2ext_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      mx2ext_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      mx2ext_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      mx2ext_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      mx2ext_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      mx2ext_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      mx2ext_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Current_Limit", strlen("Current_Limit")))
      mx2ext_state.Current_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      mx2ext_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      mx2ext_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      mx2ext_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      mx2ext_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      mx2ext_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      mx2ext_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      mx2ext_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      mx2ext_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      mx2ext_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      mx2ext_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      mx2ext_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      mx2ext_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      mx2ext_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      mx2ext_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      mx2ext_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      mx2ext_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      mx2ext_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      mx2ext_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      mx2ext_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Current", strlen("Goal_Current")))
      mx2ext_state.Goal_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      mx2ext_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      mx2ext_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      mx2ext_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      mx2ext_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      mx2ext_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      mx2ext_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      mx2ext_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      mx2ext_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Current", strlen("Present_Current")))
      mx2ext_state.Present_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      mx2ext_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      mx2ext_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      mx2ext_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      mx2ext_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      mx2ext_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      mx2ext_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(mx2ext_state);
}

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

void SingleDynamixelMonitor::XL320(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::XL320 xl320_state;

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    dynamixel_driver_->readRegister(dxl_id_, item_ptr[index].item_name, &read_value);

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      xl320_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      xl320_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      xl320_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      xl320_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      xl320_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      xl320_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      xl320_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      xl320_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      xl320_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      xl320_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      xl320_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      xl320_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      xl320_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      xl320_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      xl320_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "D_gain", strlen("D_gain")))
      xl320_state.D_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "I_gain", strlen("I_gain")))
      xl320_state.I_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "P_gain", strlen("P_gain")))
      xl320_state.P_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      xl320_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      xl320_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      xl320_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      xl320_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      xl320_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      xl320_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      xl320_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      xl320_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      xl320_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      xl320_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      xl320_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      xl320_state.Punch = read_value;
  }

  dynamixel_status_pub_.publish(xl320_state);
}

void SingleDynamixelMonitor::XL(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::XL xl_state;

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    dynamixel_driver_->readRegister(dxl_id_, item_ptr[index].item_name, &read_value);

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      xl_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      xl_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      xl_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      xl_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      xl_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      xl_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      xl_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      xl_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      xl_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      xl_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      xl_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      xl_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      xl_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      xl_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      xl_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      xl_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      xl_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      xl_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      xl_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      xl_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      xl_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      xl_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      xl_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      xl_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      xl_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      xl_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      xl_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      xl_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      xl_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      xl_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      xl_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      xl_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      xl_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      xl_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      xl_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      xl_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      xl_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      xl_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      xl_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      xl_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      xl_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      xl_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      xl_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      xl_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      xl_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      xl_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      xl_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      xl_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      xl_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(xl_state);
}

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
