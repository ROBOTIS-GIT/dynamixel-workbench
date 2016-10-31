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

#include "dynamixel_workbench_single_manager/dynamixel_workbench_single_manager.h"

using namespace dynamixel_workbench_single_manager;

DynamixelWorkbenchSingleManager::DynamixelWorkbenchSingleManager()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(""),
     baud_rate_(0),
     dynamixel_model_number_(0),
     protocol_version_(0.0),
     read_value_(0),
     dynamixel_torque_status_(false),
     dynamixel_(NULL)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchSingleManager());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1.0 PacketHandler and Protocol 2.0 PacketHandler
  packetHandler1_ = dynamixel::PacketHandler::getPacketHandler(1.0);
  packetHandler2_ = dynamixel::PacketHandler::getPacketHandler(2.0);

  // Open port
  if (portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the port(%s)!", device_name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the port!");
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baud_rate_))
  {
    ROS_INFO("Succeeded to change the baudrate(%d)!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
  }

  scanDynamixelID();
  setServer();
  setPublisher();
  setSubscriber();
}

DynamixelWorkbenchSingleManager::~DynamixelWorkbenchSingleManager()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchSingleManager());
}

bool DynamixelWorkbenchSingleManager::initDynamixelWorkbenchSingleManager(void)
{
  ROS_INFO("dynamixel_workbench_single_manager : Init OK!");
  return true;
}

bool DynamixelWorkbenchSingleManager::shutdownDynamixelWorkbenchSingleManager(void)
{
  dynamixel_->item_ = dynamixel_->ctrl_table_["torque_enable"];
  writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, false);
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

int DynamixelWorkbenchSingleManager::getch(void)
{
  struct termios oldt, newt;
  int ch;

  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 1;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

int DynamixelWorkbenchSingleManager::kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

void DynamixelWorkbenchSingleManager::setPublisher(void)
{
  // Init ROS publish
  if(!strncmp(dynamixel_->model_name_.c_str(), "AX", 2))
  {
    dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelAX>("/dynamixel_workbench_single_manager/motor_state",10);
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "MX", 2))
  {
    if (dynamixel_->model_number_ == 310) // MX_64
    {
      dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelMX64>("/dynamixel_workbench_single_manager/motor_state",10);
    }
    else if (dynamixel_->model_number_ == 320) // MX_106
    {
      dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelMX106>("/dynamixel_workbench_single_manager/motor_state",10);
    }
    else
    {
      dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelMX>("/dynamixel_workbench_single_manager/motor_state",10);
    }
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "RX", 2))
  {
    dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelRX>("/dynamixel_workbench_single_manager/motor_state",10);
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "EX", 2))
  {
    dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelEX>("/dynamixel_workbench_single_manager/motor_state",10);
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "XL", 2))
  {
    dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelXL>("/dynamixel_workbench_single_manager/motor_state",10);
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "XM", 2))
  {
    dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelXM>("/dynamixel_workbench_single_manager/motor_state",10);
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "PRO", 3))
  {
    if (dynamixel_->model_number_ == 35072) // PRO_L42_10_S300_R
    {
      dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelProL42>("/dynamixel_workbench_single_manager/motor_state",10);
    }
    else
    {
      dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelPro>("/dynamixel_workbench_single_manager/motor_state",10);
    }
  }
}

void DynamixelWorkbenchSingleManager::setSubscriber(void)
{
  // Init ROS subscriber
  dynamixel_command_sub_ = nh_.subscribe("/dynamixel_workbench_single_manager/motor_command", 10, &DynamixelWorkbenchSingleManager::dynamixelCommandMsgCallback, this);
}

void DynamixelWorkbenchSingleManager::setServer(void)
{
  workbench_param_server_ = nh_.advertiseService("/dynamixel_workbench_single_manager/get_workbench_parameter", &DynamixelWorkbenchSingleManager::getWorkbenchParamCallback, this);
}

void DynamixelWorkbenchSingleManager::setPublishedMsg(void)
{
  if (!strncmp(dynamixel_->model_name_.c_str(), "AX", 2))
  {
    axMotorMessage();
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "RX", 2))
  {
    rxMotorMessage();
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "MX", 2))
  {
    if (dynamixel_->model_number_ == 310) // MX_64
    {
      mx64MotorMessage();
    }
    else if (dynamixel_->model_number_ == 320) //MX_106
    {
      mx106MotorMessage();
    }
    else
    {
      mxMotorMessage();
    }
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "EX", 2))
  {
    exMotorMessage();
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "XL", 2))
  {
    xlMotorMessage();
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "XM", 2))
  {
    xmMotorMessage();
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "PRO", 3))
  {
    if (dynamixel_->model_number_ == 35072) // PRO_L42_10_S300_R
    {
      proL42MotorMessage();
    }
    else
    {
      proMotorMessage();
    }
  }
}

bool DynamixelWorkbenchSingleManager::scanDynamixelID(void)
{
  uint8_t dynamixel_error = 0;
  uint8_t dynamixel_id = 0;
  uint16_t dynamixel_num = 0;

  ROS_INFO("Scan Dynamixel Using Protocol 1.0");
  for (dynamixel_id = 1; dynamixel_id < 253; dynamixel_id++)
  {
    if (packetHandler1_->ping(portHandler_, dynamixel_id, &dynamixel_num, &dynamixel_error) == COMM_SUCCESS)
    {
      dynamixel_model_number_ = dynamixel_num;
      dynamixel_model_id_ = dynamixel_id;
      protocol_version_ = packetHandler1_->getProtocolVersion();
      packetHandler_ = packetHandler1_->getPacketHandler(protocol_version_);
      dynamixel_ = new dynamixel_tool::DynamixelTool(dynamixel_model_id_, dynamixel_model_number_, protocol_version_);
    }

    if (kbhit())
    {
      char c = getch();
      if (c == ESC_ASCII_VALUE) break;
    }
  }

  ROS_INFO("Scan Dynamixel Using Protocol 2.0");
  for (dynamixel_id = 1; dynamixel_id < 253; dynamixel_id++)
  {
    if (packetHandler2_->ping(portHandler_, dynamixel_id, &dynamixel_num, &dynamixel_error) == COMM_SUCCESS)
    {
      dynamixel_model_number_ = dynamixel_num;
      dynamixel_model_id_ = dynamixel_id;
      protocol_version_ = packetHandler2_->getProtocolVersion();
      packetHandler_ = packetHandler2_->getPacketHandler(protocol_version_);
      dynamixel_ = new dynamixel_tool::DynamixelTool(dynamixel_model_id_, dynamixel_model_number_, protocol_version_);
    }

    if (kbhit())
    {
      char c = getch();
      if (c == ESC_ASCII_VALUE) break;
    }
  }

  if (dynamixel_ == NULL)
  {
    ROS_ERROR("...Failed to find dynamixel!");
    shutdownDynamixelWorkbenchSingleManager();
    return false;
  }
  else
  {
    ROS_INFO("...Succeeded to find dynamixel\n");
    ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
    return true;
  }
}

bool DynamixelWorkbenchSingleManager::writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t value)
{
  uint8_t dynamixel_error = 0;
  int dynamixel_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dynamixel_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dynamixel_error);
  }
  else if (length == 2)
  {
    dynamixel_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (int16_t)value, &dynamixel_error);
  }
  else if (length == 4)
  {
    dynamixel_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (int32_t)value, &dynamixel_error);
  }

  if (dynamixel_comm_result == COMM_SUCCESS)
  {
    if (dynamixel_error != 0)
    {
      packetHandler_->printRxPacketError(dynamixel_error);
    }
    return true;
  }
  else
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!", id);
    return false;
  }
}

bool DynamixelWorkbenchSingleManager::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value)
{
  uint8_t dynamixel_error = 0;
  int dynamixel_comm_result = COMM_RX_FAIL;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dynamixel_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dynamixel_error);
  }
  else if (length == 2)
  {
    dynamixel_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dynamixel_error);
  }
  else if (length == 4)
  {
    dynamixel_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dynamixel_error);
  }

  if (dynamixel_comm_result == COMM_SUCCESS)
  {
    if (dynamixel_error != 0)
    {
      packetHandler_->printRxPacketError(dynamixel_error);
    }

    if (length == 1)
    {
      *value = value_8_bit;
      return true;

    }
    else if (length == 2)
    {
      *value = value_16_bit;
      return true;
    }
    else if (length == 4)
    {
      *value = value_32_bit;
      return true;
    }
  }
  else
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    //ROS_WARN("[ID] %u, Fail to read!, %d", id);
    return false;
  }
}

void DynamixelWorkbenchSingleManager::viewManagerMenu(void)
{
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Single Manager supports GUI (dynamixel_workbench_single_manager_gui)  ");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Command list :");
  ROS_INFO("[help|h|?]................: display this menu");
  ROS_INFO("[status]..................: status of a Dynamixel");
  ROS_INFO("[table]...................: check a control table of a dynamixel");
  ROS_INFO("[reboot]..................: reboot a Dynamixel(only protocol version 2.0)");
  ROS_INFO("[factory_reset]...........: command for all data back to the factory settings values");
  ROS_INFO("[[table_item] [value].....: change address value of a dynamixel");
  ROS_INFO("[exit]....................: shutdown");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Press SpaceBar to command a Dynamixel");
}

bool DynamixelWorkbenchSingleManager::rebootDynamixel(void)
{
  if (protocol_version_ != 2.0)
  {
    ROS_ERROR("reboot command only can support in protocol version 2.0");
  }
  else
  {
    uint8_t dynamixel_error = 0;
    uint16_t dynamixel_comm_result = COMM_RX_FAIL;

    dynamixel_comm_result = packetHandler_->reboot(portHandler_, dynamixel_->id_, &dynamixel_error);

    sleep(1);

    if (dynamixel_comm_result == COMM_SUCCESS)
    {
      if (dynamixel_error != 0)
      {
        packetHandler_->printRxPacketError(dynamixel_error);
      }
      ROS_INFO("Success to reboot!");

      dynamixel_ = new dynamixel_tool::DynamixelTool(dynamixel_->id_, dynamixel_->model_number_, packetHandler_->getProtocolVersion());
      ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
    }
    else
    {
      packetHandler_->printTxRxResult(dynamixel_comm_result);
      ROS_INFO("Fail to reboot!");
    }
  }
}

bool DynamixelWorkbenchSingleManager::resetDynamixel()
{
  uint8_t dynamixel_error = 0;
  uint16_t dynamixel_comm_result = COMM_RX_FAIL;

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
    dynamixel_comm_result = packetHandler_->factoryReset(portHandler_, dynamixel_->id_, 0x00, &dynamixel_error);
    sleep(1);

    if (dynamixel_comm_result == COMM_SUCCESS)
    {
      if (dynamixel_error != 0)
      {
        packetHandler_->printRxPacketError(dynamixel_error);
      }
      ROS_INFO("Success to reset!");

      if (portHandler_->setBaudRate(57600) == false)
      {
        sleep(1);
        ROS_INFO(" Failed to change baudrate!");
      }
      else
      {
        sleep(1);
        dynamixel_ = new dynamixel_tool::DynamixelTool(1, dynamixel_->model_number_, packetHandler_->getProtocolVersion());
        ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
      }
    }
    else
    {
      packetHandler_->printTxRxResult(dynamixel_comm_result);
      ROS_ERROR("Fail to reset!");
    }
  }
  else if (packetHandler_->getProtocolVersion() == 2.0)
  {
    dynamixel_comm_result = packetHandler_->factoryReset(portHandler_, dynamixel_->id_, 0xff, &dynamixel_error);
    sleep(1);

    if (dynamixel_comm_result == COMM_SUCCESS)
    {
      if (dynamixel_error != 0)
      {
        packetHandler_->printRxPacketError(dynamixel_error);
      }
      ROS_INFO("Success to reset!");

      if (portHandler_->setBaudRate(57600) == false)
      {
        sleep(1);
        ROS_INFO(" Failed to change baudrate!");
      }
      else
      {
        sleep(1);
        dynamixel_ = new dynamixel_tool::DynamixelTool(1, dynamixel_->model_number_, packetHandler_->getProtocolVersion());
        ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
      }
    }
    else
    {
      packetHandler_->printTxRxResult(dynamixel_comm_result);
      ROS_ERROR("Fail to reset!");
    }
  }
}

void DynamixelWorkbenchSingleManager::showControlTable(void)
{
  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    if (dynamixel_torque_status_)
    {
      if ((dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE) && (dynamixel_->item_->memory_type == dynamixel_tool::RAM))
      {
        ROS_INFO("%s", dynamixel_->item_->item_name.c_str());
      }
    }
    else
    {
      if (dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE)
      {
        ROS_INFO("%s", dynamixel_->item_->item_name.c_str());
      }
    }
  }
}

void DynamixelWorkbenchSingleManager::checkValidationCommand(bool *valid_cmd, char *cmd)
{
  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    if (cmd == dynamixel_->item_->item_name)
    {
      *valid_cmd = true;
      break;
    }
    else
    {
      *valid_cmd = false;
    }
  }
}

void DynamixelWorkbenchSingleManager::axMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelAX dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_angle_limit = read_value_;
    else if ("the_highest_limit_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dynamixel_->item_->item_name)
      dynamixel_response.max_torque = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("alarm_led" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_shutdown = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led" == dynamixel_->item_->item_name)
      dynamixel_response.led = read_value_;
    else if ("cw_compliance_margin" == dynamixel_->item_->item_name)
      dynamixel_response.cw_compliance_margin = read_value_;
    else if ("ccw_compliance_margin" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_compliance_margin = read_value_;
    else if ("cw_compliance_slope" == dynamixel_->item_->item_name)
      dynamixel_response.cw_compliance_slope = read_value_;
    else if ("ccw_compliance_margin" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_compliance_margin = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("moving_speed" == dynamixel_->item_->item_name)
      dynamixel_response.moving_speed = read_value_;
    else if ("torque_limit" == dynamixel_->item_->item_name)
      dynamixel_response.torque_limit = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_load" == dynamixel_->item_->item_name)
      dynamixel_response.present_load = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("registered" == dynamixel_->item_->item_name)
      dynamixel_response.registered = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.moving = read_value_;
    else if ("lock" == dynamixel_->item_->item_name)
      dynamixel_response.lock = read_value_;
    else if ("punch" == dynamixel_->item_->item_name)
      dynamixel_response.punch = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::rxMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelRX dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_angle_limit = read_value_;
    else if ("the_highest_limit_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dynamixel_->item_->item_name)
      dynamixel_response.max_torque = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("alarm_led" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_shutdown = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led" == dynamixel_->item_->item_name)
      dynamixel_response.led = read_value_;
    else if ("cw_compliance_margin" == dynamixel_->item_->item_name)
      dynamixel_response.cw_compliance_margin = read_value_;
    else if ("ccw_compliance_margin" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_compliance_margin = read_value_;
    else if ("cw_compliance_slope" == dynamixel_->item_->item_name)
      dynamixel_response.cw_compliance_slope = read_value_;
    else if ("ccw_compliance_margin" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_compliance_margin = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("moving_speed" == dynamixel_->item_->item_name)
      dynamixel_response.moving_speed = read_value_;
    else if ("torque_limit" == dynamixel_->item_->item_name)
      dynamixel_response.torque_limit = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_load" == dynamixel_->item_->item_name)
      dynamixel_response.present_load = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("registered" == dynamixel_->item_->item_name)
      dynamixel_response.registered = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.moving = read_value_;
    else if ("lock" == dynamixel_->item_->item_name)
      dynamixel_response.lock = read_value_;
    else if ("punch" == dynamixel_->item_->item_name)
      dynamixel_response.punch = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::mxMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelMX dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_angle_limit = read_value_;
    else if ("the_highest_limit_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dynamixel_->item_->item_name)
      dynamixel_response.max_torque = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("alarm_led" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_shutdown = read_value_;
    else if ("multi_turn_offset" == dynamixel_->item_->item_name)
      dynamixel_response.multi_turn_offset = read_value_;
    else if ("resolution_divider" == dynamixel_->item_->item_name)
      dynamixel_response.resolution_divider = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led" == dynamixel_->item_->item_name)
      dynamixel_response.led = read_value_;
    else if ("d_gain" == dynamixel_->item_->item_name)
      dynamixel_response.d_gain = read_value_;
    else if ("i_gain" == dynamixel_->item_->item_name)
      dynamixel_response.i_gain = read_value_;
    else if ("p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.p_gain = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("moving_speed" == dynamixel_->item_->item_name)
      dynamixel_response.moving_speed = read_value_;
    else if ("torque_limit" == dynamixel_->item_->item_name)
      dynamixel_response.torque_limit = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_load" == dynamixel_->item_->item_name)
      dynamixel_response.present_load = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("registered" == dynamixel_->item_->item_name)
      dynamixel_response.registered = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.moving = read_value_;
    else if ("lock" == dynamixel_->item_->item_name)
      dynamixel_response.lock = read_value_;
    else if ("punch" == dynamixel_->item_->item_name)
      dynamixel_response.punch = read_value_;
    else if ("goal_acceleration" == dynamixel_->item_->item_name)
      dynamixel_response.goal_acceleration = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::mx64MotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelMX64 dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_angle_limit = read_value_;
    else if ("the_highest_limit_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dynamixel_->item_->item_name)
      dynamixel_response.max_torque = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("alarm_led" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_shutdown = read_value_;
    else if ("multi_turn_offset" == dynamixel_->item_->item_name)
      dynamixel_response.multi_turn_offset = read_value_;
    else if ("resolution_divider" == dynamixel_->item_->item_name)
      dynamixel_response.resolution_divider = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led" == dynamixel_->item_->item_name)
      dynamixel_response.led = read_value_;
    else if ("d_gain" == dynamixel_->item_->item_name)
      dynamixel_response.d_gain = read_value_;
    else if ("i_gain" == dynamixel_->item_->item_name)
      dynamixel_response.i_gain = read_value_;
    else if ("p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.p_gain = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("moving_speed" == dynamixel_->item_->item_name)
      dynamixel_response.moving_speed = read_value_;
    else if ("torque_limit" == dynamixel_->item_->item_name)
      dynamixel_response.torque_limit = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_load" == dynamixel_->item_->item_name)
      dynamixel_response.present_load = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("registered" == dynamixel_->item_->item_name)
      dynamixel_response.registered = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.moving = read_value_;
    else if ("lock" == dynamixel_->item_->item_name)
      dynamixel_response.lock = read_value_;
    else if ("punch" == dynamixel_->item_->item_name)
      dynamixel_response.punch = read_value_;
    else if ("current" == dynamixel_->item_->item_name)
      dynamixel_response.current = read_value_;
    else if ("torque_control_mode_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_control_mode_enable = read_value_;
    else if ("goal_torque" == dynamixel_->item_->item_name)
      dynamixel_response.goal_torque = read_value_;
    else if ("goal_acceleration" == dynamixel_->item_->item_name)
      dynamixel_response.goal_acceleration = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::mx106MotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelMX106 dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_angle_limit = read_value_;
    else if ("drive_mode" == dynamixel_->item_->item_name)
      dynamixel_response.drive_mode = read_value_;
    else if ("the_highest_limit_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dynamixel_->item_->item_name)
      dynamixel_response.max_torque = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("alarm_led" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_shutdown = read_value_;
    else if ("multi_turn_offset" == dynamixel_->item_->item_name)
      dynamixel_response.multi_turn_offset = read_value_;
    else if ("resolution_divider" == dynamixel_->item_->item_name)
      dynamixel_response.resolution_divider = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led" == dynamixel_->item_->item_name)
      dynamixel_response.led = read_value_;
    else if ("d_gain" == dynamixel_->item_->item_name)
      dynamixel_response.d_gain = read_value_;
    else if ("i_gain" == dynamixel_->item_->item_name)
      dynamixel_response.i_gain = read_value_;
    else if ("p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.p_gain = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("moving_speed" == dynamixel_->item_->item_name)
      dynamixel_response.moving_speed = read_value_;
    else if ("torque_limit" == dynamixel_->item_->item_name)
      dynamixel_response.torque_limit = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_load" == dynamixel_->item_->item_name)
      dynamixel_response.present_load = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("registered" == dynamixel_->item_->item_name)
      dynamixel_response.registered = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.moving = read_value_;
    else if ("lock" == dynamixel_->item_->item_name)
      dynamixel_response.lock = read_value_;
    else if ("punch" == dynamixel_->item_->item_name)
      dynamixel_response.punch = read_value_;
    else if ("current" == dynamixel_->item_->item_name)
      dynamixel_response.current = read_value_;
    else if ("torque_control_mode_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_control_mode_enable = read_value_;
    else if ("goal_torque" == dynamixel_->item_->item_name)
      dynamixel_response.goal_torque = read_value_;
    else if ("goal_acceleration" == dynamixel_->item_->item_name)
      dynamixel_response.goal_acceleration = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::exMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelEX dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_angle_limit = read_value_;
    else if ("drive_mode" == dynamixel_->item_->item_name)
      dynamixel_response.drive_mode = read_value_;
    else if ("the_highest_limit_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dynamixel_->item_->item_name)
      dynamixel_response.max_torque = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("alarm_led" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_shutdown = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led" == dynamixel_->item_->item_name)
      dynamixel_response.led = read_value_;
    else if ("cw_compliance_margin" == dynamixel_->item_->item_name)
      dynamixel_response.cw_compliance_margin = read_value_;
    else if ("ccw_compliance_margin" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_compliance_margin = read_value_;
    else if ("cw_compliance_slope" == dynamixel_->item_->item_name)
      dynamixel_response.cw_compliance_slope = read_value_;
    else if ("ccw_compliance_slope" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_compliance_slope = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("moving_speed" == dynamixel_->item_->item_name)
      dynamixel_response.moving_speed = read_value_;
    else if ("torque_limit" == dynamixel_->item_->item_name)
      dynamixel_response.torque_limit = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_load" == dynamixel_->item_->item_name)
      dynamixel_response.present_load = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("registered" == dynamixel_->item_->item_name)
      dynamixel_response.registered = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.moving = read_value_;
    else if ("lock" == dynamixel_->item_->item_name)
      dynamixel_response.lock = read_value_;
    else if ("punch" == dynamixel_->item_->item_name)
      dynamixel_response.punch = read_value_;
    else if ("sensed_current" == dynamixel_->item_->item_name)
      dynamixel_response.sensed_current = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::xlMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelXL dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dynamixel_->item_->item_name)
      dynamixel_response.ccw_angle_limit = read_value_;
    else if ("control_mode" == dynamixel_->item_->item_name)
      dynamixel_response.control_mode = read_value_;
    else if ("limit_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.limit_temperature = read_value_;
    else if ("down_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.down_limit_voltage = read_value_;
    else if ("up_limit_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.up_limit_voltage = read_value_;
    else if ("max_torque" == dynamixel_->item_->item_name)
      dynamixel_response.max_torque = read_value_;
    else if ("return_level" == dynamixel_->item_->item_name)
      dynamixel_response.return_level = read_value_;
    else if ("alarm_shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.alarm_shutdown = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led" == dynamixel_->item_->item_name)
      dynamixel_response.led = read_value_;
    else if ("d_gain" == dynamixel_->item_->item_name)
      dynamixel_response.d_gain = read_value_;
    else if ("i_gain" == dynamixel_->item_->item_name)
      dynamixel_response.i_gain = read_value_;
    else if ("p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.p_gain = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("goal_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.goal_velocity = read_value_;
    else if ("goal_torque" == dynamixel_->item_->item_name)
      dynamixel_response.goal_torque = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_load" == dynamixel_->item_->item_name)
      dynamixel_response.present_load = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("registered_instruction" == dynamixel_->item_->item_name)
      dynamixel_response.registered_instruction = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.moving = read_value_;
    else if ("hardware_error_status" == dynamixel_->item_->item_name)
      dynamixel_response.hardware_error_status = read_value_;
    else if ("punch" == dynamixel_->item_->item_name)
      dynamixel_response.punch = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::xmMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelXM dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("operating_mode" == dynamixel_->item_->item_name)
      dynamixel_response.operating_mode = read_value_;
    else if ("protocol_version" == dynamixel_->item_->item_name)
      dynamixel_response.protocol_version = read_value_;
    else if ("homing_offset" == dynamixel_->item_->item_name)
      dynamixel_response.homing_offset = read_value_;
    else if ("moving_threshold" == dynamixel_->item_->item_name)
      dynamixel_response.moving_threshold = read_value_;
    else if ("max_temperature_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_temperature_limit = read_value_;
    else if ("max_voltage_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_voltage_limit = read_value_;
    else if ("min_voltage_limit" == dynamixel_->item_->item_name)
      dynamixel_response.min_voltage_limit = read_value_;
    else if ("pwm_limit" == dynamixel_->item_->item_name)
      dynamixel_response.pwm_limit = read_value_;
    else if ("current_limit" == dynamixel_->item_->item_name)
      dynamixel_response.current_limit = read_value_;
    else if ("acceleration_limit" == dynamixel_->item_->item_name)
      dynamixel_response.acceleration_limit = read_value_;
    else if ("velocity_limit" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_limit = read_value_;
    else if ("max_position_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_position_limit = read_value_;
    else if ("min_position_limit" == dynamixel_->item_->item_name)
      dynamixel_response.min_position_limit = read_value_;
    else if ("shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.shutdown = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led" == dynamixel_->item_->item_name)
      dynamixel_response.led = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("registered_instruction" == dynamixel_->item_->item_name)
      dynamixel_response.registered_instruction = read_value_;
    else if ("hardware_error_status" == dynamixel_->item_->item_name)
      dynamixel_response.hardware_error_status = read_value_;
    else if ("velocity_i_gain" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_i_gain = read_value_;
    else if ("velocity_p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_p_gain = read_value_;
    else if ("position_d_gain" == dynamixel_->item_->item_name)
      dynamixel_response.position_d_gain = read_value_;
    else if ("position_i_gain" == dynamixel_->item_->item_name)
      dynamixel_response.position_i_gain = read_value_;
    else if ("position_p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.position_p_gain = read_value_;
    else if ("feedforward_2nd_gain" == dynamixel_->item_->item_name)
      dynamixel_response.feedforward_2nd_gain = read_value_;
    else if ("feedforward_1st_gain" == dynamixel_->item_->item_name)
      dynamixel_response.feedforward_1st_gain = read_value_;
    else if ("goal_pwm" == dynamixel_->item_->item_name)
      dynamixel_response.goal_pwm = read_value_;
    else if ("goal_current" == dynamixel_->item_->item_name)
      dynamixel_response.goal_current = read_value_;
    else if ("goal_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.goal_velocity = read_value_;
    else if ("profile_acceleration" == dynamixel_->item_->item_name)
      dynamixel_response.profile_acceleration = read_value_;
    else if ("profile_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.profile_velocity = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("realtime_tick" == dynamixel_->item_->item_name)
      dynamixel_response.realtime_tick = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.moving = read_value_;
    else if ("moving_status" == dynamixel_->item_->item_name)
      dynamixel_response.moving_status = read_value_;
    else if ("present_pwm" == dynamixel_->item_->item_name)
      dynamixel_response.present_pwm = read_value_;
    else if ("present_current" == dynamixel_->item_->item_name)
      dynamixel_response.present_current = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("velocity_trajectory" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_trajectory = read_value_;
    else if ("position_trajectory" == dynamixel_->item_->item_name)
      dynamixel_response.position_trajectory = read_value_;
    else if ("present_input_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_input_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::proMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelPro dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("operating_mode" == dynamixel_->item_->item_name)
      dynamixel_response.operating_mode = read_value_;
    else if ("homing_offset" == dynamixel_->item_->item_name)
      dynamixel_response.homing_offset = read_value_;
    else if ("moving_threshold" == dynamixel_->item_->item_name)
      dynamixel_response.moving_threshold = read_value_;
    else if ("max_temperature_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_temperature_limit = read_value_;
    else if ("max_voltage_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_voltage_limit = read_value_;
    else if ("min_voltage_limit" == dynamixel_->item_->item_name)
      dynamixel_response.min_voltage_limit = read_value_;
    else if ("acceleration_limit" == dynamixel_->item_->item_name)
      dynamixel_response.acceleration_limit = read_value_;
    else if ("torque_limit" == dynamixel_->item_->item_name)
      dynamixel_response.torque_limit = read_value_;
    else if ("velocity_limit" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_limit = read_value_;
    else if ("max_position_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_position_limit = read_value_;
    else if ("min_position_limit" == dynamixel_->item_->item_name)
      dynamixel_response.min_position_limit = read_value_;
    else if ("external_port_mod_1" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_mod_1 = read_value_;
    else if ("external_port_mod_2" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_mod_2 = read_value_;
    else if ("external_port_mod_3" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_mod_3 = read_value_;
    else if ("external_port_mod_4" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_mod_4 = read_value_;
    else if ("shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.shutdown = read_value_;
    else if ("indirect_address_1" == dynamixel_->item_->item_name)
      dynamixel_response.indirect_address_1 = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led_red" == dynamixel_->item_->item_name)
      dynamixel_response.led_red = read_value_;
    else if ("led_green" == dynamixel_->item_->item_name)
      dynamixel_response.led_green = read_value_;
    else if ("led_blue" == dynamixel_->item_->item_name)
      dynamixel_response.led_blue = read_value_;
    else if ("velocity_i_gain" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_i_gain = read_value_;
    else if ("velocity_p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_p_gain = read_value_;
    else if ("position_p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.position_p_gain = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("goal_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.goal_velocity = read_value_;
    else if ("goal_torque" == dynamixel_->item_->item_name)
      dynamixel_response.goal_torque = read_value_;
    else if ("goal_acceleration" == dynamixel_->item_->item_name)
      dynamixel_response.goal_acceleration = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.is_moving = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_current" == dynamixel_->item_->item_name)
      dynamixel_response.present_current = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("external_port_data_1" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_data_1 = read_value_;
    else if ("external_port_data_2" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_data_2 = read_value_;
    else if ("external_port_data_3" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_data_3 = read_value_;
    else if ("external_port_data_4" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_data_4 = read_value_;
    else if ("indirect_data_1" == dynamixel_->item_->item_name)
      dynamixel_response.indirect_data_1 = read_value_;
    else if ("registered_instruction" == dynamixel_->item_->item_name)
      dynamixel_response.registered_instruction = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("hardware_error_status" == dynamixel_->item_->item_name)
      dynamixel_response.hardware_error_status = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

void DynamixelWorkbenchSingleManager::proL42MotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelPro dynamixel_response;

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, &read_value_);

    if ("model_number" == dynamixel_->item_->item_name)
      dynamixel_response.model_number = read_value_;
    else if ("version_of_firmware" == dynamixel_->item_->item_name)
      dynamixel_response.version_of_firmware = read_value_;
    else if ("id" == dynamixel_->item_->item_name)
      dynamixel_response.id = read_value_;
    else if ("baud_rate" == dynamixel_->item_->item_name)
      dynamixel_response.baud_rate = read_value_;
    else if ("return_delay_time" == dynamixel_->item_->item_name)
      dynamixel_response.return_delay_time = read_value_;
    else if ("operating_mode" == dynamixel_->item_->item_name)
      dynamixel_response.operating_mode = read_value_;
    else if ("moving_threshold" == dynamixel_->item_->item_name)
      dynamixel_response.moving_threshold = read_value_;
    else if ("max_temperature_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_temperature_limit = read_value_;
    else if ("max_voltage_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_voltage_limit = read_value_;
    else if ("min_voltage_limit" == dynamixel_->item_->item_name)
      dynamixel_response.min_voltage_limit = read_value_;
    else if ("acceleration_limit" == dynamixel_->item_->item_name)
      dynamixel_response.acceleration_limit = read_value_;
    else if ("torque_limit" == dynamixel_->item_->item_name)
      dynamixel_response.torque_limit = read_value_;
    else if ("velocity_limit" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_limit = read_value_;
    else if ("max_position_limit" == dynamixel_->item_->item_name)
      dynamixel_response.max_position_limit = read_value_;
    else if ("min_position_limit" == dynamixel_->item_->item_name)
      dynamixel_response.min_position_limit = read_value_;
    else if ("external_port_mod_1" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_mod_1 = read_value_;
    else if ("external_port_mod_2" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_mod_2 = read_value_;
    else if ("external_port_mod_3" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_mod_3 = read_value_;
    else if ("external_port_mod_4" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_mod_4 = read_value_;
    else if ("shutdown" == dynamixel_->item_->item_name)
      dynamixel_response.shutdown = read_value_;
    else if ("indirect_address_1" == dynamixel_->item_->item_name)
      dynamixel_response.indirect_address_1 = read_value_;
    else if ("torque_enable" == dynamixel_->item_->item_name)
      dynamixel_response.torque_enable = dynamixel_torque_status_ = read_value_;
    else if ("led_red" == dynamixel_->item_->item_name)
      dynamixel_response.led_red = read_value_;
    else if ("led_green" == dynamixel_->item_->item_name)
      dynamixel_response.led_green = read_value_;
    else if ("led_blue" == dynamixel_->item_->item_name)
      dynamixel_response.led_blue = read_value_;
    else if ("velocity_i_gain" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_i_gain = read_value_;
    else if ("velocity_p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.velocity_p_gain = read_value_;
    else if ("position_p_gain" == dynamixel_->item_->item_name)
      dynamixel_response.position_p_gain = read_value_;
    else if ("goal_position" == dynamixel_->item_->item_name)
      dynamixel_response.goal_position = read_value_;
    else if ("goal_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.goal_velocity = read_value_;
    else if ("goal_torque" == dynamixel_->item_->item_name)
      dynamixel_response.goal_torque = read_value_;
    else if ("goal_acceleration" == dynamixel_->item_->item_name)
      dynamixel_response.goal_acceleration = read_value_;
    else if ("moving" == dynamixel_->item_->item_name)
      dynamixel_response.is_moving = read_value_;
    else if ("present_position" == dynamixel_->item_->item_name)
      dynamixel_response.present_position = read_value_;
    else if ("present_velocity" == dynamixel_->item_->item_name)
      dynamixel_response.present_velocity = read_value_;
    else if ("present_current" == dynamixel_->item_->item_name)
      dynamixel_response.present_current = read_value_;
    else if ("present_voltage" == dynamixel_->item_->item_name)
      dynamixel_response.present_voltage = read_value_;
    else if ("present_temperature" == dynamixel_->item_->item_name)
      dynamixel_response.present_temperature = read_value_;
    else if ("external_port_data_1" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_data_1 = read_value_;
    else if ("external_port_data_2" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_data_2 = read_value_;
    else if ("external_port_data_3" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_data_3 = read_value_;
    else if ("external_port_data_4" == dynamixel_->item_->item_name)
      dynamixel_response.external_port_data_4 = read_value_;
    else if ("indirect_data_1" == dynamixel_->item_->item_name)
      dynamixel_response.indirect_data_1 = read_value_;
    else if ("registered_instruction" == dynamixel_->item_->item_name)
      dynamixel_response.registered_instruction = read_value_;
    else if ("status_return_level" == dynamixel_->item_->item_name)
      dynamixel_response.status_return_level = read_value_;
    else if ("hardware_error_status" == dynamixel_->item_->item_name)
      dynamixel_response.hardware_error_status = read_value_;
  }

  dynamixel_state_pub_.publish(dynamixel_response);
}

bool DynamixelWorkbenchSingleManager::getWorkbenchParamCallback(dynamixel_workbench_msgs::GetWorkbenchParam::Request &req, dynamixel_workbench_msgs::GetWorkbenchParam::Response &res)
{
  res.workbench_parameter.device_name = device_name_;
  res.workbench_parameter.baud_rate = portHandler_->getBaudRate();
  res.workbench_parameter.protocol_version = packetHandler_->getProtocolVersion();
  res.workbench_parameter.model_name = dynamixel_->model_name_;
  res.workbench_parameter.model_id = dynamixel_->id_;
  res.workbench_parameter.model_number = dynamixel_->model_number_;

  return true;
}

void DynamixelWorkbenchSingleManager::dynamixelCommandMsgCallback(const dynamixel_workbench_msgs::DynamixelCommand::ConstPtr &msg)
{
  if (msg->addr_name == "reboot")
  {
    rebootDynamixel();
  }
  else if (msg->addr_name == "factory_reset")
  {
    resetDynamixel();
  }
  else if (msg->addr_name == "baud_rate")
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[msg->addr_name];
    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, dynamixel_->baud_rate_table_.find(msg->value)->second);
    sleep(1);

    if (portHandler_->setBaudRate(dynamixel_->baud_rate_table_.find(msg->value)->first) == false)
    {
      sleep(1);
      ROS_INFO(" Failed to change baudrate!");
    }
    else
    {
      sleep(1);
      ROS_INFO(" Success to change baudrate! [ BAUD RATE: %d ]", dynamixel_->baud_rate_table_.find(msg->value)->first);
    }
  }
  else if (msg->addr_name == "id")
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[msg->addr_name];
    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, msg->value);
    sleep(1);

    dynamixel_ = new dynamixel_tool::DynamixelTool(msg->value, dynamixel_model_number_, protocol_version_);
    ROS_INFO("...Succeeded to set dynamixel id");
    ROS_INFO("[ID] %u, [Model Name] %s", dynamixel_->id_, dynamixel_->model_name_.c_str());
  }
  else
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[msg->addr_name];
    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, msg->value);
  }
}

bool DynamixelWorkbenchSingleManager::dynamixelSingleManagerLoop(void)
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;
  bool valid_cmd = false;

  setPublishedMsg();

  if (kbhit())
  {
    if (getchar() == SPACEBAR_ASCII_VALUE)
    {
      sleep(0.5);
      printf("[CMD]");
      fgets(input, sizeof(input), stdin);

      char *p;
      if ((p = strchr(input, '\n'))!= NULL) *p = '\0';
      fflush(stdin);

      if (strlen(input) == 0) return false;

      token = strtok(input, " ");

      if (token == 0) return false;

      strcpy(cmd, token);
      token = strtok(0, " ");
      num_param = 0;

      while (token != 0)
      {
        strcpy(param[num_param++], token);
        token = strtok(0, " ");
      }

      checkValidationCommand(&valid_cmd, cmd);

      if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
      {
          viewManagerMenu();
      }
      else if (strcmp(cmd, "status") == 0)
      {
        ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
      }
      else if (strcmp(cmd, "exit") == 0)
      {
        shutdownDynamixelWorkbenchSingleManager();
        return true;
      }
      else if (strcmp(cmd, "table") == 0)
      {
        showControlTable();
      }
      else if (strcmp(cmd, "reboot") == 0)
      {
        rebootDynamixel();
      }
      else if (strcmp(cmd, "factory_reset") == 0)
      {
        resetDynamixel();
      }
      else if (valid_cmd == true)
      {
        if (num_param == 1)
        {
          dynamixel_->item_ = dynamixel_->ctrl_table_[cmd];
          if (dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE)
          {
            if((dynamixel_torque_status_ == true) && dynamixel_->item_->memory_type == dynamixel_tool::EEPROM)
            {
              ROS_ERROR("address in EEPROM is not accessed when motor is torque on");
              ROS_ERROR("Check a ""table""");
            }
            else if ((dynamixel_torque_status_ == false) && (dynamixel_->item_->item_name != "torque_enable") && dynamixel_->item_->memory_type == dynamixel_tool::RAM)
            {
              ROS_ERROR("address in RAM is accessed when motor is torque on");
            }
            else
            {
              if (dynamixel_->item_->item_name == "id")
              {
                if (atoi(param[0]) > 0 && atoi(param[0]) < 253)
                {
                  writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, atoi(param[0]));
                  sleep(1);

                  dynamixel_ = new dynamixel_tool::DynamixelTool(atoi(param[0]), dynamixel_->model_number_, packetHandler_->getProtocolVersion());
                  ROS_INFO("...Succeeded to set dynamixel id [%u]", dynamixel_->id_);
                }
                else
                {
                  ROS_ERROR(" Dynamixel ID can be set 1~252");
                }
              }
              else if (dynamixel_->item_->item_name == "baud_rate")
              {
                if (dynamixel_->baud_rate_table_.find(atoi(param[0]))->second == dynamixel_->baud_rate_table_.end()->second)
                {
                  ROS_ERROR(" Failed to change [ BAUD RATE: %d ]", atoi(param[0]));
                  ROS_ERROR(" Please check the valid baud rate at dynamixel_tool packages or E-MANUAL");

                  writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, dynamixel_->baud_rate_table_.find(57600)->second);
                  sleep(1);

                  if (portHandler_->setBaudRate(dynamixel_->baud_rate_table_.find(57600)->first) == false)
                  {
                    sleep(1);
                    ROS_INFO(" Failed to change default baudrate(57600)!");
                  }
                  else
                  {
                    sleep(1);
                    ROS_INFO(" Success to change default baudrate! [ BAUD RATE: 57600 ]");
                  }
                }
                else
                {
                  if (atoi(param[0]) < 2250000)
                  {
                    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, dynamixel_->baud_rate_table_.find(atoi(param[0]))->second);
                    sleep(1);

                    if (portHandler_->setBaudRate(dynamixel_->baud_rate_table_.find(atoi(param[0]))->first) == false)
                    {
                      sleep(1);
                      ROS_INFO(" Failed to change baudrate!");
                    }
                    else
                    {
                      sleep(1);
                      ROS_INFO(" Success to change baudrate! [ BAUD RATE: %d ]", dynamixel_->baud_rate_table_.find(atoi(param[0]))->first);
                    }
                  }
                  else
                  {
                    ROS_ERROR(" USB2Dynamixel supports baudrate under '2250000'");
                  }
                }
              }
              else if (dynamixel_->item_->item_name == "protocol_version")
              {
                if (atoi(param[0]) == 1.0 || atoi(param[0]) == 2.0)
                {
                  // TODO: Find restriced access address in protocol_version 1.0 of XM430
                  writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, atof(param[0]));
                  sleep(1);

                  packetHandler_->getPacketHandler(atof(param[0]));
                  sleep(1);

                  ROS_INFO(" Success to change protocol version [ PROTOCOL VERSION: %.2f]", packetHandler_->getProtocolVersion());
                }
                else
                {
                  ROS_ERROR(" Dynamixel has '1.0' or '2.0' protocol version");
                }
              }
              else
              {
                writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, atoi(param[0]));
              }
            }
          }
        }
        else
        {
          ROS_ERROR("Invalid parameters! Please check control table [-table]");
        }
      }
      else
      {
        ROS_ERROR("Invalid command. Please check menu[help, ?]");
      }
    }
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_single_manager");
  DynamixelWorkbenchSingleManager manager;
  ros::Rate loop_rate(10);
  manager.viewManagerMenu();

  while (ros::ok())
  {
    manager.dynamixelSingleManagerLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
