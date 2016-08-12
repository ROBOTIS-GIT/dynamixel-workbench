#include "dynamixel_workbench_single_manager/dynamixel_workbench_single_manager.h"

using namespace dynamixel_workbench_single_manager;

DynamixelWorkbenchSingleManager::DynamixelWorkbenchSingleManager()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(DEVICENAME),
     baud_rate_(BAUDRATE),
     protocol_version_(PROTOCOL_VERSION),
     dxl_(NULL)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);
  nh_priv_.getParam("protocol_version_",protocol_version_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchSingleManager());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1.0 PacketHandler or Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

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
  setPublisher();
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
  if(!strncmp(dxl_->model_name_.c_str(), "AX", 2))
  {
    dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelAX>("/" + dxl_->model_name_ + "/motor_state",10);
  }
  else if (!strncmp(dxl_->model_name_.c_str(), "MX", 2))
  {
    if (dxl_->model_number_ == 310) // MX-64
    {
      dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelMX64>("/" + dxl_->model_name_ + "/motor_state",10);
    }
    else if (dxl_->model_number_ == 320) // MX-106
    {
      dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelMX106>("/" + dxl_->model_name_ + "/motor_state",10);
    }
    else
    {
      dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelMX>("/" + dxl_->model_name_ + "/motor_state",10);
    }
  }
  else if (!strncmp(dxl_->model_name_.c_str(), "RX", 2))
  {
    dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelRX>("/" + dxl_->model_name_ + "/motor_state",10);
  }
  else if (!strncmp(dxl_->model_name_.c_str(), "XM", 2))
  {
    dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelXM>("/" + dxl_->model_name_ + "/motor_state",10);
  }
  else if (!strncmp(dxl_->model_name_.c_str(), "PRO", 3))
  {
    if (dxl_->model_number_ == 35072) // PRO_L42_10_S300_R
    {
      dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelProL42>("/" + dxl_->model_name_ + "/motor_state",10);
    }
    else
    {
      dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelPro>("/" + dxl_->model_name_ + "/motor_state",10);
    }
  }
}

void DynamixelWorkbenchSingleManager::getPublisher(void)
{
  if (!strncmp(dxl_->model_name_.c_str(), "AX", 2))
  {
    axMotorMessage();
  }
  else if (!strncmp(dxl_->model_name_.c_str(), "RX", 2))
  {
    rxMotorMessage();
  }
  else if (!strncmp(dxl_->model_name_.c_str(), "MX", 2))
  {
    if (dxl_->model_number_ == 310) // MX-64
    {
      mx64MotorMessage();
    }
    else if (dxl_->model_number_ == 320) //MX-106
    {
      mx106MotorMessage();
    }
    else
    {
      mxMotorMessage();
    }
  }
  else if (!strncmp(dxl_->model_name_.c_str(), "XM", 2))
  {
    xmMotorMessage();
  }
  else if (!strncmp(dxl_->model_name_.c_str(), "PRO", 3))
  {
    if (dxl_->model_number_ == 35072) // PRO_L42_10_S300_R
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
  uint8_t dxl_error = 0;
  uint8_t dxl_id = 0;
  uint16_t dxl_model = 0;

  if (packetHandler_->getProtocolVersion() == 1.0)
  {
    ROS_INFO("Scan Dynamixel Using Protocol 1.0");
    for (dxl_id = 1; dxl_id < 253; dxl_id++)
    {
      if (packetHandler_->ping(portHandler_, dxl_id, &dxl_model, &dxl_error) == COMM_SUCCESS)
      {
        ROS_INFO("Model Number: %d", dxl_model);
        dxl_number_ = dxl_model;
        dxl_ = new dxl_motor::DxlMotor(dxl_id, dxl_model, protocol_version_);
      }
      else
      {
        fprintf(stderr,".");
      }

      if (kbhit())
      {
        char c = getch();
        if (c == ESC_ASCII_VALUE) break;
      }
    }
  }
  else
  {
    ROS_INFO("Scan Dynamixel Using Protocol 2.0");
    for (dxl_id = 1; dxl_id < 253; dxl_id++)
    {
      if (packetHandler_->ping(portHandler_, dxl_id, &dxl_model, &dxl_error) == COMM_SUCCESS)
      {
        ROS_INFO("Model Number: %d", dxl_model);
        dxl_number_ = dxl_model;
        dxl_ = new dxl_motor::DxlMotor(dxl_id, dxl_model, protocol_version_);        
      }
      else
      {
        fprintf(stderr,".");
      }

      if (kbhit())
      {
        char c = getch();
        if (c == ESC_ASCII_VALUE) break;
      }
    }
  }

  if (dxl_ == NULL)
  {
    ROS_INFO("");
    ROS_ERROR("...Failed to find dynamixel!");
    shutdownDynamixelWorkbenchSingleManager();
    return false;
  }
  else
  {
    ROS_INFO("");
    ROS_INFO("...Succeeded to find dynamixel");
    ROS_INFO("[ID] %u, [Model Name] %s", dxl_->id_, dxl_->model_name_.c_str());
    return true;
  }
}

bool DynamixelWorkbenchSingleManager::writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, addr, (int16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, addr, (int32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
    {
      packetHandler_->printRxPacketError(dxl_error);
    }
    return true;
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!", id);
    return false;
  }

}

bool DynamixelWorkbenchSingleManager::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t *value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_RX_FAIL;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dxl_comm_result = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
    {
      packetHandler_->printRxPacketError(dxl_error);
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
    packetHandler_->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to read!", id);
    return false;
  }
}

void DynamixelWorkbenchSingleManager::viewMangerMenu(void)
{
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Press SpaceBar to command dynamixel");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Command list :");
  ROS_INFO("[-help]....................: display this help");
  ROS_INFO("[-table]...................: check dynamixel control table");
  ROS_INFO("[-reboot]..................: reboot dynamixel(only protocol version 2.0)");
  ROS_INFO("[-factory_reset]...........: factory reset dynamixel ");
  ROS_INFO("[-[table_item] [value].....: control dynamixel write address");
  ROS_INFO("[-exit]....................: shutdown");
  ROS_INFO("----------------------------------------------------------------------");
}

bool DynamixelWorkbenchSingleManager::rebootDynamixel(void)
{
  if (protocol_version_ != 2.0)
  {
    ROS_ERROR("reboot command only can operate in protocol version 2.0");
  }
  else
  {
    uint8_t dxl_error = 0;
    uint16_t dxl_comm_result = COMM_RX_FAIL;

    dxl_comm_result = packetHandler_->reboot(portHandler_, dxl_->id_, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
      if (dxl_error != 0)
      {
        packetHandler_->printRxPacketError(dxl_error);
      }
      ROS_INFO("Success to reboot!");
    }
    else
    {
      packetHandler_->printTxRxResult(dxl_comm_result);
      ROS_INFO("Fail to reboot!");
    }
  }
}

bool DynamixelWorkbenchSingleManager::resetDynamixel(void)
{
  uint8_t dxl_error = 0;
  uint16_t dxl_comm_result = COMM_RX_FAIL;

  dxl_comm_result = packetHandler_->factoryReset(portHandler_, dxl_->id_,0x00, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
    {
      packetHandler_->printRxPacketError(dxl_error);
    }
    fprintf(stderr, "\n Success to reset! \n\n");
  }
  else
  {
    packetHandler_->printTxRxResult(dxl_comm_result);
    fprintf(stderr, "\n Fail to reset! \n\n");
  }
}

void DynamixelWorkbenchSingleManager::showControlTable(void)
{
  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    if(dxl_->dxl_item_->access_type == 1) //1 = READ_WRITE
    {
      ROS_INFO("%s", dxl_->dxl_item_->item_name.c_str());
    }
  }
}

void DynamixelWorkbenchSingleManager::checkValidationCommand(bool *valid_cmd, char *cmd)
{
  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    if (cmd == dxl_->dxl_item_->item_name)
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
  dynamixel_workbench_msgs::DynamixelAX dxl_response;

  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, &read_value_);

    if ("model_number" == dxl_->dxl_item_->item_name)
      dxl_response.model_number = read_value_;
    else if ("version_of_firmware" == dxl_->dxl_item_->item_name)
      dxl_response.version_of_firmware = read_value_;
    else if ("id" == dxl_->dxl_item_->item_name)
      dxl_response.id = read_value_;
    else if ("baud_rate" == dxl_->dxl_item_->item_name)
      dxl_response.baud_rate = read_value_;
    else if ("return_delay_time" == dxl_->dxl_item_->item_name)
      dxl_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_angle_limit = read_value_;
    else if ("the_highest_limit_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dxl_->dxl_item_->item_name)
      dxl_response.max_torque = read_value_;
    else if ("status_return_level" == dxl_->dxl_item_->item_name)
      dxl_response.status_return_level = read_value_;
    else if ("alarm_led" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_shutdown = read_value_;
    else if ("torque_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_enable = read_value_;
    else if ("led" == dxl_->dxl_item_->item_name)
      dxl_response.led = read_value_;
    else if ("cw_compliance_margin" == dxl_->dxl_item_->item_name)
      dxl_response.cw_compliance_margin = read_value_;
    else if ("ccw_compliance_margin" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_compliance_margin = read_value_;
    else if ("cw_compliance_slope" == dxl_->dxl_item_->item_name)
      dxl_response.cw_compliance_slope = read_value_;
    else if ("ccw_compliance_margin" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_compliance_margin = read_value_;
    else if ("goal_position" == dxl_->dxl_item_->item_name)
      dxl_response.goal_position = read_value_;
    else if ("moving_speed" == dxl_->dxl_item_->item_name)
      dxl_response.moving_speed = read_value_;
    else if ("torque_limit" == dxl_->dxl_item_->item_name)
      dxl_response.torque_limit = read_value_;
    else if ("present_position" == dxl_->dxl_item_->item_name)
      dxl_response.present_position = read_value_;
    else if ("present_speed" == dxl_->dxl_item_->item_name)
      dxl_response.present_speed = read_value_;
    else if ("present_load" == dxl_->dxl_item_->item_name)
      dxl_response.present_load = read_value_;
    else if ("present_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.present_voltage = read_value_;
    else if ("present_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.present_temperature = read_value_;
    else if ("registered" == dxl_->dxl_item_->item_name)
      dxl_response.registered = read_value_;
    else if ("moving" == dxl_->dxl_item_->item_name)
      dxl_response.moving = read_value_;
    else if ("lock" == dxl_->dxl_item_->item_name)
      dxl_response.lock = read_value_;
    else if ("punch" == dxl_->dxl_item_->item_name)
      dxl_response.punch = read_value_;
  }

  dxl_state_pub_.publish(dxl_response);
}

void DynamixelWorkbenchSingleManager::rxMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelRX dxl_response;

  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, &read_value_);

    if ("model_number" == dxl_->dxl_item_->item_name)
      dxl_response.model_number = read_value_;
    else if ("version_of_firmware" == dxl_->dxl_item_->item_name)
      dxl_response.version_of_firmware = read_value_;
    else if ("id" == dxl_->dxl_item_->item_name)
      dxl_response.id = read_value_;
    else if ("baud_rate" == dxl_->dxl_item_->item_name)
      dxl_response.baud_rate = read_value_;
    else if ("return_delay_time" == dxl_->dxl_item_->item_name)
      dxl_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_angle_limit = read_value_;
    else if ("the_highest_limit_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dxl_->dxl_item_->item_name)
      dxl_response.max_torque = read_value_;
    else if ("status_return_level" == dxl_->dxl_item_->item_name)
      dxl_response.status_return_level = read_value_;
    else if ("alarm_led" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_shutdown = read_value_;
    else if ("torque_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_enable = read_value_;
    else if ("led" == dxl_->dxl_item_->item_name)
      dxl_response.led = read_value_;
    else if ("cw_compliance_margin" == dxl_->dxl_item_->item_name)
      dxl_response.cw_compliance_margin = read_value_;
    else if ("ccw_compliance_margin" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_compliance_margin = read_value_;
    else if ("cw_compliance_slope" == dxl_->dxl_item_->item_name)
      dxl_response.cw_compliance_slope = read_value_;
    else if ("ccw_compliance_margin" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_compliance_margin = read_value_;
    else if ("goal_position" == dxl_->dxl_item_->item_name)
      dxl_response.goal_position = read_value_;
    else if ("moving_speed" == dxl_->dxl_item_->item_name)
      dxl_response.moving_speed = read_value_;
    else if ("torque_limit" == dxl_->dxl_item_->item_name)
      dxl_response.torque_limit = read_value_;
    else if ("present_position" == dxl_->dxl_item_->item_name)
      dxl_response.present_position = read_value_;
    else if ("present_speed" == dxl_->dxl_item_->item_name)
      dxl_response.present_speed = read_value_;
    else if ("present_load" == dxl_->dxl_item_->item_name)
      dxl_response.present_load = read_value_;
    else if ("present_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.present_voltage = read_value_;
    else if ("present_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.present_temperature = read_value_;
    else if ("registered" == dxl_->dxl_item_->item_name)
      dxl_response.registered = read_value_;
    else if ("moving" == dxl_->dxl_item_->item_name)
      dxl_response.moving = read_value_;
    else if ("lock" == dxl_->dxl_item_->item_name)
      dxl_response.lock = read_value_;
    else if ("punch" == dxl_->dxl_item_->item_name)
      dxl_response.punch = read_value_;
  }

  dxl_state_pub_.publish(dxl_response);
}

void DynamixelWorkbenchSingleManager::mxMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelMX dxl_response;

  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, &read_value_);

    if ("model_number" == dxl_->dxl_item_->item_name)
      dxl_response.model_number = read_value_;
    else if ("version_of_firmware" == dxl_->dxl_item_->item_name)
      dxl_response.version_of_firmware = read_value_;
    else if ("id" == dxl_->dxl_item_->item_name)
      dxl_response.id = read_value_;
    else if ("baud_rate" == dxl_->dxl_item_->item_name)
      dxl_response.baud_rate = read_value_;
    else if ("return_delay_time" == dxl_->dxl_item_->item_name)
      dxl_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_angle_limit = read_value_;
    else if ("the_highest_limit_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dxl_->dxl_item_->item_name)
      dxl_response.max_torque = read_value_;
    else if ("status_return_level" == dxl_->dxl_item_->item_name)
      dxl_response.status_return_level = read_value_;
    else if ("alarm_led" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_shutdown = read_value_;
    else if ("multi_turn_offset" == dxl_->dxl_item_->item_name)
      dxl_response.multi_turn_offset = read_value_;
    else if ("resolution_divider" == dxl_->dxl_item_->item_name)
      dxl_response.resolution_divider = read_value_;
    else if ("torque_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_enable = read_value_;
    else if ("led" == dxl_->dxl_item_->item_name)
      dxl_response.led = read_value_;
    else if ("d_gain" == dxl_->dxl_item_->item_name)
      dxl_response.d_gain = read_value_;
    else if ("i_gain" == dxl_->dxl_item_->item_name)
      dxl_response.i_gain = read_value_;
    else if ("p_gain" == dxl_->dxl_item_->item_name)
      dxl_response.p_gain = read_value_;
    else if ("goal_position" == dxl_->dxl_item_->item_name)
      dxl_response.goal_position = read_value_;
    else if ("moving_speed" == dxl_->dxl_item_->item_name)
      dxl_response.moving_speed = read_value_;
    else if ("torque_limit" == dxl_->dxl_item_->item_name)
      dxl_response.torque_limit = read_value_;
    else if ("present_position" == dxl_->dxl_item_->item_name)
      dxl_response.present_position = read_value_;
    else if ("present_speed" == dxl_->dxl_item_->item_name)
      dxl_response.present_speed = read_value_;
    else if ("present_load" == dxl_->dxl_item_->item_name)
      dxl_response.present_load = read_value_;
    else if ("present_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.present_voltage = read_value_;
    else if ("present_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.present_temperature = read_value_;
    else if ("registered" == dxl_->dxl_item_->item_name)
      dxl_response.registered = read_value_;
    else if ("moving" == dxl_->dxl_item_->item_name)
      dxl_response.moving = read_value_;
    else if ("lock" == dxl_->dxl_item_->item_name)
      dxl_response.lock = read_value_;
    else if ("punch" == dxl_->dxl_item_->item_name)
      dxl_response.punch = read_value_;
    else if ("goal_acceleration" == dxl_->dxl_item_->item_name)
      dxl_response.goal_acceleration = read_value_;
  }

  dxl_state_pub_.publish(dxl_response);
}

void DynamixelWorkbenchSingleManager::mx64MotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelMX64 dxl_response;

  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, &read_value_);

    if ("model_number" == dxl_->dxl_item_->item_name)
      dxl_response.model_number = read_value_;
    else if ("version_of_firmware" == dxl_->dxl_item_->item_name)
      dxl_response.version_of_firmware = read_value_;
    else if ("id" == dxl_->dxl_item_->item_name)
      dxl_response.id = read_value_;
    else if ("baud_rate" == dxl_->dxl_item_->item_name)
      dxl_response.baud_rate = read_value_;
    else if ("return_delay_time" == dxl_->dxl_item_->item_name)
      dxl_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_angle_limit = read_value_;
    else if ("the_highest_limit_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dxl_->dxl_item_->item_name)
      dxl_response.max_torque = read_value_;
    else if ("status_return_level" == dxl_->dxl_item_->item_name)
      dxl_response.status_return_level = read_value_;
    else if ("alarm_led" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_shutdown = read_value_;
    else if ("multi_turn_offset" == dxl_->dxl_item_->item_name)
      dxl_response.multi_turn_offset = read_value_;
    else if ("resolution_divider" == dxl_->dxl_item_->item_name)
      dxl_response.resolution_divider = read_value_;
    else if ("torque_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_enable = read_value_;
    else if ("led" == dxl_->dxl_item_->item_name)
      dxl_response.led = read_value_;
    else if ("d_gain" == dxl_->dxl_item_->item_name)
      dxl_response.d_gain = read_value_;
    else if ("i_gain" == dxl_->dxl_item_->item_name)
      dxl_response.i_gain = read_value_;
    else if ("p_gain" == dxl_->dxl_item_->item_name)
      dxl_response.p_gain = read_value_;
    else if ("goal_position" == dxl_->dxl_item_->item_name)
      dxl_response.goal_position = read_value_;
    else if ("moving_speed" == dxl_->dxl_item_->item_name)
      dxl_response.moving_speed = read_value_;
    else if ("torque_limit" == dxl_->dxl_item_->item_name)
      dxl_response.torque_limit = read_value_;
    else if ("present_position" == dxl_->dxl_item_->item_name)
      dxl_response.present_position = read_value_;
    else if ("present_speed" == dxl_->dxl_item_->item_name)
      dxl_response.present_speed = read_value_;
    else if ("present_load" == dxl_->dxl_item_->item_name)
      dxl_response.present_load = read_value_;
    else if ("present_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.present_voltage = read_value_;
    else if ("present_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.present_temperature = read_value_;
    else if ("registered" == dxl_->dxl_item_->item_name)
      dxl_response.registered = read_value_;
    else if ("moving" == dxl_->dxl_item_->item_name)
      dxl_response.moving = read_value_;
    else if ("lock" == dxl_->dxl_item_->item_name)
      dxl_response.lock = read_value_;
    else if ("punch" == dxl_->dxl_item_->item_name)
      dxl_response.punch = read_value_;
    else if ("current" == dxl_->dxl_item_->item_name)
      dxl_response.current = read_value_;
    else if ("torque_control_mode_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_control_mode_enable = read_value_;
    else if ("goal_torque" == dxl_->dxl_item_->item_name)
      dxl_response.goal_torque = read_value_;
    else if ("goal_acceleration" == dxl_->dxl_item_->item_name)
      dxl_response.goal_acceleration = read_value_;
  }

  dxl_state_pub_.publish(dxl_response);
}

void DynamixelWorkbenchSingleManager::mx106MotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelMX106 dxl_response;

  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, &read_value_);

    if ("model_number" == dxl_->dxl_item_->item_name)
      dxl_response.model_number = read_value_;
    else if ("version_of_firmware" == dxl_->dxl_item_->item_name)
      dxl_response.version_of_firmware = read_value_;
    else if ("id" == dxl_->dxl_item_->item_name)
      dxl_response.id = read_value_;
    else if ("baud_rate" == dxl_->dxl_item_->item_name)
      dxl_response.baud_rate = read_value_;
    else if ("return_delay_time" == dxl_->dxl_item_->item_name)
      dxl_response.return_delay_time = read_value_;
    else if ("cw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.cw_angle_limit = read_value_;
    else if ("ccw_angle_limit" == dxl_->dxl_item_->item_name)
      dxl_response.ccw_angle_limit = read_value_;
    else if ("drive_mode" == dxl_->dxl_item_->item_name)
      dxl_response.drive_mode = read_value_;
    else if ("the_highest_limit_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_temperature = read_value_;
    else if ("the_lowest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_lowest_limit_voltage = read_value_;
    else if ("the_highest_limit_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.the_highest_limit_voltage = read_value_;
    else if ("max_torque" == dxl_->dxl_item_->item_name)
      dxl_response.max_torque = read_value_;
    else if ("status_return_level" == dxl_->dxl_item_->item_name)
      dxl_response.status_return_level = read_value_;
    else if ("alarm_led" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_led = read_value_;
    else if ("alarm_shutdown" == dxl_->dxl_item_->item_name)
      dxl_response.alarm_shutdown = read_value_;
    else if ("multi_turn_offset" == dxl_->dxl_item_->item_name)
      dxl_response.multi_turn_offset = read_value_;
    else if ("resolution_divider" == dxl_->dxl_item_->item_name)
      dxl_response.resolution_divider = read_value_;
    else if ("torque_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_enable = read_value_;
    else if ("led" == dxl_->dxl_item_->item_name)
      dxl_response.led = read_value_;
    else if ("d_gain" == dxl_->dxl_item_->item_name)
      dxl_response.d_gain = read_value_;
    else if ("i_gain" == dxl_->dxl_item_->item_name)
      dxl_response.i_gain = read_value_;
    else if ("p_gain" == dxl_->dxl_item_->item_name)
      dxl_response.p_gain = read_value_;
    else if ("goal_position" == dxl_->dxl_item_->item_name)
      dxl_response.goal_position = read_value_;
    else if ("moving_speed" == dxl_->dxl_item_->item_name)
      dxl_response.moving_speed = read_value_;
    else if ("torque_limit" == dxl_->dxl_item_->item_name)
      dxl_response.torque_limit = read_value_;
    else if ("present_position" == dxl_->dxl_item_->item_name)
      dxl_response.present_position = read_value_;
    else if ("present_speed" == dxl_->dxl_item_->item_name)
      dxl_response.present_speed = read_value_;
    else if ("present_load" == dxl_->dxl_item_->item_name)
      dxl_response.present_load = read_value_;
    else if ("present_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.present_voltage = read_value_;
    else if ("present_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.present_temperature = read_value_;
    else if ("registered" == dxl_->dxl_item_->item_name)
      dxl_response.registered = read_value_;
    else if ("moving" == dxl_->dxl_item_->item_name)
      dxl_response.moving = read_value_;
    else if ("lock" == dxl_->dxl_item_->item_name)
      dxl_response.lock = read_value_;
    else if ("punch" == dxl_->dxl_item_->item_name)
      dxl_response.punch = read_value_;
    else if ("current" == dxl_->dxl_item_->item_name)
      dxl_response.current = read_value_;
    else if ("torque_control_mode_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_control_mode_enable = read_value_;
    else if ("goal_torque" == dxl_->dxl_item_->item_name)
      dxl_response.goal_torque = read_value_;
    else if ("goal_acceleration" == dxl_->dxl_item_->item_name)
      dxl_response.goal_acceleration = read_value_;
  }

  dxl_state_pub_.publish(dxl_response);
}

void DynamixelWorkbenchSingleManager::xmMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelXM dxl_response;

  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, &read_value_);

    if ("model_number" == dxl_->dxl_item_->item_name)
      dxl_response.model_number = read_value_;
    else if ("version_of_firmware" == dxl_->dxl_item_->item_name)
      dxl_response.version_of_firmware = read_value_;
    else if ("id" == dxl_->dxl_item_->item_name)
      dxl_response.id = read_value_;
    else if ("baud_rate" == dxl_->dxl_item_->item_name)
      dxl_response.baud_rate = read_value_;
    else if ("return_delay_time" == dxl_->dxl_item_->item_name)
      dxl_response.return_delay_time = read_value_;
    else if ("operating_mode" == dxl_->dxl_item_->item_name)
      dxl_response.operating_mode = read_value_;
    else if ("protocol_version" == dxl_->dxl_item_->item_name)
      dxl_response.protocol_version = read_value_;
    else if ("homing_offset" == dxl_->dxl_item_->item_name)
      dxl_response.homing_offset = read_value_;
    else if ("moving_threshold" == dxl_->dxl_item_->item_name)
      dxl_response.moving_threshold = read_value_;
    else if ("max_temperature_limit" == dxl_->dxl_item_->item_name)
      dxl_response.max_temperature_limit = read_value_;
    else if ("max_voltage_limit" == dxl_->dxl_item_->item_name)
      dxl_response.max_voltage_limit = read_value_;
    else if ("min_voltage_limit" == dxl_->dxl_item_->item_name)
      dxl_response.min_voltage_limit = read_value_;
    else if ("pwm_limit" == dxl_->dxl_item_->item_name)
      dxl_response.pwm_limit = read_value_;
    else if ("current_limit" == dxl_->dxl_item_->item_name)
      dxl_response.current_limit = read_value_;
    else if ("acceleration_limit" == dxl_->dxl_item_->item_name)
      dxl_response.acceleration_limit = read_value_;
    else if ("velocity_limit" == dxl_->dxl_item_->item_name)
      dxl_response.velocity_limit = read_value_;
    else if ("max_position_limit" == dxl_->dxl_item_->item_name)
      dxl_response.max_position_limit = read_value_;
    else if ("min_position_limit" == dxl_->dxl_item_->item_name)
      dxl_response.min_position_limit = read_value_;
    else if ("shutdown" == dxl_->dxl_item_->item_name)
      dxl_response.shutdown = read_value_;
    else if ("torque_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_enable = read_value_;
    else if ("led" == dxl_->dxl_item_->item_name)
      dxl_response.led = read_value_;
    else if ("status_return_level" == dxl_->dxl_item_->item_name)
      dxl_response.status_return_level = read_value_;
    else if ("registered_instruction" == dxl_->dxl_item_->item_name)
      dxl_response.registered_instruction = read_value_;
    else if ("hardware_error_status" == dxl_->dxl_item_->item_name)
      dxl_response.hardware_error_status = read_value_;
    else if ("velocity_i_gain" == dxl_->dxl_item_->item_name)
      dxl_response.velocity_i_gain = read_value_;
    else if ("velocity_p_gain" == dxl_->dxl_item_->item_name)
      dxl_response.velocity_p_gain = read_value_;
    else if ("velocity_d_gain" == dxl_->dxl_item_->item_name)
      dxl_response.velocity_d_gain = read_value_;
    else if ("position_i_gain" == dxl_->dxl_item_->item_name)
      dxl_response.position_i_gain = read_value_;
    else if ("position_p_gain" == dxl_->dxl_item_->item_name)
      dxl_response.position_p_gain = read_value_;
    else if ("feedforward_2nd_gain" == dxl_->dxl_item_->item_name)
      dxl_response.feedforward_2nd_gain = read_value_;
    else if ("feedforward_1st_gain" == dxl_->dxl_item_->item_name)
      dxl_response.feedforward_1st_gain = read_value_;
    else if ("goal_pwm" == dxl_->dxl_item_->item_name)
      dxl_response.goal_pwm = read_value_;
    else if ("goal_current" == dxl_->dxl_item_->item_name)
      dxl_response.goal_current = read_value_;
    else if ("goal_velocity" == dxl_->dxl_item_->item_name)
      dxl_response.goal_velocity = read_value_;
    else if ("profile_acceleration" == dxl_->dxl_item_->item_name)
      dxl_response.profile_acceleration = read_value_;
    else if ("profile_velocity" == dxl_->dxl_item_->item_name)
      dxl_response.profile_velocity = read_value_;
    else if ("goal_position" == dxl_->dxl_item_->item_name)
      dxl_response.goal_position = read_value_;
    else if ("realtime_tick" == dxl_->dxl_item_->item_name)
      dxl_response.realtime_tick = read_value_;
    else if ("moving" == dxl_->dxl_item_->item_name)
      dxl_response.moving = read_value_;
    else if ("moving_status" == dxl_->dxl_item_->item_name)
      dxl_response.moving_status = read_value_;
    else if ("present_pwm" == dxl_->dxl_item_->item_name)
      dxl_response.present_pwm = read_value_;
    else if ("present_current" == dxl_->dxl_item_->item_name)
      dxl_response.present_current = read_value_;
    else if ("present_velocity" == dxl_->dxl_item_->item_name)
      dxl_response.present_velocity = read_value_;
    else if ("present_position" == dxl_->dxl_item_->item_name)
      dxl_response.present_position = read_value_;
    else if ("velocity_trajectory" == dxl_->dxl_item_->item_name)
      dxl_response.velocity_trajectory = read_value_;
    else if ("position_trajectory" == dxl_->dxl_item_->item_name)
      dxl_response.position_trajectory = read_value_;
    else if ("present_input_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.present_input_voltage = read_value_;
    else if ("present_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.present_temperature = read_value_;
    else if ("indirect_address_1" == dxl_->dxl_item_->item_name)
      dxl_response.indirect_address_1 = read_value_;
    else if ("indirect_data_1" == dxl_->dxl_item_->item_name)
      dxl_response.indirect_data_1 = read_value_;
    else if ("indirect_address_29" == dxl_->dxl_item_->item_name)
      dxl_response.indirect_address_29 = read_value_;
    else if ("indirect_data_29" == dxl_->dxl_item_->item_name)
      dxl_response.indirect_data_29 = read_value_;
  }

  dxl_state_pub_.publish(dxl_response);
}

void DynamixelWorkbenchSingleManager::proMotorMessage(void)
{
  dynamixel_workbench_msgs::DynamixelPro dxl_response;

  for (dxl_->it_ctrl_ = dxl_->ctrl_table_.begin(); dxl_->it_ctrl_ != dxl_->ctrl_table_.end(); dxl_->it_ctrl_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_ctrl_->first.c_str()];
    readDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, &read_value_);

    if ("model_number" == dxl_->dxl_item_->item_name)
      dxl_response.model_number = read_value_;
    else if ("version_of_firmware" == dxl_->dxl_item_->item_name)
      dxl_response.version_of_firmware = read_value_;
    else if ("id" == dxl_->dxl_item_->item_name)
      dxl_response.id = read_value_;
    else if ("baud_rate" == dxl_->dxl_item_->item_name)
      dxl_response.baud_rate = read_value_;
    else if ("return_delay_time" == dxl_->dxl_item_->item_name)
      dxl_response.return_delay_time = read_value_;
    else if ("operating_mode" == dxl_->dxl_item_->item_name)
      dxl_response.operating_mode = read_value_;
    else if ("homing_offset" == dxl_->dxl_item_->item_name)
      dxl_response.homing_offset = read_value_;
    else if ("moving_threshold" == dxl_->dxl_item_->item_name)
      dxl_response.moving_threshold = read_value_;
    else if ("max_temperature_limit" == dxl_->dxl_item_->item_name)
      dxl_response.max_temperature_limit = read_value_;
    else if ("max_voltage_limit" == dxl_->dxl_item_->item_name)
      dxl_response.max_voltage_limit = read_value_;
    else if ("min_voltage_limit" == dxl_->dxl_item_->item_name)
      dxl_response.min_voltage_limit = read_value_;
    else if ("acceleration_limit" == dxl_->dxl_item_->item_name)
      dxl_response.acceleration_limit = read_value_;
    else if ("torque_limit" == dxl_->dxl_item_->item_name)
      dxl_response.torque_limit = read_value_;
    else if ("velocity_limit" == dxl_->dxl_item_->item_name)
      dxl_response.velocity_limit = read_value_;
    else if ("max_position_limit" == dxl_->dxl_item_->item_name)
      dxl_response.max_position_limit = read_value_;
    else if ("min_position_limit" == dxl_->dxl_item_->item_name)
      dxl_response.min_position_limit = read_value_;
    else if ("external_port_mod_1" == dxl_->dxl_item_->item_name)
      dxl_response.external_port_mod_1 = read_value_;
    else if ("external_port_mod_2" == dxl_->dxl_item_->item_name)
      dxl_response.external_port_mod_2 = read_value_;
    else if ("external_port_mod_3" == dxl_->dxl_item_->item_name)
      dxl_response.external_port_mod_3 = read_value_;
    else if ("external_port_mod_4" == dxl_->dxl_item_->item_name)
      dxl_response.external_port_mod_4 = read_value_;
    else if ("shutdown" == dxl_->dxl_item_->item_name)
      dxl_response.shutdown = read_value_;
    else if ("indirect_address_1" == dxl_->dxl_item_->item_name)
      dxl_response.indirect_address_1 = read_value_;
    else if ("torque_enable" == dxl_->dxl_item_->item_name)
      dxl_response.torque_enable = read_value_;
    else if ("led_red" == dxl_->dxl_item_->item_name)
      dxl_response.led_red = read_value_;
    else if ("led_green" == dxl_->dxl_item_->item_name)
      dxl_response.led_green = read_value_;
    else if ("led_blue" == dxl_->dxl_item_->item_name)
      dxl_response.led_blue = read_value_;
    else if ("velocity_i_gain" == dxl_->dxl_item_->item_name)
      dxl_response.velocity_i_gain = read_value_;
    else if ("velocity_p_gain" == dxl_->dxl_item_->item_name)
      dxl_response.velocity_p_gain = read_value_;
    else if ("position_p_gain" == dxl_->dxl_item_->item_name)
      dxl_response.position_p_gain = read_value_;
    else if ("goal_position" == dxl_->dxl_item_->item_name)
      dxl_response.goal_position = read_value_;
    else if ("goal_velocity" == dxl_->dxl_item_->item_name)
      dxl_response.goal_velocity = read_value_;
    else if ("goal_torque" == dxl_->dxl_item_->item_name)
      dxl_response.goal_torque = read_value_;
    else if ("goal_acceleration" == dxl_->dxl_item_->item_name)
      dxl_response.goal_acceleration = read_value_;
    else if ("is_moving" == dxl_->dxl_item_->item_name)
      dxl_response.is_moving = read_value_;
    else if ("present_position" == dxl_->dxl_item_->item_name)
      dxl_response.present_position = read_value_;
    else if ("present_velocity" == dxl_->dxl_item_->item_name)
      dxl_response.present_velocity = read_value_;
    else if ("present_current" == dxl_->dxl_item_->item_name)
      dxl_response.present_current = read_value_;
    else if ("present_voltage" == dxl_->dxl_item_->item_name)
      dxl_response.present_voltage = read_value_;
    else if ("present_temperature" == dxl_->dxl_item_->item_name)
      dxl_response.present_temperature = read_value_;
    else if ("external_port_data_1" == dxl_->dxl_item_->item_name)
      dxl_response.external_port_data_1 = read_value_;
    else if ("external_port_data_2" == dxl_->dxl_item_->item_name)
      dxl_response.external_port_data_2 = read_value_;
    else if ("external_port_data_3" == dxl_->dxl_item_->item_name)
      dxl_response.external_port_data_3 = read_value_;
    else if ("external_port_data_4" == dxl_->dxl_item_->item_name)
      dxl_response.external_port_data_4 = read_value_;
    else if ("indirect_data_1" == dxl_->dxl_item_->item_name)
      dxl_response.indirect_data_1 = read_value_;
    else if ("registered_instruction" == dxl_->dxl_item_->item_name)
      dxl_response.registered_instruction = read_value_;
    else if ("status_return_level" == dxl_->dxl_item_->item_name)
      dxl_response.status_return_level = read_value_;
    else if ("hardware_error_status" == dxl_->dxl_item_->item_name)
      dxl_response.hardware_error_status = read_value_;
  }

  dxl_state_pub_.publish(dxl_response);
}


bool DynamixelWorkbenchSingleManager::dynamixelSingleManagerLoop(void)
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;
  bool valid_cmd = false;

  getPublisher();

  if (kbhit())
  {
    if (getchar() == SPACEBAR_ASCII_VALUE)
    {
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
          viewMangerMenu();
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
      else if (strcmp(cmd, "scan") == 0)
      {
        scanDynamixelID();
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
          dxl_->dxl_item_ = dxl_->ctrl_table_[cmd];
          if (dxl_->dxl_item_->item_name == "id")
          {
            writeDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, atoi(param[0]));
            dxl_ = new dxl_motor::DxlMotor(atoi(param[0]), dxl_number_, protocol_version_);
            scanDynamixelID();
          }
          else if (dxl_->dxl_item_->item_name == "baud_rate")
          {
            writeDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, dxl_->baud_rate_table_.find(atoi(param[0]))->second);

            if (portHandler_->setBaudRate(dxl_->baud_rate_table_.find(atoi(param[0]))->first) == false)
            {
              ROS_INFO(" Failed to change baudrate! \n");
            }
            else
            {
              ROS_INFO(" Success to change baudrate! [ BAUD RATE: %d ]\n", dxl_->baud_rate_table_.find(atoi(param[0]))->first);
            }
          }
          else if (dxl_->dxl_item_->item_name == "protocol_version")
          {
            //TODO: Find restriced access address in protocol_version 1.0 of XM430
            writeDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, atof(param[0]));
            packetHandler_->getPacketHandler(atof(param[0]));
          }
          else
          {
            writeDynamixelRegister(dxl_->id_, dxl_->dxl_item_->address, dxl_->dxl_item_->data_length, atoi(param[0]));
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
  manager.viewMangerMenu();

  while (ros::ok())
  {
    manager.dynamixelSingleManagerLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
