#include "dynamixel_workbench_single_manager/dynamixel_workbench_single_manager.h"

using namespace dynamixel_workbench_single_manager;

DynamixelWorkbenchSingleManager::DynamixelWorkbenchSingleManager()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(DEVICENAME),
     baud_rate_(BAUDRATE),
     protocol_version_(PROTOCOL_VERSION)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);
  nh_priv_.getParam("protocol_version_",protocol_version_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchSingleManager());

  // Init ROS publish
  dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::DynamixelResponse>("/dxl_motor_state",10);

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

}

DynamixelWorkbenchSingleManager::~DynamixelWorkbenchSingleManager()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchSingleManager());
}

bool DynamixelWorkbenchSingleManager::initDynamixelWorkbenchSingleManager()
{
  ROS_INFO("dynamixel_workbench_single_manager : Init OK!");
  return true;
}

bool DynamixelWorkbenchSingleManager::shutdownDynamixelWorkbenchSingleManager()
{
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

int DynamixelWorkbenchSingleManager::getch()
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

int DynamixelWorkbenchSingleManager::kbhit()
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

bool DynamixelWorkbenchSingleManager::scanDynamixelID()
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
        dxl_ = new dxl_motor::DxlMotor(dxl_id, dxl_model, protocol_version_);
      }
      else
      {
        fprintf(stderr,".");
      }
    }

    if (dxl_ == NULL)
    {
      ROS_INFO("");
      ROS_ERROR("...Failed to find dynamixel!");
      return false;
    }
    else
    {
      ROS_INFO("");
      ROS_INFO("...Succeeded to find dynamixel");
      ROS_INFO("[ID] %u, [Model No.] %s", dxl_->id_, dxl_->model_name_.c_str());
      return true;
    }
  }
  else
  {
    ROS_INFO("Scan Dynamixel Using Protocol 2.0");
    for (dxl_id = 1; dxl_id < 253; dxl_id++)
    {
      if (packetHandler_->ping(portHandler_, dxl_id, &dxl_model, &dxl_error) == COMM_SUCCESS)
      {
        dxl_ = new dxl_motor::DxlMotor(dxl_id, dxl_model, protocol_version_);
      }
      else
      {
        fprintf(stderr,".");
      }
    }

    if (dxl_ == NULL)
    {
      ROS_INFO("");
      ROS_ERROR("...Failed to find dynamixel!");
      return false;
    }
    else
    {
      ROS_INFO("");
      ROS_INFO("...Succeeded to find dynamixel");
      ROS_INFO("[ID] %u, [Model No.] %s", dxl_->id_, dxl_->model_name_.c_str());
      return true;
    }
  }
}

bool DynamixelWorkbenchSingleManager::writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t value)
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
      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_8_bit);
      return true;

    }
    else if (length == 2)
    {
      *value = value_16_bit;
      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_16_bit);
      return true;
    }
    else if (length == 4)
    {
      *value = value_32_bit;
      //ROS_INFO("[ID] %u, [Present Value] %d", id, value_32_bit);
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

void DynamixelWorkbenchSingleManager::viewRemoteMenu()
{
  ROS_INFO("----------------------------------------------------------------------\n");
  ROS_INFO("Menu:\n");
  ROS_INFO("[-h].......................: display this help");
  ROS_INFO("[-torque onoff]............: dynamixel torque onoff");
  ROS_INFO("[-id num]..................: dynamixel ID set");
  ROS_INFO("[-exit]....................: shutdown");
  ROS_INFO("----------------------------------------------------------------------\n");
}

bool DynamixelWorkbenchSingleManager::dynamixelSingleManagerLoop(void)
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;

  printf("[CMD] ");
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

  while(token != 0)
  {
    strcpy(param[num_param++], token);
    token = strtok(0, " ");
  }

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
  {
      viewRemoteMenu();
  }
  else if (strcmp(cmd, "exit") == 0)
  {
    DynamixelWorkbenchSingleManager::shutdownDynamixelWorkbenchSingleManager();
    return true;
  }
  else if (strcmp(cmd, "id") == 0)
  {
    if (num_param == 1)
    {
      dxl_->dxl_item_ = dxl_->ctrl_table_["ID"];
      DynamixelWorkbenchSingleManager::writeDynamixelRegister(dxl_->id_,
                                                              dxl_->dxl_item_->address,
                                                              dxl_->dxl_item_->data_length,
                                                              atoi(param[0]));
    }
    else
    {
      ROS_ERROR("Invalid parameters!");
    }
  }
  else if (strcmp(cmd, "torque") == 0)
  {
    if (num_param ==1)
    {
      dxl_->dxl_item_ = dxl_->ctrl_table_["torque_enable"];
      DynamixelWorkbenchSingleManager::writeDynamixelRegister(dxl_->id_,
                                                              dxl_->dxl_item_->address,
                                                              dxl_->dxl_item_->data_length,
                                                              atoi(param[0]));
    }
    else
    {
      ROS_ERROR("Invalid parameters!");
    }
  }
  else
  {
    ROS_ERROR("Invalid command. Please check menu[help, ?]");
  }

  dynamixel_workbench_msgs::DynamixelResponse dxl_response;
  for (dxl_->it_ = dxl_->ctrl_table_.begin(); dxl_->it_ != dxl_->ctrl_table_.end(); dxl_->it_++)
  {
    dxl_->dxl_item_ = dxl_->ctrl_table_[dxl_->it_->first.c_str()];
    DynamixelWorkbenchSingleManager::readDynamixelRegister(dxl_->id_,
                                                           dxl_->dxl_item_->address,
                                                           dxl_->dxl_item_->data_length,
                                                           &read_value_);
    if(dxl_->dxl_item_->item_name == "realtime_tick")
        dxl_response.timestamps = read_value_;
    else if(dxl_->dxl_item_->item_name == "torque_enable")
        dxl_response.torque = read_value_;
  }

  dxl_state_pub_.publish(dxl_response);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_single_manager");
  DynamixelWorkbenchSingleManager manager;
  ros::Rate loop_rate(50);

  while (ros::ok())
  {
    manager.dynamixelSingleManagerLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
