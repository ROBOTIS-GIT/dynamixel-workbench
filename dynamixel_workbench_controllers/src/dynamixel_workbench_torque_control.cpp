#include "dynamixel_workbench_controllers/dynamixel_workbench_torque_control.h"

using namespace dynamixel_workbench_torque_control;

DynamixelWorkbenchTorqueControl::DynamixelWorkbenchTorqueControl()
    :nh_priv_("~"),
     is_debug_(false),
     dxl_addparam_result_(false),
     dxl_getdata_result_(false),
     dxl_comm_result_(COMM_TX_FAIL),
     device_name_(DEVICENAME),
     baud_rate_(BAUDRATE),
     motor_model_(""),
     motor_id_(0),
     protocol_version_(PROTOCOL_VERSION),
     dxl_motor_(NULL),
     read_value_(0)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchTorqueControl());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1.0 PacketHandler and Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // init ROS Publish
  dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::MotorStateList>("/dynamixel_workbench_position_control/motor_state",10);

  // init ROS Server
  position_control_server = nh_.advertiseService("/dynamixel_workbench_torque_control", &DynamixelWorkbenchTorqueControl::controlPanTiltMotor, this);

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

  nh_priv_.getParam("pan_motor_/motor_model_", motor_model_);
  nh_priv_.getParam("pan_motor_/motor_id_", motor_id_);
  nh_priv_.getParam("pan_motor_/protocol_version_", protocol_version_);
  ROS_INFO("pan_motor_id: %d", motor_id_);
  ROS_INFO("pan_motor_model: %s", motor_model_.c_str());
  ROS_INFO("pan_motor_protocol_version_: %.1f", protocol_version_);

  initMotor(dxl_motor_, motor_model_, motor_id_, protocol_version_);

  nh_priv_.getParam("tilt_motor_/motor_model_", motor_model_);
  nh_priv_.getParam("tilt_motor_/motor_id_", motor_id_);
  nh_priv_.getParam("tilt_motor_/protocol_version_", protocol_version_);
  ROS_INFO("tilt_motor_id: %d", motor_id_);
  ROS_INFO("tilt_motor_model: %s", motor_model_.c_str());
  ROS_INFO("tilt_motor_protocol_version_: %.1f", protocol_version_);

  initMotor(dxl_motor_, motor_model_, motor_id_, protocol_version_);
}

DynamixelWorkbenchTorqueControl::~DynamixelWorkbenchTorqueControl()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchTorqueControl());
}

bool DynamixelWorkbenchTorqueControl::initDynamixelWorkbenchTorqueControl(void)
{
  ROS_INFO("dynamixel_workbench_torque_control : Init OK!");
  return true;
}

bool DynamixelWorkbenchTorqueControl::shutdownDynamixelWorkbenchTorqueControl(void)
{
  writeTorque(false);
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

bool DynamixelWorkbenchTorqueControl::initMotor(dynamixel_tool::DynamixelTool *dxl_motor, std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dxl_motor = new dynamixel_tool::DynamixelTool(motor_id, motor_model, protocol_version);
  dynamixel_.push_back(dxl_motor);
}

bool DynamixelWorkbenchTorqueControl::writeTorque(bool onoff)
{
  if (onoff == true)
  {
    dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["torque_enable"];
    dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["torque_enable"];

    packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[PAN_MOTOR]->protocol_version_);
    writeDynamixelRegister(packetHandler_, dynamixel_[PAN_MOTOR]->id_, dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, 1);

    packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[TILT_MOTOR]->protocol_version_);
    writeDynamixelRegister(packetHandler_, dynamixel_[TILT_MOTOR]->id_, dynamixel_[TILT_MOTOR]->dxl_item_->address, dynamixel_[TILT_MOTOR]->dxl_item_->data_length, 1);
  }
  else
  {
    dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["torque_enable"];
    dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["torque_enable"];

    packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[PAN_MOTOR]->protocol_version_);
    writeDynamixelRegister(packetHandler_, dynamixel_[PAN_MOTOR]->id_, dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, 0);

    packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[TILT_MOTOR]->protocol_version_);
    writeDynamixelRegister(packetHandler_, dynamixel_[TILT_MOTOR]->id_, dynamixel_[TILT_MOTOR]->dxl_item_->address, dynamixel_[TILT_MOTOR]->dxl_item_->data_length, 0);
  }
  return true;
}

bool DynamixelWorkbenchTorqueControl::writeDynamixelRegister(dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, uint32_t value)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler_, id, addr, (int8_t)value, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler_, id, addr, (int16_t)value, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler_, id, addr, (int32_t)value, &dxl_error);
  }

  if (dxl_comm_result == COMM_SUCCESS)
  {
    if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }
    return true;
  }
  else
  {
    packetHandler->printTxRxResult(dxl_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!", id);
    return false;
  }
}

bool DynamixelWorkbenchTorqueControl::writeSyncDynamixel(uint16_t addr, uint8_t length, uint32_t pan_motor_value, uint32_t tilt_motor_value)
{
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, addr,length);


  dxl_addparam_result_ = groupSyncWrite.addParam(dynamixel_[PAN_MOTOR]->id_, (uint8_t*)&pan_motor_value);
  if (dxl_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[PAN_MOTOR]->id_);
    return false;
  }

  dxl_addparam_result_ = groupSyncWrite.addParam(dynamixel_[TILT_MOTOR]->id_, (uint8_t*)&tilt_motor_value);
  if (dxl_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[TILT_MOTOR]->id_);
    return false;
  }

  dxl_comm_result_ = groupSyncWrite.txPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }
  groupSyncWrite.clearParam();
  return true;
}

bool DynamixelWorkbenchTorqueControl::readDynamixelRegister(dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, uint32_t *value)
{
  uint8_t dxl_error = 0;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dxl_comm_result_ = packetHandler->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result_ = packetHandler->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result_ = packetHandler->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dxl_error);
  }

  if (dxl_comm_result_ == COMM_SUCCESS)
  {
    if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
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
    packetHandler->printTxRxResult(dxl_comm_result_);
    ROS_ERROR("[ID] %u, Fail to read!", id);
    return false;
  }
}

bool DynamixelWorkbenchTorqueControl::readSyncDynamixel(uint16_t addr, uint8_t length, ReadData *data)
{
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, addr, length);

  for (int i = 0; i < dynamixel_.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dynamixel_[i]->id_);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dynamixel_[i]->id_);
      return false;
    }
  }

  // Syncread present position
  dxl_comm_result_ = groupSyncRead.txRxPacket();
  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
  }

  // Check if groupsyncread data is available
  for (int i = 0; i < dynamixel_.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dynamixel_[i]->id_, addr, length);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dynamixel_[i]->id_);
      return false;
    }
  }

  // Get present position value
  for (int i = 0; i < dynamixel_.size(); i++)
  {
    if (length == 1)
      data->dxl_bool_data.push_back(groupSyncRead.getData(dynamixel_[i]->id_, addr, length));
    else
      data->dxl_int_data.push_back(groupSyncRead.getData(dynamixel_[i]->id_, addr, length));
  }
  groupSyncRead.clearParam();
}

bool DynamixelWorkbenchTorqueControl::readTorque(void)
{
  ReadData *data = new ReadData;
  dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["torque_enable"];
  dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["torque_enable"];

  if (dynamixel_[PAN_MOTOR]->protocol_version_ == 1.0 || dynamixel_[TILT_MOTOR]->protocol_version_ == 1.0)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[i]->protocol_version_);
      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_bool_data.push_back(read_value_);
    }
    read_data_["torque_enable"] = data;
    return true;
  }
  else if (dynamixel_[PAN_MOTOR]->dxl_item_->address != dynamixel_[TILT_MOTOR]->dxl_item_->address)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_bool_data.push_back(read_value_);
    }
    read_data_["torque_enable"] = data;
    return true;
  }
  else
  {
    readSyncDynamixel(dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, data);
    read_data_["torque_enable"] = data;
    return true;
  }
}

bool DynamixelWorkbenchTorqueControl::readMoving(void)
{
  ReadData *data = new ReadData;
  dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["moving"];
  dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["moving"];

  if (dynamixel_[PAN_MOTOR]->protocol_version_ == 1.0 || dynamixel_[TILT_MOTOR]->protocol_version_ == 1.0)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[i]->protocol_version_);

      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_bool_data.push_back(read_value_);
    }
    read_data_["moving"] = data;
    return true;
  }
  else if (dynamixel_[PAN_MOTOR]->dxl_item_->address != dynamixel_[TILT_MOTOR]->dxl_item_->address)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_bool_data.push_back(read_value_);
    }
    read_data_["moving"] = data;
    return true;
  }
  else
  {
    readSyncDynamixel(dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, data);
    read_data_["moving"] = data;
    return true;
  }
}

bool DynamixelWorkbenchTorqueControl::readPresentPosition(void)
{
  ReadData *data = new ReadData;
  dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["present_position"];
  dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["present_position"];

  if (dynamixel_[PAN_MOTOR]->protocol_version_ == 1.0 || dynamixel_[TILT_MOTOR]->protocol_version_ == 1.0)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[i]->protocol_version_);

      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_int_data.push_back(read_value_);
    }
    read_data_["present_position"] = data;
    return true;
  }
  else if (dynamixel_[PAN_MOTOR]->dxl_item_->address != dynamixel_[TILT_MOTOR]->dxl_item_->address)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_int_data.push_back(read_value_);
    }
    read_data_["present_position"] = data;
    return true;
  }
  else
  {
    readSyncDynamixel(dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, data);
    read_data_["present_position"] = data;
    return true;
  }
}

bool DynamixelWorkbenchTorqueControl::readPresentVelocity(void)
{
  ReadData *data = new ReadData;
  dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["present_velocity"];
  dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["present_velocity"];

  if (dynamixel_[PAN_MOTOR]->protocol_version_ == 1.0 || dynamixel_[TILT_MOTOR]->protocol_version_ == 1.0)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[i]->protocol_version_);

      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_int_data.push_back(read_value_);
    }
    read_data_["present_velocity"] = data;
    return true;
  }
  else if (dynamixel_[PAN_MOTOR]->dxl_item_->address != dynamixel_[TILT_MOTOR]->dxl_item_->address)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_int_data.push_back(read_value_);
    }
    read_data_["present_velocity"] = data;
    return true;
  }
  else
  {
    readSyncDynamixel(dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, data);
    read_data_["present_velocity"] = data;
    return true;
  }
}

bool DynamixelWorkbenchTorqueControl::readGoalPosition(void)
{
  ReadData *data = new ReadData;
  dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["goal_position"];
  dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["goal_position"];

  if (dynamixel_[PAN_MOTOR]->protocol_version_ == 1.0 || dynamixel_[TILT_MOTOR]->protocol_version_ == 1.0)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[i]->protocol_version_);

      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_int_data.push_back(read_value_);
    }
    read_data_["goal_position"] = data;
    return true;
  }
  else if (dynamixel_[PAN_MOTOR]->dxl_item_->address != dynamixel_[TILT_MOTOR]->dxl_item_->address)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_int_data.push_back(read_value_);
    }
    read_data_["goal_position"] = data;
    return true;
  }
  else
  {
    readSyncDynamixel(dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, data);
    read_data_["goal_position"] = data;
    return true;
  }
}

bool DynamixelWorkbenchTorqueControl::readGoalVelocity(void)
{
  ReadData *data = new ReadData;
  dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["goal_velocity"];
  dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["goal_velocity"];

  if (dynamixel_[PAN_MOTOR]->protocol_version_ == 1.0 || dynamixel_[TILT_MOTOR]->protocol_version_ == 1.0)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      packetHandler_ = packetHandler_->getPacketHandler(dynamixel_[i]->protocol_version_);

      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_int_data.push_back(read_value_);
    }
    read_data_["goal_velocity"] = data;
    return true;
  }
  else if (dynamixel_[PAN_MOTOR]->dxl_item_->address != dynamixel_[TILT_MOTOR]->dxl_item_->address)
  {
    for (int i = 0; i < dynamixel_.size(); i++)
    {
      readDynamixelRegister(packetHandler_, dynamixel_[i]->id_, dynamixel_[i]->dxl_item_->address, dynamixel_[i]->dxl_item_->data_length, &read_value_);
      data->dxl_int_data.push_back(read_value_);
    }
    read_data_["goal_velocity"] = data;
    return true;
  }
  else
  {
    readSyncDynamixel(dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, data);
    read_data_["goal_velocity"] = data;
    return true;
  }
}

bool DynamixelWorkbenchTorqueControl::dynamixelControlLoop(void)
{
  readTorque();
  readMoving();
  readPresentPosition();
  readPresentVelocity();
  readGoalPosition();
  readGoalVelocity();

    dynamixel_workbench_msgs::MotorState dxl_response[dynamixel_.size()];
    dynamixel_workbench_msgs::MotorStateList dxl_response_list;

    for (int i = 0; i < dynamixel_.size(); i++)
    {
      dxl_response[i].id = dynamixel_[i]->id_;
      //dxl_response[i].torque_enable = read_data_["torque_enable"]->dxl_bool_data.at(i);
      //dxl_response[i].moving = read_data_["moving"]->dxl_bool_data.at(i);
      dxl_response[i].present_position = read_data_["present_position"]->dxl_int_data.at(i);
      dxl_response[i].present_velocity = read_data_["present_velocity"]->dxl_int_data.at(i);
      dxl_response[i].goal_position = read_data_["goal_position"]->dxl_int_data.at(i);
      dxl_response[i].goal_velocity = read_data_["goal_velocity"]->dxl_int_data.at(i);

      dxl_response_list.motor_states.push_back(dxl_response[i]);
    }
    dxl_state_pub_.publish(dxl_response_list);
}

bool DynamixelWorkbenchTorqueControl::controlPanTiltMotor(dynamixel_workbench_msgs::SetPosition::Request &req,
                                                            dynamixel_workbench_msgs::SetPosition::Response &res)
{
  dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["goal_position"];
  dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["goal_position"];

  dynamixel_workbench_msgs::MotorState dxl_response[dynamixel_.size()];
  dynamixel_workbench_msgs::MotorStateList dxl_response_list;

  for (int i = 0; i < dynamixel_.size(); i++)
  {
    dxl_response[i].id = dynamixel_[i]->id_;
    dxl_response[i].torque_enable = read_data_["torque_enable"]->dxl_bool_data.at(i);
    dxl_response[i].moving = read_data_["moving"]->dxl_bool_data.at(i);
    dxl_response[i].present_position = read_data_["present_position"]->dxl_int_data.at(i);
    dxl_response[i].present_velocity = read_data_["present_velocity"]->dxl_int_data.at(i);
    dxl_response[i].goal_position = read_data_["goal_position"]->dxl_int_data.at(i);
    dxl_response[i].goal_velocity = read_data_["goal_velocity"]->dxl_int_data.at(i);

    dxl_response_list.motor_states.push_back(dxl_response[i]);
  }
  dxl_state_pub_.publish(dxl_response_list);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_torque_control");
  DynamixelWorkbenchTorqueControl dxl_torque_ctrl;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    dxl_torque_ctrl.dynamixelControlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
