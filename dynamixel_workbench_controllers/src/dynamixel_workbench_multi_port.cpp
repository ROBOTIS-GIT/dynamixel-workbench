#include "dynamixel_workbench_controllers/dynamixel_workbench_multi_port.h"

using namespace dynamixel_workbench_multi_port;

DynamixelWorkbenchMultiPort::DynamixelWorkbenchMultiPort()
    :nh_priv_("~"),
     is_debug_(false),
     pan_motor_device_name_(""),
     pan_motor_baud_rate_(0),
     pan_motor_model_(""),
     pan_motor_id_(0),
     pan_motor_protocol_version_(0.0),
     tilt_motor_device_name_(""),
     tilt_motor_baud_rate_(0),
     tilt_motor_model_(""),
     tilt_motor_id_(0),
     tilt_motor_protocol_version_(0.0)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);

  nh_priv_.getParam("pan_motor_/device_name_", pan_motor_device_name_);
  nh_priv_.getParam("pan_motor_/baud_rate_", pan_motor_baud_rate_);
  nh_priv_.getParam("pan_motor_/motor_model_", pan_motor_model_);
  nh_priv_.getParam("pan_motor_/protocol_version_", pan_motor_protocol_version_);
  nh_priv_.getParam("pan_motor_/motor_id_", pan_motor_id_);

  nh_priv_.getParam("tilt_motor_/device_name_", tilt_motor_device_name_);
  nh_priv_.getParam("tilt_motor_/baud_rate_", tilt_motor_baud_rate_);
  nh_priv_.getParam("tilt_motor_/motor_model_", tilt_motor_model_);
  nh_priv_.getParam("tilt_motor_/protocol_version_", tilt_motor_protocol_version_);
  nh_priv_.getParam("tilt_motor_/motor_id_", tilt_motor_id_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchMultiPort());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  pan_motor_portHandler_ = dynamixel::PortHandler::getPortHandler(pan_motor_device_name_.c_str());
  tilt_motor_portHandler_ = dynamixel::PortHandler::getPortHandler(tilt_motor_device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol 2.0 PacketHandler
  pan_motor_packetHandler_ = dynamixel::PacketHandler::getPacketHandler(pan_motor_protocol_version_);
  tilt_motor_packetHandler_ = dynamixel::PacketHandler::getPacketHandler(tilt_motor_protocol_version_);

  // init ROS Publish
  dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::MotorStateList>("/dynamixel_workbench_multi_port/motor_state",10);

  // init ROS Server
  multi_port_control_server = nh_.advertiseService("/dynamixel_workbench_tutorials/pan_tilt", &DynamixelWorkbenchMultiPort::controlPanTiltMotorCallback, this);

  // Open port
  if (pan_motor_portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the pan_motor_port(%s)!", pan_motor_device_name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the pan_motor_port!");
  }

  // Set port baudrate
  if (pan_motor_portHandler_->setBaudRate(pan_motor_baud_rate_))
  {
    ROS_INFO("Succeeded to change the pan_motor_baudrate(%d)\n!", pan_motor_portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the pan_motor_baudrate!");
  }

  ROS_INFO("pan_motor_device_name: %s", pan_motor_device_name_.c_str());
  ROS_INFO("pan_motor_baud_rate: %d", pan_motor_baud_rate_);
  ROS_INFO("pan_motor_protocol_version_: %.1f", pan_motor_protocol_version_);
  ROS_INFO("pan_motor_id: %d", pan_motor_id_);
  ROS_INFO("pan_motor_model: %s\n", pan_motor_model_.c_str());

  initMotor(pan_motor_model_, pan_motor_id_, pan_motor_protocol_version_);

  // Open port
  if (tilt_motor_portHandler_->openPort())
  {
    ROS_INFO("Succeeded to open the tilt_motor_port(%s)!", tilt_motor_device_name_.c_str());
  }
  else
  {
    ROS_ERROR("Failed to open the tilt_motor_port!");
  }

  // Set port baudrate
  if (tilt_motor_portHandler_->setBaudRate(tilt_motor_baud_rate_))
  {
    ROS_INFO("Succeeded to change the tilt_motor_baudrate(%d)\n!", tilt_motor_portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the tilt_motor_baudrate!");
  }

  ROS_INFO("tilt_motor_device_name: %s", tilt_motor_device_name_.c_str());
  ROS_INFO("tilt_motor_baud_rate: %d", tilt_motor_baud_rate_);
  ROS_INFO("tilt_motor_protocol_version_: %.1f", tilt_motor_protocol_version_);
  ROS_INFO("tilt_motor_id: %d", tilt_motor_id_);
  ROS_INFO("tilt_motor_model: %s\n", tilt_motor_model_.c_str());

  initMotor(tilt_motor_model_, tilt_motor_id_, tilt_motor_protocol_version_);

  writeTorque(true);
  writeProfile();
}

DynamixelWorkbenchMultiPort::~DynamixelWorkbenchMultiPort()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchMultiPort());
}

bool DynamixelWorkbenchMultiPort::initDynamixelWorkbenchMultiPort(void)
{
  ROS_INFO("dynamixel_workbench_multi_port : Init OK!");
  return true;
}

bool DynamixelWorkbenchMultiPort::shutdownDynamixelWorkbenchMultiPort(void)
{
  writeTorque(false);
  pan_motor_portHandler_->closePort();
  tilt_motor_portHandler_->closePort();
  ros::shutdown();
  return true;
}

bool DynamixelWorkbenchMultiPort::initMotor(std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dynamixel_tool::DynamixelTool *dynamixel_motor = new dynamixel_tool::DynamixelTool(motor_id, motor_model, protocol_version);
  dynamixel_.push_back(dynamixel_motor);
}

bool DynamixelWorkbenchMultiPort::writeDynamixelRegister(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t value)
{
  uint8_t dynamixel_error = 0;
  int dynamixel_comm_result = COMM_TX_FAIL;

  if (length == 1)
  {
    dynamixel_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (int8_t)value, &dynamixel_error);
  }
  else if (length == 2)
  {
    dynamixel_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (int16_t)value, &dynamixel_error);
  }
  else if (length == 4)
  {
    dynamixel_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, (int32_t)value, &dynamixel_error);
  }

  if (dynamixel_comm_result == COMM_SUCCESS)
  {
    if (dynamixel_error != 0)
    {
      packetHandler->printRxPacketError(dynamixel_error);
    }
    return true;
  }
  else
  {
    packetHandler->printTxRxResult(dynamixel_comm_result);
    ROS_ERROR("[ID] %u, Fail to write!", id);
    return false;
  }
}

bool DynamixelWorkbenchMultiPort::readDynamixelRegister(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t *value)
{
  uint8_t dynamixel_error = 0;
  int8_t dynamixel_comm_result_;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dynamixel_comm_result_ = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&value_8_bit, &dynamixel_error);
  }
  else if (length == 2)
  {
    dynamixel_comm_result_ = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&value_16_bit, &dynamixel_error);
  }
  else if (length == 4)
  {
    dynamixel_comm_result_ = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&value_32_bit, &dynamixel_error);
  }

  if (dynamixel_comm_result_ == COMM_SUCCESS)
  {
    if (dynamixel_error != 0)
    {
      packetHandler->printRxPacketError(dynamixel_error);
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
    packetHandler->printTxRxResult(dynamixel_comm_result_);
    ROS_ERROR("[ID] %u, Fail to read!", id);
    return false;
  }
}

bool DynamixelWorkbenchMultiPort::writeTorque(bool onoff)
{
  dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["torque_enable"];
  dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["torque_enable"];

  if (onoff == true)
  {
    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, true);
    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, true);
  }
  else
  {
    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, false);
    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, false);
  }
}

bool DynamixelWorkbenchMultiPort::writeProfile()
{
  if (!strncmp(pan_motor_model_.c_str(), "AX", 2) || !strncmp(pan_motor_model_.c_str(), "RX", 2) || !strncmp(pan_motor_model_.c_str(), "EX", 2))
  {
    dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["moving_speed"];

    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, VELOCITY);
  }
  else if (!strncmp(pan_motor_model_.c_str(), "MX", 2))
  {
    dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["moving_speed"];

    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, VELOCITY);

    dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["goal_acceleration"];

    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, ACCELERATION);
  }
  else if (!strncmp(pan_motor_model_.c_str(), "PRO", 3))
  {
    dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["goal_velocity"];

    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, VELOCITY);

    dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["goal_acceleration"];

    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, ACCELERATION);
  }
  else
  {
    dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["profile_velocity"];

    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, VELOCITY);

    dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["profile_acceleration"];

    writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, ACCELERATION);
  }

  if (!strncmp(tilt_motor_model_.c_str(), "AX", 2) || !strncmp(tilt_motor_model_.c_str(), "RX", 2) || !strncmp(tilt_motor_model_.c_str(), "EX", 2))
  {
    dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["moving_speed"];

    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, VELOCITY);
  }
  else if (!strncmp(tilt_motor_model_.c_str(), "MX", 2))
  {
    dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["moving_speed"];

    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, VELOCITY);

    dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["goal_acceleration"];

    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, ACCELERATION);
  }
  else if (!strncmp(tilt_motor_model_.c_str(), "PRO", 3))
  {
    dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["goal_velocity"];

    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, VELOCITY);

    dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["goal_acceleration"];

    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, ACCELERATION);
  }
  else
  {
    dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["profile_velocity"];

    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, VELOCITY);

    dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["profile_acceleration"];

    writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, ACCELERATION);
 }
}

bool DynamixelWorkbenchMultiPort::writePosition(int64_t pan_pos, int64_t tilt_pos)
{
  dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_["goal_position"];
  dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_["goal_position"];

  writeDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, pan_pos);
  writeDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, tilt_pos);
}

bool DynamixelWorkbenchMultiPort::readMotorState(int8_t motor, std::string addr_name)
{
  int64_t read_value;

  if (motor == PAN_MOTOR)
  {
    dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_[addr_name];

    readDynamixelRegister(pan_motor_portHandler_, pan_motor_packetHandler_, pan_motor_id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, &read_value);

    read_pan_motor_data_[addr_name] = read_value;
    return true;
  }
  else
  {
    dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_[addr_name];

    readDynamixelRegister(tilt_motor_portHandler_, tilt_motor_packetHandler_, tilt_motor_id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, &read_value);

    read_tilt_motor_data_[addr_name] = read_value;
    return true;
  }
}

bool DynamixelWorkbenchMultiPort::getPublishedMsg(void)
{
  readMotorState(PAN_MOTOR, "torque_enable");
  readMotorState(PAN_MOTOR, "moving");
  readMotorState(PAN_MOTOR, "goal_position");
  readMotorState(PAN_MOTOR, "present_position");
  readMotorState(PAN_MOTOR, "present_velocity");

  if (!strncmp(pan_motor_model_.c_str(), "AX", 2) || !strncmp(pan_motor_model_.c_str(), "RX", 2) || !strncmp(pan_motor_model_.c_str(), "MX", 2) || !strncmp(pan_motor_model_.c_str(), "EX", 2))
  {
    readMotorState(PAN_MOTOR, "moving_speed");
  }
  else
  {
    readMotorState(PAN_MOTOR, "goal_velocity");
  }

  if (!strncmp(pan_motor_model_.c_str(), "MX", 2) || !strncmp(pan_motor_model_.c_str(), "PRO", 3))
  {
    readMotorState(PAN_MOTOR, "goal_acceleration");
  }

  if (!strncmp(pan_motor_model_.c_str(), "XM", 2))
  {
    readMotorState(PAN_MOTOR, "profile_velocity");
    readMotorState(PAN_MOTOR, "profile_acceleration");
  }

  if (!strncmp(pan_motor_model_.c_str(), "XM", 2) || !strncmp(pan_motor_model_.c_str(), "PRO", 3))
  {
    readMotorState(PAN_MOTOR, "min_position_limit");
    readMotorState(PAN_MOTOR, "max_position_limit");
  }
  else
  {
    readMotorState(PAN_MOTOR, "cw_angle_limit");
    readMotorState(PAN_MOTOR, "ccw_angle_limit");
  }

  readMotorState(TILT_MOTOR, "torque_enable");
  readMotorState(TILT_MOTOR, "moving");
  readMotorState(TILT_MOTOR, "goal_position");
  readMotorState(TILT_MOTOR, "present_position");
  readMotorState(TILT_MOTOR, "present_velocity");

  if (!strncmp(tilt_motor_model_.c_str(), "AX", 2) || !strncmp(tilt_motor_model_.c_str(), "RX", 2) || !strncmp(tilt_motor_model_.c_str(), "MX", 2) || !strncmp(tilt_motor_model_.c_str(), "EX", 2))
  {
    readMotorState(TILT_MOTOR, "moving_speed");
  }
  else
  {
    readMotorState(TILT_MOTOR, "goal_velocity");
  }

  if (!strncmp(tilt_motor_model_.c_str(), "MX", 2) || !strncmp(tilt_motor_model_.c_str(), "PRO", 3))
  {
    readMotorState(TILT_MOTOR, "goal_acceleration");
  }

  if (!strncmp(tilt_motor_model_.c_str(), "XM", 2))
  {
    readMotorState(TILT_MOTOR, "profile_velocity");
    readMotorState(TILT_MOTOR, "profile_acceleration");
  }

  if (!strncmp(tilt_motor_model_.c_str(), "XM", 2) || !strncmp(tilt_motor_model_.c_str(), "PRO", 3))
  {
    readMotorState(TILT_MOTOR, "min_position_limit");
    readMotorState(TILT_MOTOR, "max_position_limit");
  }
  else
  {
    readMotorState(TILT_MOTOR, "cw_angle_limit");
    readMotorState(TILT_MOTOR, "ccw_angle_limit");
  }
}

int64_t DynamixelWorkbenchMultiPort::convertRadian2Value(int8_t motor, double radian)
{
  int64_t value = 0;
  if (radian > 0)
  {
    if (dynamixel_[motor]->value_of_max_radian_position_ <= dynamixel_[motor]->value_of_0_radian_position_)
      return dynamixel_[motor]->value_of_max_radian_position_;

    value = (radian * (dynamixel_[motor]->value_of_max_radian_position_ - dynamixel_[motor]->value_of_0_radian_position_) / dynamixel_[motor]->max_radian_)
                + dynamixel_[motor]->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (dynamixel_[motor]->value_of_min_radian_position_ >= dynamixel_[motor]->value_of_0_radian_position_)
      return dynamixel_[motor]->value_of_min_radian_position_;

    value = (radian * (dynamixel_[motor]->value_of_min_radian_position_ - dynamixel_[motor]->value_of_0_radian_position_) / dynamixel_[motor]->min_radian_)
                + dynamixel_[motor]->value_of_0_radian_position_;
  }
  else
    value = dynamixel_[motor]->value_of_0_radian_position_;

  if (value > dynamixel_[motor]->value_of_max_radian_position_)
    return dynamixel_[motor]->value_of_max_radian_position_;
  else if (value < dynamixel_[motor]->value_of_min_radian_position_)
    return dynamixel_[motor]->value_of_min_radian_position_;

  return value;
}

bool DynamixelWorkbenchMultiPort::dynamixelControlLoop(void)
{
  getPublishedMsg();

  dynamixel_workbench_msgs::MotorState dynamixel_response[dynamixel_.size()];
  dynamixel_workbench_msgs::MotorStateList dynamixel_response_list;

  dynamixel_response[PAN_MOTOR].motor_model = dynamixel_[PAN_MOTOR]->model_name_;
  dynamixel_response[PAN_MOTOR].id = dynamixel_[PAN_MOTOR]->id_;
  dynamixel_response[PAN_MOTOR].torque_enable = read_pan_motor_data_["torque_enable"];
  dynamixel_response[PAN_MOTOR].present_position = read_pan_motor_data_["present_position"];
  dynamixel_response[PAN_MOTOR].present_velocity = read_pan_motor_data_["present_velocity"];
  dynamixel_response[PAN_MOTOR].goal_position = read_pan_motor_data_["goal_position"];
  dynamixel_response[PAN_MOTOR].moving = read_pan_motor_data_["moving"];

  if (!strncmp(pan_motor_model_.c_str(), "AX", 2) || !strncmp(pan_motor_model_.c_str(), "RX", 2) || !strncmp(pan_motor_model_.c_str(), "MX", 2) || !strncmp(pan_motor_model_.c_str(), "EX", 2))
  {
    dynamixel_response[PAN_MOTOR].moving_speed = read_pan_motor_data_["moving_speed"];
  }
  else
  {
    dynamixel_response[PAN_MOTOR].goal_velocity = read_pan_motor_data_["goal_velocity"];
  }

  if (!strncmp(pan_motor_model_.c_str(), "MX", 2) || !strncmp(pan_motor_model_.c_str(), "PRO", 3))
  {
    dynamixel_response[PAN_MOTOR].goal_acceleration = read_pan_motor_data_["goal_acceleration"];
  }

  if (!strncmp(pan_motor_model_.c_str(), "XM", 2))
  {
    dynamixel_response[PAN_MOTOR].profile_velocity = read_pan_motor_data_["profile_velocity"];
    dynamixel_response[PAN_MOTOR].profile_acceleration = read_pan_motor_data_["profile_acceleration"];
  }

  if (!strncmp(pan_motor_model_.c_str(), "XM", 2) || !strncmp(pan_motor_model_.c_str(), "PRO", 3))
  {
    dynamixel_response[PAN_MOTOR].max_position_limit = read_pan_motor_data_["max_position_limit"];
    dynamixel_response[PAN_MOTOR].min_position_limit = read_pan_motor_data_["min_position_limit"];
  }
  else
  {
    dynamixel_response[PAN_MOTOR].cw_angle_limit = read_pan_motor_data_["cw_angle_limit"];
    dynamixel_response[PAN_MOTOR].ccw_angle_limit = read_pan_motor_data_["ccw_angle_limit"];
  }

  dynamixel_response_list.motor_states.push_back(dynamixel_response[PAN_MOTOR]);

  dynamixel_response[TILT_MOTOR].motor_model = dynamixel_[TILT_MOTOR]->model_name_;
  dynamixel_response[TILT_MOTOR].id = dynamixel_[TILT_MOTOR]->id_;
  dynamixel_response[TILT_MOTOR].torque_enable = read_tilt_motor_data_["torque_enable"];
  dynamixel_response[TILT_MOTOR].present_position = read_tilt_motor_data_["present_position"];
  dynamixel_response[TILT_MOTOR].present_velocity = read_tilt_motor_data_["present_velocity"];
  dynamixel_response[TILT_MOTOR].goal_position = read_tilt_motor_data_["goal_position"];
  dynamixel_response[TILT_MOTOR].moving = read_tilt_motor_data_["moving"];

  if (!strncmp(tilt_motor_model_.c_str(), "AX", 2) || !strncmp(tilt_motor_model_.c_str(), "RX", 2) || !strncmp(tilt_motor_model_.c_str(), "MX", 2) || !strncmp(tilt_motor_model_.c_str(), "EX", 2))
  {
    dynamixel_response[TILT_MOTOR].moving_speed = read_tilt_motor_data_["moving_speed"];
  }
  else
  {
    dynamixel_response[TILT_MOTOR].goal_velocity = read_tilt_motor_data_["goal_velocity"];
  }

  if (!strncmp(tilt_motor_model_.c_str(), "MX", 2) || !strncmp(tilt_motor_model_.c_str(), "PRO", 3))
  {
    dynamixel_response[TILT_MOTOR].goal_acceleration = read_tilt_motor_data_["goal_acceleration"];
  }

  if (!strncmp(tilt_motor_model_.c_str(), "XM", 2))
  {
    dynamixel_response[TILT_MOTOR].profile_velocity = read_tilt_motor_data_["profile_velocity"];
    dynamixel_response[TILT_MOTOR].profile_acceleration = read_tilt_motor_data_["profile_acceleration"];
  }

  if (!strncmp(tilt_motor_model_.c_str(), "XM", 2) || !strncmp(tilt_motor_model_.c_str(), "PRO", 3))
  {
    dynamixel_response[TILT_MOTOR].max_position_limit = read_tilt_motor_data_["max_position_limit"];
    dynamixel_response[TILT_MOTOR].min_position_limit = read_tilt_motor_data_["min_position_limit"];
  }
  else
  {
    dynamixel_response[TILT_MOTOR].cw_angle_limit = read_tilt_motor_data_["cw_angle_limit"];
    dynamixel_response[TILT_MOTOR].ccw_angle_limit = read_tilt_motor_data_["ccw_angle_limit"];
  }

  dynamixel_response_list.motor_states.push_back(dynamixel_response[TILT_MOTOR]);

  dynamixel_state_pub_.publish(dynamixel_response_list);
}

bool DynamixelWorkbenchMultiPort::controlPanTiltMotorCallback(dynamixel_workbench_msgs::SetPosition::Request &req,
                                                              dynamixel_workbench_msgs::SetPosition::Response &res)
{
  int64_t pan_pos = convertRadian2Value(PAN_MOTOR, req.pan_pos);
  int64_t tilt_pos = convertRadian2Value(TILT_MOTOR, req.tilt_pos);

  writePosition(pan_pos, tilt_pos);

  res.pan_pos = pan_pos;
  res.tilt_pos = tilt_pos;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_multi_port");
  DynamixelWorkbenchMultiPort dynamixel_multi_port;
  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    dynamixel_multi_port.dynamixelControlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
