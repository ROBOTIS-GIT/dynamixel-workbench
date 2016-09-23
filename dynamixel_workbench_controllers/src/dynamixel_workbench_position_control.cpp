#include "dynamixel_workbench_controllers/dynamixel_workbench_position_control.h"

using namespace dynamixel_workbench_position_control;

DynamixelWorkbenchPositionControl::DynamixelWorkbenchPositionControl()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(""),
     baud_rate_(0),
     motor_model_(""),
     motor_id_(0),
     protocol_version_(0.0)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);
  nh_priv_.getParam("motor_model_", motor_model_);
  nh_priv_.getParam("protocol_version_", protocol_version_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchPositionControl());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // init ROS Publish
  dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::MotorStateList>("/dynamixel_workbench_position_control/motor_state",10);

  // init ROS Server
  position_control_server = nh_.advertiseService("/dynamixel_workbench_tutorials/pan_tilt", &DynamixelWorkbenchPositionControl::controlPanTiltMotorCallback, this);

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
    ROS_INFO("Succeeded to change the baudrate(%d)\n!", portHandler_->getBaudRate());
  }
  else
  {
    ROS_ERROR("Failed to change the baudrate!");
  }

  nh_priv_.getParam("pan_motor_/motor_id_", motor_id_);
  ROS_INFO("pan_motor_id: %d", motor_id_);
  ROS_INFO("pan_motor_model: %s", motor_model_.c_str());
  ROS_INFO("pan_motor_protocol_version_: %.1f\n", protocol_version_);

  initMotor(motor_model_, motor_id_, protocol_version_);

  nh_priv_.getParam("tilt_motor_/motor_id_", motor_id_);
  ROS_INFO("tilt_motor_id: %d", motor_id_);
  ROS_INFO("tilt_motor_model: %s", motor_model_.c_str());
  ROS_INFO("tilt_motor_protocol_version_: %.1f", protocol_version_);

  initMotor(motor_model_, motor_id_, protocol_version_);
  writeTorque(true);
  writeProfile();
}

DynamixelWorkbenchPositionControl::~DynamixelWorkbenchPositionControl()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchPositionControl());
}

bool DynamixelWorkbenchPositionControl::initDynamixelWorkbenchPositionControl(void)
{
  ROS_INFO("dynamixel_workbench_position_control : Init OK!");
  return true;
}

bool DynamixelWorkbenchPositionControl::shutdownDynamixelWorkbenchPositionControl(void)
{
  writeTorque(false);
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

bool DynamixelWorkbenchPositionControl::initMotor(std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dynamixel_tool::DynamixelTool *dynamixel_motor = new dynamixel_tool::DynamixelTool(motor_id, motor_model, protocol_version);
  dynamixel_.push_back(dynamixel_motor);
}

bool DynamixelWorkbenchPositionControl::writeSyncDynamixel(uint16_t addr, uint8_t length, int64_t pan_motor_value, int64_t tilt_motor_value)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, addr,length);

  dynamixel_addparam_result_ = groupSyncWrite.addParam(dynamixel_[PAN_MOTOR]->id_, (uint8_t*)&pan_motor_value);
  if (dynamixel_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[PAN_MOTOR]->id_);
    return false;
  }

  dynamixel_addparam_result_ = groupSyncWrite.addParam(dynamixel_[TILT_MOTOR]->id_, (uint8_t*)&tilt_motor_value);
  if (dynamixel_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[TILT_MOTOR]->id_);
    return false;
  }

  dynamixel_comm_result_ = groupSyncWrite.txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWrite.clearParam();
  return true;
}

bool DynamixelWorkbenchPositionControl::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value)
{
  uint8_t dynamixel_error = 0;
  int8_t dynamixel_comm_result_;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dynamixel_comm_result_ = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dynamixel_error);
  }
  else if (length == 2)
  {
    dynamixel_comm_result_ = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dynamixel_error);
  }
  else if (length == 4)
  {
    dynamixel_comm_result_ = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dynamixel_error);
  }

  if (dynamixel_comm_result_ == COMM_SUCCESS)
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
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    ROS_ERROR("[ID] %u, Fail to read!", id);
    return false;
  }
}

bool DynamixelWorkbenchPositionControl::writeTorque(bool onoff)
{
  dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["torque_enable"];
  if (onoff == true)
  {
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, true, true);
  }
  else
  {
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, false, false);
  }
}

bool DynamixelWorkbenchPositionControl::writeProfile()
{
  if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
  {
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["moving_speed"];
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, VELOCITY, VELOCITY);
  }
  else if (!strncmp(motor_model_.c_str(), "MX", 2))
  {
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["moving_speed"];
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, VELOCITY, VELOCITY);
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_acceleration"];
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, ACCELERATION, ACCELERATION);
  }
  else if (!strncmp(motor_model_.c_str(), "PRO", 3))
  {
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_velocity"];
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, VELOCITY, VELOCITY);
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_acceleration"];
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, ACCELERATION, ACCELERATION);
  }
  else
  {
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["profile_velocity"];
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, VELOCITY, VELOCITY);
    dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["profile_acceleration"];
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, ACCELERATION, ACCELERATION);
  }
}

bool DynamixelWorkbenchPositionControl::writePosition(int64_t pan_pos, int64_t tilt_pos)
{
  dynamixel_[PAN_TILT_MOTOR]->item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_position"];
  writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->item_->address, dynamixel_[PAN_TILT_MOTOR]->item_->data_length, pan_pos, tilt_pos);
}

bool DynamixelWorkbenchPositionControl::readMotorState(std::string addr_name)
{
  std::vector<int64_t> *read_data = new std::vector<int64_t>;
  int64_t read_value;

  dynamixel_[PAN_MOTOR]->item_ = dynamixel_[PAN_MOTOR]->ctrl_table_[addr_name];
  dynamixel_[TILT_MOTOR]->item_ = dynamixel_[TILT_MOTOR]->ctrl_table_[addr_name];

  readDynamixelRegister(dynamixel_[PAN_MOTOR]->id_, dynamixel_[PAN_MOTOR]->item_->address, dynamixel_[PAN_MOTOR]->item_->data_length, &read_value);
  read_data->push_back(read_value);

  readDynamixelRegister(dynamixel_[TILT_MOTOR]->id_, dynamixel_[TILT_MOTOR]->item_->address, dynamixel_[TILT_MOTOR]->item_->data_length, &read_value);
  read_data->push_back(read_value);

  read_data_[addr_name] = read_data;
  return true;
}

bool DynamixelWorkbenchPositionControl::getPublishedMsg(void)
{
  readMotorState("torque_enable");
  readMotorState("moving");
  readMotorState("goal_position");
  readMotorState("present_position");
  readMotorState("present_velocity");

  if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
  {
    readMotorState("moving_speed");
  }
  else
  {
    readMotorState("goal_velocity");
  }

  if (!strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "PRO", 3))
  {
    readMotorState("goal_acceleration");
  }

  if (!strncmp(motor_model_.c_str(), "XM", 2))
  {
    readMotorState("profile_velocity");
    readMotorState("profile_acceleration");
  }

  if (!strncmp(motor_model_.c_str(), "XM", 2) || !strncmp(motor_model_.c_str(), "PRO", 3))
  {
    readMotorState("min_position_limit");
    readMotorState("max_position_limit");
  }
  else
  {
    readMotorState("cw_angle_limit");
    readMotorState("ccw_angle_limit");
  }
}

int64_t DynamixelWorkbenchPositionControl::convertRadian2Value(double radian)
{
  int64_t value = 0;
  if (radian > 0)
  {
    if (dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_ <= dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_)
      return dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_;

    value = (radian * (dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_ - dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_) / dynamixel_[PAN_TILT_MOTOR]->max_radian_)
                + dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_ >= dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_)
      return dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_;

    value = (radian * (dynamixel_[PAN_MOTOR]->value_of_min_radian_position_ - dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_) / dynamixel_[PAN_TILT_MOTOR]->min_radian_)
                + dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_;
  }
  else
    value = dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_;

  if (value > dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_)
    return dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_;
  else if (value < dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_)
    return dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_;

  return value;
}

bool DynamixelWorkbenchPositionControl::dynamixelControlLoop(void)
{
  getPublishedMsg();

  dynamixel_workbench_msgs::MotorState dynamixel_response[dynamixel_.size()];
  dynamixel_workbench_msgs::MotorStateList dynamixel_response_list;

  for (int i = 0; i < dynamixel_.size(); i++)
  {
    dynamixel_response[i].motor_model = dynamixel_[i]->model_name_;
    dynamixel_response[i].id = dynamixel_[i]->id_;
    dynamixel_response[i].torque_enable = read_data_[i, "torque_enable"]->at(i);
    dynamixel_response[i].present_position = read_data_["present_position"]->at(i);
    dynamixel_response[i].present_velocity = read_data_["present_velocity"]->at(i);
    dynamixel_response[i].goal_position = read_data_["goal_position"]->at(i);
    dynamixel_response[i].moving = read_data_["moving"]->at(i);

    if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
    {
      dynamixel_response[i].moving_speed = read_data_["moving_speed"]->at(i);
    }
    else
    {
      dynamixel_response[i].goal_velocity = read_data_["goal_velocity"]->at(i);
    }

    if (!strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "PRO", 3))
    {
      dynamixel_response[i].goal_acceleration = read_data_["goal_acceleration"]->at(i);
    }

    if (!strncmp(motor_model_.c_str(), "XM", 2))
    {
      dynamixel_response[i].profile_velocity = read_data_["profile_velocity"]->at(i);
      dynamixel_response[i].profile_acceleration = read_data_["profile_acceleration"]->at(i);
    }

    if (!strncmp(motor_model_.c_str(), "XM", 2) || !strncmp(motor_model_.c_str(), "PRO", 3))
    {
      dynamixel_response[i].max_position_limit = read_data_["max_position_limit"]->at(i);
      dynamixel_response[i].min_position_limit = read_data_["min_position_limit"]->at(i);
    }
    else
    {
      dynamixel_response[i].cw_angle_limit = read_data_["cw_angle_limit"]->at(i);
      dynamixel_response[i].ccw_angle_limit = read_data_["ccw_angle_limit"]->at(i);
    }

    dynamixel_response_list.motor_states.push_back(dynamixel_response[i]);
  }
  dynamixel_state_pub_.publish(dynamixel_response_list);
}

bool DynamixelWorkbenchPositionControl::controlPanTiltMotorCallback(dynamixel_workbench_msgs::SetPosition::Request &req,
                                                                    dynamixel_workbench_msgs::SetPosition::Response &res)
{
  int64_t pan_pos = convertRadian2Value(req.pan_pos);
  int64_t tilt_pos = convertRadian2Value(req.tilt_pos);

  writePosition(pan_pos, tilt_pos);

  res.pan_pos = pan_pos;
  res.tilt_pos = tilt_pos;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_position_control");
  DynamixelWorkbenchPositionControl dynamixel_pos_ctrl;
  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    dynamixel_pos_ctrl.dynamixelControlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
