#include "dynamixel_workbench_controllers/dynamixel_workbench_torque_control.h"

using namespace dynamixel_workbench_torque_control;

DynamixelWorkbenchTorqueControl::DynamixelWorkbenchTorqueControl()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(""),
     baud_rate_(0.0),
     motor_model_(""),
     motor_id_(0),
     protocol_version_(0.0),
     pan_des_pos_(0),
     pan_pre_pos_(0),
     pan_torque_(0.0),
     tilt_des_pos_(0),
     tilt_pre_pos_(0),
     tilt_torque_(0.0)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);
  nh_priv_.getParam("motor_model_", motor_model_);
  nh_priv_.getParam("protocol_version_", protocol_version_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchTorqueControl());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // init ROS Publish
  dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::MotorStateList>("/dynamixel_workbench_torque_control/motor_state",10);

  // init ROS Server
  position_control_server = nh_.advertiseService("/dynamixel_workbench_tutorials/pan_tilt", &DynamixelWorkbenchTorqueControl::controlPanTiltMotorCallback, this);

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

  readMotorState("present_position");
  pan_des_pos_ = read_data_["present_position"]->at(PAN_MOTOR);
  tilt_des_pos_ = read_data_["present_position"]->at(TILT_MOTOR);
  pan_pre_pos_ = read_data_["present_position"]->at(PAN_MOTOR);
  tilt_pre_pos_ = read_data_["present_position"]->at(TILT_MOTOR);
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

bool DynamixelWorkbenchTorqueControl::initMotor(std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dynamixel_tool::DynamixelTool *dxl_motor = new dynamixel_tool::DynamixelTool(motor_id, motor_model, protocol_version);
  dynamixel_.push_back(dxl_motor);
}

bool DynamixelWorkbenchTorqueControl::writeSyncDynamixel(uint16_t addr, uint8_t length, int64_t pan_motor_value, int64_t tilt_motor_value)
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

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

bool DynamixelWorkbenchTorqueControl::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value)
{
  uint8_t dxl_error = 0;
  int8_t dxl_comm_result_;

  int8_t value_8_bit = 0;
  int16_t value_16_bit = 0;
  int32_t value_32_bit = 0;

  if (length == 1)
  {
    dxl_comm_result_ = packetHandler_->read1ByteTxRx(portHandler_, id, addr, (uint8_t*)&value_8_bit, &dxl_error);
  }
  else if (length == 2)
  {
    dxl_comm_result_ = packetHandler_->read2ByteTxRx(portHandler_, id, addr, (uint16_t*)&value_16_bit, &dxl_error);
  }
  else if (length == 4)
  {
    dxl_comm_result_ = packetHandler_->read4ByteTxRx(portHandler_, id, addr, (uint32_t*)&value_32_bit, &dxl_error);
  }

  if (dxl_comm_result_ == COMM_SUCCESS)
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
    packetHandler_->printTxRxResult(dxl_comm_result_);
    ROS_ERROR("[ID] %u, Fail to read!", id);
    return false;
  }
}

bool DynamixelWorkbenchTorqueControl::writeTorque(bool onoff)
{
  dynamixel_[PAN_TILT_MOTOR]->dxl_item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["torque_enable"];
  if (onoff == true)
  {
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->dxl_item_->address, dynamixel_[PAN_TILT_MOTOR]->dxl_item_->data_length, true, true);
  }
  else
  {
    writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->dxl_item_->address, dynamixel_[PAN_TILT_MOTOR]->dxl_item_->data_length, false, false);
  }
}

bool DynamixelWorkbenchTorqueControl::writeCurrent(int64_t pan_cur, int64_t tilt_cur)
{
  dynamixel_[PAN_TILT_MOTOR]->dxl_item_ = dynamixel_[PAN_TILT_MOTOR]->ctrl_table_["goal_current"];
  writeSyncDynamixel(dynamixel_[PAN_TILT_MOTOR]->dxl_item_->address, dynamixel_[PAN_TILT_MOTOR]->dxl_item_->data_length, pan_cur, tilt_cur);
}

bool DynamixelWorkbenchTorqueControl::readMotorState(std::string addr_name)
{
  std::vector<int64_t> *read_data = new std::vector<int64_t>;
  int64_t read_value;

  dynamixel_[PAN_MOTOR]->dxl_item_ = dynamixel_[PAN_MOTOR]->ctrl_table_[addr_name];
  dynamixel_[TILT_MOTOR]->dxl_item_ = dynamixel_[TILT_MOTOR]->ctrl_table_[addr_name];

  readDynamixelRegister(dynamixel_[PAN_MOTOR]->id_, dynamixel_[PAN_MOTOR]->dxl_item_->address, dynamixel_[PAN_MOTOR]->dxl_item_->data_length, &read_value);
  read_data->push_back(read_value);

  readDynamixelRegister(dynamixel_[TILT_MOTOR]->id_, dynamixel_[TILT_MOTOR]->dxl_item_->address, dynamixel_[TILT_MOTOR]->dxl_item_->data_length, &read_value);
  read_data->push_back(read_value);

  read_data_[addr_name] = read_data;
  return true;
}

bool DynamixelWorkbenchTorqueControl::getPublishedMsg(void)
{
  readMotorState("torque_enable");
  readMotorState("moving");
  readMotorState("goal_position");
  readMotorState("goal_velocity");
  readMotorState("goal_current");
  readMotorState("profile_velocity");
  readMotorState("profile_acceleration");
  readMotorState("present_position");
  readMotorState("present_velocity");
  readMotorState("present_current");
  readMotorState("min_position_limit");
  readMotorState("max_position_limit");
}

int64_t DynamixelWorkbenchTorqueControl::convertRadian2Value(double radian)
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

double DynamixelWorkbenchTorqueControl::convertValue2Radian(int32_t value)
{
  double radian = 0.0;
  if (value > dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_)
  {
    if (dynamixel_[PAN_TILT_MOTOR]->max_radian_ <= 0)
      return dynamixel_[PAN_TILT_MOTOR]->max_radian_;

    radian = (double) (value - dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_) * dynamixel_[PAN_TILT_MOTOR]->max_radian_
               / (double) (dynamixel_[PAN_TILT_MOTOR]->value_of_max_radian_position_ - dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_);
  }
  else if (value < dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_)
  {
    if (dynamixel_[PAN_TILT_MOTOR]->min_radian_ >= 0)
      return dynamixel_[PAN_TILT_MOTOR]->min_radian_;

    radian = (double) (value - dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_) * dynamixel_[PAN_TILT_MOTOR]->min_radian_
               / (double) (dynamixel_[PAN_TILT_MOTOR]->value_of_min_radian_position_ - dynamixel_[PAN_TILT_MOTOR]->value_of_0_radian_position_);
  }

  if (radian > dynamixel_[PAN_TILT_MOTOR]->max_radian_)
    return dynamixel_[PAN_TILT_MOTOR]->max_radian_;
  else if (radian < dynamixel_[PAN_TILT_MOTOR]->min_radian_)
    return dynamixel_[PAN_TILT_MOTOR]->min_radian_;

  return radian;
}

int16_t DynamixelWorkbenchTorqueControl::convertTorque2Value(double torque)
{
  return (int16_t) (torque * dynamixel_[PAN_TILT_MOTOR]->torque_to_current_value_ratio_);
}

bool DynamixelWorkbenchTorqueControl::dynamixelControlLoop(void)
{
  getPublishedMsg();

  dynamixel_workbench_msgs::MotorState dxl_response[dynamixel_.size()];
  dynamixel_workbench_msgs::MotorStateList dxl_response_list;

  for (int i = 0; i < dynamixel_.size(); i++)
  {
    dxl_response[i].motor_model = dynamixel_[i]->model_name_;
    dxl_response[i].id = dynamixel_[i]->id_;
    dxl_response[i].torque_enable = read_data_[i, "torque_enable"]->at(i);
    dxl_response[i].moving = read_data_["moving"]->at(i);
    dxl_response[i].goal_position = read_data_["goal_position"]->at(i);
    dxl_response[i].goal_velocity = read_data_["goal_velocity"]->at(i);
    dxl_response[i].goal_current = read_data_["goal_current"]->at(i);
    dxl_response[i].profile_velocity = read_data_["profile_velocity"]->at(i);
    dxl_response[i].profile_acceleration = read_data_["profile_acceleration"]->at(i);
    dxl_response[i].present_position = read_data_["present_position"]->at(i);
    dxl_response[i].present_velocity = read_data_["present_velocity"]->at(i);
    dxl_response[i].present_current = read_data_["present_current"]->at(i);
    dxl_response[i].max_position_limit = read_data_["max_position_limit"]->at(i);
    dxl_response[i].min_position_limit = read_data_["min_position_limit"]->at(i);

    dxl_response_list.motor_states.push_back(dxl_response[i]);
  }
  dxl_state_pub_.publish(dxl_response_list);

  pan_cur_pos_ = read_data_["present_position"]->at(PAN_MOTOR);
  tilt_cur_pos_ = read_data_["present_position"]->at(TILT_MOTOR);

  pan_torque_ = PROPORTION_GAIN * (pan_des_pos_ - pan_cur_pos_) + DIFFERENTIAL_GAIN * ((pan_pre_pos_ - pan_cur_pos_) / 0.004);
  tilt_torque_ = PROPORTION_GAIN * (tilt_des_pos_ - tilt_cur_pos_) + DIFFERENTIAL_GAIN * ((tilt_pre_pos_ - tilt_cur_pos_) / 0.004)
                 + TILT_MOTOR_MASS * GRAVITY * LINK_LENGTH * cos(convertValue2Radian(tilt_cur_pos_));

  writeCurrent(convertTorque2Value(pan_torque_), convertTorque2Value(tilt_torque_));

  pan_pre_pos_ = read_data_["present_position"]->at(PAN_MOTOR);
  tilt_pre_pos_ = read_data_["present_position"]->at(TILT_MOTOR);
}

bool DynamixelWorkbenchTorqueControl::controlPanTiltMotorCallback(dynamixel_workbench_msgs::SetPosition::Request &req,
                                                                    dynamixel_workbench_msgs::SetPosition::Response &res)
{
  pan_des_pos_ = convertRadian2Value(req.pan_pos);
  tilt_des_pos_ = convertRadian2Value(req.tilt_pos);

  res.pan_pos = pan_des_pos_;
  res.tilt_pos = tilt_des_pos_;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_torque_control");
  DynamixelWorkbenchTorqueControl dxl_torque_ctrl;
  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    dxl_torque_ctrl.dynamixelControlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
