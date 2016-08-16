#include "dynamixel_workbench_controllers/dynamixel_workbench_position_control.h"

using namespace dynamixel_workbench_position_control;

DynamixelWorkbenchPositionControl::DynamixelWorkbenchPositionControl()
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
     dxl_tilt_motor_(NULL),
     dxl_pan_motor_(NULL)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name_", device_name_);
  nh_priv_.getParam("baud_rate_", baud_rate_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchPositionControl());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1.0 PacketHandler or Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // init ROS Publish
  dxl_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::MotorStateList>("/motor_state",10);

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

  initMotor(dxl_pan_motor_, motor_model_, motor_id_, protocol_version_);

  nh_priv_.getParam("tilt_motor_/motor_model_", motor_model_);
  nh_priv_.getParam("tilt_motor_/motor_id_", motor_id_);
  nh_priv_.getParam("tilt_motor_/protocol_version_", protocol_version_);
  ROS_INFO("tilt_motor_id: %d", motor_id_);
  ROS_INFO("tilt_motor_model: %s", motor_model_.c_str());
  ROS_INFO("tilt_motor_protocol_version_: %.1f", protocol_version_);

  initMotor(dxl_tilt_motor_, motor_model_, motor_id_, protocol_version_);
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
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

bool DynamixelWorkbenchPositionControl::initMotor(dxl_motor::DxlMotor *dxl_motor, std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dxl_motor = new dxl_motor::DxlMotor(motor_id, motor_model, protocol_version);
  dynamixel_.push_back(dxl_motor);
}

bool DynamixelWorkbenchPositionControl::readTorque(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<bool> *dxl_read_data)
{
  dynamixel[0]->dxl_item_ = dynamixel[0]->ctrl_table_["torque_enable"];
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, dynamixel[0]->dxl_item_->address, dynamixel_[0]->dxl_item_->data_length);

  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dynamixel[i]->id_);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dynamixel[i]->id_);
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
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dynamixel[i]->id_);
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchPositionControl::readMoving(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<bool> *dxl_read_data)
{
  dynamixel[0]->dxl_item_ = dynamixel[0]->ctrl_table_["moving"];
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, dynamixel[0]->dxl_item_->address, dynamixel_[0]->dxl_item_->data_length);

  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dynamixel[i]->id_);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dynamixel[i]->id_);
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
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dynamixel[i]->id_);
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchPositionControl::readPresentPosition(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data)
{
  dynamixel[0]->dxl_item_ = dynamixel[0]->ctrl_table_["present_position"];
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, dynamixel[0]->dxl_item_->address, dynamixel_[0]->dxl_item_->data_length);

  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dynamixel[i]->id_);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dynamixel[i]->id_);
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
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dynamixel[i]->id_);
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchPositionControl::readPresentVelocity(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data)
{
  dynamixel[0]->dxl_item_ = dynamixel[0]->ctrl_table_["present_velocity"];
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, dynamixel[0]->dxl_item_->address, dynamixel_[0]->dxl_item_->data_length);

  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dynamixel[i]->id_);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dynamixel[i]->id_);
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
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dynamixel[i]->id_);
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchPositionControl::readGoalPosition(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data)
{
  dynamixel[0]->dxl_item_ = dynamixel[0]->ctrl_table_["goal_position"];
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, dynamixel[0]->dxl_item_->address, dynamixel_[0]->dxl_item_->data_length);

  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dynamixel[i]->id_);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dynamixel[i]->id_);
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
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dynamixel[i]->id_);
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchPositionControl::readGoalVelocity(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data)
{
  dynamixel[0]->dxl_item_ = dynamixel[0]->ctrl_table_["goal_velocity"];
  dynamixel::GroupSyncRead groupSyncRead(portHandler_, packetHandler_, dynamixel[0]->dxl_item_->address, dynamixel_[0]->dxl_item_->data_length);

  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_addparam_result_ = groupSyncRead.addParam(dynamixel[i]->id_);
    if (dxl_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncRead addparam failed", dynamixel[i]->id_);
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
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_getdata_result_ = groupSyncRead.isAvailable(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length);
    if (dxl_getdata_result_ != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", dynamixel[i]->id_);
      return false;
    }
  }

  dxl_read_data->clear();

  // Get present position value
  for (int i = 0; i < dynamixel.size(); i++)
  {
    dxl_read_data->push_back(groupSyncRead.getData(dynamixel[i]->id_, dynamixel[0]->dxl_item_->address, dynamixel[0]->dxl_item_->data_length));
  }

  groupSyncRead.clearParam();
  return true;
}

bool DynamixelWorkbenchPositionControl::dynamixelControlLoop(void)
{
  dynamixel_workbench_msgs::MotorState dxl_response[dynamixel_.size()];
  dynamixel_workbench_msgs::MotorStateList dxl_response_list;

  readTorque(dynamixel_, &dxl_torque_read_data_);
  readPresentPosition(dynamixel_, &dxl_present_position_read_data_);
  readPresentVelocity(dynamixel_, &dxl_present_velocity_read_data_);
  readGoalPosition(dynamixel_, &dxl_goal_position_read_data_);
  readGoalVelocity(dynamixel_, &dxl_goal_velocity_read_data_);
  readMoving(dynamixel_, &dxl_moving_read_data_);

  for (int i = 0; i < dynamixel_.size(); i++)
  {
    dxl_response[i].id = dynamixel_[i]->id_;
    dxl_response[i].torque_enable = dxl_torque_read_data_.at(i);
    dxl_response[i].moving = dxl_moving_read_data_.at(i);
    dxl_response[i].present_position = dxl_present_position_read_data_.at(i);
    dxl_response[i].present_velocity = dxl_present_velocity_read_data_.at(i);
    dxl_response[i].goal_position = dxl_goal_position_read_data_.at(i);
    dxl_response[i].goal_velocity = dxl_goal_velocity_read_data_.at(i);
  }

  for (int i = 0; i < dynamixel_.size(); i++)
  {
    dxl_response_list.motor_states.push_back(dxl_response[i]);
  }

  dxl_state_pub_.publish(dxl_response_list);

//  +  dynamixel_workbench_msgs::DynamixelResponse response[dxl_id_vec_.size()];
//   +  dynamixel_workbench_msgs::DynamixelResponseList response_list;
//   +
//   +  dynamixel_tool_->readRealtimeTick(dxl_id_vec_, &dxl_realtime_tick_read_data_);
//   +  dynamixel_tool_->readOperatingMode(dxl_id_vec_, &dxl_operating_mode_read_data_);
//   +  dynamixel_tool_->readTorque(dxl_id_vec_, &dxl_torque_read_data_);
//   +  dynamixel_tool_->readGoalPosition(dxl_id_vec_, &dxl_goal_position_read_data_);
//   +  dynamixel_tool_->readPresentPosition(dxl_id_vec_, &dxl_present_position_read_data_);
//   +  dynamixel_tool_->readProfileVelocity(dxl_id_vec_, &dxl_profile_velocity_read_data_);
//   +  dynamixel_tool_->readPresentVelocity(dxl_id_vec_, &dxl_present_velocity_read_data_);
//   +  dynamixel_tool_->readVoltage(dxl_id_vec_, &dxl_voltage_);
//   +  dynamixel_tool_->readTemperature(dxl_id_vec_, &dxl_temperature_);
//   +  dynamixel_tool_->readIsMoving(dxl_id_vec_, &dxl_is_moving_);
//   +
//   +  for (int i = 0; i < dxl_id_vec_.size(); i++)
//   +  {
//   +    response[i].id = dxl_id_vec_.at(i);
//   +    response[i].timestamps = dxl_realtime_tick_read_data_.at(i);
//   +    response[i].operating_mode = dxl_operating_mode_read_data_.at(i);
//   +    response[i].torque = dxl_torque_read_data_.at(i);
//   +    response[i].goal_position = dxl_goal_position_read_data_.at(i);
//   +    response[i].present_position = dxl_present_position_read_data_.at(i);
//   +    response[i].profile_velocity = dxl_profile_velocity_read_data_.at(i);
//   +    response[i].present_velocity = dxl_present_velocity_read_data_.at(i);
//   +    response[i].voltage = dxl_voltage_.at(i);
//   +    response[i].temperature = dxl_temperature_.at(i);
//   +    response[i].is_moving = dxl_is_moving_.at(i);
//   +  }
//   +
//   +  for(int i = 0; i < dxl_id_vec_.size(); i++)
//   +  {
//   +    response_list.dynamixel_responses.push_back(response[i]);
//   +  }
//   +  dxl_position_pub_.publish(response_list);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_position_control");
  DynamixelWorkbenchPositionControl dxl_pos_ctrl;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    dxl_pos_ctrl.dynamixelControlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
