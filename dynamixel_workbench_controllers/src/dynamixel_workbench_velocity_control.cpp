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

/*
 *
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *     |-----------|
 *    O-------------O
 *  left          right
 *   1              2
 *
 *
 * */

#include "dynamixel_workbench_controllers/dynamixel_workbench_velocity_control.h"

using namespace dynamixel_workbench_velocity_control;

DynamixelWorkbenchVelocityControl::DynamixelWorkbenchVelocityControl()
    :nh_priv_("~"),
     is_debug_(false),
     device_name_(""),
     baud_rate_(0),
     motor_model_(""),
     motor_id_(0),
     protocol_version_(0),
     read_value_(0),
     right_motor_velocity_(0.0),
     left_motor_velocity_(0.0)
{
  // Init parameter
  nh_.param("is_debug", is_debug_, is_debug_);
  nh_priv_.getParam("device_name", device_name_);
  nh_priv_.getParam("baud_rate", baud_rate_);
  nh_priv_.getParam("motor_model", motor_model_);
  nh_priv_.getParam("protocol_version", protocol_version_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchVelocityControl());

  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux
  portHandler_ = dynamixel::PortHandler::getPortHandler(device_name_.c_str());

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol 2.0 PacketHandler
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(protocol_version_);

  // init ROS Publish
  dynamixel_state_pub_ = nh_.advertise<dynamixel_workbench_msgs::MotorStateList>("/dynamixel_workbench_velocity_control/wheel_state",10);

  // init ROS Server
  wheel_control_server_ = nh_.advertiseService("/dynamixel_workbench_tutorials/wheel", &DynamixelWorkbenchVelocityControl::controlWheelVelocityCallback, this);

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

  nh_priv_.getParam("left_motor/motor_id", motor_id_);
  ROS_INFO("left_motor_id: %d", motor_id_);
  ROS_INFO("left_motor_model: %s", motor_model_.c_str());
  ROS_INFO("left_motor_protocol_version_: %.1f\n", protocol_version_);

  initMotor(motor_model_, motor_id_, protocol_version_);

  nh_priv_.getParam("right_motor/motor_id", motor_id_);
  ROS_INFO("right_motor_id: %d", motor_id_);
  ROS_INFO("right_motor_model: %s", motor_model_.c_str());
  ROS_INFO("right_motor_protocol_version_: %.1f", protocol_version_);

  initMotor(motor_model_, motor_id_, protocol_version_);
  writeTorque(true);
}

DynamixelWorkbenchVelocityControl::~DynamixelWorkbenchVelocityControl()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchVelocityControl());
}

bool DynamixelWorkbenchVelocityControl::initDynamixelWorkbenchVelocityControl(void)
{
  ROS_INFO("dynamixel_workbench_velocity_control : Init OK!");
  return true;
}

bool DynamixelWorkbenchVelocityControl::shutdownDynamixelWorkbenchVelocityControl(void)
{
  writeTorque(false);
  portHandler_->closePort();
  ros::shutdown();
  return true;
}

bool DynamixelWorkbenchVelocityControl::initMotor(std::string motor_model, uint8_t motor_id, float protocol_version)
{
  dynamixel_tool::DynamixelTool *dynamixel_motor = new dynamixel_tool::DynamixelTool(motor_id, motor_model, protocol_version);
  dynamixel_.push_back(dynamixel_motor);
}

bool DynamixelWorkbenchVelocityControl::writeTorque(bool onoff)
{
  dynamixel_[LEFT_RIGHT_WHEEL]->item_ = dynamixel_[LEFT_RIGHT_WHEEL]->ctrl_table_["torque_enable"];
  if (onoff == true)
  {
    syncWriteDynamixels(dynamixel_[LEFT_RIGHT_WHEEL]->item_->address, dynamixel_[LEFT_RIGHT_WHEEL]->item_->data_length, true, true);
  }
  else
  {
    syncWriteDynamixels(dynamixel_[LEFT_RIGHT_WHEEL]->item_->address, dynamixel_[LEFT_RIGHT_WHEEL]->item_->data_length, false, false);
  }
}

bool DynamixelWorkbenchVelocityControl::writeVelocity(int64_t left_wheel_velocity, int64_t right_wheel_velocity)
{
  if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
  {
      dynamixel_[LEFT_RIGHT_WHEEL]->item_ = dynamixel_[LEFT_RIGHT_WHEEL]->ctrl_table_["moving_speed"];
  }
  else
  {
      dynamixel_[LEFT_RIGHT_WHEEL]->item_ = dynamixel_[LEFT_RIGHT_WHEEL]->ctrl_table_["goal_velocity"];
  }

  syncWriteDynamixels(dynamixel_[LEFT_RIGHT_WHEEL]->item_->address, dynamixel_[LEFT_RIGHT_WHEEL]->item_->data_length, left_wheel_velocity, right_wheel_velocity);
}

bool DynamixelWorkbenchVelocityControl::syncWriteDynamixels(uint16_t addr, uint8_t length, int64_t left_wheel_value, int64_t right_wheel_value)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, addr,length);

  dynamixel_addparam_result_ = groupSyncWrite.addParam(dynamixel_[LEFT_WHEEL]->id_, (uint8_t*)&left_wheel_value);
  if (dynamixel_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[LEFT_WHEEL]->id_);
    return false;
  }

  dynamixel_addparam_result_ = groupSyncWrite.addParam(dynamixel_[RIGHT_WHEEL]->id_, (uint8_t*)&right_wheel_value);
  if (dynamixel_addparam_result_ != true)
  {
    ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", dynamixel_[RIGHT_WHEEL]->id_);
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

bool DynamixelWorkbenchVelocityControl::readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value)
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

bool DynamixelWorkbenchVelocityControl::readMotorState(std::string addr_name)
{
  std::vector<int64_t> *read_data = new std::vector<int64_t>;

  dynamixel_[LEFT_WHEEL]->item_ = dynamixel_[LEFT_WHEEL]->ctrl_table_[addr_name];
  dynamixel_[RIGHT_WHEEL]->item_ = dynamixel_[RIGHT_WHEEL]->ctrl_table_[addr_name];

  readDynamixelRegister(dynamixel_[LEFT_WHEEL]->id_, dynamixel_[LEFT_WHEEL]->item_->address, dynamixel_[LEFT_WHEEL]->item_->data_length, &read_value_);
  read_data->push_back(read_value_);

  readDynamixelRegister(dynamixel_[RIGHT_WHEEL]->id_, dynamixel_[RIGHT_WHEEL]->item_->address, dynamixel_[RIGHT_WHEEL]->item_->data_length, &read_value_);
  read_data->push_back(read_value_);

  read_data_[addr_name] = read_data;
  return true;
}

int64_t DynamixelWorkbenchVelocityControl::convertVelocity2Value(double velocity)
{
  return (int64_t) (velocity * dynamixel_[LEFT_RIGHT_WHEEL]->velocity_to_value_ratio_);
}

bool DynamixelWorkbenchVelocityControl::getPublishedMsg(void)
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

bool DynamixelWorkbenchVelocityControl::dynamixelControlLoop(void)
{
  getPublishedMsg();

  dynamixel_workbench_msgs::MotorState dynamixel_response[dynamixel_.size()];
  dynamixel_workbench_msgs::MotorStateList dynamixel_response_list;

  for (int i = 0; i < dynamixel_.size(); i++)
  {
    dynamixel_response[i].motor_model = dynamixel_[i]->model_name_;
    dynamixel_response[i].id = dynamixel_[i]->id_;
    dynamixel_response[i].torque_enable = read_data_["torque_enable"]->at(i);
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

bool DynamixelWorkbenchVelocityControl::controlWheelVelocityCallback(dynamixel_workbench_msgs::SetDirection::Request &req,
                                                                     dynamixel_workbench_msgs::SetDirection::Response &res)
{
  if (req.right_wheel_velocity == 0.0 || req.left_wheel_velocity == 0.0)
  {
    right_motor_velocity_ = 0.0;
    left_motor_velocity_ = 0.0;
  }
  else
  {
    right_motor_velocity_ += req.right_wheel_velocity;
    left_motor_velocity_  += req.left_wheel_velocity;
  }


  if (!strncmp(motor_model_.c_str(), "AX", 2) || !strncmp(motor_model_.c_str(), "RX", 2) || !strncmp(motor_model_.c_str(), "MX", 2) || !strncmp(motor_model_.c_str(), "EX", 2))
  {
    if (right_motor_velocity_ < 0.0 && left_motor_velocity_ < 0.0)
    {
      writeVelocity(convertVelocity2Value(left_motor_velocity_ * (-1)) + 1024, convertVelocity2Value(right_motor_velocity_) * (-1));
    }
    else if (right_motor_velocity_ < 0.0 && left_motor_velocity_ > 0.0)
    {
      writeVelocity(convertVelocity2Value(left_motor_velocity_), convertVelocity2Value(right_motor_velocity_) * (-1));
    }
    else if (right_motor_velocity_ > 0.0 && left_motor_velocity_ < 0.0)
    {
      writeVelocity(convertVelocity2Value(left_motor_velocity_ * (-1)) + 1024, convertVelocity2Value(right_motor_velocity_) + 1024);
    }
    else
    {
      writeVelocity(convertVelocity2Value(left_motor_velocity_), (convertVelocity2Value(right_motor_velocity_) + 1024));
    }
  }
  else
  {
     writeVelocity(convertVelocity2Value(left_motor_velocity_), convertVelocity2Value(right_motor_velocity_) * (-1));
  }

  res.right_wheel_velocity = right_motor_velocity_;
  res.left_wheel_velocity = left_motor_velocity_;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_velocity_control");
  DynamixelWorkbenchVelocityControl dynamixel_vel_ctrl;
  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    dynamixel_vel_ctrl.dynamixelControlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
