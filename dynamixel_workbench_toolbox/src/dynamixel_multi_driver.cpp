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

#include "dynamixel_workbench_toolbox/dynamixel_multi_driver.h"

using namespace dynamixel_multi_driver;

DynamixelMultiDriver::DynamixelMultiDriver(std::string device_name, int baud_rate, float protocol_version)
  :dynamixel_driver::DynamixelDriver(device_name, baud_rate, protocol_version)
{
  portHandler_   = dynamixel_driver::DynamixelDriver::portHandler_;
  packetHandler_ = dynamixel_driver::DynamixelDriver::packetHandler_;
}

DynamixelMultiDriver::~DynamixelMultiDriver()
{

}

bool DynamixelMultiDriver::loadDynamixel(std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info)
{
  uint8_t error = 0;

  for (std::vector<dynamixel_driver::DynamixelInfo>::size_type num = 0; num < dynamixel_info.size(); ++num)
  {
    if (packetHandler_->ping(portHandler_, dynamixel_info[num]->model_id, &dynamixel_info[num]->model_number, &error) == COMM_SUCCESS)
    {
      dynamixel_tool::DynamixelTool *dynamixel = new dynamixel_tool::DynamixelTool(dynamixel_info[num]->model_id, dynamixel_info[num]->model_number);
      multi_dynamixel_.push_back(dynamixel);

      dynamixel_info[num]->model_name = dynamixel->model_name_;
    }
    else
    {
      return false;
    }
  }

  return true;
}

dynamixel::GroupSyncWrite* DynamixelMultiDriver::setSyncWrite(std::string addr_name)
{
  dynamixel_tool::DynamixelTool *dynamixel = multi_dynamixel_[0]; // TODO fix adress only dependent on first joint

  dynamixel->item_ = dynamixel->ctrl_table_[addr_name];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel->item_;

  dynamixel::GroupSyncWrite * groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  return groupSyncWrite;
}

dynamixel::GroupSyncRead* DynamixelMultiDriver::setSyncRead(std::string addr_name)
{
  dynamixel_tool::DynamixelTool *dynamixel = multi_dynamixel_[0];

  dynamixel->item_ = dynamixel->ctrl_table_[addr_name];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel->item_;

  dynamixel::GroupSyncRead* groupSyncRead = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  return groupSyncRead;
}

bool DynamixelMultiDriver::initSyncWrite()
{
  groupSyncWriteTorque_   = setSyncWrite("torque_enable");
  groupSyncWritePosition_ = setSyncWrite("goal_position");

  if (getProtocolVersion() == 2.0)
  {
    groupSyncWriteVelocity_ = setSyncWrite("goal_velocity");

    if (multi_dynamixel_[0]->model_name_.find("XM") != std::string::npos)
    {
      groupSyncWriteCurrent_ = setSyncWrite("goal_current");
    }

    if (!(multi_dynamixel_[0]->model_name_.find("PRO") != std::string::npos))
    {
      groupSyncWriteProfileVelocity_     = setSyncWrite("profile_velocity");
      groupSyncWriteProfileAcceleration_ = setSyncWrite("profile_acceleration");
    }
  }
  else
  {
    groupSyncWriteMovingSpeed_ = setSyncWrite("moving_speed");
  }

  return true;
}

bool DynamixelMultiDriver::initSyncRead()
{
  groupSyncReadPosition_ = setSyncRead("present_position");

  return true;
}

bool DynamixelMultiDriver::readMultiRegister(std::string addr_name)
{
  std::vector<int64_t> *read_data = new std::vector<int64_t>;
  int32_t value;

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dynamixel_= multi_dynamixel_[num];

    if (readRegister(addr_name, &value))
      read_data->push_back(value);
    else
      return false;
  }

  read_value_[addr_name] = read_data;

  return true;
}

bool DynamixelMultiDriver::syncWritePosition(std::vector<uint32_t> pos)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_position[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(pos[num]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(pos[num]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(pos[num]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(pos[num]));

    dynamixel_addparam_result_ = groupSyncWritePosition_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_position);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWritePosition_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWritePosition(const std::vector<double>& pos)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_position[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    uint32_t pos_ticks = multi_dynamixel_[num]->convertRadian2Value(pos[num]);
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(pos_ticks));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(pos_ticks));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(pos_ticks));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(pos_ticks));

    dynamixel_addparam_result_ = groupSyncWritePosition_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_position);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWritePosition_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWritePosition_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteVelocity(std::vector<int32_t> vel)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_velocity[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel[num]));

    dynamixel_addparam_result_ = groupSyncWriteVelocity_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_velocity);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteVelocity_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteVelocity_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteMovingSpeed(std::vector<uint16_t> spd)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_speed[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_speed[0] = DXL_LOBYTE(DXL_LOWORD(spd[num]));
    param_goal_speed[1] = DXL_HIBYTE(DXL_LOWORD(spd[num]));
    param_goal_speed[2] = DXL_LOBYTE(DXL_HIWORD(spd[num]));
    param_goal_speed[3] = DXL_HIBYTE(DXL_HIWORD(spd[num]));

    dynamixel_addparam_result_ = groupSyncWriteMovingSpeed_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_speed);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteMovingSpeed_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteMovingSpeed_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteCurrent(std::vector<int16_t> cur)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_current[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_current[0] = DXL_LOBYTE(DXL_LOWORD(cur[num]));
    param_goal_current[1] = DXL_HIBYTE(DXL_LOWORD(cur[num]));
    param_goal_current[2] = DXL_LOBYTE(DXL_HIWORD(cur[num]));
    param_goal_current[3] = DXL_HIBYTE(DXL_HIWORD(cur[num]));

    dynamixel_addparam_result_ = groupSyncWriteCurrent_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_current);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteCurrent_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteCurrent_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteTorque(std::vector<uint8_t> &onoff)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_torque[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_torque[0] = DXL_LOBYTE(DXL_LOWORD(onoff[num]));
    param_goal_torque[1] = DXL_HIBYTE(DXL_LOWORD(onoff[num]));
    param_goal_torque[2] = DXL_LOBYTE(DXL_HIWORD(onoff[num]));
    param_goal_torque[3] = DXL_HIBYTE(DXL_HIWORD(onoff[num]));

    dynamixel_addparam_result_ = groupSyncWriteTorque_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_torque);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteTorque_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteTorque_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncWriteProfileVelocity(std::vector<uint32_t> vel)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_profile_velocity[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_profile_velocity[0] = DXL_LOBYTE(DXL_LOWORD(vel[num]));
    param_goal_profile_velocity[1] = DXL_HIBYTE(DXL_LOWORD(vel[num]));
    param_goal_profile_velocity[2] = DXL_LOBYTE(DXL_HIWORD(vel[num]));
    param_goal_profile_velocity[3] = DXL_HIBYTE(DXL_HIWORD(vel[num]));

    dynamixel_addparam_result_ = groupSyncWriteProfileVelocity_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_velocity);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteProfileVelocity_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteProfileVelocity_->clearParam();
  return true;
}

 bool DynamixelMultiDriver::syncWriteProfileAcceleration(std::vector<uint32_t> acc)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_profile_acceleration[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_profile_acceleration[0] = DXL_LOBYTE(DXL_LOWORD(acc[num]));
    param_goal_profile_acceleration[1] = DXL_HIBYTE(DXL_LOWORD(acc[num]));
    param_goal_profile_acceleration[2] = DXL_LOBYTE(DXL_HIWORD(acc[num]));
    param_goal_profile_acceleration[3] = DXL_HIBYTE(DXL_HIWORD(acc[num]));

    dynamixel_addparam_result_ = groupSyncWriteProfileAcceleration_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_profile_acceleration);
    if (dynamixel_addparam_result_ != true)
    {
      ROS_ERROR("[ID:%03d] groupSyncWrite addparam failed", multi_dynamixel_[num]->id_);
      return false;
    }
  }

  dynamixel_comm_result_ = groupSyncWriteProfileAcceleration_->txPacket();
  if (dynamixel_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result_);
    return false;
  }
  groupSyncWriteProfileAcceleration_->clearParam();
  return true;
}

bool DynamixelMultiDriver::syncReadPosition(std::vector<uint32_t> &pos)
{
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint32_t position;

  dynamixel_= multi_dynamixel_[0];
  dynamixel_->item_ = dynamixel_->ctrl_table_["present_position"];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  pos.clear(); // TODO remove this

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_addparam_result = groupSyncReadPosition_->addParam(multi_dynamixel_[num]->id_);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncReadPosition_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_getdata_result = groupSyncReadPosition_->isAvailable(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);

    if (dxl_getdata_result)
    {
      position  = groupSyncReadPosition_->getData(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
      pos.push_back(position);
    }
    else
    {
      return false;
    }
  }

  groupSyncReadPosition_->clearParam();

  return true;
}

bool DynamixelMultiDriver::syncReadPosition(std::vector<double> &pos)
{
  int  dxl_comm_result = COMM_TX_FAIL;
  bool dxl_addparam_result = false;
  bool dxl_getdata_result = false;

  uint32_t position;

  dynamixel_= multi_dynamixel_[0];
  dynamixel_->item_ = dynamixel_->ctrl_table_["present_position"];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel_->item_;

  pos.clear(); // TODO remove this, wtf

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_addparam_result = groupSyncReadPosition_->addParam(multi_dynamixel_[num]->id_);
    if (dxl_addparam_result != true)
      return false;
  }

  dxl_comm_result = groupSyncReadPosition_->txRxPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    packetHandler_->printTxRxResult(dxl_comm_result);

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    dxl_getdata_result = groupSyncReadPosition_->isAvailable(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);

    if (dxl_getdata_result)
    {
      position  = groupSyncReadPosition_->getData(multi_dynamixel_[num]->id_, addr_item->address, addr_item->data_length);
      pos.push_back(multi_dynamixel_[num]->convertValue2Radian(position));
    }
    else
    {
      return false;
    }
  }

  groupSyncReadPosition_->clearParam();

  return true;
}
