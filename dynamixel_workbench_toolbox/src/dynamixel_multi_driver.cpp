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

#include "dynamixel_workbench_toolbox/dynamixel_multi_driver.h"

using namespace dynamixel_multi_driver;

DynamixelMultiDriver::DynamixelMultiDriver(std::string device_name, int baud_rate, float protocol_version)
  :dynamixel_driver::DynamixelDriver(device_name, baud_rate, protocol_version)
{
  portHandler_   = dynamixel_driver::DynamixelDriver::portHandler_;//dynamixel_driver::DynamixelDriver::getPortHandler();
  packetHandler_ = dynamixel_driver::DynamixelDriver::packetHandler_; //dynamixel_driver::DynamixelDriver::getPacketHandler();
}

DynamixelMultiDriver::~DynamixelMultiDriver()
{

}

bool DynamixelMultiDriver::loadDynamixel(std::vector<dynamixel_driver::DynamixelInfo*> dynamixel_info)
{
  uint8_t error      = 0;
  uint16_t model_num = 0;

  for (std::vector<dynamixel_driver::DynamixelInfo>::size_type num = 0; num < dynamixel_info.size(); ++num)
  {
    //ROS_INFO("ID : %u", dynamixel_info[num]->model_id);
    if (packetHandler_->ping(portHandler_, dynamixel_info[num]->model_id, &model_num, &error) == COMM_SUCCESS)
    {
      dynamixel_tool::DynamixelTool *dynamixel = new dynamixel_tool::DynamixelTool(dynamixel_info[num]->model_id, model_num);
      multi_dynamixel_.push_back(dynamixel);
      // ROS_INFO("IsD : %u", multi_dynamixel_[num]->id_);
    }
//    else
//    {
//      ROS_WARN("aA");
//    }
//    return true;
  }

//  return false;
}

bool DynamixelMultiDriver::initSyncWrite()
{
  dynamixel_tool::DynamixelTool *dynamixel = multi_dynamixel_[0];

  dynamixel->item_ = dynamixel->ctrl_table_["goal_position"];
  dynamixel_tool::ControlTableItem *addr_item = dynamixel->item_;

  groupSyncWritePosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  dynamixel->item_ = dynamixel->ctrl_table_["goal_velocity"];
  addr_item = dynamixel->item_;

  groupSyncWriteVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  dynamixel->item_ = dynamixel->ctrl_table_["goal_current"];
  addr_item = dynamixel->item_;

  groupSyncWriteCurrent_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  dynamixel->item_ = dynamixel->ctrl_table_["torque_enable"];
  addr_item = dynamixel->item_;

  groupSyncWriteTorque_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  dynamixel->item_ = dynamixel->ctrl_table_["profile_velocity"];
  addr_item = dynamixel->item_;

  groupSyncWriteProfileVelocity_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);

  dynamixel->item_ = dynamixel->ctrl_table_["profile_acceleration"];
  addr_item = dynamixel->item_;

  groupSyncWriteProfileAcceleration_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, addr_item->address, addr_item->data_length);
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

bool DynamixelMultiDriver::syncWriteTorque(std::vector<uint8_t> &onoff)
{
  bool dynamixel_addparam_result_;
  int8_t dynamixel_comm_result_;

  uint8_t param_goal_position[4];

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_dynamixel_.size(); ++num)
  {
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(onoff[num]));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(onoff[num]));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(onoff[num]));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(onoff[num]));

    dynamixel_addparam_result_ = groupSyncWriteTorque_->addParam(multi_dynamixel_[num]->id_, (uint8_t*)&param_goal_position);
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
