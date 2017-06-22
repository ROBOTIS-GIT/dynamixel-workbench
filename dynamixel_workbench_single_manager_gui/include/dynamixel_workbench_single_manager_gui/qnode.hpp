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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_QNODE_HPP
#define DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_QNODE_HPP

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include "dynamixel_workbench_toolbox/message_header.h"

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "dynamixel_workbench_msgs/DynamixelInfo.h"

#endif

namespace qnode
{
class QNode : public QThread
{
 Q_OBJECT

 public:
  QNode(int argc, char** argv );
  virtual ~QNode();

  bool init();
  void run();

  QStringListModel* loggingModel() { return &logging_model_; }
  void log(const std::string &msg, int64_t sender);
  void log(const std::string &msg);

  // TODO : Add new Dynamixel
  void AXStatusMsgCallback(const dynamixel_workbench_msgs::AX::ConstPtr &msg);
  void RXStatusMsgCallback(const dynamixel_workbench_msgs::RX::ConstPtr &msg);
  void MXStatusMsgCallback(const dynamixel_workbench_msgs::MX::ConstPtr &msg);
//  void dynamixelEXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelEX::ConstPtr &msg);
//  void dynamixelXL320StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXL320::ConstPtr &msg);
//  void dynamixelXLStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXL::ConstPtr &msg);
  void XMStatusMsgCallback(const dynamixel_workbench_msgs::XM::ConstPtr &msg);
//  void dynamixelXHStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXH::ConstPtr &msg);
//  void dynamixelProStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelPro::ConstPtr &msg);
//  void dynamixelProL42StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelProL42::ConstPtr &msg);

  bool sendSetIdMsg(uint8_t set_id);
  bool sendSetOperatingModeMsg(std::string index, float protocol_version, std::string model_name, int32_t value_of_max_radian_position);
  bool sendSetBaudrateMsg(int64_t baud_rate);

  bool sendTorqueMsg(int64_t onoff);
  bool sendRebootMsg(void);
  bool sendResetMsg(void);
  bool sendAddressValueMsg(std::string addr_name, int64_t value);
  bool setPositionZeroMsg(int32_t zero_position);

  void getDynamixelInfo();
  // TODO : Add new Dynamixel
  void initDynamixelStateSubscriber();
  bool sendCommandMsg(std::string cmd, std::string addr = "", int64_t value = 0);

 Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void updateDynamixelInfo(dynamixel_workbench_msgs::DynamixelInfo);

 private:
  int init_argc;
  char** init_argv;

  QStringListModel logging_model_;

  // ROS Topic Publisher

  // ROS Topic Subscriber
  ros::Subscriber dynamixel_status_msg_sub_;

  // ROS Service Server

  // ROS Service Client
  ros::ServiceClient dynamixel_info_client_;
  ros::ServiceClient dynamixel_command_client_;

  // Single Manager GUI variable
  int64_t row_count_;

  dynamixel_workbench_msgs::DynamixelInfo dynamixel_info_;

};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_QNODE_HPP
