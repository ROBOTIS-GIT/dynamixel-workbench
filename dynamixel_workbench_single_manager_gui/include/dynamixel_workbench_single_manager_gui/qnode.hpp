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

#include <dynamixel_workbench_msgs/DynamixelAX.h>
#include <dynamixel_workbench_msgs/DynamixelRX.h>
#include <dynamixel_workbench_msgs/DynamixelMX.h>
#include <dynamixel_workbench_msgs/DynamixelMX64.h>
#include <dynamixel_workbench_msgs/DynamixelMX106.h>
#include <dynamixel_workbench_msgs/DynamixelEX.h>
#include <dynamixel_workbench_msgs/DynamixelXL.h>
#include <dynamixel_workbench_msgs/DynamixelXM.h>
#include <dynamixel_workbench_msgs/DynamixelPro.h>
#include <dynamixel_workbench_msgs/DynamixelProL42.h>

#include "dynamixel_workbench_toolbox/dynamixel_tool.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/WorkbenchParam.h"
#include "dynamixel_workbench_msgs/GetWorkbenchParam.h"

#endif

namespace dynamixel_workbench_single_manager_gui
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

  void dynamixelAXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelAX::ConstPtr &msg);
  void dynamixelRXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelRX::ConstPtr &msg);
  void dynamixelMXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX::ConstPtr &msg);
  void dynamixelMX64StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX64::ConstPtr &msg);
  void dynamixelMX106StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX106::ConstPtr &msg);
  void dynamixelEXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelEX::ConstPtr &msg);
  void dynamixelXLStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXL::ConstPtr &msg);
  void dynamixelXMStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXM::ConstPtr &msg);
  void dynamixelProStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelPro::ConstPtr &msg);
  void dynamixelProL42StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelProL42::ConstPtr &msg);

  void sendTorqueMsg(std::string addr_name, int64_t onoff);
  void sendRebootMsg(void);
  void sendResetMsg(void);
  void sendSetIdMsg(int64_t id);
  void sendSetOperatingModeMsg(std::__cxx11::string index);
  void sendSetBaudrateMsg(float baud_rate);
  void sendControlTableValueMsg(QString table_item, int64_t value);
  void setPositionZeroMsg(int32_t zero_position);

  void getWorkbenchParam(void);
  void setSubscriber();

 Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void updateWorkbenchParam(dynamixel_workbench_msgs::WorkbenchParam);

 private:
  int init_argc;
  char** init_argv;

  QStringListModel logging_model_;

  ros::Publisher dynamixel_command_msg_pub_;
  ros::Publisher set_workbench_param_msg_pub_;
  ros::Subscriber dynamixel_status_msg_sub_;
  ros::ServiceClient get_workbench_param_client_;

  int64_t row_count_;

  std::string dynamixel_model_name_;
  uint16_t dynamixel_model_number_;

};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_QNODE_HPP
