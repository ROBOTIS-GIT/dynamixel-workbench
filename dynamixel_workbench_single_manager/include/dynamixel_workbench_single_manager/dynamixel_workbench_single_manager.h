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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H
#define DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H

#include <unistd.h>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>

#include <dynamixel_workbench_msgs/DynamixelAX.h>
#include <dynamixel_workbench_msgs/DynamixelRX.h>
#include <dynamixel_workbench_msgs/DynamixelMX.h>
#include <dynamixel_workbench_msgs/DynamixelMX64.h>
#include <dynamixel_workbench_msgs/DynamixelMX106.h>
#include <dynamixel_workbench_msgs/DynamixelEX.h>
#include <dynamixel_workbench_msgs/DynamixelXL.h>
#include <dynamixel_workbench_msgs/DynamixelXM.h>
#include <dynamixel_workbench_msgs/DynamixelXH.h>
#include <dynamixel_workbench_msgs/DynamixelPro.h>
#include <dynamixel_workbench_msgs/DynamixelProL42.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_workbench_msgs/WorkbenchParam.h>
#include <dynamixel_workbench_msgs/GetWorkbenchParam.h>

#include <dynamixel_workbench_toolbox/dynamixel_tool.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

#define ESC_ASCII_VALUE             0x1b
#define SPACEBAR_ASCII_VALUE        0x20

namespace dynamixel_workbench_single_manager
{
class DynamixelWorkbenchSingleManager
{
 public:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  dynamixel::PacketHandler *packetHandler1_;
  dynamixel::PacketHandler *packetHandler2_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dynamixel_state_pub_;
  // ROS Topic Subscriber
  ros::Subscriber dynamixel_command_sub_;
  // ROS Server
  ros::ServiceServer workbench_param_server_;
  // Parameters
  std::string device_name_;
  //uint64_t baud_rate_list_[BAUD_RATE_NUM];
  int baud_rate_;
  float protocol_version_;
  uint16_t dynamixel_model_number_;
  uint8_t dynamixel_model_id_;
  bool dynamixel_torque_status_;
  dynamixel_tool::DynamixelTool *dynamixel_;
  int64_t read_value_;

 public:
  DynamixelWorkbenchSingleManager();
  ~DynamixelWorkbenchSingleManager();
  void viewManagerMenu(void);
  bool dynamixelSingleManagerLoop(void);

 private:
  bool initDynamixelWorkbenchSingleManager(void);
  bool shutdownDynamixelWorkbenchSingleManager(void);

  int getch(void);
  int kbhit(void);

  bool scanDynamixelID();
  void showControlTable(void);
  bool rebootDynamixel();
  bool resetDynamixel();
  void checkValidationCommand(bool *valid_cmd, char *cmd);

  void setPublisher(void);
  void setSubscriber(void);
  void setPublishedMsg(void);
  void setServer(void);

  void dynamixelCommandMsgCallback(const dynamixel_workbench_msgs::DynamixelCommand::ConstPtr &msg);
  bool getWorkbenchParamCallback(dynamixel_workbench_msgs::GetWorkbenchParam::Request &req, dynamixel_workbench_msgs::GetWorkbenchParam::Response &res);

  bool writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t value);
  bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value);

  void axMotorMessage(void);
  void rxMotorMessage(void);
  void mxMotorMessage(void);
  void mx64MotorMessage(void);
  void mx106MotorMessage(void);
  void exMotorMessage(void);
  void xlMotorMessage(void);
  void xmMotorMessage(void);
  void xhMotorMessage(void);
  void proMotorMessage(void);
  void proL42MotorMessage(void);
};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H
