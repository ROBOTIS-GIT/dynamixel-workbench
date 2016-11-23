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

#ifndef DYNAMIXEL_WORKBENCH_MULTI_PORT_H
#define DYNAMIXEL_WORKBENCH_MULTI_PORT_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_workbench_msgs/MotorStateList.h>
#include <dynamixel_workbench_msgs/SetPosition.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

#define PAN_MOTOR      0
#define TILT_MOTOR     1

#define VELOCITY      100
#define ACCELERATION  20

namespace dynamixel_workbench_multi_port
{
class DynamixelWorkbenchMultiPort
{
 public:
  dynamixel::PortHandler *pan_motor_portHandler_;
  dynamixel::PortHandler *tilt_motor_portHandler_;
  dynamixel::PacketHandler *pan_motor_packetHandler_;
  dynamixel::PacketHandler *tilt_motor_packetHandler_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dynamixel_state_pub_;
  // ROS Service Server
  ros::ServiceServer multi_port_control_server;
  // Parameters
  std::vector<dynamixel_tool::DynamixelTool *> dynamixel_;

  std::string pan_motor_device_name_;
  std::string pan_motor_model_;
  int pan_motor_id_;
  float pan_motor_protocol_version_;
  int pan_motor_baud_rate_;

  std::string tilt_motor_device_name_;
  std::string tilt_motor_model_;
  int tilt_motor_id_;
  float tilt_motor_protocol_version_;
  int tilt_motor_baud_rate_;

  std::map<std::string, int64_t> read_pan_motor_data_;
  std::map<std::string, int64_t> read_tilt_motor_data_;

 public:
  DynamixelWorkbenchMultiPort();
  ~DynamixelWorkbenchMultiPort();
  bool dynamixelControlLoop(void);

 private:
  bool initDynamixelWorkbenchMultiPort(void);
  bool shutdownDynamixelWorkbenchMultiPort(void);

  bool initMotor(std::string motor_model, uint8_t motor_id, float protocol_version);

  bool readDynamixelRegister(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t *value);
  bool readMotorState(int8_t motor, std::string addr_name);

  bool writeDynamixelRegister(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t value);
  bool writeTorque(bool onoff);
  bool writeProfile();
  bool writePosition(int64_t pan_pos, int64_t tilt_pos);

  int64_t convertRadian2Value(int8_t motor, double radian);

  bool getPublishedMsg(void);
  bool controlPanTiltMotorCallback(dynamixel_workbench_msgs::SetPosition::Request &req,
                                   dynamixel_workbench_msgs::SetPosition::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_MULTI_PORT_H
