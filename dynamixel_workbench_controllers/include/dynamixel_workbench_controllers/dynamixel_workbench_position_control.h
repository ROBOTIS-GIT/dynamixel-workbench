#ifndef DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
#define DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_tool/dxl_motor.h>
#include <dynamixel_workbench_msgs/MotorStateList.h>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK Library

// Default setting
#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"
#define PROTOCOL_VERSION            2.0

namespace dynamixel_workbench_position_control
{
class DynamixelWorkbenchPositionControl
{
 public:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dxl_state_pub_;
  // Parameters
  bool dxl_addparam_result_;
  int dxl_comm_result_;
  bool dxl_getdata_result_;
  dxl_motor::DxlMotor *dxl_pan_motor_;
  dxl_motor::DxlMotor *dxl_tilt_motor_;
  std::vector<dxl_motor::DxlMotor *> dynamixel_;
  std::vector<bool> dxl_torque_read_data_;
  std::vector<bool> dxl_moving_read_data_;
  std::vector<uint16_t> dxl_goal_position_read_data_;
  std::vector<uint16_t> dxl_goal_velocity_read_data_;
  std::vector<uint16_t> dxl_present_position_read_data_;
  std::vector<uint16_t> dxl_present_velocity_read_data_;
  std::string device_name_;
  std::string motor_model_;
  int motor_id_;
  float protocol_version_;
  float baud_rate_;

 public:
  DynamixelWorkbenchPositionControl();
  ~DynamixelWorkbenchPositionControl();
  bool dynamixelControlLoop(void);

 private:
  bool initDynamixelWorkbenchPositionControl(void);
  bool shutdownDynamixelWorkbenchPositionControl(void);
  bool initMotor(dxl_motor::DxlMotor *dxl_motor, std::string motor_model, uint8_t motor_id, float protocol_version);
  bool readTorque(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<bool> *dxl_read_data);
  bool readMoving(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<bool> *dxl_read_data);
  bool readGoalPosition(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data);
  bool readGoalVelocity(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data);
  bool readPresentPosition(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data);
  bool readPresentVelocity(std::vector<dxl_motor::DxlMotor *> dynamixel, std::vector<uint16_t> *dxl_read_data);
};
}

#endif //DYNAMIXEL_WORKBENCH_POSITION_CONTROL_H
