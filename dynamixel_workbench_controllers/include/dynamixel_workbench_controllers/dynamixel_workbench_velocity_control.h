#ifndef DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
#define DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_workbench_msgs/MotorStateList.h>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK Library

// Default setting
#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"
#define PROTOCOL_VERSION            2.0

namespace dynamixel_workbench_velocity_control
{
struct ReadData{
 std::vector<bool> dxl_bool_data;
 std::vector<uint16_t> dxl_int_data;
};

class DynamixelWorkbenchVelocityControl
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
  dynamixel_tool::DynamixelTool *dxl_pan_motor_;
  dynamixel_tool::DynamixelTool *dxl_tilt_motor_;
  std::vector<dynamixel_tool::DynamixelTool *> dynamixel_;
  std::map<std::string, ReadData *> read_data_;
  std::string device_name_;
  std::string motor_model_;
  int motor_id_;
  float protocol_version_;
  float baud_rate_;
  uint32_t value_;

 public:
  DynamixelWorkbenchVelocityControl();
  ~DynamixelWorkbenchVelocityControl();
  bool dynamixelControlLoop(void);

 private:
  bool initDynamixelWorkbenchVelocityControl(void);
  bool shutdownDynamixelWorkbenchVelocityControl(void);
  bool initMotor(dynamixel_tool::DynamixelTool *dxl_motor, std::string motor_model, uint8_t motor_id, float protocol_version);
  bool readDynamixelRegister(dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, uint32_t *value);
  bool readTorque(void);
  bool readMoving(void);
  bool readGoalPosition(void);
  bool readGoalVelocity(void);
  bool readPresentPosition(void);
  bool readPresentVelocity(void);
};
}

#endif //DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
