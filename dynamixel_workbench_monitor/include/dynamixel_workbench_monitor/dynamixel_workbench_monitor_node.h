#ifndef DYNAMIXEL_WORKBENCH_MONITOR_H_
#define DYNAMIXEL_WORKBENCH_MONITOR_H_

#include <unistd.h>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
//#include <geometry_msgs/Twist.h>
#include <dynamixel_workbench_msgs/DynamixelResponse.h>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK Library

// Control table address (Dynamixel XM series)
#define ADDR_XM_OPERATION_MODE      11
#define ADDR_XM_TORQUE_ENABLE       64
#define ADDR_XM_GOAL_VELOCITY       104
#define ADDR_XM_GOAL_POSITION       116
#define ADDR_XM_REALTIRM_TICK       120
#define ADDR_XM_PRESENT_VELOCITY    128
#define ADDR_XM_PRESENT_POSITION    132

// Protocol version
#define PROTOCOL_VERSION            2.0

// Default setting
#define DXL_ID                      1
#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"

#define TORQUE_CONTROL_MODE                 0
#define VELOCITY_CONTROL_MODE               1
#define POSITION_CONTROL_MODE               3       // Default
#define EXTENDED_POSITION_CONTROL_MODE      4
#define CURRENT_BASED_POSITION_CONTROL_MODE 5
#define PWM_CONTROL_MODE                    16

#define TORQUE_ENABLE                       1
#define TORQUE_DISABLE                      0

#define ESC_ASCII_VALUE                      0x1b

namespace dynamixel_workbench_monitor
{
class DynamixelWorkbenchMonitor
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
  ros::Publisher dxl_position_pub_;
  // Parameters
  std::string device_name_;
  uint8_t dxl_id_;
  float baud_rate_;
  float protocol_version_;

 public:
  DynamixelWorkbenchMonitor();
  ~DynamixelWorkbenchMonitor();
  void closeDynamixel(void);

 private:
  bool initDynamixelController(void);
  bool shutdownDynamixelWorkbenchMonitor(void);
  bool setTorque(uint8_t id, bool onoff);
};
}


#endif // DYNAMIXEL_WORKBENCH_MONITOR_H_
