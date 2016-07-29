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
#include "dynamixel_workbench_monitor/MonitorCommand.h"

// Control table address (Dynamixel XM series)
#define ADDR_XM_GET_ID              7
#define ADDR_XM_OPERATIING_MODE     11
#define ADDR_XM_TORQUE_ENABLE       64
#define ADDR_XM_PROFILE_VELOCITY    112
#define ADDR_XM_GOAL_VELOCITY       104
#define ADDR_XM_GOAL_POSITION       116
#define ADDR_XM_REALTIME_TICK       120
#define ADDR_XM_PRESENT_VELOCITY    128
#define ADDR_XM_PRESENT_POSITION    132
#define ADDR_XM_PRESENT_VOLTAGE     144
#define ADDR_XM_PRESENT_TEMPERATURE 146
#define ADDR_XM_MOVING              122

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
  // ROS Service server
  ros::ServiceServer dynamixel_workbench_monitor_server_;
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
  void dynamixelMonitorLoop(void);
  void closeDynamixel(void);

 private:
  int getch(void);
  int kbhit(void);
  bool initDynamixelController(void);
  bool shutdownDynamixelWorkbenchMonitor(void);
  uint8_t scanDynamixelId(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler);
  bool writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t value);
  bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t *value);
  bool setTorque(uint8_t id, bool onoff);
  // Read various state of Dynamixel
  bool readOperatingMode(uint8_t id, int8_t *operating_mode);
  bool readTorque(uint8_t id, int8_t *torque);
  bool readPresentPosition(uint8_t id, int32_t *position);
  bool readRealtimeTick(uint8_t id, int16_t *realtime_tick);
  bool readGoalPosition(uint8_t id, int32_t *goal_position);
  bool readPresentVelocity(uint8_t id, int32_t *velocity);
  bool readGoalVelocity(uint8_t id, int32_t *velocity);
  bool readPresentVoltage(uint8_t id, int16_t *voltage);
  bool readPresentTemperature(uint8_t id, int8_t *temperature);
  bool readIsMoving(uint8_t id, int8_t *is_moving);
  // ROS Service Server
  bool dynamixelCommandServer(dynamixel_workbench_monitor::MonitorCommand::Request &req,
                              dynamixel_workbench_monitor::MonitorCommand::Response &res);
};
}

#endif // DYNAMIXEL_WORKBENCH_MONITOR_H_
