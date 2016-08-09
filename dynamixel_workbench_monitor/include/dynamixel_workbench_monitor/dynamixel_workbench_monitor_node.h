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
#include <dynamixel_workbench_msgs/DynamixelResponseList.h>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK Library
#include "dynamixel_workbench_tool/dynamixel_position_control.h"
#include "dynamixel_workbench_monitor/MonitorCommand.h"

// Protocol version
#define PROTOCOL_VERSION            2.0

// Default setting
#define DXL_ID                      1
#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"
#define CONTROL_MODE                "position"

#define ESC_ASCII_VALUE             0x1b

namespace dynamixel_workbench_monitor
{
class DynamixelWorkbenchMonitor
{
 public:
  dynamixel_position_control::DynamixelPositionControl *dxl_position_control_tool_;
  std::vector<dxl_motor::DxlMotor *> dxl_control_;

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
  std::vector<dxl_motor::DxlMotor *> dxl_;
  std::vector<uint16_t> dxl_realtime_tick_read_data_;
  std::vector<uint16_t> dxl_operating_mode_read_data_;
  std::vector<bool> dxl_torque_read_data_;
  std::vector<uint32_t> dxl_goal_position_read_data_;
  std::vector<uint32_t> dxl_present_position_read_data_;
  std::vector<uint32_t> dxl_profile_velocity_read_data_;
  std::vector<uint32_t> dxl_present_velocity_read_data_;
  std::vector<uint16_t> dxl_voltage_;
  std::vector<uint8_t> dxl_temperature_;
  std::vector<bool> dxl_is_moving_;

  std::string device_name_;
  float baud_rate_;
  float protocol_version_;
  std::string control_mode_set_;

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
  //bool writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int32_t value);
  //bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t *value);

  // ROS Service Server
  bool dynamixelCommandServer(dynamixel_workbench_monitor::MonitorCommand::Request &req,
                              dynamixel_workbench_monitor::MonitorCommand::Response &res);
};
}

#endif // DYNAMIXEL_WORKBENCH_MONITOR_H_
