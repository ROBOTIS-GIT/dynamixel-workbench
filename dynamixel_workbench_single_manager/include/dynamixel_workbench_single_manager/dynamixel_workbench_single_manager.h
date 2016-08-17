#ifndef DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H
#define DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H

#include <unistd.h>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_msgs/DynamixelAX.h>
#include <dynamixel_workbench_msgs/DynamixelRX.h>
#include <dynamixel_workbench_msgs/DynamixelMX.h>
#include <dynamixel_workbench_msgs/DynamixelMX64.h>
#include <dynamixel_workbench_msgs/DynamixelMX106.h>
#include <dynamixel_workbench_msgs/DynamixelXL.h>
#include <dynamixel_workbench_msgs/DynamixelXM.h>
#include <dynamixel_workbench_msgs/DynamixelPro.h>
#include <dynamixel_workbench_msgs/DynamixelProL42.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK Library


// Protocol version
#define PROTOCOL_VERSION            2.0

// Default setting
#define BAUDRATE                    1000000
#define DEVICENAME                  "/dev/ttyUSB0"

#define ESC_ASCII_VALUE             0x1b
#define SPACEBAR_ASCII_VALUE        0x20

namespace dynamixel_workbench_single_manager
{
class DynamixelWorkbenchSingleManager
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
  dynamixel_tool::DynamixelTool *dxl_;
  std::string device_name_;
  float baud_rate_;
  float protocol_version_;
  uint32_t read_value_;
  uint16_t dxl_number_;

 public:
  DynamixelWorkbenchSingleManager();
  ~DynamixelWorkbenchSingleManager();
  void viewManagerMenu(void);
  bool dynamixelSingleManagerLoop(void);

 private:
  int getch(void);
  int kbhit(void);
  bool initDynamixelWorkbenchSingleManager(void);
  bool shutdownDynamixelWorkbenchSingleManager(void);
  void setPublisher(void);
  void getPublisher(void);
  bool writeDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t value);
  bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, uint32_t *value);
  bool scanDynamixelID();
  void showControlTable(void);
  bool rebootDynamixel();
  bool resetDynamixel();
  void checkValidationCommand(bool *valid_cmd, char *cmd);

  void axMotorMessage(void);
  void rxMotorMessage(void);
  void mxMotorMessage(void);
  void mx64MotorMessage(void);
  void mx106MotorMessage(void);
  void xlMotorMessage(void);
  void xmMotorMessage(void);
  void proMotorMessage(void);
  void proL42MotorMessage(void);
};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H
