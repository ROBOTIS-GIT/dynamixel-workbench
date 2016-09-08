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
#include <dynamixel_workbench_msgs/DynamixelEX.h>
#include <dynamixel_workbench_msgs/DynamixelXL.h>
#include <dynamixel_workbench_msgs/DynamixelXM.h>
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

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dxl_state_pub_;
  // ROS Topic Subscriber
  ros::Subscriber dxl_command_sub_;
  // ROS Server
  ros::ServiceServer workbench_param_server_;
  // Parameters
  std::string device_name_;
  float baud_rate_;
  float protocol_version_;
  uint16_t dxl_model_number_;
  uint8_t dxl_model_id_;
  bool dxl_torque_status_;

  dynamixel_tool::DynamixelTool *dxl_;

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
  void proMotorMessage(void);
  void proL42MotorMessage(void);
};
}

#endif //DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_H
