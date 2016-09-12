#ifndef DYNAMIXEL_WORKBENCH_WHEEL_H
#define DYNAMIXEL_WORKBENCH_WHEEL_H

#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>          // FILE control
#include <termios.h>        // Terminal IO
#include <stdio.h>
#include <ros/ros.h>

#include <dynamixel_workbench_msgs/SetDirection.h>

#define ESC_ASCII_VALUE             0x1b
#define FORWARD                     0x77
#define BACKWARD                    0x73
#define LEFT                        0x61
#define RIGHT                       0x64

namespace dynamixel_workbench_wheel
{
class DynamixelWorkbenchWheel
{
 public:
  // ROS Server Client
  ros::ServiceClient wheel_control_client_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // Parameters

 public:
  DynamixelWorkbenchWheel();
  ~DynamixelWorkbenchWheel();
  int getch(void);
  int kbhit(void);

 private:
  bool initDynamixelWorkbenchWheel(void);
  bool shutdownDynamixelWorkbenchWheel(void);
};
}

#endif //DYNAMIXEL_WORKBENCH_WHEEL_H
