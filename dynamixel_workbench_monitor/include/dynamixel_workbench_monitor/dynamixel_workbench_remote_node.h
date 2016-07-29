#ifndef DYNAMIXEL_WORKBENCH_REMOTE_NODE_H_
#define DYNAMIXEL_WORKBENCH_REMOTE_NODE_H_

#include <unistd.h>
#include <fcntl.h>
#include <termio.h>>
#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK Library
#include "dynamixel_workbench_monitor/MonitorCommand.h"

#define ESC_ASCII_VALUE         0x1b

namespace dynamixel_workbench_remote
{
class DynamixelWorkbenchRemote
{
 public:

 private:
   // ROS NodeHandle
   ros::NodeHandle nh_;
   ros::NodeHandle nh_priv_;
   // ROS Parameters
   bool is_debug_;
   //ROS Service client
   ros::ServiceClient dynamixel_workbench_remote_client_;


 public:
  DynamixelWorkbenchRemote();
  ~DynamixelWorkbenchRemote();
  bool remoteLoop(void);

 private:
  bool initDynamixelWorkbenchRemote(void);
  bool shutDownDynamixelWorkbenchRemote(void);
  void viewRemoteMenu(void);
  int getch(void);
  int kbhit(void);
};
}



#endif // DYNAMIXEL_WORKBENCH_REMOTE_NODE_H_
