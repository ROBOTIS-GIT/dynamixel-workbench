#ifndef DYNAMIXEL_WORKBENCH_PAN_TILT_H
#define DYNAMIXEL_WORKBENCH_PAN_TILT_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_msgs/GetPosition.h>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK Library

namespace dynamixel_workbench_pan_tilt
{
class DynamixelWorkbenchPanTilt
{
 public:

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dxl_state_pub_;
  // Parameters

 public:
  DynamixelWorkbenchPanTilt();
  ~DynamixelWorkbenchPanTilt();

 private:
  bool initDynamixelWorkbenchPanTilt(void);
  bool shutdownDynamixelWorkbenchPanTilt(void);

};
}

#endif //DYNAMIXEL_WORKBENCH_PAN_TILT_H
