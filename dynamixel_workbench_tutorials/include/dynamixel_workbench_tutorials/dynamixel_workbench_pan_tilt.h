#ifndef DYNAMIXEL_WORKBENCH_PAN_TILT_H
#define DYNAMIXEL_WORKBENCH_PAN_TILT_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_msgs/SetPosition.h>

namespace dynamixel_workbench_pan_tilt
{
class DynamixelWorkbenchPanTilt
{
 public:
  // ROS Server Client
  ros::ServiceClient position_control_client_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
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
