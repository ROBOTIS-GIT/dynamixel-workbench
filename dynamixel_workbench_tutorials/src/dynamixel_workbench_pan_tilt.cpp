#include "dynamixel_workbench_tutorials/dynamixel_workbench_pan_tilt.h"

using namespace dynamixel_workbench_pan_tilt;

DynamixelWorkbenchPanTilt::DynamixelWorkbenchPanTilt()
    :nh_priv_("~"),
     is_debug_(false)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchPanTilt());

  // init ROS Subscribe
}

DynamixelWorkbenchPanTilt::~DynamixelWorkbenchPanTilt()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchPanTilt());
}

bool DynamixelWorkbenchPanTilt::initDynamixelWorkbenchPanTilt(void)
{
  ROS_INFO("dynamixel_workbench_pan_tilt : Init OK!");
  return true;
}

bool DynamixelWorkbenchPanTilt::shutdownDynamixelWorkbenchPanTilt(void)
{
  ros::shutdown();
  return true;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_pan_tilt");
  DynamixelWorkbenchPanTilt dxl_pan_tilt;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
