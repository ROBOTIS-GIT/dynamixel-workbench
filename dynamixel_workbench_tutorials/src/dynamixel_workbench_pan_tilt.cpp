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

  // init ROS Client
  position_control_client_ = nh_.serviceClient<dynamixel_workbench_msgs::SetPosition>("/dynamixel_workbench_tutorials/pan_tilt");
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

  DynamixelWorkbenchPanTilt dynamixel_pan_tilt;
  dynamixel_workbench_msgs::SetPosition srv;

  if (argc != 3)
  {
    ROS_ERROR("rosrun dynamixel_workbench_tutorials dynamixel_workbench_pan_tilt [pan_pos(radian)] [tilt_pos(radian)]");
    return 1;
  }

  srv.request.pan_pos = atof(argv[1]);
  srv.request.tilt_pos = atof(argv[2]);

  if (dynamixel_pan_tilt.position_control_client_.call(srv))
  {
    ROS_INFO("send messages: [pan_pos: %ld (value)] [tilt_pos: %ld (value)]", srv.response.pan_pos, srv.response.tilt_pos);
  }
  else
  {
    ROS_ERROR("Failed to call service /pan_tilt or /multi_port");
  }

  return 0;
}
