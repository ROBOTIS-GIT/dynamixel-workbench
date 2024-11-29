// ros_control
#include <controller_manager/controller_manager.h>

#include "dynamixel_general_hw/dynamixel_general_hw.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamixel_general_control_node");

  std::string port_name;
  int baud_rate;
  XmlRpc::XmlRpcValue dxl_info;
  int rate_hz;

  if (!(ros::param::get("~port_name", port_name) && ros::param::get("~baud_rate", baud_rate) &&
        ros::param::get("~dynamixel_info", dxl_info) && ros::param::get("~control_rate", rate_hz)))
  {
    ROS_ERROR("Couldn't get necessary parameters");
    return 0;
  }

  dynamixel_general_hw::DynamixelGeneralHw dxl_hw;

  bool result = false;

  result = dxl_hw.initWorkbench(port_name, baud_rate);
  if (result == false)
  {
    ROS_ERROR("Please check USB port name");
    return 0;
  }

  result = dxl_hw.getDynamixelsInfo(dxl_info);
  if (result == false)
  {
    ROS_ERROR("Please check dynamixel_info");
    return 0;
  }

  result = dxl_hw.loadDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check Dynamixel ID or BaudRate");
    return 0;
  }

  result = dxl_hw.initDynamixels();
  if (result == false)
  {
    ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
    return 0;
  }

  result = dxl_hw.initControlItems();
  if (result == false)
  {
    ROS_ERROR("Please check control items");
    return 0;
  }

  result = dxl_hw.initSDKHandlers();
  if (result == false)
  {
    ROS_ERROR("Failed to set Dynamixel SDK Handler");
    return 0;
  }

  result = dxl_hw.initRosInterface();
  if (result == false)
  {
    return 0;
  }

  controller_manager::ControllerManager cm(&dxl_hw);

  // For non-realtime spinner thread
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Control loop
  ros::Rate rate(rate_hz);
  ros::Time prev_time = ros::Time::now();
  bool prev_controller_ignored = false;

  while (ros::ok())
  {
    const ros::Time now = ros::Time::now();
    const ros::Duration elapsed_time = now - prev_time;

    dxl_hw.read();
    const bool controller_ignored = dxl_hw.isJntCmdIgnored();
    const bool reset_controllers = (prev_controller_ignored && !controller_ignored);
    cm.update(now, elapsed_time, reset_controllers);
    dxl_hw.write();
    prev_time = now;
    prev_controller_ignored = controller_ignored;

    rate.sleep();
  }
  spinner.stop();
  dxl_hw.cleanup();

  return 0;
}
