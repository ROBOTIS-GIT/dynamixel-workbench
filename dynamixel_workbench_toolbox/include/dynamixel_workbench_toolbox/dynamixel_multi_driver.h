#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_multi_driver");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
