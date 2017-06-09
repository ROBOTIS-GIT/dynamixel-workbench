#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "single_dynamixel_state_pub");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
