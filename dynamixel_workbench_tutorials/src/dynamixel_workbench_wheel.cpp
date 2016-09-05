#include "dynamixel_workbench_tutorials/dynamixel_workbench_wheel.h"

using namespace dynamixel_workbench_wheel;

DynamixelWorkbenchWheel::DynamixelWorkbenchWheel()
    :nh_priv_("~"),
     is_debug_(false)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchWheel());

  // init ROS Client
  wheel_control_client_ = nh_.serviceClient<dynamixel_workbench_msgs::SetDirection>("/dynamixel_workbench_velocity_control");
}

DynamixelWorkbenchWheel::~DynamixelWorkbenchWheel()
{
  ROS_ASSERT(shutdownDynamixelWorkbenchWheel());
}

bool DynamixelWorkbenchWheel::initDynamixelWorkbenchWheel(void)
{
  ROS_INFO("dynamixel_workbench_wheel : Init OK!");
  return true;
}

bool DynamixelWorkbenchWheel::shutdownDynamixelWorkbenchWheel(void)
{
  ros::shutdown();
  return true;
}

int DynamixelWorkbenchWheel::getch(void)
{
  struct termios oldt, newt;
  int ch;

  tcgetattr( STDIN_FILENO, &oldt );
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_cc[VMIN] = 0;
  newt.c_cc[VTIME] = 1;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt );
  ch = getchar();
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

  return ch;
}

int DynamixelWorkbenchWheel::kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_wheel");
  ROS_INFO("Control your Turtlebot!! by using keyboard");
  ROS_INFO("w : Forward");
  ROS_INFO("x : Backward");
  ROS_INFO("a : Left");
  ROS_INFO("d : Right");
  ROS_INFO("s : Stop");
  ROS_INFO("");
  ROS_INFO("ESC : exit");

  DynamixelWorkbenchWheel dxl_wheel;
  dynamixel_workbench_msgs::SetDirection srv;
  ros::Rate loop_rate(10);

  while(1)
  {
    if (dxl_wheel.kbhit())
    {
      char c = dxl_wheel.getch();
      if (c == ESC_ASCII_VALUE)
      {
        break;
      }
      if (c == FORWARD)
      {
        srv.request.forward = true;
        srv.request.backward = false;
        srv.request.left = false;
        srv.request.right = false;
        srv.request.stop = false;
      }
      else if (c == BACKWARD)
      {
        srv.request.forward = false;
        srv.request.backward = true;
        srv.request.left = false;
        srv.request.right = false;
        srv.request.stop = false;
      }
      else if (c == LEFT)
      {
        srv.request.forward = false;
        srv.request.backward = false;
        srv.request.left = true;
        srv.request.right = false;
        srv.request.stop = false;
      }
      else if (c == RIGHT)
      {
        srv.request.forward = false;
        srv.request.backward = false;
        srv.request.left = false;
        srv.request.right = true;
        srv.request.stop = false;
      }
      else if (c == STOP)
      {
        srv.request.forward = false;
        srv.request.backward = false;
        srv.request.left = false;
        srv.request.right = false;
        srv.request.stop = true;
      }
    }
    dxl_wheel.wheel_control_client_.call(srv);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
