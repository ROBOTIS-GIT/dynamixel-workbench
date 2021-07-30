/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#if defined(__linux__) || defined(__APPLE__)
  #include <fcntl.h>          // FILE control
  #include <termios.h>        // Terminal IO
#elif defined(_WIN32)
  #include <conio.h>
#endif

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define ESC_ASCII_VALUE             0x1b
#define FORWARD                     0x77
#define BACKWARD                    0x78
#define LEFT                        0x61
#define RIGHT                       0x64
#define STOPS                       0x73

int getch(void)
{
  #if defined(__linux__) || defined(__APPLE__)

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

  #elif defined(_WIN32) || defined(_WIN64)
    return _getch();
  #endif
}

int kbhit(void)
{
  #if defined(__linux__) || defined(__APPLE__)
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
  #elif defined(_WIN32)
    return _kbhit();
  #endif
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "wheel_operator");
  ros::NodeHandle node_handle("");

  double lin_vel_step = 0.01;
  double ang_vel_step = 0.1;

  ROS_INFO("You can set '-lin_vel_step' and  '-ang_vel_step' arguments (default is 0.01 and 0.1)");

  if (argc > 1)
  {
    lin_vel_step = atof(argv[1]);
    ang_vel_step = atof(argv[2]);
  }

  ros::Publisher cmd_vel_pub = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  geometry_msgs::Twist twist_msg;

  std::string msg =
  "\n\
  Control Your Mobile Robot! \n\
  --------------------------- \n\
  Moving around:\n\
          w\n\
     a    s    d\n\
          x\n\
  \n\
  w/x : increase/decrease linear velocity\n\
  a/d : increase/decrease angular velocity\n\
  \n\
  s : force stop\n\
  \n\
  CTRL-C to quit\n\
  ";

  ROS_INFO("%s", msg.c_str());

  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    if (kbhit())
    {
      char c = getch();

      if (c == FORWARD)
      {
        twist_msg.linear.x += lin_vel_step;
      }
      else if (c == BACKWARD)
      {
        twist_msg.linear.x -= lin_vel_step;
      }
      else if (c == LEFT)
      {
        twist_msg.angular.z += ang_vel_step;
      }
      else if (c == RIGHT)
      {
        twist_msg.angular.z -= ang_vel_step;
      }
      else if (c == STOPS)
      {
        twist_msg.linear.x  = 0.0f;
        twist_msg.angular.z = 0.0f;
      }
      else
      {
        twist_msg.linear.x  = twist_msg.linear.x;
        twist_msg.angular.z = twist_msg.angular.z;
      }
    }

    cmd_vel_pub.publish(twist_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
