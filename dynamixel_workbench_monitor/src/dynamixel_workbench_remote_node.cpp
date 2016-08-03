#include "dynamixel_workbench_monitor/dynamixel_workbench_remote_node.h"

using namespace dynamixel_workbench_remote;

DynamixelWorkbenchRemote::DynamixelWorkbenchRemote()
    :nh_priv_("~"),
     is_debug_(false)
{
  // Init parameter
  nh_priv_.param("is_debug", is_debug_, is_debug_);

  // Init target name
  ROS_ASSERT(initDynamixelWorkbenchRemote());

  // Init ROS Service client
  dynamixel_workbench_remote_client_ = nh_.serviceClient<dynamixel_workbench_monitor::MonitorCommand>("dynamixel_workbench_monitor_remote");
}

DynamixelWorkbenchRemote::~DynamixelWorkbenchRemote()
{
  ROS_ASSERT(shutDownDynamixelWorkbenchRemote());
}

bool DynamixelWorkbenchRemote::initDynamixelWorkbenchRemote()
{
  ROS_INFO("dynamixel_workbench_remote_node : Init OK!");
  return true;
}

bool DynamixelWorkbenchRemote::shutDownDynamixelWorkbenchRemote()
{
  ros::shutdown();
  return true;
}

int DynamixelWorkbenchRemote::getch()
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

int DynamixelWorkbenchRemote::kbhit()
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

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void DynamixelWorkbenchRemote::viewRemoteMenu()
{
  ROS_INFO("----------------------------------------------------------------------\n");
  ROS_INFO("Menu:\n");
  ROS_INFO("[-h].......................: display this help");
  ROS_INFO("[-pos dxl_id pos_value]....: dynamixel position control");
  ROS_INFO("[-vel dxl_id vel_value]....: dynamixel velocity set");
  ROS_INFO("----------------------------------------------------------------------\n");
}

bool DynamixelWorkbenchRemote::remoteLoop()
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;
  dynamixel_workbench_monitor::MonitorCommand monitor_command_service;

  printf("[CMD] ");
  fgets(input, sizeof(input), stdin);

  char *p;
  if ((p = strchr(input, '\n'))!= NULL) *p = '\0';
  fflush(stdin);

  if (strlen(input) == 0) return false;

  token = strtok(input, " ");

  if (token == 0) return false;

  strcpy(cmd, token);
  token = strtok(0, " ");
  num_param = 0;

  while(token != 0)
  {
    strcpy(param[num_param++], token);
    token = strtok(0, " ");
  }

  if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
  {
      viewRemoteMenu();
  }
  else if (strcmp(cmd, "exit") == 0)
  {
    ros::shutdown();
    return true;
  }
  else if (strcmp(cmd, "push") == 0)
  {
    if (num_param == 1)
    {
      monitor_command_service.request.cmd = cmd;
      monitor_command_service.request.dxl_id = atoi(param[0]);
      dynamixel_workbench_remote_client_.call(monitor_command_service);
    }
    else
    {
      ROS_ERROR("Invalid parameters!");
    }
  }
  else if (strcmp(cmd, "pop") == 0)
  {
    monitor_command_service.request.cmd = cmd;
    dynamixel_workbench_remote_client_.call(monitor_command_service);
  }
  else if (strcmp(cmd, "check") == 0)
  {
    monitor_command_service.request.cmd = cmd;
    dynamixel_workbench_remote_client_.call(monitor_command_service);
  }
  else if (strcmp(cmd, "torque") == 0)
  {
    if (num_param == 1)
    {
      monitor_command_service.request.cmd = cmd;
      monitor_command_service.request.onoff = atoi(param[0]);
      dynamixel_workbench_remote_client_.call(monitor_command_service);
    }
    else
    {
      ROS_ERROR("Invalid parameters!");
    }
  }
  else if (strcmp(cmd, "pos") == 0)
  {
    if (num_param == 1)
    {
      monitor_command_service.request.cmd = cmd;
      monitor_command_service.request.pos_value = atoi(param[0]);
      dynamixel_workbench_remote_client_.call(monitor_command_service);
    }
    else
    {
      ROS_ERROR("Invalid parameters!");
    }
  }
  else if (strcmp(cmd, "vel") == 0)
  {
    if (num_param == 1)
    {
      monitor_command_service.request.cmd = cmd;
      monitor_command_service.request.vel_value = atoi(param[0]);
      dynamixel_workbench_remote_client_.call(monitor_command_service);
    }
    else
    {
      ROS_ERROR("Invalid parameters!");
    }
  }
  else
  {
    ROS_ERROR("Bad command! Please input 'help'");
  }

  return true;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "dynamixel_workbench_remote_node");
  DynamixelWorkbenchRemote dwr;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    dwr.remoteLoop();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
