/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Taehoon Lim (Darby) */

#include "dynamixel_workbench_single_manager/single_dynamixel_controller.h"

using namespace single_dynamixel_controller;

SingleDynamixelController::SingleDynamixelController()
{
  // init Service Client
  dynamixel_info_client_    = node_handle_.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("dynamixel/info");
  dynamixel_command_client_ = node_handle_.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel/command");
}

SingleDynamixelController::~SingleDynamixelController()
{

}

bool SingleDynamixelController::initSingleDynamixelController()
{

}

bool SingleDynamixelController::shutdownSingleDynamixelController(void)
{
  ros::shutdown();
  return true;
}

int SingleDynamixelController::getch()
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

int SingleDynamixelController::kbhit()
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

void SingleDynamixelController::viewManagerMenu()
{
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Single Manager supports GUI (dynamixel_workbench_single_manager_gui)  ");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Command list :");
  ROS_INFO("[help|h|?]...........: display this menu");
  ROS_INFO("[info]...............: information of a Dynamixel");
  ROS_INFO("[table]..............: check a control table of a dynamixel");
  ROS_INFO("[reboot].............: reboot a Dynamixel(only protocol version 2.0)");
  ROS_INFO("[factory_reset]......: command for all data back to factory settings values");
  ROS_INFO("[[table_item] [value]: change address value of a dynamixel");
  ROS_INFO("[exit]...............: shutdown");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Press Enter Key To Command A DYNAMIXEL");
}

//bool SingleManager::getWorkbenchParamCallback(dynamixel_workbench_msgs::GetWorkbenchParam::Request &req, dynamixel_workbench_msgs::GetWorkbenchParam::Response &res)
//{
//  res.workbench_parameter.device_name = device_name_;
//  res.workbench_parameter.baud_rate = portHandler_->getBaudRate();
//  res.workbench_parameter.protocol_version = packetHandler_->getProtocolVersion();
//  res.workbench_parameter.model_name = dynamixel_->model_name_;
//  res.workbench_parameter.model_id = dynamixel_->id_;
//  res.workbench_parameter.model_number = dynamixel_->model_number_;

//  return true;
//}

//void SingleManager::dynamixelCommandMsgCallback(const dynamixel_workbench_msgs::DynamixelCommand::ConstPtr &msg)
//{
//  if (msg->addr_name == "reboot")
//  {
//    rebootDynamixel();
//  }
//  else if (msg->addr_name == "factory_reset")
//  {
//    resetDynamixel();
//  }
//  else if (msg->addr_name == "baud_rate")
//  {
//    dynamixel_->item_ = dynamixel_->ctrl_table_[msg->addr_name];
//    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, dynamixel_->baud_rate_table_.find(msg->value)->second);

//    usleep(dynamixel_->item_->data_length* 55 * 1000 *10);

//    if (portHandler_->setBaudRate(dynamixel_->baud_rate_table_.find(msg->value)->first) == false)
//    {
//      ROS_INFO(" Failed to change baudrate!");
//    }
//    else
//    {
//      ROS_INFO(" Success to change baudrate! [ BAUD RATE: %d ]", dynamixel_->baud_rate_table_.find(msg->value)->first);
//    }
//  }
//  else if (msg->addr_name == "id")
//  {
//    dynamixel_->item_ = dynamixel_->ctrl_table_[msg->addr_name];
//    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, msg->value);

//    usleep(dynamixel_->item_->data_length* 55 * 1000 *10);

//    dynamixel_ = new dynamixel_tool::DynamixelTool(msg->value, dynamixel_model_number_, protocol_version_);
//    ROS_INFO("...Succeeded to set dynamixel id");
//    ROS_INFO("[ID] %u, [Model Name] %s", dynamixel_->id_, dynamixel_->model_name_.c_str());
//  }
//  else
//  {
//    dynamixel_->item_ = dynamixel_->ctrl_table_[msg->addr_name];
//    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, msg->value);

//    if (dynamixel_->item_->memory_type == dynamixel_tool::EEPROM)
//    {
//      usleep(dynamixel_->item_->data_length* 55 * 1000 *10);
//    }
//  }
//}

bool SingleDynamixelController::controlLoop()
{
  char input[128];
  char cmd[80];
  char param[20][30];
  int num_param = 0;
  char *token;
  bool valid_cmd = false;

  if (kbhit())
  {
    if (getchar() == ENTER_ASCII_VALUE)
    {
      viewManagerMenu();
      printf("[CMD]");
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

      while (token != 0)
      {
        strcpy(param[num_param++], token);
        token = strtok(0, " ");
      }

      if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
      {
        viewManagerMenu();
      }
      else if (strcmp(cmd, "info") == 0)
      {
        dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;

        if (dynamixel_info_client_.call(get_dynamixel_info))
        {
          ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %ld", get_dynamixel_info.response.dynamixel_info.model_id,
                                                                get_dynamixel_info.response.dynamixel_info.model_name.c_str(),
                                                                get_dynamixel_info.response.dynamixel_info.baud_rate);
        }
      }
      else if (strcmp(cmd, "exit") == 0)
      {
        dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

        set_dynamixel_command.request.command = "exit";

        if (dynamixel_command_client_.call(set_dynamixel_command))
        {
          if (set_dynamixel_command.response.comm_result)
            shutdownSingleDynamixelController();
        }
        return true;
      }
      else if (strcmp(cmd, "table") == 0)
      {
        dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

        set_dynamixel_command.request.command = "table";

        if (dynamixel_command_client_.call(set_dynamixel_command))
        {
          if (!set_dynamixel_command.response.comm_result)
            ROS_ERROR("It didn't load DYNAMIXEL Control Table");
        }
      }
      else if (strcmp(cmd, "reboot") == 0)
      {
        dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

        set_dynamixel_command.request.command = "reboot";

        if (dynamixel_command_client_.call(set_dynamixel_command))
        {
          if (!set_dynamixel_command.response.comm_result)
            ROS_ERROR("It didn't reboot to DYNAMIXEL");
        }
      }
      else if (strcmp(cmd, "factory_reset") == 0)
      {
        dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

        set_dynamixel_command.request.command = "factory_reset";

        if (dynamixel_command_client_.call(set_dynamixel_command))
        {
          if (!set_dynamixel_command.response.comm_result)
            ROS_ERROR("It didn't factory reset to DYNAMIXEL");
        }
      }
      else if (num_param == 1)
      {
        dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

        set_dynamixel_command.request.command = "addr";
        set_dynamixel_command.request.addr_name = cmd;
        set_dynamixel_command.request.value = atoi(param[0]);

        if (!dynamixel_command_client_.call(set_dynamixel_command))
        {
          if (!set_dynamixel_command.response.comm_result)
            ROS_ERROR("It didn't works!!");
        }
        else
        {
          if (set_dynamixel_command.response.comm_result)
            ROS_INFO("It works!!");
        }
      }
    }
  }
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "single_dynamixel_controller");
  SingleDynamixelController single_dynamixel_controller;
  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    single_dynamixel_controller.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
