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
  ROS_ASSERT(initSingleDynamixelController());
}

SingleDynamixelController::~SingleDynamixelController()
{
  ROS_ASSERT(shutdownSingleDynamixelController());
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

void SingleDynamixelController::viewManagerMenu(void)
{
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Single Manager supports GUI (dynamixel_workbench_single_manager_gui)  ");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Command list :");
  ROS_INFO("[help|h|?]................: display this menu");
  ROS_INFO("[status]..................: status of a Dynamixel");
  ROS_INFO("[table]...................: check a control table of a dynamixel");
  ROS_INFO("[reboot]..................: reboot a Dynamixel(only protocol version 2.0)");
  ROS_INFO("[factory_reset]...........: command for all data back to the factory settings values");
  ROS_INFO("[[table_item] [value].....: change address value of a dynamixel");
  ROS_INFO("[exit]....................: shutdown");
  ROS_INFO("----------------------------------------------------------------------");
  ROS_INFO("Press SpaceBar to command a Dynamixel");
}

//void SingleManager::showControlTable(void)
//{
//  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
//  {
//    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
//    if (dynamixel_torque_status_)
//    {
//      if ((dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE) && (dynamixel_->item_->memory_type == dynamixel_tool::RAM))
//      {
//        ROS_INFO("%s", dynamixel_->item_->item_name.c_str());
//      }
//    }
//    else
//    {
//      if (dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE)
//      {
//        ROS_INFO("%s", dynamixel_->item_->item_name.c_str());
//      }
//    }
//  }
//}

//void SingleManager::checkValidationCommand(bool *valid_cmd, char *cmd)
//{
//  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
//  {
//    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
//    if (cmd == dynamixel_->item_->item_name)
//    {
//      *valid_cmd = true;
//      break;
//    }
//    else
//    {
//      *valid_cmd = false;
//    }
//  }
//}

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

//bool SingleManager::dynamixelSingleManagerLoop(void)
//{
//  char input[128];
//  char cmd[80];
//  char param[20][30];
//  int num_param = 0;
//  char *token;
//  bool valid_cmd = false;

//  setPublishedMsg();

//  if (kbhit())
//  {
//    if (getchar() == SPACEBAR_ASCII_VALUE)
//    {
//      printf("[CMD]");
//      fgets(input, sizeof(input), stdin);

//      char *p;
//      if ((p = strchr(input, '\n'))!= NULL) *p = '\0';
//      fflush(stdin);

//      if (strlen(input) == 0) return false;

//      token = strtok(input, " ");

//      if (token == 0) return false;

//      strcpy(cmd, token);
//      token = strtok(0, " ");
//      num_param = 0;

//      while (token != 0)
//      {
//        strcpy(param[num_param++], token);
//        token = strtok(0, " ");
//      }

//      checkValidationCommand(&valid_cmd, cmd);

//      if (strcmp(cmd, "help") == 0 || strcmp(cmd, "h") == 0 || strcmp(cmd, "?") == 0)
//      {
//          viewManagerMenu();
//      }
//      else if (strcmp(cmd, "status") == 0)
//      {
//        ROS_INFO("[ID] %u, [Model Name] %s, [BAUD RATE] %d", dynamixel_->id_, dynamixel_->model_name_.c_str(), portHandler_->getBaudRate());
//      }
//      else if (strcmp(cmd, "exit") == 0)
//      {
//        shutdownDynamixelWorkbenchSingleManager();
//        return true;
//      }
//      else if (strcmp(cmd, "table") == 0)
//      {
//        showControlTable();
//      }
//      else if (strcmp(cmd, "reboot") == 0)
//      {
//        rebootDynamixel();
//      }
//      else if (strcmp(cmd, "factory_reset") == 0)
//      {
//        resetDynamixel();
//      }
//      else if (valid_cmd == true)
//      {
//        if (num_param == 1)
//        {
//          dynamixel_->item_ = dynamixel_->ctrl_table_[cmd];
//          if (dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE)
//          {
//            if((dynamixel_torque_status_ == true) && dynamixel_->item_->memory_type == dynamixel_tool::EEPROM)
//            {
//              ROS_ERROR("address in EEPROM is not accessed when motor is torque on");
//              ROS_ERROR("Check a ""table""");
//            }
//            else if ((dynamixel_torque_status_ == false) && (dynamixel_->item_->item_name != "torque_enable") && dynamixel_->item_->memory_type == dynamixel_tool::RAM)
//            {
//              ROS_ERROR("address in RAM is accessed when motor is torque on");
//            }
//            else
//            {
//              if (dynamixel_->item_->item_name == "id")
//              {
//                if (atoi(param[0]) > 0 && atoi(param[0]) < 253)
//                {
//                  writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, atoi(param[0]));
//                  usleep(dynamixel_->item_->data_length* 55 * 1000 * 10);

//                  dynamixel_ = new dynamixel_tool::DynamixelTool(atoi(param[0]), dynamixel_->model_number_, packetHandler_->getProtocolVersion());
//                  ROS_INFO("...Succeeded to set dynamixel id [%u]", dynamixel_->id_);
//                }
//                else
//                {
//                  ROS_ERROR(" Dynamixel ID can be set 1~252");
//                }
//              }
//              else if (dynamixel_->item_->item_name == "baud_rate")
//              {
//                if (dynamixel_->baud_rate_table_.find(atoi(param[0]))->second == dynamixel_->baud_rate_table_.end()->second)
//                {
//                  ROS_ERROR(" Failed to change [ BAUD RATE: %d ]", atoi(param[0]));
//                  ROS_ERROR(" Please check the valid baud rate at dynamixel_tool packages or E-MANUAL");

//                  writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, dynamixel_->baud_rate_table_.find(57600)->second);
//                  usleep(dynamixel_->item_->data_length* 55 * 1000 *10);

//                  if (portHandler_->setBaudRate(dynamixel_->baud_rate_table_.find(57600)->first) == false)
//                  {
//               //     sleep(1);
//                    ROS_INFO(" Failed to change default baudrate(57600)!");
//                  }
//                  else
//                  {
//                //    sleep(1);
//                    ROS_INFO(" Success to change default baudrate! [ BAUD RATE: 57600 ]");
//                  }
//                }
//                else
//                {
//                  if (atoi(param[0]) < 2250000)
//                  {
//                    writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, dynamixel_->baud_rate_table_.find(atoi(param[0]))->second);
//                    usleep(dynamixel_->item_->data_length* 55 * 1000 *10);

//                    if (portHandler_->setBaudRate(dynamixel_->baud_rate_table_.find(atoi(param[0]))->first) == false)
//                    {
//                      ROS_INFO(" Failed to change baudrate!");
//                    }
//                    else
//                    {
//                      ROS_INFO(" Success to change baudrate! [ BAUD RATE: %d ]", dynamixel_->baud_rate_table_.find(atoi(param[0]))->first);
//                    }
//                  }
//                  else
//                  {
//                    ROS_ERROR(" USB2Dynamixel supports baudrate under '2250000'");
//                  }
//                }
//              }
//              else if (dynamixel_->item_->item_name == "protocol_version")
//              {
//                if (atoi(param[0]) == 1.0 || atoi(param[0]) == 2.0)
//                {
//                  // TODO: Find restriced access address in protocol_version 1.0 of XM430
//                  writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, atof(param[0]));
//                  usleep(dynamixel_->item_->data_length* 55 * 1000 *10);

//                  packetHandler_->getPacketHandler(atof(param[0]));

//                  ROS_INFO(" Success to change protocol version [ PROTOCOL VERSION: %.2f]", packetHandler_->getProtocolVersion());
//                }
//                else
//                {
//                  ROS_ERROR(" Dynamixel has '1.0' or '2.0' protocol version");
//                }
//              }
//              else
//              {
//                writeDynamixelRegister(dynamixel_->id_, dynamixel_->item_->address, dynamixel_->item_->data_length, atoi(param[0]));
//                if (dynamixel_->item_->memory_type == dynamixel_tool::EEPROM)
//                {
//                  usleep(dynamixel_->item_->data_length* 55 * 1000 *10);
//                }
//              }
//            }
//          }
//        }
//        else
//        {
//          ROS_ERROR("Invalid parameters! Please check control table [-table]");
//        }
//      }
//      else
//      {
//        ROS_ERROR("Invalid command. Please check menu[help, ?]");
//      }
//    }
//  }
//}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "single_dynamixel_controller");
  SingleDynamixelController single_dynamixel_controller;
  ros::Rate loop_rate(125);
  single_dynamixel_controller.viewManagerMenu();

  while (ros::ok())
  {
    //single_manager.dynamixelSingleManagerLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
