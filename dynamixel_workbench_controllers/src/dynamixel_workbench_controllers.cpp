/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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

#include "dynamixel_workbench_controllers/dynamixel_workbench_controllers.h"

//DynamixelController::DynamixelController()
//{
//  std::string port_name   = priv_node_handle_.param<std::string>("port_name", "/dev/ttyUSB0");
//  uint32_t baud_rate      = priv_node_handle_.param<int>("baud_rate", 57600);

//  uint8_t scan_range      = priv_node_handle_.param<int>("scan_range", 253);

//  bool result = false;
//  const char *log = NULL;

//  dxl_wb_ = new DynamixelWorkbench;

//  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
//  if (result == false)
//  {
//    printf("%s\n", log);
//    printf("Please check USB port name");
//    return;
//  }
//  else
//    printf("\n%s\n", log);



//  if (dxl_wb_->scan(dxl_id_, &dxl_cnt_, scan_range) != true)
//  {
//    ROS_ERROR("Not found Motors, Please check scan range or baud rate");
//    ros::shutdown();
//    return;
//  }

//  initMsg();

//  for (int index = 0; index < dxl_cnt_; index++)
//    dxl_wb_->jointMode(dxl_id_[index], profile_velocity, profile_acceleration);

//  dxl_wb_->addSyncWrite("Goal_Position");

//  initPublisher();
//  initSubscriber();
//  initServer();
//}

//DynamixelController::~DynamixelController(){}

//PositionControl::~PositionControl()
//{
//  for (int index = 0; index < dxl_cnt_; index++)
//    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

//  ros::shutdown();
//}

//void PositionControl::initMsg()
//{
//  printf("-----------------------------------------------------------------------\n");
//  printf("  dynamixel_workbench controller; position control example             \n");
//  printf("-----------------------------------------------------------------------\n");
//  printf("\n");

//  for (int index = 0; index < dxl_cnt_; index++)
//  {
//    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
//    printf("ID      : %d\n", dxl_id_[index]);
//    printf("\n");
//  }
//  printf("-----------------------------------------------------------------------\n");
//}

//void PositionControl::dynamixelStatePublish()
//{
//  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
//  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

//  for (int index = 0; index < dxl_cnt_; index++)
//  {
//    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
//    dynamixel_state[index].id                  = dxl_id_[index];
//    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
//    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
//    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
//    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
//    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
//    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

//    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
//  }
//  dynamixel_state_list_pub_.publish(dynamixel_state_list);
//}

//void PositionControl::jointStatePublish()
//{
//  int32_t present_position[dxl_cnt_] = {0, };

//  for (int index = 0; index < dxl_cnt_; index++)
//    present_position[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");

//  int32_t present_velocity[dxl_cnt_] = {0, };

//  for (int index = 0; index < dxl_cnt_; index++)
//    present_velocity[index] = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");

//  sensor_msgs::JointState dynamixel_;
//  dynamixel_.header.stamp = ros::Time::now();

//  for (int index = 0; index < dxl_cnt_; index++)
//  {
//    std::stringstream id_num;
//    id_num << "id_" << (int)(dxl_id_[index]);

//    dynamixel_.name.push_back(id_num.str());

//    dynamixel_.position.push_back(dxl_wb_->convertValue2Radian(dxl_id_[index], present_position[index]));
//    dynamixel_.velocity.push_back(dxl_wb_->convertValue2Velocity(dxl_id_[index], present_velocity[index]));
//  }
//  joint_states_pub_.publish(dynamixel_);
//}

//void PositionControl::controlLoop()
//{
//  dynamixelStatePublish();
//  jointStatePublish();
//}

//bool PositionControl::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
//                                              dynamixel_workbench_msgs::JointCommand::Response &res)
//{
//  int32_t goal_position = 0;
//  int32_t present_position = 0;

//  if (req.unit == "rad")
//  {
//    goal_position = dxl_wb_->convertRadian2Value(req.id, req.goal_position);
//  }
//  else if (req.unit == "raw")
//  {
//    goal_position = req.goal_position;
//  }
//  else
//  {
//    goal_position = req.goal_position;
//  }

//  bool ret = dxl_wb_->goalPosition(req.id, goal_position);

//  res.result = ret;
//}

//void PositionControl::goalJointPositionCallback(const sensor_msgs::JointState::ConstPtr &msg)
//{
//  double goal_position[dxl_cnt_] = {0.0, };

//  for (int index = 0; index < dxl_cnt_; index++)
//    goal_position[index] = msg->position.at(index);

//  int32_t goal_dxl_position[dxl_cnt_] = {0, };

//  for (int index = 0; index < dxl_cnt_; index++)
//  {
//    goal_dxl_position[index] = dxl_wb_->convertRadian2Value(dxl_id_[index], goal_position[index]);
//  }

//  dxl_wb_->syncWrite("Goal_Position", goal_dxl_position);
//}

DynamixelController::DynamixelController():node_handle_("")
{
  joint_state_topic_ = node_handle_.param<bool>("use_joint_states_topic", true);
  cmd_vel_topic_ = node_handle_.param<bool>("use_cmd_vel_topic", false);

  dxl_wb_ = new DynamixelWorkbench;
}

DynamixelController::~DynamixelController(){}

bool DynamixelController::initWorkbench(const std::string port_name, const uint32_t baud_rate)
{
  bool result = false;
  const char* log;

  result = dxl_wb_->init(port_name.c_str(), baud_rate, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  return result;
}

bool DynamixelController::getDynamixelsInfo(const std::string yaml_file)
{
  YAML::Node dynamixel;
  dynamixel = YAML::LoadFile(yaml_file.c_str());

  if (dynamixel == NULL)
    return false;

  for (YAML::const_iterator it_file = dynamixel.begin(); it_file != dynamixel.end(); it_file++)
  {
    std::string name = it_file->first.as<std::string>();
    if (name.size() == 0)
    {
      continue;
    }
    else
    {
//      ROS_INFO("Dynamixel Name: %s", name.c_str());
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      uint32_t value = it_item->second.as<uint32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      std::pair<std::string, uint32_t> info(item_name, value);
      dynamixel_info_.push_back(info);
    }
  }

//  for (auto const& item:dynamixel_info_)
//  {
//    ROS_INFO("item_name: %s, value: %d", item.first.c_str(), item.second);
//  }

  return true;
}

bool DynamixelController::loadDynamixels(void)
{
  bool result = false;
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    uint16_t model_number = 0;
    result = dxl_wb_->ping((uint8_t)dxl.second, &model_number, &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
      return result;
    }
    else
    {
      ROS_INFO("Name : %s, ID : %d, Model Number : %d", dxl.first.c_str(), dxl.second, model_number);
    }
  }

  return result;
}

bool DynamixelController::initDynamixels(void)
{
  const char* log;

  for (auto const& dxl:dynamixel_)
  {
    dxl_wb_->torqueOff((uint8_t)dxl.second);

    for (auto const& info:dynamixel_info_)
    {
      if (info.first != "ID" && info.first != "Baud_Rate")
      {
        bool result = dxl_wb_->writeRegister((uint8_t)dxl.second, info.first.c_str(), info.second, &log);
        if (result == false)
        {
          ROS_ERROR("%s", log);
          ROS_ERROR("Failed to write items[%s] to Dynamixel[Name : %s, ID : %d]", info.first.c_str(), dxl.first.c_str(), dxl.second);
        }
      }
    }
  }

  return true;
}

bool DynamixelController::initHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto const& it = dynamixel_.begin();
  result = dxl_wb_->addSyncWriteHandler(it.second, "Goal_Position", &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxl_wb_->addSyncReadHandler(ADDR_PRESENT_CURRENT_2,
                                      (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2),
                                      &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }

  return result;
}

void DynamixelController::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
  if (joint_state_topic_) joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initSubscriber()
{
  if (cmd_vel_topic_) cmd_vel_sub_ = node_handle_.subscribe("cmd_vel", 10, &DynamixelController::commandVelocityCallback, this);
}

void DynamixelController::initServer()
{
  joint_command_server_     = node_handle_.advertiseService("joint_command", &DynamixelController::jointCommandMsgCallback, this);
  dynamixel_command_server_ = node_handle_.advertiseService("dynamixel_command", &DynamixelController::dynamixelCommandMsgCallback, this);
}

void DynamixelController::readCallback(const ros::TimerEvent&)
{


//  ROS_INFO("callback_1");
}

void DynamixelController::writeCallback(const ros::TimerEvent&)
{
//  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dynamixel_.size()];
//  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

//  for (int index = 0; index < dxl_cnt_; index++)
//  {
//    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
//    dynamixel_state[index].id                  = dxl_id_[index];
//    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
//    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
//    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
//    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
//    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
//    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

//    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
//  }
//  dynamixel_state_list_pub_.publish(dynamixel_state_list);

//  if (joint_state_topic_)
//  {
//    sensor_msgs::JointState joint_state_msg_;
//    joint_state_msg_.name.push_back(name);
//  }
//  ROS_INFO("callback_2");
}

void DynamixelController::publishCallback(const ros::TimerEvent&)
{
//  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dynamixel_.size()];
//  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

//  for (int index = 0; index < dxl_cnt_; index++)
//  {
//    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
//    dynamixel_state[index].id                  = dxl_id_[index];
//    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
//    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
//    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
//    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
//    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
//    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

//    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
//  }
//  dynamixel_state_list_pub_.publish(dynamixel_state_list);

//  if (joint_state_topic_)
//  {
//    sensor_msgs::JointState joint_state_msg_;
//    joint_state_msg_.name.push_back(name);
//  }
//  ROS_INFO("callback_3");
}

bool DynamixelController::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                                  dynamixel_workbench_msgs::JointCommand::Response &res)
{
  return true;
}

void DynamixelController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

}

bool DynamixelController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                      dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_workbench_controllers");
  ros::NodeHandle node_handle("");

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

  std::string port_name = "/dev/ttyUSB0";
  uint32_t baud_rate = 57600;

  if (argc < 2)
  {
    ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    port_name = argv[1];
    baud_rate = atoi(argv[2]);
  }

  DynamixelController dynamixel_controller;
  bool result = false;

  result = dynamixel_controller.initWorkbench(port_name, baud_rate);
  if (result == false)  ROS_ERROR("Please check USB port name");

  result = dynamixel_controller.getDynamixelsInfo(yaml_file);
  if (result == false)  ROS_ERROR("Please check YAML file");

  result = dynamixel_controller.loadDynamixels();
  if (result == false) ROS_ERROR("Please check Dynamixel ID or BaudRate");

  result = dynamixel_controller.initDynamixels();

  result = dynamixel_controller.initHandlers();
  if (result == false) ROS_ERROR("Failed to set Dynamixel SDK Handler");

  dynamixel_controller.initPublisher();
  dynamixel_controller.initSubscriber();
  dynamixel_controller.initServer();

  ros::Timer read_timer = node_handle.createTimer(ros::Duration(0.010), &DynamixelController::readCallback, &dynamixel_controller);
  ros::Timer write_timer = node_handle.createTimer(ros::Duration(0.010), &DynamixelController::writeCallback, &dynamixel_controller);
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(0.010), &DynamixelController::publishCallback, &dynamixel_controller);

  ros::spin();

  return 0;
}
