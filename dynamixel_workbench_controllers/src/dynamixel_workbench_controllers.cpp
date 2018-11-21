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

DynamixelController::DynamixelController():node_handle_(""), priv_node_handle_("~")
{
  is_joint_state_topic_ = priv_node_handle_.param<bool>("use_joint_states_topic", true);
  is_cmd_vel_topic_ = priv_node_handle_.param<bool>("use_cmd_vel_topic", false);

  if (is_cmd_vel_topic_ == true)
  {
    wheel_separation_ = priv_node_handle_.param<double>("mobile_robot_config/seperation_between_wheels", 0.0);
    wheel_radius_ = priv_node_handle_.param<double>("mobile_robot_config/radius_of_wheel", 0.0);
  }

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
      if (is_joint_state_topic_)
      {
        joint_state_msg_.name.push_back(name);
      }
//      ROS_INFO("Dynamixel Name: %s", name.c_str());
    }

    YAML::Node item = dynamixel[name];
    for (YAML::const_iterator it_item = item.begin(); it_item != item.end(); it_item++)
    {
      std::string item_name = it_item->first.as<std::string>();
      int32_t value = it_item->second.as<int32_t>();

      if (item_name == "ID")
        dynamixel_[name] = value;

      ItemValue item_value = {item_name, value};
      std::pair<std::string, ItemValue> info(name, item_value);
//      ROS_INFO("item_name: %s, value: %d", item_value.item_name.c_str(), item_value.value);
      dynamixel_info_.push_back(info);
    }
  }

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
      if (dxl.first == info.first)
      {
        if (info.second.item_name != "ID" && info.second.item_name != "Baud_Rate")
        {
          bool result = dxl_wb_->itemWrite((uint8_t)dxl.second, info.second.item_name.c_str(), info.second.value, &log);
          if (result == false)
          {
            ROS_ERROR("%s", log);
            ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[Name : %s, ID : %d]", info.second.value, info.second.item_name.c_str(), dxl.first.c_str(), dxl.second);
            return false;
          }
        }
      }
    }
  }

  return true;
}

bool DynamixelController::initSDKHandlers(void)
{
  bool result = false;
  const char* log = NULL;

  auto it = dynamixel_.begin();
  result = dxl_wb_->addSyncWriteHandler(it->second, "Goal_Position", &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }

  result = dxl_wb_->addSyncWriteHandler(it->second, "Goal_Velocity", &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    return result;
  }

  if (dxl_wb_->getProtocolVersion() == 2.0f)
  {
    result = dxl_wb_->addSyncReadHandler(ADDR_PRESENT_CURRENT_2,
                                        (LENGTH_PRESENT_CURRENT_2 + LENGTH_PRESENT_VELOCITY_2 + LENGTH_PRESENT_POSITION_2),
                                        &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
      return result;
    }
  }

  return result;
}

void DynamixelController::initPublisher()
{
  dynamixel_state_list_pub_ = priv_node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
  if (is_joint_state_topic_) joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initSubscriber()
{
  trajectory_sub_ = node_handle_.subscribe("joint_command", 100, &DynamixelController::trajectoryMsgCallback, this);
  if (is_cmd_vel_topic_) cmd_vel_sub_ = node_handle_.subscribe("cmd_vel", 10, &DynamixelController::commandVelocityCallback, this);
}

void DynamixelController::initServer()
{
  dynamixel_command_server_ = node_handle_.advertiseService("dynamixel_command", &DynamixelController::dynamixelCommandMsgCallback, this);
}

static double priv_secs[3] = {0.0, 0.0, 0.0};
void DynamixelController::readCallback(const ros::TimerEvent&)
{
  double secs =ros::Time::now().toSec();

  bool result = false;
  const char* log = NULL;

  dynamixel_workbench_msgs::DynamixelState  dynamixel_state[dynamixel_.size()];
  dynamixel_state_list_.dynamixel_state.clear();

  int32_t get_value[dynamixel_.size()];

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    dynamixel_state[id_cnt].name = dxl.first;
    dynamixel_state[id_cnt].id = (uint8_t)dxl.second;

    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

  result = dxl_wb_->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                             id_array,
                             dynamixel_.size(),
                             &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }

  id_cnt = 0;
  for (auto const& dxl:dynamixel_)
  {
    uint8_t id = id_array[id_cnt];

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  &id,
                                                  1,
                                                  ADDR_PRESENT_CURRENT_2,
                                                  LENGTH_PRESENT_CURRENT_2,
                                                  &get_value[id_cnt],
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
    else
    {
      dynamixel_state[id_cnt].present_current = get_value[id_cnt];
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  &id,
                                                  1,
                                                  ADDR_PRESENT_VELOCITY_2,
                                                  LENGTH_PRESENT_VELOCITY_2,
                                                  &get_value[id_cnt],
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
    else
    {
      dynamixel_state[id_cnt].present_velocity = get_value[id_cnt];
    }

    result = dxl_wb_->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT,
                                                  &id,
                                                  1,
                                                  ADDR_PRESENT_POSITION_2,
                                                  LENGTH_PRESENT_POSITION_2,
                                                  &get_value[id_cnt],
                                                  &log);
    if (result == false)
    {
      ROS_ERROR("%s", log);
    }
    else
    {
      dynamixel_state[id_cnt].present_position = get_value[id_cnt];
    }

//    ROS_INFO("ID : %d, present_position : %d, present_velocity : %d, present_current : %d", id, dynamixel_state[id_cnt].present_position, dynamixel_state[id_cnt].present_velocity, dynamixel_state[id_cnt].present_current);
    dynamixel_state_list_.dynamixel_state.push_back(dynamixel_state[id_cnt]);
    id_cnt++;
  }

//  ROS_WARN("[readCallback] diff_secs : %f", secs - priv_secs[0]);
  priv_secs[0] = secs;
}

void DynamixelController::publishCallback(const ros::TimerEvent&)
{
//  double secs =ros::Time::now().toSec();

  dynamixel_state_list_pub_.publish(dynamixel_state_list_);

  if (is_joint_state_topic_)
  {
    joint_state_msg_.header.stamp = ros::Time::now();

    joint_state_msg_.position.clear();
    joint_state_msg_.velocity.clear();
    joint_state_msg_.effort.clear();

    uint8_t id_cnt = 0;
    for (auto const& dxl:dynamixel_)
    {
      double position = 0.0;
      double velocity = 0.0;
      double effort = 0.0;

      effort = dxl_wb_->convertValue2Current((uint8_t)dxl.second, (int16_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_current);
      velocity = dxl_wb_->convertValue2Velocity((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_velocity);
      position = dxl_wb_->convertValue2Radian((uint8_t)dxl.second, (int32_t)dynamixel_state_list_.dynamixel_state[id_cnt].present_position);

      joint_state_msg_.effort.push_back(position);
      joint_state_msg_.velocity.push_back(velocity);
      joint_state_msg_.position.push_back(position);

      id_cnt++;
    }

    joint_states_pub_.publish(joint_state_msg_);
  }

//  ROS_WARN("[publishCallback] diff_secs : %f", secs - priv_secs[1]);
//  priv_secs[1] = secs;
}

void DynamixelController::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  double wheel_velocity[dynamixel_.size()];
  int32_t dynamixel_velocity[dynamixel_.size()];

  const uint8_t LEFT = 0;
  const uint8_t RIGHT = 1;

  double robot_lin_vel = msg->linear.x;
  double robot_ang_vel = msg->angular.z;

  uint8_t id_array[dynamixel_.size()];
  uint8_t id_cnt = 0;

  float rpm = 0.0;
  for (auto const& dxl:dynamixel_)
  {
    const ModelInfo *modelInfo = dxl_wb_->getModelInfo((uint8_t)dxl.second);
    rpm = modelInfo->rpm;
    id_array[id_cnt++] = (uint8_t)dxl.second;
  }

//  V = r * w = r * (RPM * 0.10472) (Change rad/sec to RPM)
//       = r * ((RPM * Goal_Velocity) * 0.10472)		=> Goal_Velocity = V / (r * RPM * 0.10472) = V * VELOCITY_CONSTATNE_VALUE

  double velocity_constant_value = 1 / (wheel_radius_ * rpm * 0.10472);

  wheel_velocity[LEFT]  = robot_lin_vel - (robot_ang_vel * wheel_separation_ / 2);
  wheel_velocity[RIGHT] = robot_lin_vel + (robot_ang_vel * wheel_separation_ / 2);

  dynamixel_velocity[LEFT]  = wheel_velocity[LEFT] * velocity_constant_value;
  dynamixel_velocity[RIGHT] = wheel_velocity[RIGHT] * velocity_constant_value;

  result = dxl_wb_->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY, id_array, dynamixel_.size(), dynamixel_velocity, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
  }
}

void DynamixelController::trajectoryMsgCallback(const trajectory_msgs::JointTrajectory::ConstPtr &msg)
{
  bool result = false;
  const char* log = NULL;

  JointTrajectory joint_trajectory_;

  std::string joint_name = msg->joint_names.at(0);
  trajectory_msgs::JointTrajectoryPoint jnt_tra_point = msg->points.at(0);

  uint8_t id_array[msg->joint_names.size()];
  uint8_t id_cnt = 0;

  for (auto const& dxl:dynamixel_)
  {
    for (auto const& joint:msg->joint_names)
    {
      if (dxl.first == joint)
      {
        id_array[id_cnt++] = dxl.second;
      }
    }
  }

//  jnt_tra_point.time_from_start.

//  uint8_t id_array[req.id.size()];
//  int32_t goal_position[req.id.size()];

//  for (auto const& id:req.id)
//  {
//    goal_position[index] = dynamixel_workbench_->convertRadian2Value(actuator_id.at(index), radian_vector.at(index));
//  }

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

//  return true;
}

bool DynamixelController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                      dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
  bool result = false;
  const char* log;

  uint8_t id = req.id;
  std::string item_name = req.addr_name;
  int32_t value = req.value;

  result = dxl_wb_->itemWrite(id, item_name.c_str(), value, &log);
  if (result == false)
  {
    ROS_ERROR("%s", log);
    ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, item_name.c_str(), id);
  }

  res.comm_result = result;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamixel_workbench_controllers");
  ros::NodeHandle node_handle("");

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

  std::string yaml_file = node_handle.param<std::string>("dynamixel_info", "");

  result = dynamixel_controller.initWorkbench(port_name, baud_rate);
  if (result == false)  ROS_ERROR("Please check USB port name");

  result = dynamixel_controller.getDynamixelsInfo(yaml_file);
  if (result == false)  ROS_ERROR("Please check YAML file");

  result = dynamixel_controller.loadDynamixels();
  if (result == false) ROS_ERROR("Please check Dynamixel ID or BaudRate");

  result = dynamixel_controller.initDynamixels();
  if (result == false) ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");

  result = dynamixel_controller.initSDKHandlers();
  if (result == false) ROS_ERROR("Failed to set Dynamixel SDK Handler");

  dynamixel_controller.initPublisher();
  dynamixel_controller.initSubscriber();
  dynamixel_controller.initServer();

  ros::Timer read_timer = node_handle.createTimer(ros::Duration(0.010f), &DynamixelController::readCallback, &dynamixel_controller);
  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(0.010f), &DynamixelController::publishCallback, &dynamixel_controller);

  ros::spin();

  return 0;
}
