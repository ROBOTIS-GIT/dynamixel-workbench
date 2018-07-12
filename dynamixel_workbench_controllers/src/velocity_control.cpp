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

#include "dynamixel_workbench_controllers/velocity_control.h"

VelocityControl::VelocityControl()
    :node_handle_(""),
     dxl_cnt_(2)
{
  std::string device_name   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  uint32_t dxl_baud_rate    = node_handle_.param<int>("baud_rate", 57600);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  wheel_separation_ = node_handle_.param<float>("wheel_separation", 0.160);
  wheel_radius_     = node_handle_.param<float>("wheel_radius", 0.033);

  dxl_id_[0] = node_handle_.param<int>("left_wheel", 1);
  dxl_id_[1] = node_handle_.param<int>("right_wheel", 2);

  dxl_wb_ = new DynamixelWorkbench;

  dxl_wb_->begin(device_name.c_str(), dxl_baud_rate);

  for (int index = 0; index < dxl_cnt_; index++)
  {
    uint16_t get_model_number;
    if (dxl_wb_->ping(dxl_id_[index], &get_model_number) != true)
    {
      ROS_ERROR("Not found Motors, Please check id and baud rate");

      ros::shutdown();
      return;
    }
  }

  initMsg();

  // Set Normal Mode to Right Motor(ID : 1)
  // Set Reverse Mode to Left  Motor(ID : 2)
  dxl_wb_->itemWrite(dxl_id_[0], "Drive_Mode", 0);//ID :2 LEFT 
  dxl_wb_->itemWrite(dxl_id_[1], "Drive_Mode", 1);//ID :1 RIGHT 

  for (int index = 0; index < dxl_cnt_; index++)
    dxl_wb_->wheelMode(dxl_id_[index], profile_velocity, profile_acceleration);

  dxl_wb_->addSyncWrite("Goal_Velocity");

  initPublisher();
  initSubscriber();
  initServer();
  initOdom();
}

VelocityControl::~VelocityControl()
{
  for (int index = 0; index < 2; index++)
    dxl_wb_->itemWrite(dxl_id_[index], "Torque_Enable", 0);

  ros::shutdown();
}

void VelocityControl::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("        dynamixel_workbench controller; velocity control example       \n");
  printf("              -This example supports MX2.0 and X Series-               \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int index = 0; index < dxl_cnt_; index++)
  {
    printf("MODEL   : %s\n", dxl_wb_->getModelName(dxl_id_[index]));
    printf("ID      : %d\n", dxl_id_[index]);
    printf("\n");
  }
  printf("-----------------------------------------------------------------------\n");
}

void VelocityControl::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
  odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>("odom", 10);
}

void VelocityControl::initSubscriber()
{
  cmd_vel_sub_ = node_handle_.subscribe("cmd_vel", 10, &VelocityControl::commandVelocityCallback, this);
  reset_odom_sub_ = node_handle_.subscribe("reset_odometry", 10, &VelocityControl::subscribeResetOdometry, this);
}

void VelocityControl::initServer()
{
  wheel_command_server_ = node_handle_.advertiseService("wheel_command", &VelocityControl::wheelCommandMsgCallback, this);
}

void VelocityControl::dynamixelStatePublish()
{
  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_cnt_];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (int index = 0; index < dxl_cnt_; index++)
  {
    dynamixel_state[index].model_name          = std::string(dxl_wb_->getModelName(dxl_id_[index]));
    dynamixel_state[index].id                  = dxl_id_[index];
    dynamixel_state[index].torque_enable       = dxl_wb_->itemRead(dxl_id_[index], "Torque_Enable");
    dynamixel_state[index].present_position    = dxl_wb_->itemRead(dxl_id_[index], "Present_Position");
    dynamixel_state[index].present_velocity    = dxl_wb_->itemRead(dxl_id_[index], "Present_Velocity");
    dynamixel_state[index].goal_position       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Position");
    dynamixel_state[index].goal_velocity       = dxl_wb_->itemRead(dxl_id_[index], "Goal_Velocity");
    dynamixel_state[index].moving              = dxl_wb_->itemRead(dxl_id_[index], "Moving");

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[index]);
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);
  updateMotorInfo(dynamixel_state[LEFT].present_position,dynamixel_state[RIGHT].present_position);
}

void VelocityControl::initOdom(void)
{
  init_encoder = true;

  for (int index = 0; index < 3; index++)
  {
    odom_pose[index] = 0.0;
    odom_vel[index]  = 0.0;
  }

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;

  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 0.0;

  odom.twist.twist.linear.x  = 0.0;
  odom.twist.twist.angular.z = 0.0;

  timestamp_ = ros::Time::now();
}

/*******************************************************************************
* Update motor information
*******************************************************************************/
void VelocityControl::updateMotorInfo(double left_tick, double right_tick)
{
  double current_tick = 0;
  static double last_tick[WHEEL_NUM] = {0.0, 0.0};
  
  if (init_encoder)
  {
    for (int index = 0; index < WHEEL_NUM; index++)
    {
      last_diff_tick[index] = 0.0;
      last_tick[index]      = 0.0;
      last_rad[index]       = 0.0;

      last_velocity[index]  = 0.0;
    }  

    last_tick[LEFT] = left_tick;
    last_tick[RIGHT] = right_tick;

    init_encoder = false;
    return;
  }

  current_tick = left_tick;

  last_diff_tick[LEFT] = current_tick - last_tick[LEFT];
  last_tick[LEFT]      = current_tick;
  last_rad[LEFT]       += TICK2RAD * (double)last_diff_tick[LEFT];

  current_tick = right_tick;

  last_diff_tick[RIGHT] = current_tick - last_tick[RIGHT];
  last_tick[RIGHT]      = current_tick;
  last_rad[RIGHT]       += TICK2RAD * (double)last_diff_tick[RIGHT];
}


/*******************************************************************************
* Calculate the odometry
*******************************************************************************/
bool VelocityControl::calcOdometry(const ros::Time &time)
{


  //float* orientation;//for imu
  double wheel_l, wheel_r;      // rotation value of wheel [rad]
  double delta_s, theta, delta_theta;
  static double last_theta = 0.0;
  double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]

  wheel_l = wheel_r = 0.0;
  delta_s = delta_theta = theta = 0.0;
  v = w = 0.0;





  wheel_l = TICK2RAD * (double)last_diff_tick[LEFT];
  wheel_r = TICK2RAD * (double)last_diff_tick[RIGHT];

  if (std::isnan(wheel_l))
    wheel_l = 0.0;

  if (std::isnan(wheel_r))
    wheel_r = 0.0;

  delta_s     = wheel_radius_ * (wheel_r + wheel_l) / 2.0;
  theta = wheel_radius_ * (wheel_r - wheel_l) / wheel_separation_;  
  //orientation = sensors.getOrientation();//using IMU
  //theta       = atan2f(orientation[1]*orientation[2] + orientation[0]*orientation[3], 
  //              0.5f - orientation[2]*orientation[2] - orientation[3]*orientation[3]);

  delta_theta = theta - last_theta;

  // compute odometric pose
  odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
  odom_pose[2] += delta_theta;

  // compute odometric instantaneouse velocity

  // We cannot estimate the speed with very small time intervals:
  const double dt = (time - timestamp_).toSec();
  if (dt < 0.0001)
      return false; // Interval too small to integrate with

  timestamp_ = time;

  v = delta_s / dt;
  w = delta_theta / dt;

  odom_vel[0] = v;
  odom_vel[1] = 0.0;
  odom_vel[2] = w;

  last_velocity[LEFT]  = wheel_l / dt;
  last_velocity[RIGHT] = wheel_r / dt;
  last_theta = theta;

  return true;
}

void VelocityControl::odomPublish()
{
    ros::Time current_time = ros::Time::now();

    // calculate odometry
    calcOdometry(current_time);


    //filling the odometry
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);

    odom.twist.twist.linear.x  = odom_vel[0];
    odom.twist.twist.angular.z = odom_vel[2];

    odom_pub_.publish(odom);


    // update odom TF
    odom_tf.header = odom.header;
    odom_tf.header.stamp = current_time;
    odom_tf.child_frame_id = odom.child_frame_id;

    odom_tf.transform.translation.x = odom.pose.pose.position.x;
    odom_tf.transform.translation.y = odom.pose.pose.position.y;
    odom_tf.transform.translation.z = odom.pose.pose.position.z;
    odom_tf.transform.rotation      = odom.pose.pose.orientation;

    tf_broadcaster.sendTransform(odom_tf);

}

void VelocityControl::subscribeResetOdometry(const std_msgs::EmptyConstPtr  msg)
{
  //ROS_INFO_STREAM("Controller: Resetting the odometry. [" << name << "].");
  initOdom();
  return;
}



void VelocityControl::controlLoop()
{
  dynamixelStatePublish();
  odomPublish();
}

bool VelocityControl::wheelCommandMsgCallback(dynamixel_workbench_msgs::WheelCommand::Request &req,
                                              dynamixel_workbench_msgs::WheelCommand::Response &res)
{
  static int32_t goal_velocity[2] = {0, 0};

  goal_velocity[0] = dxl_wb_->convertVelocity2Value(dxl_id_[0], req.left_vel);
  goal_velocity[1] = dxl_wb_->convertVelocity2Value(dxl_id_[1], req.right_vel);

  printf("Wheel Request: %d %d\n",goal_velocity[0],goal_velocity[1] );

  bool ret = dxl_wb_->syncWrite("Goal_Velocity", goal_velocity);

  res.result = ret;
}

void VelocityControl::commandVelocityCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  bool dxl_comm_result = false;

  float wheel_velocity[2] = {0.0, 0.0};
  int32_t dynamixel_velocity[2] = {0, 0};

  float lin_vel = msg->linear.x;
  float ang_vel = msg->angular.z;
  printf("Twist Request Lin:%f Ang:%f\n",lin_vel,ang_vel );

  const float RPM_OF_DXL = 0.229;
  const float VELOCITY_CONSTANT_VALUE = 1 / (wheel_radius_ * RPM_OF_DXL * 0.10472);

  wheel_velocity[0]  = lin_vel - (ang_vel * wheel_separation_ / 2);
  wheel_velocity[1]  = lin_vel + (ang_vel * wheel_separation_ / 2);
  printf("Wheel Velocity : %f %f\n",wheel_velocity[0],wheel_velocity[1] );

  dynamixel_velocity[0] = wheel_velocity[0] * VELOCITY_CONSTANT_VALUE;
  dynamixel_velocity[1] = wheel_velocity[1] * VELOCITY_CONSTANT_VALUE;
  printf("Velocity Request: %d %d\n",dynamixel_velocity[0],dynamixel_velocity[1] );

  dxl_wb_->syncWrite("Goal_Velocity", dynamixel_velocity);
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "velocity_control");
  VelocityControl vel_ctrl;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    vel_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
