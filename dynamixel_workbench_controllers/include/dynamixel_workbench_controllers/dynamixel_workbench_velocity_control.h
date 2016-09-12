#ifndef DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
#define DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_workbench_msgs/MotorStateList.h>
#include <dynamixel_workbench_msgs/SetDirection.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

#define PI 3.14159265358979323846
#define DEGREE2RADIAN (PI / 180.0)
#define RADIAN2DEGREE (180.0 / PI)

#define LEFT_RIGHT_WHEEL            0
#define LEFT_WHEEL                  0
#define RIGHT_WHEEL                 1

namespace dynamixel_workbench_velocity_control
{
class DynamixelWorkbenchVelocityControl
{
 public:
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dxl_state_pub_;
  // ROS Service Server
  ros::ServiceServer wheel_control_server_;
  // Parameters
  std::vector<dynamixel_tool::DynamixelTool *> dynamixel_;

  std::string device_name_;
  std::string motor_model_;
  int motor_id_;
  float protocol_version_;
  int baud_rate_;
  double right_motor_velocity_;
  double left_motor_velocity_;

  std::map<std::string, std::vector<int64_t> *> read_data_;
  int64_t read_value_;

 public:
  DynamixelWorkbenchVelocityControl();
  ~DynamixelWorkbenchVelocityControl();
  bool dynamixelControlLoop(void);

 private:
  bool initDynamixelWorkbenchVelocityControl(void);
  bool shutdownDynamixelWorkbenchVelocityControl(void);

  bool initMotor(std::string motor_model, uint8_t motor_id, float protocol_version);
  int64_t convertVelocity2Value(double velocity);

  bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value);
  bool readMotorState(std::string addr_name);

  bool writeSyncDynamixel(uint16_t addr, uint8_t length, int64_t pan_motor_value, int64_t tilt_motor_value);
  bool writeTorque(bool onoff);
  bool writeVelocity(int64_t left_wheel_velocity, int64_t right_wheel_velocity);

  bool getPublishedMsg(void);
  bool controlWheelVelocityCallback(dynamixel_workbench_msgs::SetDirection::Request &req,
                                    dynamixel_workbench_msgs::SetDirection::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_VELOCITY_CONTROL_H
