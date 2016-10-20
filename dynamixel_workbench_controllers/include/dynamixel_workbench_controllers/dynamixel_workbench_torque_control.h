#ifndef DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H
#define DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_tool.h>
#include <dynamixel_workbench_msgs/MotorStateList.h>
#include <dynamixel_workbench_msgs/SetPosition.h>

#include <dynamixel_sdk/dynamixel_sdk.h>

#define PI 3.14159265358979323846
#define DEGREE2RADIAN (PI / 180.0)
#define RADIAN2DEGREE (180.0 / PI)

#define PAN_TILT_MOTOR 0
#define PAN_MOTOR      0
#define TILT_MOTOR     1

#define PROPORTION_GAIN   0.001
#define DIFFERENTIAL_GAIN 0.00002

#define TILT_MOTOR_MASS 0.082
#define GRAVITY         9.8
#define LINK_LENGTH     0.018

namespace dynamixel_workbench_torque_control
{
class DynamixelWorkbenchTorqueControl
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
  ros::Publisher dynamixel_state_pub_;
  // ROS Service Server
  ros::ServiceServer position_control_server;
  // Parameters
  std::vector<dynamixel_tool::DynamixelTool *> dynamixel_;

  std::string device_name_;
  std::string motor_model_;
  int motor_id_;
  float protocol_version_;
  int baud_rate_;

  int64_t pan_des_pos_;
  int64_t tilt_des_pos_;

  int64_t pan_pre_pos_;
  int64_t tilt_pre_pos_;

  int64_t pan_cur_pos_;
  int64_t tilt_cur_pos_;

  double pan_torque_;
  double tilt_torque_;

  std::map<std::string, std::vector<int64_t> *> read_data_;

 public:
  DynamixelWorkbenchTorqueControl();
  ~DynamixelWorkbenchTorqueControl();
  bool dynamixelControlLoop(void);

 private:
  bool initDynamixelWorkbenchTorqueControl(void);
  bool shutdownDynamixelWorkbenchTorqueControl(void);

  bool initMotor(std::string motor_model, uint8_t motor_id, float protocol_version);

  bool readDynamixelRegister(uint8_t id, uint16_t addr, uint8_t length, int64_t *value);
  bool readMotorState(std::string addr_name);

  bool writeSyncDynamixel(uint16_t addr, uint8_t length, int64_t pan_motor_value, int64_t tilt_motor_value);
  bool writeTorque(bool onoff);
  bool writeCurrent(int64_t pan_cur, int64_t tilt_cur);

  int16_t convertTorque2Value(double torque);
  int64_t convertRadian2Value(double radian);
  double convertValue2Radian(int32_t value);

  bool getPublishedMsg(void);
  bool controlPanTiltMotorCallback(dynamixel_workbench_msgs::SetPosition::Request &req,
                                   dynamixel_workbench_msgs::SetPosition::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_TORQUE_CONTROL_H
