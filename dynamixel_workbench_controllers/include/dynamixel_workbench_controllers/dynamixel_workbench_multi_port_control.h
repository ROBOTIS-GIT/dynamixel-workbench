#ifndef DYNAMIXEL_WORKBENCH_MULTI_PORT_CONTROL_H
#define DYNAMIXEL_WORKBENCH_MULTI_PORT_CONTROL_H

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

#define PAN_MOTOR      0
#define TILT_MOTOR     1

#define VELOCITY      100
#define ACCELERATION  20

namespace dynamixel_workbench_multi_port_control
{
class DynamixelWorkbenchMultiPortControl
{
 public:
  dynamixel::PortHandler *pan_motor_portHandler_;
  dynamixel::PortHandler *tilt_motor_portHandler_;
  dynamixel::PacketHandler *pan_motor_packetHandler_;
  dynamixel::PacketHandler *tilt_motor_packetHandler_;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  // ROS Parameters
  bool is_debug_;
  // ROS Topic Publisher
  ros::Publisher dxl_state_pub_;
  // ROS Service Server
  ros::ServiceServer multi_port_control_server;
  // Parameters
  std::vector<dynamixel_tool::DynamixelTool *> dynamixel_;

  std::string pan_motor_device_name_;
  std::string pan_motor_model_;
  int pan_motor_id_;
  float pan_motor_protocol_version_;
  int pan_motor_baud_rate_;

  std::string tilt_motor_device_name_;
  std::string tilt_motor_model_;
  int tilt_motor_id_;
  float tilt_motor_protocol_version_;
  int tilt_motor_baud_rate_;

  std::map<std::string, int64_t> read_pan_motor_data_;
  std::map<std::string, int64_t> read_tilt_motor_data_;

 public:
  DynamixelWorkbenchMultiPortControl();
  ~DynamixelWorkbenchMultiPortControl();
  bool dynamixelControlLoop(void);

 private:
  bool initDynamixelWorkbenchMultiPortControl(void);
  bool shutdownDynamixelWorkbenchMultiPortControl(void);

  bool initMotor(std::string motor_model, uint8_t motor_id, float protocol_version);

  bool readDynamixelRegister(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t *value);
  bool readMotorState(int8_t motor, std::string addr_name);

  bool writeDynamixelRegister(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint8_t length, int64_t value);
  bool writeTorque(bool onoff);
  bool writeProfile();
  bool writePosition(int64_t pan_pos, int64_t tilt_pos);

  int64_t convertRadian2Value(int8_t motor, double radian);

  bool getPublishedMsg(void);
  bool controlPanTiltMotorCallback(dynamixel_workbench_msgs::SetPosition::Request &req,
                                   dynamixel_workbench_msgs::SetPosition::Response &res);
};
}

#endif //DYNAMIXEL_WORKBENCH_MULTI_PORT_CONTROL_H
