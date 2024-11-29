#ifndef DYNAMIXEL_GENERAL_HW_H
#define DYNAMIXEL_GENERAL_HW_H

// C++
#include <algorithm>
#include <cmath>
#include <mutex>

// Dynamixel workbench
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

// ROS base
#include <ros/ros.h>
#include <ros/callback_queue.h>

// ros_control
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_interface_loader.h>

// ROS msg and srv
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <dynamixel_general_hw/DynamixelStateList.h>
#include <std_msgs/Bool.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define SYNC_READ_HANDLER_FOR_PRESENT_INFO 0

namespace dynamixel_general_hw
{

typedef struct
{
  std::string item_name;
  int32_t value;
} ItemValue;

class DynamixelGeneralHw : public hardware_interface::RobotHW
{
protected:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Dynamixel workbench parameters
  std::unique_ptr<DynamixelWorkbench> dxl_wb_;
  std::map<std::string, uint32_t> dynamixel_;
  std::map<std::string, const ControlItem*> control_items_;
  std::vector<std::pair<std::string, ItemValue>> dynamixel_info_;
  dynamixel_general_hw::DynamixelStateList dynamixel_state_list_;

  // Transmission loader
  transmission_interface::RobotTransmissions robot_transmissions_;
  std::unique_ptr<transmission_interface::TransmissionInterfaceLoader> transmission_loader_;

  // Actuator interface to transmission loader
  hardware_interface::ActuatorStateInterface actr_state_interface_;
  hardware_interface::PositionActuatorInterface pos_actr_interface_;

  // Actuator raw data
  std::vector<std::string> actr_names_;
  std::vector<double> actr_curr_pos_;
  std::vector<double> actr_curr_vel_;
  std::vector<double> actr_curr_eff_;
  std::vector<double> actr_cmd_pos_;

  // Actuator parameters
  std::map<std::string, double> torque_consts_;

  // E-stop interface
  ros::Subscriber servo_sub_;
  bool is_servo_raw_;
  bool is_servo_;
  bool prev_is_servo_;
  ros::Subscriber hold_pos_sub_;
  bool is_hold_pos_raw_;
  bool is_hold_pos_;

  // Dynamixel-specific interfaces
  ros::Publisher dynamixel_state_pub_;
  ros::ServiceServer dynamixel_cmd_srv_;

  // For multi-threaded spinning
  std::shared_ptr<ros::AsyncSpinner> subscriber_spinner_;
  ros::CallbackQueue subscriber_queue_;
  std::mutex mtx_;

  bool is_calc_effort_;
  bool is_pub_temp_;
  bool is_pub_volt_;

public:
  DynamixelGeneralHw();

  bool initWorkbench(const std::string port_name, const uint32_t baud_rate);
  bool getDynamixelsInfo(XmlRpc::XmlRpcValue& dxl_info);
  bool loadDynamixels(void);
  bool initDynamixels(void);
  bool initControlItems(void);
  bool initSDKHandlers(void);
  bool initRosInterface(void);

  void cleanup(void);

  void readDynamixelState(void);
  void read(void);
  void write(void);

  bool isJntCmdIgnored(void);

  void servoCallback(const std_msgs::BoolConstPtr& msg);
  void holdPosCallback(const std_msgs::BoolConstPtr& msg);
  bool dynamixelCmdCallback(dynamixel_workbench_msgs::DynamixelCommand::Request& req,
                            dynamixel_workbench_msgs::DynamixelCommand::Response& res);
};  // end class DynamixelGeneralHw

};  // end namespace dynamixel_general_hw

#endif  // DYNAMIXEL_GENERAL_HW_H
