/**
 * @file /include/dynamixel_workbench_single_manager_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef dynamixel_workbench_single_manager_gui_QNODE_HPP_
#define dynamixel_workbench_single_manager_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <dynamixel_workbench_msgs/DynamixelAX.h>
#include <dynamixel_workbench_msgs/DynamixelRX.h>
#include <dynamixel_workbench_msgs/DynamixelMX.h>
#include <dynamixel_workbench_msgs/DynamixelMX64.h>
#include <dynamixel_workbench_msgs/DynamixelMX106.h>
#include <dynamixel_workbench_msgs/DynamixelEX.h>
#include <dynamixel_workbench_msgs/DynamixelXL.h>
#include <dynamixel_workbench_msgs/DynamixelXM.h>
#include <dynamixel_workbench_msgs/DynamixelPro.h>
#include <dynamixel_workbench_msgs/DynamixelProL42.h>

#include "dynamixel_workbench_toolbox/dynamixel_tool.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/WorkbenchParam.h"
#include "dynamixel_workbench_msgs/GetWorkbenchParam.h"

#endif
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dynamixel_workbench_single_manager_gui {

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
Q_OBJECT
 public:
  QNode(int argc, char** argv );
  virtual ~QNode();
  bool init();
  void run();

  QStringListModel* loggingModel() { return &logging_model_; }
  void log(const std::string &msg, int64_t sender);
  void log(const std::string &msg);

  void dynamixelAXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelAX::ConstPtr &msg);
  void dynamixelRXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelRX::ConstPtr &msg);
  void dynamixelMXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX::ConstPtr &msg);
  void dynamixelMX64StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX64::ConstPtr &msg);
  void dynamixelMX106StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelMX106::ConstPtr &msg);
  void dynamixelEXStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelEX::ConstPtr &msg);
  void dynamixelXLStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXL::ConstPtr &msg);
  void dynamixelXMStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXM::ConstPtr &msg);
  void dynamixelProStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelPro::ConstPtr &msg);
  void dynamixelProL42StatusMsgCallback(const dynamixel_workbench_msgs::DynamixelProL42::ConstPtr &msg);

  void sendTorqueMsg(std::string addr_name, int64_t onoff);
  void sendRebootMsg(void);
  void sendResetMsg(void);
  void sendSetIdMsg(int64_t id);
  void sendSetOperatingModeMsg(std::__cxx11::string index);
  void sendSetBaudrateMsg(float baud_rate);
  void sendControlTableValueMsg(QString table_item, int64_t value);
  void setPositionZeroMsg(int32_t zero_position);

  void getWorkbenchParam(void);
  void setSubscriber(ros::NodeHandle nh);

Q_SIGNALS:
  void loggingUpdated();
  void rosShutdown();

  void updateWorkbenchParam(dynamixel_workbench_msgs::WorkbenchParam);

 private:
  int init_argc;
  char** init_argv;

  QStringListModel logging_model_;

  ros::Publisher dxl_command_msg_pub_;
  ros::Publisher set_workbench_param_msg_pub_;
  ros::Subscriber dxl_status_msg_sub_;
  ros::ServiceClient get_workbench_param_client_;

  int64_t row_count_;

  std::string dxl_model_name_;
  uint16_t dxl_model_number_;

};

}  // namespace dynamixel_workbench_single_manager_gui

#endif /* dynamixel_workbench_single_manager_gui_QNODE_HPP_ */
