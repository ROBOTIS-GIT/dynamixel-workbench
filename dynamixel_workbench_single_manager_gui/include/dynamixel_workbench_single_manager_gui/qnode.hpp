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

#include "dynamixel_workbench_msgs/DynamixelXM.h"

#endif
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dynamixel_workbench_single_manager_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
        void log(const LogLevel &level, const std::string &msg, int64_t sender);

        void dynamixelStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXM::ConstPtr &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    QStringListModel logging_model;

    ros::Subscriber dynamixel_status_msg_sub_;
};

}  // namespace dynamixel_workbench_single_manager_gui

#endif /* dynamixel_workbench_single_manager_gui_QNODE_HPP_ */
