/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/dynamixel_workbench_single_manager_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dynamixel_workbench_single_manager_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"dynamixel_workbench_single_manager_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    dynamixel_status_msg_sub_ = n.subscribe("XM430_W210_R/motor_state", 10, &QNode::dynamixelStatusMsgCallback, this);

	start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"dynamixel_workbench_single_manager_gui");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);

    dynamixel_status_msg_sub_ = n.subscribe("XM430_W210_R/motor_state", 10, &QNode::dynamixelStatusMsgCallback, this);

	start();
	return true;
}

void QNode::dynamixelStatusMsgCallback(const dynamixel_workbench_msgs::DynamixelXM::ConstPtr &msg)
{
  log(Info, std::string("model_number: "), msg->model_number);
//  log(Info, std::string("version_of_firmware: "), msg->version_of_firmware);
//  log(Info, std::string("id: "), msg->id);
//  log(Info, std::string("baud_rate: "), msg->baud_rate);
//  log(Info, std::string("return_delay_tiem: "), msg->return_delay_time);
//  log(Info, std::string("operating_mode: "), msg->operating_mode);
//  log(Info, std::string("protocol_version: "), msg->protocol_version);
//  log(Info, std::string("homing_offset: "), msg->homing_offset);
//  log(Info, std::string("moving_threshold: "), msg->moving_threshold);
//  log(Info, std::string("max_temperature_limit: "), msg->max_temperature_limit);
//  log(Info, std::string("max_voltage_limit: "), msg->max_voltage_limit);
//  log(Info, std::string("min_voltage_limit: "), msg->min_voltage_limit);
//  log(Info, std::string("pwm_limit: "), msg->pwm_limit);
//  log(Info, std::string("current_limit: "), msg->current_limit);
//  log(Info, std::string("acceleration_liit: "), msg->acceleration_limit);
//  log(Info, std::string("velocity_limit: "), msg->velocity_limit);
//  log(Info, std::string("max_position_limit: "), msg->max_position_limit);
//  log(Info, std::string("min_position_limit: "), msg->min_position_limit);
//  log(Info, std::string("shutdown: "), msg->shutdown);
//  log(Info, std::string("torque_enable: "), msg->torque_enable);
//  log(Info, std::string("led: "), msg->led);
//  log(Info, std::string("status_return_level: "), msg->status_return_level);
//  log(Info, std::string("registered_instruction: "), msg->registered_instruction);
//  log(Info, std::string("hardware_error_status: "), msg->hardware_error_status);
//  log(Info, std::string("velocity_i_gain: "), msg->velocity_i_gain);
//  log(Info, std::string("velocity_p_gain: "), msg->velocity_p_gain);
//  log(Info, std::string("velocity_d_gain: "), msg->velocity_d_gain);
//  log(Info, std::string("position_i_gain: "), msg->position_i_gain);
//  log(Info, std::string("position_p_gain: "), msg->position_p_gain);
//  log(Info, std::string("feedforward_2nd_gain: "), msg->feedforward_2nd_gain);
//  log(Info, std::string("feedforward_1st_gain: "), msg->feedforward_1st_gain);
//  log(Info, std::string("goal_pwm: "), msg->goal_pwm);
//  log(Info, std::string("goal_current: "), msg->goal_current);
//  log(Info, std::string("goal_velocity: "), msg->goal_velocity);
//  log(Info, std::string("profile_acceleration: "), msg->profile_acceleration);
//  log(Info, std::string("profile_velocity: "), msg->profile_velocity);
//  log(Info, std::string("goal_position: "), msg->goal_position);
//  log(Info, std::string("realtime_tick: "), msg->realtime_tick);
//  log(Info, std::string("moving: "), msg->moving);
//  log(Info, std::string("moving_status: "), msg->moving_status);
//  log(Info, std::string("present_pwm: "), msg->present_pwm);
//  log(Info, std::string("present_current: "), msg->present_current);
//  log(Info, std::string("present_velocity: "), msg->present_velocity);
//  log(Info, std::string("present_position: "), msg->present_position);
//  log(Info, std::string("velocity_trajectory: "), msg->velocity_trajectory);
//  log(Info, std::string("position_trajectory: "), msg->position_trajectory);
//  log(Info, std::string("present_input_voltage: "), msg->present_input_voltage);
//  log(Info, std::string("present_temperature: "), msg->present_temperature);
//  log(Info, std::string("indirect_address_1: "), msg->indirect_address_1);
//  log(Info, std::string("indirect_data_1: "), msg->indirect_data_1);
//  log(Info, std::string("indirect_address_29: "), msg->indirect_address_29);
//  log(Info, std::string("indirect_data_29: "), msg->indirect_data_29);
}

void QNode::run() {
    ros::Rate loop_rate(10);
	int count = 0;
	while ( ros::ok() ) {

		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
        //chatter_publisher.publish(msg);
        //log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::log( const LogLevel &level, const std::string &msg, int64_t sender){
    logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] " << msg << sender;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}  // namespace dynamixel_workbench_single_manager_gui
