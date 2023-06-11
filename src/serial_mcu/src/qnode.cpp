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
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include "../include/serial_mcu/qnode.hpp"
#include "../include/serial_mcu/serial_msg.h"
#include "../include/serial_mcu/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace serial_mcu {
using namespace std;
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

void QNode::DxCallback(const serial_mcu::Motor_msg::ConstPtr &msg)
{
    Dx_msg.length = msg->length;
    Dx_msg.mode = msg->mode;
    Dx_msg.id = msg->id;
    Dx_msg.speed = msg->speed;
    Dx_msg.position = msg->position;
    Dx_msg.type = msg->type;
    Dx_msg.acceleration = msg->acceleration;
    Q_EMIT Rx_MotorData();
}

void QNode::MotionCallback(const serial_mcu::Mt2Serial_msg::ConstPtr &motion_msg)
{
    Motion_msg.Motion_Mode = motion_msg->Motion_Mode;
    Motion_msg.Motion_Num = motion_msg->Motion_Num;
    Q_EMIT Rx_MotionData();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"serial_mcu");

	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    Motor_sub = n.subscribe("Dynamixel", 100, &QNode::DxCallback,this);
    Motion_sub = n.subscribe("Motion", 100, &QNode::MotionCallback,this);
    Serial_pub = n.advertise<serial_mcu::motion_end>("motion_end", 100);
	start();
	return true;
}


void QNode::run() {
    ros::spin();
//	ros::Rate loop_rate(1);
//    while ( ros::ok() )
//    {
//		ros::spinOnce();
//		loop_rate.sleep();
//	}
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


}  // namespace serial_mcu
