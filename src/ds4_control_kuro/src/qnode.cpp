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
#include "../include/ds4_control_kuro/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ds4_control_kuro {

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
	ros::init(init_argc,init_argv,"ds4_control_kuro");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
        ps4_sub = n.subscribe("ps4",100,&QNode::ps4Callback,this);
        ps4arm_pub = n.advertise<msg_generate::ps4arm_msg>("ps4arm",100);

        start();
	return true;
}
void QNode::ps4Callback(const msg_generate::ps4::ConstPtr &msg)
{
        ps4Info.buttonCircle = msg->buttonCircle;
        ps4Info.buttonCross = msg->buttonCross;
        ps4Info.buttonSquare = msg->buttonSquare;
        ps4Info.buttonTriangle = msg->buttonTriangle;

        ps4Info.buttonShare = msg->buttonShare;
        ps4Info.buttonLogo = msg->buttonLogo;
        ps4Info.buttonOptions = msg->buttonOptions;

        ps4Info.buttonL1 = msg->buttonL1;
        ps4Info.buttonL2 = msg->buttonL2;
        ps4Info.buttonR1 = msg->buttonR1;
        ps4Info.buttonR2 = msg->buttonR2;

        ps4Info.buttonUpDown = msg->buttonUpDown;
        ps4Info.buttonLeftRight = msg->buttonLeftRight;

        ps4Info.lJoystickUpDown = msg->lJoystickUpDown;
        ps4Info.lJoystickLeftRight = msg->lJoystickLeftRight;
        ps4Info.rJoystickUpDown = msg->rJoystickUpDown;
        ps4Info.rJoystickLeftRight = msg->rJoystickLeftRight;
        Q_EMIT QnodePs4Callback();
}
void QNode::run() {
        ros::Rate loop_rate(33);
	while ( ros::ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

}  // namespace ds4_control_kuro
