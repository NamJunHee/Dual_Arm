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
#include "../include/dynamixel_serial/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dynamixel_serial {


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
	ros::init(init_argc,init_argv,"dynamixel_serial");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

    Motor_sub = n.subscribe("Dynamixel", 100, &QNode::DxCallback,this);
    Motion_sub = n.subscribe("Motion", 100, &QNode::MotionCallback,this);
    dxl_id_sub = n.subscribe("dxl_id", 100, &QNode::Dxl_idCallback, this);
    wheel_sub = n.subscribe("wheel_mode",100,&QNode::wheel_mode_Callback, this);

    dxl_position_pub = n.advertise<msg_generate::Arm_path_position_msg>("dxl_position", 100);
    motion_end_pub = n.advertise<msg_generate::motion_end>("motion_end", 100);

	start();
	return true;
}


void QNode::run() {
    ros::Rate loop_rate(33);

    while ( ros::ok() ) {

		ros::spinOnce();
		loop_rate.sleep();
	}

	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::DxCallback(const msg_generate::Motor_msg::ConstPtr &msg)
{
    Dx_msg.length = msg->length;
    Dx_msg.mode = msg->mode;
    Dx_msg.id = msg->id;
    Dx_msg.speed = msg->speed;
    Dx_msg.position = msg->position;
    Dx_msg.type = msg->type;
    Dx_msg.acceleration = msg->acceleration;
    std::cout<<"call"<<std::endl;

    Q_EMIT Rx_MotorData();
}

void QNode::MotionCallback(const msg_generate::Motion_msg::ConstPtr &msg)
{
    Motion_msg.max_step = msg->max_step;
    Motion_msg.motion_data = msg->motion_data;
    Motion_msg.repeat = msg->repeat;
    Q_EMIT Rx_MotionData();
//    std::cout<<"call"<<std::endl;
}

void QNode::Dxl_idCallback(const order_kuro::Rec_arm_id::ConstPtr &msg)
{
    dxl_id_msg.r_arm = msg->r_arm;
    dxl_id_msg.l_arm = msg->l_arm;

    Q_EMIT Rx_Dxl_id();
}

void QNode::wheel_mode_Callback(const msg_generate::dxl_wheel_msg::ConstPtr &msg)
{
    std::cout << "wheel_mode_callback1" << std::endl;

    wheel_msg.id = msg->id;
    wheel_msg.speed = msg->speed;

    Q_EMIT wheel_mode();

    std::cout << "wheel_mode_callback2" << std::endl;

}

}  // namespace dynamixel_serial
