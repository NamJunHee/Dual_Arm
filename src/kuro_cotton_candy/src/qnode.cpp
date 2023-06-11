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
#include "../include/kuro_cotton_candy/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kuro_cotton_candy {

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
    ros::init(init_argc,init_argv,"kuro_cotton_candy");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    motion_name_pub = n.advertise<kuro_cotton_candy::motion_name>("motion_name",100);
    elbow_updown_pub = n.advertise<msg_generate::elbow_updown_msg>("elbow_updown",100);
    wheel_mode_pub = n.advertise<msg_generate::dxl_wheel_msg>("wheel_mode", 100);

    motion_end_sub = n.subscribe("motion_end",100, &QNode::motion_end_callback, this);

    start();
    return true;
}

void QNode::motion_end_callback(const msg_generate::motion_end::ConstPtr &msg)
{
    motion_end_info.motion_end = msg->motion_end;

    Q_EMIT motion_end_callback();
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


}  // namespace kuro_cotton_candy
