#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/order_kuro/qnode.hpp"

namespace order_kuro {


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

void QNode::joint_stateCallback(const move_DualArm::joint_state::ConstPtr &msg)
{
    jointinfo.l_sh_pitch  = msg->l_sh_pitch ;
    jointinfo.l_sh_roll   = msg->l_sh_roll  ;
    jointinfo.l_sh_yaw    = msg->l_sh_yaw   ;
    jointinfo.l_el_pitch  = msg->l_el_pitch ;
    jointinfo.l_wri_roll  = msg->l_wri_roll ;
    jointinfo.l_wri_pitch = msg->l_wri_pitch;
    jointinfo.l_wri_roll  = msg->l_wri_roll ;

    jointinfo.r_sh_pitch  = msg->r_sh_pitch ;
    jointinfo.r_sh_roll   = msg->r_sh_roll  ;
    jointinfo.r_sh_yaw    = msg->r_sh_yaw   ;
    jointinfo.r_el_pitch  = msg->r_el_pitch ;
    jointinfo.r_wri_roll  = msg->r_wri_roll ;
    jointinfo.r_wri_pitch = msg->r_wri_pitch;
    jointinfo.r_wri_roll  = msg->r_wri_roll ;

    Q_EMIT js_callback();
}

void QNode::fk_location_Callback(const move_DualArm::fk_loca::ConstPtr &msg)
{
    fk_info.l_fk_pX = msg->l_fk_pX;
    fk_info.l_fk_pY = msg->l_fk_pY;
    fk_info.l_fk_pZ = msg->l_fk_pZ;

    fk_info.r_fk_pX = msg->r_fk_pX;
    fk_info.r_fk_pY = msg->r_fk_pY;
    fk_info.r_fk_pZ = msg->r_fk_pZ;

    Q_EMIT fk_callback();
}

void QNode::moving_path_coordinate_Callback(const msg_generate::Arm_path_coordinate_msg::ConstPtr &msg)
{
    for(int i = 0; i < 3; i++)
    {
        moving_path_info.r_arm_coordinate.push_back(msg->r_arm_coordinate[i]);
        moving_path_info.l_arm_coordinate.push_back(msg->l_arm_coordinate[i]);
    }

    //if(moving_path_info)

    moving_path_info.r_elbow_dir = msg->r_elbow_dir;
    moving_path_info.l_elbow_dir = msg->l_elbow_dir;

    moving_path_info.r_yaw_ang = msg->r_yaw_ang;
    moving_path_info.l_yaw_ang = msg->l_yaw_ang;

    Q_EMIT moving_path_callback();

    cout << "l_elbow_dir >> " << moving_path_info.l_elbow_dir << endl;
    cout << "l_wrist_ang >> " << moving_path_info.l_yaw_ang << endl;

    moving_path_info.r_arm_coordinate.clear();
    moving_path_info.l_arm_coordinate.clear();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"order_kuro");

    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    order_pub = n.advertise<order_kuro::order>("order",100);
    loca_order_pub = n.advertise<order_kuro::loca_order>("loca_order",100);
    loca_play_pub = n.advertise<order_kuro::motion_name>("motion_name",100);
    dxl_id_pub = n.advertise<order_kuro::Rec_arm_id>("dxl_id",100);
    motion_name_pub = n.advertise<order_kuro::motion_name>("motion_name",100);
    elbow_updown_pub = n.advertise<msg_generate::elbow_updown_msg>("elbow_updown",100);

    state_sub = n.subscribe("joint_state",100, &QNode::joint_stateCallback, this);
    fk_sub = n.subscribe("fk_location",100, &QNode::fk_location_Callback,this);
    moving_path_coor_sub = n.subscribe("moving_path_coor", 100, &QNode::moving_path_coordinate_Callback, this);


    start();
    return true;
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

}  // namespace order_kuro
