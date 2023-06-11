#include "ros/ros.h"
#include "cmath"

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <fstream>
#include <string>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>

#include <skeleton_message/Users.h>
#include <skeleton_message/UserData.h>
#include <skeleton_message/SkeletonJoint.h>

#include "spline.hpp"
#include "kinematics.hpp"

#include "order.h"
#include "angle.h"
#include "joint_state.h"
#include "loca_order.h"
#include "fk_loca.h"
#include "motion_name.h"
#include <msg_generate/Arm_path_position_msg.h>
#include <msg_generate/Arm_path_coordinate_msg.h>
#include <msg_generate/elbow_updown_msg.h>

#define JOINT_MOVE 0
#define LOCATION_MOVE 1

#define INIT_POS 2048

ros::Subscriber order_sub;
ros::Subscriber loca_order_sub;
ros::Subscriber skeleton_Sub;
ros::Subscriber dxl_pos_sub;
ros::Subscriber dxl_id_sub;
ros::Subscriber motion_name_sub;
ros::Subscriber elbow_updown_sub;

solve kinematics;

struct pos{
    double pX = 0;
    double pY = 0;
    double pZ = 0;
};

struct rot
{
    double rX = 0;
    double rY = 0;
    double rZ = 0;
};

struct order_parameter
{
    double r_ang[7];
    double l_ang[7];
};

struct loca_parameter
{
    double r_pX;
    double r_pY;
    double r_pZ;
    double r_el_dir;

    double l_pX;
    double l_pY;
    double l_pZ;
    double l_el_dir;
};

struct target_pos{

    double r_pX;
    double r_pY;
    double r_pZ;
    double r_el_dir;
    double r_wrist_ang;

    double l_pX;
    double l_pY;
    double l_pZ;
    double l_el_dir;
    double l_wrist_ang;
};

struct location{

    double r_pX;
    double r_pY;
    double r_pZ;

    double r_rX;
    double r_rY;
    double r_rZ;


    double l_pX;
    double l_pY;
    double l_pZ;

    double l_rX;
    double l_rY;
    double l_rZ;
};

struct current_dxl_position{

    int r_dxl_pos[5];
    int l_dxl_pos[5];
};

//struct connect_motion_data{

//    int pX;
//    int pY;
//    int pZ;
//    int el_dir;
//    int wrist_ang;
//};

order_parameter order_param;
loca_parameter location_param;

target_pos target;
location location;
current_dxl_position current_dxl_angle;

int motion_connect_data[10];

int cnt = 0;
int move_type;
static double real_t = 2.0;

double now_r_px = 0.0;
double now_r_py = 0.0;
double now_r_pz = -519.0;
double now_r_el_dir = 0.0;

double now_l_px = 0.0;
double now_l_py = 0.0;
double now_l_pz = -519.0;
double now_l_el_dir = 0.0;

double r_elbow = 0.0;
double l_elbow = 0.0;

int Goal_flag = 0;
int sibal = 0;

int g_DXL_ID_position[10] = {0,};

string motion_path = "/home/robit/catkin_ws/src/order_kuro/work/";
string now_motion_name;
string before_motion_name;
string temp_motion_name;

vector<string> motion_name_data;
vector<target_pos> now_motion_data;

string motion_name_list;

string ik_info_path = "/home/robit/catkin_ws/src/move_DualArm/work/";
string now_ik_info_file;
string before_ik_info_file;

string now_motion_file;
string before_motion_file;

int unused_data;

int before_motion_flag = 0;
int motion_file_flag = 0;
int R_connect_flag = 0;
int L_connect_flag = 0;
int exis_flag = 0;

int now_motion_size = 0;
int now_motion_speed = 0;

int before_motion_size = 0;
int before_motion_speed = 0;

int slow_cnt = 2;

int R_id[5] = {1,3,5,7,9};
int L_id[5] = {0,2,4,6,8};

kuro_test::angle gazebo_info;
move_DualArm::joint_state joint_info;
move_DualArm::fk_loca fk_info;
msg_generate::Arm_path_position_msg dxl_pos_info;
msg_generate::Arm_path_coordinate_msg moving_path_info;
msg_generate::Motion_msg Motion_Info;
msg_generate::elbow_updown_msg  elbow_updown_info;

spline R_Arm_x;
spline R_Arm_y;
spline R_Arm_z;
spline R_Arm_el_dir;

spline L_Arm_x;
spline L_Arm_y;
spline L_Arm_z;
spline L_Arm_el_dir;

int elbow_updown_value = 1;

void angle_order_callback(const order_kuro::order::ConstPtr& msg);
void loca_order_callback(const order_kuro::loca_order::ConstPtr& msg);
void play_order_callback(const order_kuro::motion_name::ConstPtr& msg);
void dxl_pos_callback(const msg_generate::Arm_path_position_msg::ConstPtr& msg);
void gazebo_simulation();
void motionData();
void elbow_updown_callback(const msg_generate::elbow_updown_msg::ConstPtr& msg);

void move_kuro_arm(double t);
void motor_packet();

void skeleton_Callback(const skeleton_message::Users &msg);
double pos2ang(double pos);

float joint[45];
int r_id[] = {1,3,5,7,9};
int l_id[] = {0,2,4,6,8};



