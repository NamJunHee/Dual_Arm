#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <iostream>
#include <fstream>
#include <cmath>
#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include <limits>
#include <cstring>
#include <assert.h>
#include <msg_generate/Motor_msg.h>
#include <msg_generate/Motion_msg.h>
#include <msg_generate/Step_msg.h>

#define PI 3.141592653589793
#define rad2deg (double)180.0/PI
#define deg2rad PI/(double)180

#define HAND_END 121
#define COTTON_STICK_END 299

using namespace std;
using namespace Eigen;

class solve
{
public:
    ros::Publisher gazebo_pub;
    ros::Publisher joint_state;
    ros::Publisher fk_pub;
    ros::Publisher dynamixel_pub;
    ros::Publisher moveing_path_coor_pub;
    ros::Publisher play_motion_pub;
    ros::Publisher elbow_dir_pub;

    double ang2pos(double angle);
    double pos2ang(double pos);
    double to360(double deg);
    double to180(double deg);

    void init_save();
    void update_motorangle();
    void motor_packet(int speed);

    void R_fk_solve(double th1, double th2, double th3, double th4, double th5);
    void L_fk_solve(double th1, double th2, double th3, double th4, double th5);

    int R_ik_solve(double pX, double pY, double pZ, double rX, double rY, double rZ, double el_dir);
    int L_ik_solve(double pX, double pY, double pZ, double rX, double rY, double rZ, double el_dir);

    void angle_dxl_position(double th_l1, double th_l2, double th_l3, double th_l4, double th_l5, double th_r1,double th_r2,double th_r3, double th_r4, double th_r5);

    struct dynamicxel_packet{
        vector<int> dynamicxel_id;
        vector<int> dynamicxel_position;
        vector<int> dynamicxel_speed;
        int dynamicxel_mode;
    };


    int g_DXL_ID_position[30] = {0,};
    // Arm Link Length (mm)
    double L3 = 200; //upper_arm
    double L5 = 117+COTTON_STICK_END; //under_arm + Endeffector

    double shoulder_x = 0;
    double shoulder_y = 0;
    double shoulder_z = 0;

    Matrix4d A1, A2, A3, A4, A5, A6, A7;

    double r_th[7]  = {0.0,};
    double r_th_[7] = {0.0,};

    double l_th[7]  = {0.0,};
    double l_th_[7] = {0.0,};

    double r_fk_pX = 0;
    double r_fk_pY = 0;
    double r_fk_pZ = 0;

    double l_fk_pX = 0;
    double l_fk_pY = 0;
    double l_fk_pZ = 0;

private:

    int g_DXL_ID_Save_position[30] = {0,};

    int position[23] = {0,};


    //angle_limit
    int Ang_Limit_max[7] = {0, 0, 0, 0, 0, 0};
    int Ang_limit_min[7] = {0, 0, 0, 0, 0, 0};
};
#endif /* KINEMATICS_HPP_ */

