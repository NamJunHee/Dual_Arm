#include "../include/move_DualArm/move_DualArm.hpp"
#include "../include/move_DualArm/kinematics.hpp"

using namespace std;

double ang2pos(double angle)
{
    return (double)((angle * 4096.0) / 360.0);
}

double pos2ang(double pos)
{
    return (double)((pos*360.0) / 4096.0) - 180.0;
}

void timer_callback(const ros::TimerEvent&)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_DualArm");
    ros::NodeHandle n;

    //..Timer.......................
    ros::Timer timer = n.createTimer(ros::Duration(1), timer_callback);

    //..msg.............................................................................
    //sub
    order_sub = n.subscribe("order",100,angle_order_callback);
    loca_order_sub = n.subscribe("loca_order",100,loca_order_callback);
    skeleton_Sub = n.subscribe("Skeleton_User",100,skeleton_Callback);
    dxl_pos_sub = n.subscribe("dxl_position",100,dxl_pos_callback);
    motion_name_sub = n.subscribe("motion_name", 100, play_order_callback);
    elbow_updown_sub = n.subscribe("elbow_updown",100,elbow_updown_callback);

    //pub
    kinematics.gazebo_pub = n.advertise<kuro_test::angle>("angle",100);
    kinematics.joint_state = n.advertise<move_DualArm::joint_state>("joint_state",100);
    kinematics.fk_pub = n.advertise<move_DualArm::fk_loca>("fk_location",100);
    kinematics.dynamixel_pub = n.advertise<msg_generate::Motor_msg>("Dynamixel", 100);
    kinematics.moveing_path_coor_pub = n.advertise<msg_generate::Arm_path_coordinate_msg>("moving_path_coor",100);
    kinematics.play_motion_pub = n.advertise<msg_generate::Motion_msg>("Motion", 100);
    //..................................................................................

    //..init.....................
    target.r_pX = 0;
    target.r_pY = 0;
    target.r_pZ = -(kinematics.L3 + kinematics.L5 - 5); //initial setting Z
    target.r_el_dir = 0;

    target.l_pX = 0;
    target.l_pY = 0;
    target.l_pZ = -(kinematics.L3 + kinematics.L5 - 5); //initial setting Z
    target.l_el_dir = 0;

    //...........................

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}

void angle_order_callback(const order_kuro::order::ConstPtr& msg)
{
    order_param.r_ang[0] = msg->r_ang_1;
    order_param.r_ang[1] = msg->r_ang_2;
    order_param.r_ang[2] = msg->r_ang_3;
    order_param.r_ang[3] = msg->r_ang_4;
    order_param.r_ang[6] = msg->r_ang_7;

    order_param.l_ang[0] = msg->l_ang_1;
    order_param.l_ang[1] = msg->l_ang_2;
    order_param.l_ang[2] = msg->l_ang_3;
    order_param.l_ang[3] = msg->l_ang_4;
    order_param.l_ang[6] = msg->l_ang_7;

    kinematics.L_fk_solve(order_param.l_ang[2],order_param.l_ang[0],order_param.l_ang[1],order_param.l_ang[3],order_param.l_ang[6]);

    fk_info.l_fk_pX = kinematics.l_fk_pX;
    fk_info.l_fk_pY = kinematics.l_fk_pY;
    fk_info.l_fk_pZ = kinematics.l_fk_pZ;

    kinematics.R_fk_solve(order_param.r_ang[2],order_param.r_ang[0],order_param.r_ang[1],order_param.r_ang[3],order_param.l_ang[6]);

    fk_info.r_fk_pX = kinematics.r_fk_pX;
    fk_info.r_fk_pY = kinematics.r_fk_pY;
    fk_info.r_fk_pZ = kinematics.r_fk_pZ;

    kinematics.fk_pub.publish(fk_info);

    move_type = JOINT_MOVE;
    gazebo_simulation();

    r_elbow = order_param.r_ang[0];
    l_elbow = order_param.l_ang[0];
}

void loca_order_callback(const order_kuro::loca_order::ConstPtr& msg)
{
    location_param.r_pX = msg->r_pX;
    location_param.r_pY = msg->r_pY;
    location_param.r_pZ = msg->r_pZ;
    location_param.r_el_dir = r_elbow;

    location_param.l_pX = msg->l_pX;
    location_param.l_pY = msg->l_pY;
    location_param.l_pZ = msg->l_pZ;
    location_param.l_el_dir = l_elbow;

    kinematics.R_ik_solve(location_param.r_pX,location_param.r_pY, location_param.r_pZ,0,0,0,r_elbow);
    kinematics.L_ik_solve(location_param.l_pX,location_param.l_pY, location_param.l_pZ,0,0,0,l_elbow);

    joint_info.r_sh_roll   = kinematics.r_th[1];
    joint_info.r_sh_pitch  = kinematics.r_th[2];
    joint_info.r_sh_yaw    = kinematics.r_th[0];
    joint_info.r_el_pitch  = kinematics.r_th[3];
    joint_info.r_wri_roll  = kinematics.r_th[4];
    joint_info.r_wri_pitch = kinematics.r_th[5];
    joint_info.r_wri_yaw   = kinematics.r_th[6];

    joint_info.l_sh_roll   = kinematics.l_th[1];
    joint_info.l_sh_pitch  = kinematics.l_th[2];
    joint_info.l_sh_yaw    = kinematics.l_th[0];
    joint_info.l_el_pitch  = kinematics.l_th[3];
    joint_info.l_wri_roll  = kinematics.l_th[4];
    joint_info.l_wri_pitch = kinematics.l_th[5];
    joint_info.l_wri_yaw   = kinematics.l_th[6];

    kinematics.joint_state.publish(joint_info);

    move_type = LOCATION_MOVE;
    gazebo_simulation();

    kinematics.angle_dxl_position(kinematics.l_th[0],kinematics.l_th[1],kinematics.l_th[2],kinematics.l_th[3],kinematics.l_th[4],kinematics.r_th[0],kinematics.r_th[1],kinematics.r_th[2],kinematics.r_th[3],kinematics.r_th[4]);
}

void play_order_callback(const order_kuro::motion_name::ConstPtr& msg)
{
    fill_n(g_DXL_ID_position, 10, INIT_POS);

    before_motion_name = msg -> before_motion_name;
    now_motion_name = msg -> now_motion_name;

    cout << before_motion_name << endl;
    cout << now_motion_name << endl;

    motion_name_list = ik_info_path + "motion_name_list";

    before_ik_info_file = ik_info_path + before_motion_name;
    now_ik_info_file = ik_info_path + now_motion_name;

    before_motion_file = motion_path + before_motion_name;
    now_motion_file = motion_path + now_motion_name;

    ifstream inFile;
    inFile.open(now_motion_file);

    if(inFile.is_open() == true)
    {
        inFile.close();
        exis_flag = 0;

        if(!exis_flag)
        {
            if(now_motion_name != "temp_motion")
            {
                motion_name_data.push_back(now_motion_name);
                now_motion_data.clear();

                ofstream outFile;
                outFile.open(motion_name_list,ios::app);
                outFile << now_motion_name << endl;
                outFile.close();
            }

            ifstream inFile;
            inFile.open(now_motion_file);
            if(inFile.is_open() == true)
            {
                motion_file_flag = 1;

                inFile >> now_motion_size;
                inFile >> now_motion_speed;

                now_motion_data.resize(now_motion_size);

                for(int i = 0; i < now_motion_size; i++)
                {
                    inFile >> now_motion_data[i].r_pX;
                    inFile >> now_motion_data[i].r_pY;
                    inFile >> now_motion_data[i].r_pZ;
                    inFile >> now_motion_data[i].r_el_dir;
                    inFile >> now_motion_data[i].r_wrist_ang;
                }

                for(int i = 0; i < now_motion_size; i++)
                {
                    inFile >> now_motion_data[i].l_pX;
                    inFile >> now_motion_data[i].l_pY;
                    inFile >> now_motion_data[i].l_pZ;
                    inFile >> now_motion_data[i].l_el_dir;
                    inFile >> now_motion_data[i].l_wrist_ang;
                }
                inFile.close();

                inFile.open(before_ik_info_file);
                if(inFile.is_open())
                {
                    before_motion_flag = 1;

                    inFile >> before_motion_size;
                    inFile >> before_motion_speed;

                    for(int i = 0; i < before_motion_size - 1; i++)
                    {
                        for(int j = 0; j < 10; j++)
                            inFile >> unused_data;
                    }

                    inFile >> motion_connect_data[1];
                    inFile >> motion_connect_data[3];
                    inFile >> motion_connect_data[5];
                    inFile >> motion_connect_data[7];
                    inFile >> motion_connect_data[9];
                    inFile >> motion_connect_data[0];
                    inFile >> motion_connect_data[2];
                    inFile >> motion_connect_data[4];
                    inFile >> motion_connect_data[6];
                    inFile >> motion_connect_data[8];
                }
                else
                {
                    before_motion_flag = 0;
                    cout << "before_motion does not open!!" << endl;
                }
                inFile.close();

                ofstream outFile;
                outFile.open(now_ik_info_file);
                if(before_motion_flag)
                {
                    outFile << now_motion_size + 1 << endl;
                    outFile << now_motion_speed << endl;

                    outFile << motion_connect_data[1] << endl
                    << motion_connect_data[3] << endl
                    << motion_connect_data[5] << endl
                    << motion_connect_data[7] << endl
                    << motion_connect_data[9] << endl
                    << motion_connect_data[0] << endl
                    << motion_connect_data[2] << endl
                    << motion_connect_data[4] << endl
                    << motion_connect_data[6] << endl
                    << motion_connect_data[8] << endl;

                    before_motion_flag = 0;
                }
                else
                {
                    outFile << now_motion_size << endl;
                    outFile << now_motion_speed << endl;
                }

                for(int i = 0 ; i < now_motion_size; i++)
                {
                    kinematics.R_ik_solve(now_motion_data[i].r_pX,now_motion_data[i].r_pY,now_motion_data[i].r_pZ,0,0,0,now_motion_data[i].r_el_dir);

                    g_DXL_ID_position[1] += ang2pos(kinematics.r_th[0]);
                    g_DXL_ID_position[3] += ang2pos(kinematics.r_th[1]);
                    g_DXL_ID_position[5] += ang2pos(kinematics.r_th[2]);
                    g_DXL_ID_position[7] += ang2pos(kinematics.r_th[3]);
                    g_DXL_ID_position[9] += ang2pos(now_motion_data[i].r_wrist_ang);

                    outFile << g_DXL_ID_position[1] << endl
                    << g_DXL_ID_position[3] << endl
                    << g_DXL_ID_position[5] << endl
                    << g_DXL_ID_position[7] << endl
                    << g_DXL_ID_position[9] << endl;

                    kinematics.L_ik_solve(now_motion_data[i].l_pX,now_motion_data[i].l_pY,now_motion_data[i].l_pZ,0,0,0,now_motion_data[i].l_el_dir);

                    g_DXL_ID_position[0] += ang2pos(kinematics.l_th[0]);
                    g_DXL_ID_position[2] += ang2pos(kinematics.l_th[1]);
                    g_DXL_ID_position[4] += ang2pos(kinematics.l_th[2]);
                    g_DXL_ID_position[6] += ang2pos(kinematics.l_th[3]);
                    g_DXL_ID_position[8] += ang2pos(now_motion_data[i].l_wrist_ang);

                    outFile << g_DXL_ID_position[0] << endl
                    << g_DXL_ID_position[2] << endl
                    << g_DXL_ID_position[4] << endl
                    << g_DXL_ID_position[6] << endl
                    << g_DXL_ID_position[8] << endl;

                    fill_n(g_DXL_ID_position, 10, INIT_POS);
                }
                outFile.close();
            }
            else
            {
                motion_file_flag = 0;
                cout << "Motion_file does not open!!" << endl;
                cout << "Check the file name or file path.." << endl;
            }
        }
        else
        {
            motion_file_flag = 1;
        }

        if(motion_file_flag)
        {
            msg_generate::Step_msg Step_Info;

//            now_motion_file = motion_path + now_motion_name;

//            inFile.open(now_motion_file);
//            inFile >> now_motion_size;
//            inFile >> now_motion_speed;
//            inFile.close();

            inFile.open(now_ik_info_file);
            inFile >> Motion_Info.max_step;
            inFile >> Step_Info.time;

            Motion_Info.repeat = 1;
            Step_Info.delay = 0;

            for(int i = 0 ; i < Motion_Info.max_step; i++)
            {
                slow_cnt++;

                inFile >> g_DXL_ID_position[1];
                inFile >> g_DXL_ID_position[3];
                inFile >> g_DXL_ID_position[5];
                inFile >> g_DXL_ID_position[7];
                inFile >> g_DXL_ID_position[9];

                for(int j = 0; j < 5; j++)
                {
                    Step_Info.id.push_back(r_id[j]);
                    Step_Info.position.push_back(g_DXL_ID_position[r_id[j]]);
                }

                inFile >> g_DXL_ID_position[0];
                inFile >> g_DXL_ID_position[2];
                inFile >> g_DXL_ID_position[4];
                inFile >> g_DXL_ID_position[6];
                inFile >> g_DXL_ID_position[8];

                for(int j = 0; j < 5; j++)
                {
                    Step_Info.id.push_back(l_id[j]);
                    Step_Info.position.push_back(g_DXL_ID_position[l_id[j]]);
                }

                Step_Info.time = now_motion_speed;

                Motion_Info.motion_data.push_back(Step_Info);
                Step_Info.id.clear();
                Step_Info.position.clear();
            }
            inFile.close();

            kinematics.play_motion_pub.publish(Motion_Info);

            Motion_Info.max_step = 0;
            Motion_Info.repeat = 0;
            Motion_Info.motion_data.clear();
            Step_Info.position.clear();
            slow_cnt = 0;
        }
        else
        {
            cout << "motion_file does not open!!" << endl;
            cout << "Check the file name or file path.." << endl;
        }
    }
    else
    {
        cout << "\"" << now_motion_name << "\"" << " file does not exist" << endl;
    }

    cout << endl;
}

void dxl_pos_callback(const msg_generate::Arm_path_position_msg::ConstPtr& msg)
{
    for(int i = 0 ; i < 5; i ++)
    {
        current_dxl_angle.r_dxl_pos[i] = pos2ang(msg->r_dxl_position[i]);
        current_dxl_angle.l_dxl_pos[i] = pos2ang(msg->l_dxl_position[i]);
    }

    kinematics.R_fk_solve(current_dxl_angle.r_dxl_pos[0],
                          current_dxl_angle.r_dxl_pos[1],
                          current_dxl_angle.r_dxl_pos[2],
                          current_dxl_angle.r_dxl_pos[3],
                          current_dxl_angle.r_dxl_pos[4]);

    kinematics.L_fk_solve(current_dxl_angle.l_dxl_pos[0],
                          current_dxl_angle.l_dxl_pos[1],
                          current_dxl_angle.l_dxl_pos[2],
                          current_dxl_angle.l_dxl_pos[3],
                          current_dxl_angle.l_dxl_pos[4]);

    if(abs(kinematics.r_fk_pX) < 200 & abs(kinematics.r_fk_pY) < 50 & kinematics.r_fk_pZ < -540)
    {
        kinematics.r_fk_pX = 207;
        kinematics.r_fk_pY = 9;
        kinematics.r_fk_pZ = -556;
    }
    if(abs(kinematics.l_fk_pX) < 200 & abs(kinematics.l_fk_pY) < 50 & kinematics.l_fk_pZ < -540)
    {
        kinematics.l_fk_pX = 207;
        kinematics.l_fk_pY = 9;
        kinematics.l_fk_pZ = -556;
    }

    kinematics.R_ik_solve(kinematics.r_fk_pX, kinematics.r_fk_pY,kinematics.r_fk_pZ,0,0,0,current_dxl_angle.r_dxl_pos[1]);
    kinematics.L_ik_solve(kinematics.l_fk_pX, kinematics.l_fk_pY,kinematics.l_fk_pZ,0,0,0,current_dxl_angle.l_dxl_pos[1]);

    moving_path_info.r_arm_coordinate.push_back(kinematics.r_fk_pX);
    moving_path_info.r_arm_coordinate.push_back(kinematics.r_fk_pY);
    moving_path_info.r_arm_coordinate.push_back(kinematics.r_fk_pZ);

    moving_path_info.r_elbow_dir = current_dxl_angle.r_dxl_pos[1];
    moving_path_info.r_yaw_ang = current_dxl_angle.r_dxl_pos[4];

    moving_path_info.l_arm_coordinate.push_back(kinematics.l_fk_pX);
    moving_path_info.l_arm_coordinate.push_back(kinematics.l_fk_pY);
    moving_path_info.l_arm_coordinate.push_back(kinematics.l_fk_pZ);
    moving_path_info.l_elbow_dir = current_dxl_angle.l_dxl_pos[1];
    moving_path_info.l_yaw_ang = current_dxl_angle.l_dxl_pos[4];

    kinematics.moveing_path_coor_pub.publish(moving_path_info);

    moving_path_info.r_arm_coordinate.clear();
    moving_path_info.l_arm_coordinate.clear();
}

void elbow_updown_callback(const msg_generate::elbow_updown_msg::ConstPtr& msg)
{
    elbow_updown_value = msg->elbow_updown;
}

void gazebo_simulation()
{
    if(move_type == JOINT_MOVE)
    {
        gazebo_info.r_ang_1 = order_param.r_ang[0];
        gazebo_info.r_ang_2 = order_param.r_ang[1];
        gazebo_info.r_ang_3 = order_param.r_ang[2];
        gazebo_info.r_ang_4 = order_param.r_ang[3];
        gazebo_info.r_ang_5 = order_param.r_ang[4];
        gazebo_info.r_ang_6 = order_param.r_ang[5];
        gazebo_info.r_ang_7 = order_param.r_ang[6];

        gazebo_info.l_ang_1 = order_param.l_ang[0];
        gazebo_info.l_ang_2 = order_param.l_ang[1];
        gazebo_info.l_ang_3 = order_param.l_ang[2];
        gazebo_info.l_ang_4 = order_param.l_ang[3];
        gazebo_info.l_ang_5 = order_param.l_ang[4];
        gazebo_info.l_ang_6 = order_param.l_ang[5];
        gazebo_info.l_ang_7 = order_param.l_ang[6];
    }
    else if(move_type == LOCATION_MOVE)
    {
        gazebo_info.r_ang_1 = kinematics.r_th[1];
        gazebo_info.r_ang_2 = kinematics.r_th[2];
        gazebo_info.r_ang_3 = kinematics.r_th[0];
        gazebo_info.r_ang_4 = kinematics.r_th[3];
        gazebo_info.r_ang_5 = kinematics.r_th[4];
        gazebo_info.r_ang_6 = kinematics.r_th[5];
        gazebo_info.r_ang_7 = kinematics.r_th[6];

        gazebo_info.l_ang_1 = kinematics.l_th[1];
        gazebo_info.l_ang_2 = kinematics.l_th[2];
        gazebo_info.l_ang_3 = kinematics.l_th[0];
        gazebo_info.l_ang_4 = kinematics.l_th[3];
        gazebo_info.l_ang_5 = kinematics.l_th[4];
        gazebo_info.l_ang_6 = kinematics.l_th[5];
        gazebo_info.l_ang_7 = kinematics.l_th[6];
    }

    kinematics.gazebo_pub.publish(gazebo_info);
}

void skeleton_Callback(const skeleton_message::Users &msg)
{
    //GLWidget skeleton;
    //  cout<<msg<<endl;
    //    cout<<"------------------------"<<endl;

    skeleton_message::Users test =msg;
    if(test.Users.size()>0){
        for(int i = 0; i< test.Users.size(); i ++){
            if(test.Users[i].UserID == 1){
                cout<<"UserStateSTATEATEATEATE========================="<<test.Users[i].UserState<<endl;
                if(test.Users[i].UserState == 1){

                    //                    cout<<"xXXX"<<test.Users[i].skeletonJoints[8].location.x<<"  YYYY"<<test.Users[i].skeletonJoints[8].location.y<<"  ZZZZ"<<test.Users[i].skeletonJoints[8].location.z<<endl;

                    for(int k = 0; k < 45; k+=3){

                        joint[k] = test.Users[i].skeletonJoints[k/3].location.x;
                        joint[k+1] = test.Users[i].skeletonJoints[k/3].location.y;
                        joint[k+2] = test.Users[i].skeletonJoints[k/3].location.z;
                    }
                }
            }
        }
    }
    int i = 0;

    //    for(int k = 0; k < 45; k+=3){

    //        cout<<"Joint "<<i<<":  x    "<<joint[k]<<"     ";
    //        cout<<"Joint "<<i<<":  y    "<<joint[k+1]<<"     ";
    //        cout<<"Joint "<<i<<":  z    "<<joint[k+2]<<endl;
    //        i++;
    //    }

    kinematics.L_ik_solve(joint[21],joint[22],joint[23],0,0,0,joint[0]);
    kinematics.R_ik_solve(joint[18],joint[19],joint[20],0,0,0,joint[1]);

    kinematics.angle_dxl_position(kinematics.l_th[0],kinematics.l_th[1],kinematics.l_th[2],kinematics.l_th[3],kinematics.l_th[4],kinematics.r_th[0],kinematics.r_th[1],kinematics.r_th[2],kinematics.r_th[3],kinematics.r_th[4]);

//    cout << "joint_pX >> " << joint[18] << endl;
//    cout << "joint_pY >> " << joint[19] << endl;
//    cout << "joint_pZ >> " << joint[20] << endl;

    move_type = LOCATION_MOVE;
    if(test.Users.size()>0){
        for(int i = 0; i< test.Users.size(); i ++){
            if(test.Users[i].UserID == 1){
                if(test.Users[i].UserState == 1){
                    gazebo_simulation();
                }
            }
        }
    }
}

//void move_kuro_arm(double real_t)
//{
//    if(real_t <= 1.0)
//    {
//        R_Arm_x.put_point(0.0, now_r_px, 0.0, 0.0, 0.0);
//        R_Arm_x.put_point(1.0, target.r_pX, 0.0, 0.0, 0.0);

//        R_Arm_y.put_point(0.0, now_r_py, 0.0, 0.0, 0.0);
//        R_Arm_y.put_point(1.0, target.r_pY, 0.0, 0.0, 0.0);

//        R_Arm_z.put_point(0.0, now_r_pz, 0.0, 0.0, 0.0);
//        R_Arm_z.put_point(1.0, target.r_pZ, 0.0, 0.0, 0.0);

//        R_Arm_el_dir.put_point(0.0, now_r_el_dir, 0.0, 0.0, 0.0);
//        R_Arm_el_dir.put_point(1.0, target.r_el_dir, 0.0, 0.0, 0.0);

//        L_Arm_x.put_point(0.0, now_l_px, 0.0, 0.0, 0.0);
//        L_Arm_x.put_point(1.0, target.l_pX, 0.0, 0.0, 0.0);

//        L_Arm_y.put_point(0.0, now_l_py, 0.0, 0.0, 0.0);
//        L_Arm_y.put_point(1.0, target.l_pY, 0.0, 0.0, 0.0);

//        L_Arm_z.put_point(0.0, now_l_pz, 0.0, 0.0, 0.0);
//        L_Arm_z.put_point(1.0, target.l_pZ, 0.0, 0.0, 0.0);

//        L_Arm_el_dir.put_point(0.0, now_l_el_dir, 0.0, 0.0, 0.0);
//        L_Arm_el_dir.put_point(1.0, target.l_el_dir, 0.0, 0.0, 0.0);

//        kinematics.R_ik_solve(R_Arm_x.result(real_t),R_Arm_y.result(real_t), R_Arm_z.result(real_t),0,0,0,R_Arm_el_dir.result(real_t));
//        kinematics.L_ik_solve(L_Arm_x.result(real_t),L_Arm_y.result(real_t), L_Arm_z.result(real_t),0,0,0,L_Arm_el_dir.result(real_t));

//        move_type = LOCATION_MOVE;
//        //gazebo_simulation();

//        kinematics.angle_dxl_position(kinematics.l_th[0],kinematics.l_th[1],kinematics.l_th[2],kinematics.l_th[3],kinematics.l_th[4],kinematics.r_th[0],kinematics.r_th[1],kinematics.r_th[2],kinematics.r_th[3],kinematics.r_th[4]);
//    }

//    if(real_t > 1.0)
//    {
//        now_r_px = target.r_pX;
//        now_r_py = target.r_pY;
//        now_r_pz = target.r_pZ;
//        now_r_el_dir = target.r_el_dir;

//        now_l_px = target.l_pX;
//        now_l_py = target.l_pY;
//        now_l_pz = target.l_pZ;
//        now_l_el_dir = target.l_el_dir;

//        Goal_flag = 1;
//    }
//}

