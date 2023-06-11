#include "../include/move_DualArm/kinematics.hpp"
#include "../include/move_DualArm/move_DualArm.hpp"
#include <Eigen/LU>

//extern ros::Publisher kinematics;

double solve::ang2pos(double angle)
{
    return (double)((angle * 4096.0) / 360.0);
}

double solve::pos2ang(double pos)
{
    return(double)((pos*360.0) / 4096.0);
}

double solve::to360(double deg)
{
    if(deg < 0) deg+=360;
    else if(deg > 360) deg -= 360;
}

void solve::init_save()
{
    ifstream is;

    is.open("/home/robit/ros2_ws/src/move_DualArm/init/init");

    for(int i = 0; i < 23; i++)
        is >> position[i];

    is.close();

    for(int DXL_ID = 0; DXL_ID < 16; DXL_ID++)
    {
        g_DXL_ID_position[DXL_ID] = position[DXL_ID] * 4;
        g_DXL_ID_Save_position[DXL_ID] = position[DXL_ID] * 4;

        cout<<"g_DXL_ID_Save_position["<<DXL_ID<<"] = "<<g_DXL_ID_Save_position[DXL_ID]<<endl;
    }
}

void solve::R_fk_solve(double th1, double th2, double th3, double th4, double th5)
{
    Matrix4d A1, A2, A3, A4, A5;
    Matrix4d T;

    double th_[7] = {0,};

    th_[0] = th1 * deg2rad;
    th_[1] = th2 * deg2rad;
    th_[2] = th3 * deg2rad;
    th_[3] = th4 * deg2rad;
    th_[4] = th5 * deg2rad;

    A1 << cos(-th_[0]), -sin(-th_[0]), 0, 0,
          sin(-th_[0]),  cos(-th_[0]), 0, 0,
                0,             0,      1, 0,
                0,             0,      0, 1;

    A2 << 1,      0,            0,      0,
          0, cos(th_[1]), -sin(th_[1]), 0,
          0, sin(th_[1]),  cos(th_[1]), 0,
          0,      0,            0,      1;

    A3 << cos(-th_[2]), 0, sin(-th_[2]), 0,
               0,       1,      0,       0,
         -sin(-th_[2]), 0, cos(-th_[2]), 0,
               0,       0,      0,       1;

    A4 << cos(-th_[3]), 0, sin(-th_[3]), 0,
               0,       1,      0,       0,
         -sin(-th_[3]), 0, cos(-th_[3]), -L3,
               0,       0,      0,       1;

    A5 << cos(-th_[4]), -sin(-th_[4]), 0, 0,
          sin(-th_[4]),  cos(-th_[4]), 0, 0,
                0,             0,      1, -L5,
                0,             0,      0, 1;

    T = A1 * A2 * A3 * A4 * A5;

    r_fk_pX = T(0,3);
    r_fk_pY = T(1,3);
    r_fk_pZ = T(2,3);
}

void solve::L_fk_solve(double th1, double th2, double th3, double th4, double th5)
{
    Matrix4d A1, A2, A3, A4, A5;
    Matrix4d T;

    double th_[7] = {0,};

    th_[0] = th1 * deg2rad;
    th_[1] = th2 * deg2rad;
    th_[2] = th3 * deg2rad;
    th_[3] = th4 * deg2rad;
    th_[4] = th5 * deg2rad;

    A1 << cos(-th_[0]), -sin(-th_[0]), 0, 0,
          sin(-th_[0]),  cos(-th_[0]), 0, 0,
                0,             0,      1, 0,
                0,             0,      0, 1;

    A2 << 1,      0,            0,      0,
          0, cos(th_[1]), -sin(th_[1]), 0,
          0, sin(th_[1]),  cos(th_[1]), 0,
          0,      0,            0,      1;

    A3 << cos(th_[2]), 0, sin(th_[2]), 0,
               0,      1,      0,      0,
         -sin(th_[2]), 0, cos(th_[2]), 0,
               0,      0,      0,      1;

    A4 << cos(th_[3]), 0, sin(th_[3]), 0,
               0,      1,      0,      0,
         -sin(th_[3]), 0, cos(th_[3]), -L3,
               0,      0,      0,      1;

    A5 << cos(-th_[4]), -sin(-th_[4]), 0, 0,
          sin(-th_[4]),  cos(-th_[4]), 0, 0,
                0,             0,      1, -L5,
                0,             0,      0, 1;

    T = A1 * A2 * A3 * A4 * A5;

    l_fk_pX = T(0,3);
    l_fk_pY = T(1,3);
    l_fk_pZ = T(2,3);
}


int solve::R_ik_solve(double pX, double pY, double pZ, double rX, double rY, double rZ,double el_dir)
{
    Matrix3d Rz;
    Vector3d P,PxRz, per_P;

    float C1, S1;
    float C2, S2;
    float r, a, dX, dY;

    float theta1, alpha, beta;
    float sol;

    int den, num;

    P << pX,
         pY,
         pZ;

    if(abs(el_dir) > 90)
    {
        if(el_dir > 0)
            el_dir = 90;
        else if(el_dir < 0)
            el_dir = -90;
    }

    //cout << "el_dir >> " << el_dir << endl << endl;
    //cout << "pZ >> " << abs(pZ) << endl << endl;

    r = abs(pZ) / cos(el_dir * deg2rad);
    r = abs(r);
    //cout << "r >> " << r << endl<< endl;

    a = abs(pZ) * tan(el_dir * deg2rad);
    a = abs(a);
    //cout << "a >> " << a << endl<< endl;

    theta1 = atan2(pY,pX);
    //cout << "theta1 >> " << theta1 * rad2deg<< endl<< endl;

    alpha = acos(a/sqrt(pow(pX,2)+pow(pY,2)));

    num = a *10;
    den = sqrt(pow(pX,2)+pow(pY,2)) * 10;

    //cout << num << endl;
    //cout << den << endl;

    if(num == den)
    {
        alpha = 0;
    }

    if(pX == 0 && pY == 0)
        alpha = 0;

    //cout << "alpha >> " << alpha << endl<< endl;

    beta = (90 * deg2rad) - alpha;

    //cout << "beta >> " << beta * rad2deg<< endl<< endl;

    if(pZ <= 0)
    {
        if(el_dir >= 0)
            sol = theta1 - beta;
        else if(el_dir < 0)
            sol = theta1 + beta;
    }
    else if(pZ >0)
    {
        if(el_dir >= 0)
            sol = theta1 + beta;
        else if(el_dir < 0)
            sol = theta1 - beta;
    }

    //cout << "sol >> " << sol * rad2deg<< endl<< endl;


    dX = a * cos((90 * deg2rad)-sol);

    //cout << "dX >> " << dX << endl;

    dY = a * sin((90 * deg2rad)-sol);

    //cout << "dY >> " << dY << endl<< endl;

    per_P[0] = (double)(P[0] - dX);
    per_P[1] = (double)(P[1] + dY);

    if(abs(el_dir) <= 90)
    {
        if(pZ <= 0)
        {
            per_P[2] = (double)-r;
        }
        else if(pZ > 0)
        {
            per_P[2] = (double)r;
        }
    }
    else if(abs(el_dir) > 90)
    {
        if(pZ <= 0)
        {
            per_P[2] = (double)r;
        }
        else if(pZ > 0)
        {
            per_P[2] = (double)-r;
        }
    }

    //cout << "P" << endl;
    //cout << P << endl << endl;

    if(el_dir != 0)
        r_th_[0] = -sol;
    else if(el_dir == 0)
        r_th_[0] = -atan2(pY, pX);

    if(alpha == 0 && pX != 0 && pY != 0)
    {
        r_th_[0] = -(atan2(pY, pX)-(90 * deg2rad));

        per_P[0] = 0;
        per_P[1] = 0;
    }

    //cout << "per_P" << endl;
    //cout << per_P << endl << endl;

    per_P[0] = int(per_P[0]);
    per_P[1] = int(per_P[1]);
    per_P[2] = int(per_P[2]);

    if(abs(per_P[0] * 100) < 1)
        per_P[0] = 0;
    if(abs(per_P[1] * 100) < 1)
        per_P[1] = 0;
    if(abs(per_P[2] * 100) < 1)
        per_P[2] = 0;

    if(!(abs(per_P[0]) <= L3+L5))
        per_P[0] = 0;
    if(!(abs(per_P[1]) <= L3+L5))
        per_P[1] = 0;
    if(!(abs(per_P[2]) <= L3+L5))
        per_P[2] = 0;

    per_P[0] = int(per_P[0]);
    per_P[1] = int(per_P[1]);
    per_P[2] = int(per_P[2]);


    //cout << "per_P" << endl;
    //cout << per_P << endl << endl;

    Rz << cos(r_th_[0]), -sin(r_th_[0]), 0,
          sin(r_th_[0]),  cos(r_th_[0]), 0,
               0,             0,       1;

    PxRz = Rz * per_P;

    C2 = (pow(PxRz[2],2)+pow(PxRz[0],2)-(pow(L3,2)+pow(L5,2)))/(2*L3*L5);
    S2 = sqrt(1-pow(C2,2));

    r_th_[3] = atan2(S2,C2);

    PxRz[2] = (-1)*PxRz[2];
    C1 = (((L3 + L5*C2)*PxRz[2])+(L5*S2)*PxRz[0])/(pow(L3+L5*C2,2)+pow(L5*S2,2));
    S1 = (-((L5*S2)*PxRz[2])+(L3+L5*C2)*PxRz[0])/(pow(L3+L5*C2,2)+pow(L5*S2,2));

    r_th_[2] = atan2(S1, C1);
    r_th_[1] = el_dir * deg2rad;

    for(int i = 0; i < 7; i++)
    {
        r_th[i] = r_th_[i]*rad2deg;
        //cout << "r_th[" << i << "]" << r_th[i] << endl;
    }
    //cout << endl;
}

int solve::L_ik_solve(double pX, double pY, double pZ, double rX, double rY, double rZ,double el_dir)
{
    Matrix3d Rz;
    Vector3d P, P_Rz_Rx,per_P;

    double C1, S1;
    double C2, S2;
    double r, a, dX, dY;

    double theta1, alpha, beta;
    double sol;

    int den, num;

    P << pX,
         pY,
         pZ;

    if(abs(el_dir) > 90)
    {
        if(el_dir > 0)
            el_dir = 90;
        else if(el_dir < 0)
            el_dir = -90;
    }

    r = abs(pZ) / cos(el_dir * deg2rad);
    r = abs(r);

    a = abs(pZ) * tan(el_dir * deg2rad);
    a = abs(a);

    theta1 = atan2(pY,pX);
    alpha = acos(a/sqrt(pow(pX,2)+pow(pY,2)));

    num = a * 10;
    den = sqrt(pow(pX,2)+pow(pY,2)) * 10;

    if(num == den)
        alpha = 0;

    if(pX == 0 && pY == 0)
        alpha = 0;

    beta = (90 * deg2rad) - alpha;

    if(pZ <= 0)
    {
        if(el_dir >= 0)
            sol = theta1 - beta;
        else if(el_dir < 0)
            sol = theta1 + beta;
    }
    else if(pZ >0)
    {
        if(el_dir >= 0)
            sol = theta1 + beta;
        else if(el_dir < 0)
            sol = theta1 - beta;
    }

    dX = a * cos((90 * deg2rad)-sol);
    dY = a * sin((90 * deg2rad)-sol);

    per_P[0] = (double)(P[0] + dX);
    per_P[1] = (double)(P[1] - dY);

    if(abs(el_dir) <= 90)
    {
        if(pZ <= 0)
        {
            per_P[2] = (double)-r;
        }
        else if(pZ > 0)
        {
            per_P[2] = (double)r;
        }
    }
    else if(abs(el_dir) > 90)
    {
        if(pZ <= 0)
        {
            per_P[2] = (double)r;
        }
        else if(pZ > 0)
        {
            per_P[2] = (double)-r;
        }
    }

    if(el_dir != 0)
        l_th_[0] = -sol;
    else if(el_dir == 0)
        l_th_[0] = -atan2(pY, pX);

    if(alpha == 0 && pX != 0 && pY != 0)
    {
        l_th_[0] = -(atan2(pY, pX)-(90 * deg2rad));

        per_P[0] = 0;
        per_P[1] = 0;
    }

    per_P[0] = int(per_P[0]);
    per_P[1] = int(per_P[1]);
    per_P[2] = int(per_P[2]);

    if(abs(per_P[0] * 100) < 1)
        per_P[0] = 0;
    if(abs(per_P[1] * 100) < 1)
        per_P[1] = 0;
    if(abs(per_P[2] * 100) < 1)
        per_P[2] = 0;

    if(!(abs(per_P[0]) <= L3+L5))
        per_P[0] = 0;
    if(!(abs(per_P[1]) <= L3+L5))
        per_P[1] = 0;
    if(!(abs(per_P[2]) <= L3+L5))
        per_P[2] = 0;

    per_P[0] = int(per_P[0]);
    per_P[1] = int(per_P[1]);
    per_P[2] = int(per_P[2]);

    Rz << cos(l_th_[0]), -sin(l_th_[0]), 0,
          sin(l_th_[0]),  cos(l_th_[0]), 0,
               0,             0,       1;

    P_Rz_Rx = Rz * per_P;

    C2 = ((pow(P_Rz_Rx[2],2)+pow(P_Rz_Rx[0],2)-(pow(L3,2)+pow(L5,2)))/(2*L3*L5));
    S2 = elbow_updown_value * sqrt(1-pow(C2,2)); //elbow_updown_value  + -> under, - -> upper

    l_th_[3] = -atan2(S2,C2);

    P_Rz_Rx[2] = (-1)*P_Rz_Rx[2];
    C1 = (((L3 + L5*C2)*P_Rz_Rx[2])+(L5*S2)*P_Rz_Rx[0])/(pow(L3+L5*C2,2)+pow(L5*S2,2));
    S1 = (-((L5*S2)*P_Rz_Rx[2])+(L3+L5*C2)*P_Rz_Rx[0])/(pow(L3+L5*C2,2)+pow(L5*S2,2));

    l_th_[2] = -atan2(S1, C1);
    l_th_[1] = el_dir * deg2rad;

    for(int i = 0; i < 7; i++)
    {
        l_th[i] = l_th_[i]*rad2deg;
//       cout << "l_th[" << i << "]" << l_th[i] << endl;
    }
//    cout << endl;
}


void solve::angle_dxl_position(double th_l1, double th_l2, double th_l3, double th_l4, double th_l5, double th_r1, double th_r2, double th_r3, double th_r4, double th_r5)
{
    int init_position=2048;
    g_DXL_ID_position[1]=init_position;
    g_DXL_ID_position[3]=init_position;
    g_DXL_ID_position[5]=init_position;
    g_DXL_ID_position[7]=init_position;
    g_DXL_ID_position[9]=init_position;

    g_DXL_ID_position[0]=init_position;
    g_DXL_ID_position[2]=init_position;
    g_DXL_ID_position[4]=init_position;
    g_DXL_ID_position[6]=init_position;
    g_DXL_ID_position[8]=init_position;

    //Right
    g_DXL_ID_position[1] += ang2pos(th_r1);
    g_DXL_ID_position[3] += ang2pos(th_r2);
    g_DXL_ID_position[5] += ang2pos(th_r3);
    g_DXL_ID_position[7] += ang2pos(th_r4);
    g_DXL_ID_position[9] += ang2pos(th_r5);
    //Left
    g_DXL_ID_position[0] += ang2pos(th_l1);
    g_DXL_ID_position[2] += ang2pos(th_l2);
    g_DXL_ID_position[4] += ang2pos(th_l3);
    g_DXL_ID_position[6] += ang2pos(th_l4);
    g_DXL_ID_position[8] += ang2pos(th_l5);

    msg_generate::Motor_msg dxl_info;

    for(int i =0;i<=9;i++)
    {
        dxl_info.id.push_back(i);
        dxl_info.mode = 3;
        int pos = g_DXL_ID_position[i];
        dxl_info.position.push_back(pos);
        dxl_info.speed.push_back(120);
    }

     dynamixel_pub.publish(dxl_info);
}


