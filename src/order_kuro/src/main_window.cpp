/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <QString>
#include <fstream>

#include "../include/order_kuro/qnode.hpp"
#include "../include/order_kuro/main_window.hpp"
#include "../include/order_kuro/joint_state.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace order_kuro {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(js_callback()), this, SLOT(joint_state_callback()));
    QObject::connect(&qnode, SIGNAL(fk_callback()), this, SLOT(fk_location_callback()));
    QObject::connect(&qnode, SIGNAL(moving_path_callback()), this, SLOT(moving_path_coordinate_callback()));
    qnode.init();
}

MainWindow::~MainWindow() {}

//..R_Angle_Text.......................................................................
void order_kuro::MainWindow::on_r_sh_roll_edit_textChanged(const QString &arg1)
{
    ui.r_sh_roll_hori->setValue(arg1.toInt());
    order_info.r_ang_1 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_r_sh_pitch_edit_textChanged(const QString &arg1)
{
    ui.r_sh_pitch_hori->setValue(arg1.toInt());
    order_info.r_ang_2 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_r_sh_yaw_edit_textChanged(const QString &arg1)
{
    ui.r_sh_yaw_hori->setValue(arg1.toInt());
    order_info.r_ang_3 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_r_el_pitch_edit_textChanged(const QString &arg1)
{
    ui.r_el_pitch_hori->setValue(arg1.toInt());
    order_info.r_ang_4 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_r_wri_yaw_edit_textChanged(const QString &arg1)
{
    ui.r_wri_yaw_hori->setValue(arg1.toInt());
    order_info.r_ang_7 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}


//..R_Angle_Horizon_Bar..........................................................
void order_kuro::MainWindow::on_r_sh_roll_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.r_sh_roll_edit->setText(position);

    order_info.r_ang_1 = value;
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_r_sh_pitch_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.r_sh_pitch_edit->setText(position);

    order_info.r_ang_2 = value;
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_r_sh_yaw_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.r_sh_yaw_edit->setText(position);

    order_info.r_ang_3 = value;
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_r_el_pitch_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.r_el_pitch_edit->setText(position);

    order_info.r_ang_4 = value;
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_r_wri_yaw_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.r_wri_yaw_edit->setText(position);

    order_info.r_ang_7 = value;
    qnode.order_pub.publish(order_info);
}

//..R_set_value.........................................................
void order_kuro::MainWindow::on_r_sh_yaw_set_clicked()
{
    ui.r_sh_yaw_edit ->setText("0");
    ui.r_sh_yaw_hori ->setValue(0);
}

void order_kuro::MainWindow::on_r_sh_roll_set_clicked()
{
    ui.r_sh_roll_edit ->setText("0");
    ui.r_sh_roll_hori ->setValue(0);
}

void order_kuro::MainWindow::on_r_sh_pitch_set_clicked()
{
    ui.r_sh_pitch_edit ->setText("0");
    ui.r_sh_pitch_hori->setValue(0);
}

void order_kuro::MainWindow::on_r_el_pitch_set_clicked()
{
    ui.r_el_pitch_edit ->setText("0");
    ui.r_el_pitch_hori ->setValue(0);
}

void order_kuro::MainWindow::on_r_wri_yaw_set_clicked()
{
    ui.r_wri_yaw_edit ->setText("0");
    ui.r_wri_yaw_hori ->setValue(0);
}


//..L_Angle_Text.................................................................
void order_kuro::MainWindow::on_l_sh_roll_edit_textChanged(const QString &arg1)
{
    ui.l_sh_roll_hori->setValue(arg1.toInt());
    order_info.l_ang_1 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_l_sh_pitch_edit_textChanged(const QString &arg1)
{
    ui.l_sh_pitch_hori->setValue(arg1.toInt());
    order_info.l_ang_2 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_l_sh_yaw_edit_textChanged(const QString &arg1)
{
    ui.l_sh_yaw_hori->setValue(arg1.toInt());
    order_info.l_ang_3 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_l_el_pitch_edit_textChanged(const QString &arg1)
{
    ui.l_el_pitch_hori->setValue(arg1.toInt());
    order_info.l_ang_4 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}

void order_kuro::MainWindow::on_l_wri_yaw_edit_textChanged(const QString &arg1)
{
    ui.l_wri_yaw_hori->setValue(arg1.toInt());
    order_info.l_ang_7 = arg1.toFloat();
    qnode.order_pub.publish(order_info);
}


//..L_Angle_horizon_Bar..................................................
void order_kuro::MainWindow::on_l_sh_roll_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.l_sh_roll_edit->setText(position);

    order_info.l_ang_1 = value;
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_l_sh_pitch_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.l_sh_pitch_edit->setText(position);

    order_info.l_ang_2 = value;
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_l_sh_yaw_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.l_sh_yaw_edit->setText(position);

    order_info.l_ang_3 = value;
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_l_el_pitch_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.l_el_pitch_edit->setText(position);

    order_info.l_ang_4 = value;
    qnode.order_pub.publish(order_info);
}
void order_kuro::MainWindow::on_l_wri_yaw_hori_valueChanged(int value)
{
    QString position = QString::number(value);
    ui.l_wri_yaw_edit->setText(position);

    order_info.l_ang_7 = value;
    qnode.order_pub.publish(order_info);
}

//..L_set_value...........................................................
void order_kuro::MainWindow::on_l_sh_yaw_set_clicked()
{
    ui.l_sh_yaw_edit ->setText("0");
    ui.l_sh_yaw_hori ->setValue(0);
}

void order_kuro::MainWindow::on_l_sh_roll_set_clicked()
{
    ui.l_sh_roll_edit ->setText("0");
    ui.l_sh_roll_hori ->setValue(0);
}

void order_kuro::MainWindow::on_l_sh_pitch_set_clicked()
{
    ui.l_sh_pitch_edit ->setText("0");
    ui.l_sh_pitch_hori ->setValue(0);
}

void order_kuro::MainWindow::on_l_el_pitch_set_clicked()
{
    ui.l_el_pitch_edit ->setText("0");
    ui.l_el_pitch_hori ->setValue(0);
}

void order_kuro::MainWindow::on_l_wri_yaw_set_clicked()
{
    ui.l_wri_yaw_edit ->setText("0");
    ui.l_wri_yaw_hori ->setValue(0);
}

//..Location................................................................
void MainWindow::fk_location_callback()
{
    fk_info.l_fk_pX = qnode.fk_info.l_fk_pX;
    fk_info.l_fk_pY = qnode.fk_info.l_fk_pY;
    fk_info.l_fk_pZ = qnode.fk_info.l_fk_pZ;

    fk_info.r_fk_pX = qnode.fk_info.r_fk_pX;
    fk_info.r_fk_pY = qnode.fk_info.r_fk_pY;
    fk_info.r_fk_pZ = qnode.fk_info.r_fk_pZ;

    ui.l_fk_px->setText(QString::number(fk_info.l_fk_pX));
    ui.l_fk_py->setText(QString::number(fk_info.l_fk_pY));
    ui.l_fk_pz->setText(QString::number(fk_info.l_fk_pZ));

    ui.r_fk_px->setText(QString::number(fk_info.r_fk_pX));
    ui.r_fk_py->setText(QString::number(fk_info.r_fk_pY));
    ui.r_fk_pz->setText(QString::number(fk_info.r_fk_pZ));
}
void order_kuro::MainWindow::on_r_fk_px_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_fk_py_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_fk_pz_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_fk_px_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_fk_py_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_fk_pz_textChanged(const QString &arg1)
{
}

void order_kuro::MainWindow::on_r_reset_clicked()
{
    ui.r_sh_yaw_edit   ->setText("0");
    ui.r_sh_roll_edit  ->setText("0");
    ui.r_sh_pitch_edit ->setText("0");
    ui.r_el_pitch_edit ->setText("0");
    ui.r_wri_yaw_edit  ->setText("0");

    ui.r_sh_roll_hori  ->setValue(0);
    ui.r_sh_pitch_hori ->setValue(0);
    ui.r_sh_yaw_hori   ->setValue(0);
    ui.r_el_pitch_hori ->setValue(0);
    ui.r_wri_yaw_hori  ->setValue(0);
}

void order_kuro::MainWindow::on_l_reset_clicked()
{
    ui.l_sh_yaw_edit   ->setText("0");
    ui.l_sh_roll_edit  ->setText("0");
    ui.l_sh_pitch_edit ->setText("0");
    ui.l_el_pitch_edit ->setText("0");
    ui.l_wri_yaw_edit  ->setText("0");

    ui.l_sh_roll_hori  ->setValue(0);
    ui.l_sh_pitch_hori ->setValue(0);
    ui.l_sh_yaw_hori   ->setValue(0);
    ui.l_el_pitch_hori ->setValue(0);
    ui.l_wri_yaw_hori  ->setValue(0);
}


//..Location_Order............................................................
void order_kuro::MainWindow::on_r_x_coordinate_textChanged(const QString &arg1)
{
    loca_info.r_pX = arg1.toFloat();
}
void order_kuro::MainWindow::on_r_x_increase_clicked()
{
    loca_info.r_pX += 10;
    ui.r_x_coordinate->setText(QString::number(loca_info.r_pX));
}
void order_kuro::MainWindow::on_r_x_decrease_clicked()
{
    loca_info.r_pX -= 10;
    ui.r_x_coordinate->setText(QString::number(loca_info.r_pX));
}

void order_kuro::MainWindow::on_r_y_coordinate_textChanged(const QString &arg1)
{
    loca_info.r_pY = arg1.toFloat();
}
void order_kuro::MainWindow::on_r_y_increase_clicked()
{
    loca_info.r_pY += 10;
    ui.r_y_coordinate->setText(QString::number(loca_info.r_pY));
}
void order_kuro::MainWindow::on_r_y_decrease_clicked()
{
    loca_info.r_pY -= 10;
    ui.r_y_coordinate->setText(QString::number(loca_info.r_pY));
}

void order_kuro::MainWindow::on_r_z_coordinate_textChanged(const QString &arg1)
{
    loca_info.r_pZ = arg1.toFloat();
}
void order_kuro::MainWindow::on_r_z_increase_clicked()
{
    loca_info.r_pZ += 10;
    ui.r_z_coordinate->setText(QString::number(loca_info.r_pZ));
}
void order_kuro::MainWindow::on_r_z_decrease_clicked()
{
    loca_info.r_pZ -= 10;
    ui.r_z_coordinate->setText(QString::number(loca_info.r_pZ));
}

//..L
void order_kuro::MainWindow::on_l_x_coordinate_textChanged(const QString &arg1)
{
    loca_info.l_pX = arg1.toFloat();
}
void order_kuro::MainWindow::on_l_x_increase_clicked()
{
    loca_info.l_pX += 10;
    ui.l_x_coordinate->setText(QString::number(loca_info.l_pX));
}
void order_kuro::MainWindow::on_l_x_decrease_clicked()
{
    loca_info.l_pX -= 10;
    ui.l_x_coordinate->setText(QString::number(loca_info.l_pX));
}

void order_kuro::MainWindow::on_l_y_coordinate_textChanged(const QString &arg1)
{
    loca_info.l_pY = arg1.toFloat();
}
void order_kuro::MainWindow::on_l_y_increase_clicked()
{
    loca_info.l_pY += 10;
    ui.l_y_coordinate->setText(QString::number(loca_info.l_pY));
}
void order_kuro::MainWindow::on_l_y_decrease_clicked()
{
    loca_info.l_pY -= 10;
    ui.l_y_coordinate->setText(QString::number(loca_info.l_pY));
}

void order_kuro::MainWindow::on_l_z_coordinate_textChanged(const QString &arg1)
{
    loca_info.l_pZ = arg1.toFloat();
}
void order_kuro::MainWindow::on_l_z_increase_clicked()
{
    loca_info.l_pZ += 10;
    ui.l_z_coordinate->setText(QString::number(loca_info.l_pZ));
}
void order_kuro::MainWindow::on_l_z_decrease_clicked()
{
    loca_info.l_pZ -= 10;
    ui.l_z_coordinate->setText(QString::number(loca_info.l_pZ));
}


void order_kuro::MainWindow::on_Location_move_clicked()
{
    qnode.loca_order_pub.publish(loca_info);
}

void order_kuro::MainWindow::on_r_insert_clicked()
{
    fk_info.r_fk_pX = qnode.fk_info.r_fk_pX;
    fk_info.r_fk_pY = qnode.fk_info.r_fk_pY;
    fk_info.r_fk_pZ = qnode.fk_info.r_fk_pZ;

    ui.r_x_coordinate->setText(QString::number(fk_info.r_fk_pX));
    ui.r_y_coordinate->setText(QString::number(fk_info.r_fk_pY));
    ui.r_z_coordinate->setText(QString::number(fk_info.r_fk_pZ));
 }

void order_kuro::MainWindow::on_l_insert_clicked()
{
    fk_info.l_fk_pX = qnode.fk_info.l_fk_pX;
    fk_info.l_fk_pY = qnode.fk_info.l_fk_pY;
    fk_info.l_fk_pZ = qnode.fk_info.l_fk_pZ;

    ui.l_x_coordinate->setText(QString::number(fk_info.l_fk_pX));
    ui.l_y_coordinate->setText(QString::number(fk_info.l_fk_pY));
    ui.l_z_coordinate->setText(QString::number(fk_info.l_fk_pZ));
 }

//..JOINT_STATE....................................................................
void MainWindow::joint_state_callback()
{
    joint_info.r_sh_roll   = qnode.jointinfo.r_sh_roll;
    joint_info.r_sh_pitch  = qnode.jointinfo.r_sh_pitch;
    joint_info.r_sh_yaw    = qnode.jointinfo.r_sh_yaw;
    joint_info.r_el_pitch  = qnode.jointinfo.r_el_pitch;
    joint_info.r_wri_yaw   = qnode.jointinfo.r_wri_yaw;

    joint_info.l_sh_roll   = qnode.jointinfo.l_sh_roll;
    joint_info.l_sh_pitch  = qnode.jointinfo.l_sh_pitch;
    joint_info.l_sh_yaw    = qnode.jointinfo.l_sh_yaw;
    joint_info.l_el_pitch  = qnode.jointinfo.l_el_pitch;
    joint_info.l_wri_yaw   = qnode.jointinfo.l_wri_yaw;

    ui.r_sh_roll_state->setText(QString::number(joint_info.r_sh_roll));
    ui.r_sh_pitch_state->setText(QString::number(joint_info.r_sh_pitch));
    ui.r_sh_yaw_state->setText(QString::number(joint_info.r_sh_yaw));
    ui.r_el_pitch_state->setText(QString::number(joint_info.r_el_pitch));
    ui.r_wri_yaw_state->setText(QString::number(joint_info.r_wri_yaw));

    ui.l_sh_roll_state->setText(QString::number(joint_info.l_sh_roll));
    ui.l_sh_pitch_state->setText(QString::number(joint_info.l_sh_pitch));
    ui.l_sh_yaw_state->setText(QString::number(joint_info.l_sh_yaw));
    ui.l_el_pitch_state->setText(QString::number(joint_info.l_el_pitch));
    ui.l_wri_yaw_state->setText(QString::number(joint_info.l_wri_yaw));
}


void order_kuro::MainWindow::on_r_sh_roll_state_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_sh_pitch_state_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_sh_yaw_state_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_el_pitch_state_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_wri_yaw_state_textChanged(const QString &arg1)
{
}

//..L..
void order_kuro::MainWindow::on_l_sh_roll_state_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_sh_pitch_state_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_sh_yaw_state_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_el_pitch_state_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_wri_yaw_state_textChanged(const QString &arg1)
{
}

//..Moving_Path_Recording...........................................................
void order_kuro::MainWindow::on_recording_clicked()
{
    R_moving_path_save.clear();
    L_moving_path_save.clear();

    if(Rec_flag == 0)
    {
        Recording_Arm = BOTH;
        ui.recording->setStyleSheet("background-color : red");
        Rec_flag = 1;

        t1 = new QTimer(this);
        connect(t1, SIGNAL(timeout()), this, SLOT(recording_start()));
        t1->start(ui.recording_period->text().toInt());

        rec_motion_exis_flag = 1;
    }
}

void MainWindow::recording_start()
{
    dxl_id_info.r_arm = 1;
    dxl_id_info.l_arm = 1;
    qnode.dxl_id_pub.publish(dxl_id_info);
}

void order_kuro::MainWindow::on_recording_stop_clicked()
{
    vect_empty_check = 1;

    dxl_id_info.r_arm = 0;
    dxl_id_info.l_arm = 0;
    qnode.dxl_id_pub.publish(dxl_id_info);

    if(Rec_flag)
    {
        ui.recording->setStyleSheet("background-color : white");
        t1->stop();
        delete t1;
        Rec_flag = 0;

        ofstream outFile;
        outFile.open("/home/robit/catkin_ws/src/order_kuro/work/temp_motion");

        outFile << R_moving_path_save.size() << endl
        << ui.recording_play_speed->text().toInt() << endl;

        for(int i = 0; i < R_moving_path_save.size(); i++)
        {
            outFile << R_moving_path_save[i].pX << endl
            << R_moving_path_save[i].pY << endl
            << R_moving_path_save[i].pZ << endl
            << R_moving_path_save[i].elbow_dir << endl;
        }
        for(int i = 0; i < L_moving_path_save.size(); i++)
        {
            outFile << L_moving_path_save[i].pX << endl
            << L_moving_path_save[i].pY << endl
            << L_moving_path_save[i].pZ << endl
            << L_moving_path_save[i].elbow_dir << endl;
        }
        outFile.close();
    }
    else
    {
        cout << "Do not Record!!" << endl;
    }
}

void order_kuro::MainWindow::on_recording_play_clicked()
{
    if(!Rec_flag && rec_motion_exis_flag)
    {
        ofstream outFile;
        outFile.open("/home/robit/catkin_ws/src/order_kuro/work/temp_motion");

        outFile << R_moving_path_save.size() << endl
        << ui.recording_play_speed->text().toInt() << endl;

        for(int i = 0; i < R_moving_path_save.size(); i++)
        {
            outFile << R_moving_path_save[i].pX << endl
            << R_moving_path_save[i].pY << endl
            << R_moving_path_save[i].pZ << endl
            << R_moving_path_save[i].elbow_dir << endl
            << R_moving_path_save[i].wrist_yaw << endl;
        }
        for(int i = 0; i < L_moving_path_save.size(); i++)
        {
            outFile << L_moving_path_save[i].pX << endl
            << L_moving_path_save[i].pY << endl
            << L_moving_path_save[i].pZ << endl
            << L_moving_path_save[i].elbow_dir << endl
            << L_moving_path_save[i].wrist_yaw << endl;
        }
        outFile.close();

        name_info.now_motion_name = "temp_motion";
        name_info.before_motion_name = " ";
        qnode.motion_name_pub.publish(name_info);
    }
    else
    {
        cout << "Do not Record!!" << endl;
    }
}

void MainWindow::moving_path_play()
{
    if(play_mode == RECORDING)
    {
        loca_info.r_pX = R_moving_path_save[path_count].pX;
        loca_info.r_pY = R_moving_path_save[path_count].pY;
        loca_info.r_pZ = R_moving_path_save[path_count].pZ;
        loca_info.r_elbow_dir = R_moving_path_save[path_count].elbow_dir;

        loca_info.l_pX = L_moving_path_save[path_count].pX;
        loca_info.l_pY = L_moving_path_save[path_count].pY;
        loca_info.l_pZ = L_moving_path_save[path_count].pZ;
        loca_info.l_elbow_dir = L_moving_path_save[path_count].elbow_dir;

        loca_info.max_step = R_moving_path_save.size();
        loca_info.motion_time = ui.recording_play_speed->text().toFloat();

        ui.r_path_px->setText(QString::number(R_moving_path_save[path_count].pX));
        ui.r_path_py->setText(QString::number(R_moving_path_save[path_count].pY));
        ui.r_path_pz->setText(QString::number(R_moving_path_save[path_count].pY));

        ui.l_path_px->setText(QString::number(L_moving_path_save[path_count].pX));
        ui.l_path_py->setText(QString::number(L_moving_path_save[path_count].pY));
        ui.l_path_pz->setText(QString::number(L_moving_path_save[path_count].pZ));

        qnode.loca_play_pub.publish(loca_info);

        if(path_count >= R_moving_path_save.size())
        {
            path_t->stop();
            delete path_t;
            play_flag = 0;
        }

        path_count ++;
    }
    else if(play_mode == PLAY)
    {
        loca_info.r_pX = R_moving_path_play[path_count].pX;
        loca_info.r_pY = R_moving_path_play[path_count].pY;
        loca_info.r_pZ = R_moving_path_play[path_count].pZ;
        loca_info.r_elbow_dir = R_moving_path_play[path_count].elbow_dir;

        loca_info.l_pX = L_moving_path_play[path_count].pX;
        loca_info.l_pY = L_moving_path_play[path_count].pY;
        loca_info.l_pZ = L_moving_path_play[path_count].pZ;
        loca_info.l_elbow_dir = L_moving_path_play[path_count].elbow_dir;

        loca_info.max_step = read_file_size;
        loca_info.motion_time = ui.play_speed->text().toFloat();

        qnode.loca_play_pub.publish(loca_info);

        if(path_count >= R_moving_path_play.size())
        {
            path_t->stop();
            delete path_t;
            play_flag = 0;
        }

        ui.r_path_play_px->setText(QString::number(R_moving_path_play[path_count].pX));
        ui.r_path_play_py->setText(QString::number(R_moving_path_play[path_count].pY));
        ui.r_path_play_pz->setText(QString::number(R_moving_path_play[path_count].pZ));

        ui.l_path_play_px->setText(QString::number(L_moving_path_play[path_count].pX));
        ui.l_path_play_py->setText(QString::number(L_moving_path_play[path_count].pY));
        ui.l_path_play_pz->setText(QString::number(L_moving_path_play[path_count].pZ));

        path_count ++;
        cout << path_count << endl;
        cout << R_moving_path_play.size() - 1 << endl << endl;
    }

}

void order_kuro::MainWindow::on_recording_save_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save file"), "/home/robit/catkin_ws/src/order_kuro/work/");

    if(fileName.isEmpty() == true)
        qDebug() << "Save Cancel";
    else
    {
        QFile *file = new QFile;
        file->setFileName(fileName);
        file->open(QIODevice::WriteOnly);
        QTextStream out(file);

        moving_path_size = R_moving_path_save.size();

        out << moving_path_size << endl
        << ui.recording_play_speed->text().toInt() << endl;

        for(int i = 0 ; i < moving_path_size; i++)
        {
            out << R_moving_path_save[i].pX << endl
            << R_moving_path_save[i].pY << endl
            << R_moving_path_save[i].pZ << endl
            << R_moving_path_save[i].elbow_dir << endl
            << R_moving_path_save[i].wrist_yaw << endl;
        }

        for(int i = 0 ; i < moving_path_size; i++)
        {
            out << L_moving_path_save[i].pX << endl
            << L_moving_path_save[i].pY << endl
            << L_moving_path_save[i].pZ << endl
            << L_moving_path_save[i].elbow_dir << endl
            << L_moving_path_save[i].wrist_yaw << endl;
        }

//        out << ",";

        file->close();
    }
}


//..Setting............
void order_kuro::MainWindow::on_recording_play_speed_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_recording_period_textChanged(const QString &arg1)
{
}

//..Location...........
void MainWindow::moving_path_coordinate_callback()  //moving path coordinate info save
{
    for(int i = 0; i < 3; i++)
    {
        moving_path_info.r_arm_coordinate.push_back(qnode.moving_path_info.r_arm_coordinate[i]);
        moving_path_info.l_arm_coordinate.push_back(qnode.moving_path_info.l_arm_coordinate[i]);
    }

    moving_path_info.r_elbow_dir = qnode.moving_path_info.r_elbow_dir;
    moving_path_info.l_elbow_dir = qnode.moving_path_info.l_elbow_dir;

    moving_path_info.r_yaw_ang = qnode.moving_path_info.r_yaw_ang;
    moving_path_info.l_yaw_ang = qnode.moving_path_info.l_yaw_ang;

    ui.r_path_px->setText(QString::number((moving_path_info.r_arm_coordinate[0])));
    ui.r_path_py->setText(QString::number((moving_path_info.r_arm_coordinate[1])));
    ui.r_path_pz->setText(QString::number((moving_path_info.r_arm_coordinate[2])));

    ui.l_path_px->setText(QString::number((moving_path_info.l_arm_coordinate[0])));
    ui.l_path_py->setText(QString::number((moving_path_info.l_arm_coordinate[1])));
    ui.l_path_pz->setText(QString::number((moving_path_info.l_arm_coordinate[2])));

    R_path_coor.pX = moving_path_info.r_arm_coordinate[0];
    R_path_coor.pY = moving_path_info.r_arm_coordinate[1];
    R_path_coor.pZ = moving_path_info.r_arm_coordinate[2];
    R_path_coor.elbow_dir = moving_path_info.r_elbow_dir;
    R_path_coor.wrist_yaw = moving_path_info.r_yaw_ang;

    L_path_coor.pX = moving_path_info.l_arm_coordinate[0];
    L_path_coor.pY = moving_path_info.l_arm_coordinate[1];
    L_path_coor.pZ = moving_path_info.l_arm_coordinate[2];
    L_path_coor.elbow_dir = moving_path_info.l_elbow_dir;
    L_path_coor.wrist_yaw = moving_path_info.l_yaw_ang;

    R_moving_path_save.push_back(R_path_coor);
    L_moving_path_save.push_back(L_path_coor);

    moving_path_info.r_arm_coordinate.clear();
    moving_path_info.l_arm_coordinate.clear();
}

void order_kuro::MainWindow::on_r_path_px_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_path_py_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_path_pz_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_path_px_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_path_py_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_path_pz_textChanged(const QString &arg1)
{
}

//Path_Play...................................................................
void order_kuro::MainWindow::on_play_save_clicked()
{
}

void order_kuro::MainWindow::on_play_open_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open file"), "/home/robit/catkin_ws/src/order_kuro/work/");

    if(fileName.isEmpty() == true)
        qDebug() << "Open Cancel";
    else
    {
        now_motion_name = fileName.toStdString();
        now_motion_name = now_motion_name.substr(now_motion_name.find_last_of("/")+1);
        cout << now_motion_name << endl;

        R_moving_path_play.clear();
        L_moving_path_play.clear();

        play_open_flag = 1;
        ui.play_file_name->setText(fileName);

        ifstream is;

        is.open(fileName.toStdString().c_str());

        is >> read_file_size;
        is >> read_play_speed;

        R_moving_path_play.resize(read_file_size);
        L_moving_path_play.resize(read_file_size);

        ui.play_speed->setText(QString::number(read_play_speed));

        for(int i = 0 ; i < read_file_size; i++)
        {
            is >> R_moving_path_play[i].pX;
            is >> R_moving_path_play[i].pY;
            is >> R_moving_path_play[i].pZ;
            is >> R_moving_path_play[i].elbow_dir;
            is >> R_moving_path_play[i].wrist_yaw;
        }

        for(int i = 0 ; i < read_file_size; i++)
        {
            is >> L_moving_path_play[i].pX;
            is >> L_moving_path_play[i].pY;
            is >> L_moving_path_play[i].pZ;
            is >> L_moving_path_play[i].elbow_dir;
            is >> L_moving_path_play[i].wrist_yaw;
        }

        is.close();
    }
}

void order_kuro::MainWindow::on_path_play_clicked()
{
    if(play_open_flag)
    {
        if(read_play_speed != ui.play_speed->text().toInt())
        {
            ofstream outFile;
            outFile.open("/home/robit/catkin_ws/src/order_kuro/work/" + now_motion_name);

            outFile << read_file_size << endl
            << ui.play_speed->text().toInt() << endl;

            for(int i = 0 ; i < read_file_size; i++)
            {
                outFile << R_moving_path_play[i].pX << endl
                << R_moving_path_play[i].pY << endl
                << R_moving_path_play[i].pZ << endl
                << R_moving_path_play[i].elbow_dir << endl
                << R_moving_path_play[i].wrist_yaw << endl;
            }

            for(int i = 0 ; i < read_file_size; i++)
            {
                outFile << L_moving_path_play[i].pX << endl
                << L_moving_path_play[i].pY << endl
                << L_moving_path_play[i].pZ << endl
                << L_moving_path_play[i].elbow_dir << endl
                << L_moving_path_play[i].wrist_yaw << endl;
            }

            outFile.close();
        }
        name_info.now_motion_name = now_motion_name;
        name_info.before_motion_name = " ";
        qnode.motion_name_pub.publish(name_info);
    }
    else
    {
        cout << "Do not open File !!"  << endl;
    }
}

void order_kuro::MainWindow::on_play_file_name_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_play_repeat_count_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_play_speed_textChanged(const QString &arg1)
{
}

//..coordinate.........
void order_kuro::MainWindow::on_r_path_play_px_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_path_play_py_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_r_path_play_pz_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_path_play_px_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_path_play_py_textChanged(const QString &arg1)
{
}
void order_kuro::MainWindow::on_l_path_play_pz_textChanged(const QString &arg1)
{
}

void order_kuro::MainWindow::on_elbow_up_clicked()
{
    elbow_updown_info.elbow_updown = -1;
    qnode.elbow_updown_pub.publish(elbow_updown_info);
}
void order_kuro::MainWindow::on_elbow_down_clicked()
{
    elbow_updown_info.elbow_updown = 1;
    qnode.elbow_updown_pub.publish(elbow_updown_info);
}

/*****************************************************************************
** Functions
*****************************************************************************/


}  // namespace order_kuro




