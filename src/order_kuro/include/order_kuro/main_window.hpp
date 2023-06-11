/**
 * @file /include/order_kuro/main_window.hpp
 *
 * @brief Qt based gui for order_kuro.
 *
 * @date November 2010
 **/
#ifndef order_kuro_MAIN_WINDOW_H
#define order_kuro_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <string>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "joint_state.h"
#include "Rec_arm_id.h"
//#include "motion_name.h"

#define STOP      0
#define RIGHT     1
#define LEFT      2
#define BOTH      3

#define RECORDING 4
#define PLAY      5

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace order_kuro {

using namespace std;

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
        MainWindow(int argc, char** argv, QWidget *parent = 0);
        ~MainWindow();

        order_kuro::order order_info;
        order_kuro::loca_order loca_info;
        order_kuro::Rec_arm_id dxl_id_info;
        order_kuro::motion_name name_info;

        move_DualArm::joint_state joint_info;
        move_DualArm::fk_loca fk_info;
        msg_generate::Arm_path_coordinate_msg moving_path_info;
        msg_generate::elbow_updown_msg elbow_updown_info;

        int recording_state = STOP;
        int r_recording_state = STOP;
        int l_recording_state = STOP;

        int Recording_Arm;
        int Rec_flag  = 0;
        int play_flag = 0;
        int vect_empty_check = 0;

        struct path_coor{
            int pX;
            int pY;
            int pZ;
            int elbow_dir;
            int wrist_yaw;
        };

        vector<path_coor> R_moving_path_save;
        vector<path_coor> L_moving_path_save;

        path_coor L_path_coor;
        path_coor R_path_coor;

        int path_count = 0;

        vector<path_coor> R_moving_path_play;
        vector<path_coor> L_moving_path_play;

        int read_file_size;
        int read_play_speed;

        int moving_path_size;

        int play_mode;
        int play_open_flag = 0;

        string now_file_name;
        string now_motion_name;
        string next_motion_name;

        int rec_motion_exis_flag = 0;

public Q_SLOTS:
        void recording_start();

private Q_SLOTS:

        void on_r_sh_roll_edit_textChanged(const QString &arg1);

        void on_r_sh_pitch_edit_textChanged(const QString &arg1);

        void on_r_sh_yaw_edit_textChanged(const QString &arg1);

        void on_r_el_pitch_edit_textChanged(const QString &arg1);

        void on_r_wri_yaw_edit_textChanged(const QString &arg1);


        void on_r_sh_roll_hori_valueChanged(int value);

        void on_r_sh_pitch_hori_valueChanged(int value);

        void on_r_sh_yaw_hori_valueChanged(int value);

        void on_r_el_pitch_hori_valueChanged(int value);

        void on_r_wri_yaw_hori_valueChanged(int value);


        void on_r_sh_yaw_set_clicked();

        void on_r_sh_roll_set_clicked();

        void on_r_sh_pitch_set_clicked();

        void on_r_el_pitch_set_clicked();

        void on_r_wri_yaw_set_clicked();


        void on_l_sh_roll_edit_textChanged(const QString &arg1);

        void on_l_sh_pitch_edit_textChanged(const QString &arg1);

        void on_l_sh_yaw_edit_textChanged(const QString &arg1);

        void on_l_el_pitch_edit_textChanged(const QString &arg1);

        void on_l_wri_yaw_edit_textChanged(const QString &arg1);


        void on_l_sh_roll_hori_valueChanged(int value);

        void on_l_sh_pitch_hori_valueChanged(int value);

        void on_l_sh_yaw_hori_valueChanged(int value);

        void on_l_el_pitch_hori_valueChanged(int value);

        void on_l_wri_yaw_hori_valueChanged(int value);


        void on_l_sh_yaw_set_clicked();

        void on_l_sh_roll_set_clicked();

        void on_l_sh_pitch_set_clicked();

        void on_l_el_pitch_set_clicked();

        void on_l_wri_yaw_set_clicked();


        void fk_location_callback();


        void on_r_fk_px_textChanged(const QString &arg1);

        void on_r_fk_py_textChanged(const QString &arg1);

        void on_r_fk_pz_textChanged(const QString &arg1);


        void on_l_fk_px_textChanged(const QString &arg1);

        void on_l_fk_py_textChanged(const QString &arg1);

        void on_l_fk_pz_textChanged(const QString &arg1);


        void on_r_x_coordinate_textChanged(const QString &arg1);

        void on_r_x_increase_clicked();

        void on_r_x_decrease_clicked();

        void on_r_y_coordinate_textChanged(const QString &arg1);

        void on_r_y_increase_clicked();

        void on_r_y_decrease_clicked();


        void on_r_z_coordinate_textChanged(const QString &arg1);

        void on_r_z_increase_clicked();

        void on_r_z_decrease_clicked();



        void on_l_x_coordinate_textChanged(const QString &arg1);

        void on_l_x_increase_clicked();

        void on_l_x_decrease_clicked();


        void on_l_y_coordinate_textChanged(const QString &arg1);

        void on_l_y_increase_clicked();

        void on_l_y_decrease_clicked();


        void on_l_z_coordinate_textChanged(const QString &arg1);

        void on_l_z_increase_clicked();

        void on_l_z_decrease_clicked();


        void on_r_insert_clicked();

        void on_l_insert_clicked();

        void on_Location_move_clicked();

        void joint_state_callback();


        void on_r_sh_roll_state_textChanged(const QString &arg1);

        void on_r_sh_pitch_state_textChanged(const QString &arg1);

        void on_r_sh_yaw_state_textChanged(const QString &arg1);

        void on_r_el_pitch_state_textChanged(const QString &arg1);

        void on_r_wri_yaw_state_textChanged(const QString &arg1);


        void on_l_sh_roll_state_textChanged(const QString &arg1);

        void on_l_sh_pitch_state_textChanged(const QString &arg1);

        void on_l_sh_yaw_state_textChanged(const QString &arg1);

        void on_l_el_pitch_state_textChanged(const QString &arg1);

        void on_l_wri_yaw_state_textChanged(const QString &arg1);


        void on_r_reset_clicked();

        void on_l_reset_clicked();


        //Moving_path_SLOT.............................................

        //..Recording
        void on_recording_clicked();

        void on_recording_stop_clicked();

        void on_recording_play_clicked();

        void on_recording_save_clicked();

        void on_recording_period_textChanged(const QString &arg1);

        void on_recording_play_speed_textChanged(const QString &arg1);


        void moving_path_coordinate_callback();

        void moving_path_play();


        void on_r_path_px_textChanged(const QString &arg1);

        void on_r_path_py_textChanged(const QString &arg1);

        void on_r_path_pz_textChanged(const QString &arg1);

        void on_l_path_px_textChanged(const QString &arg1);

        void on_l_path_py_textChanged(const QString &arg1);

        void on_l_path_pz_textChanged(const QString &arg1);


        //..PLAY
        void on_play_save_clicked();

        void on_play_open_clicked();

        void on_path_play_clicked();


        void on_play_file_name_textChanged(const QString &arg1);

        void on_play_repeat_count_textChanged(const QString &arg1);

        void on_play_speed_textChanged(const QString &arg1);


        void on_r_path_play_px_textChanged(const QString &arg1);

        void on_r_path_play_py_textChanged(const QString &arg1);

        void on_r_path_play_pz_textChanged(const QString &arg1);

        void on_l_path_play_px_textChanged(const QString &arg1);

        void on_l_path_play_py_textChanged(const QString &arg1);

        void on_l_path_play_pz_textChanged(const QString &arg1);

        void on_elbow_up_clicked();

        void on_elbow_down_clicked();

private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QTimer* t1,* t2,* t3,* path_t;

};
}  // namespace order_kuro

#endif // order_kuro_MAIN_WINDOW_H
