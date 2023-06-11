/**
 * @file /include/kuro_cotton_candy/main_window.hpp
 *
 * @brief Qt based gui for kuro_cotton_candy.
 *
 * @date November 2010
 **/
#ifndef kuro_cotton_candy_MAIN_WINDOW_H
#define kuro_cotton_candy_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "motion_name.h"
#include <msg_generate/elbow_updown_msg.h>

#define READY_MOTION          0
#define TAKE_SUGAR_MOTION     1
#define POUR_SUGAR_MOTION     2
#define CIRCLE_SETTING_MOTION 3
#define CIRCLE_MOTION         4
#define CIRCLE_END_MOTION     5
#define SPIN_MOTION           6
#define END_MOTION            7

#define EL_DOWN 1
#define EL_UP -1

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kuro_cotton_candy {

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

    bool eventFilter(QObject *object, QEvent *event);

    kuro_cotton_candy::motion_name motion_name_info;
    msg_generate::elbow_updown_msg elbow_updown_info;
    msg_generate::dxl_wheel_msg wheel_mode_info;

    struct motion_setting
    {
        int circle_time;
        int spin_time;
    };

    motion_setting motion_set;

    struct motion_path_coor{
        int pX;
        int pY;
        int pZ;
        int elbow_dir;
    };

    struct motion_param{
        int size;
        int speed;
    };

    vector<motion_path_coor> R_take_sugar_motion;
    vector<motion_path_coor> L_take_sugar_motion;

    vector<motion_path_coor> R_pour_motion;
    vector<motion_path_coor> L_pour_motion;

    vector<motion_path_coor> R_circle_motion;
    vector<motion_path_coor> L_circle_motion;

    vector<motion_path_coor> R_spin_motion;
    vector<motion_path_coor> L_spin_motion;

    motion_param take_param;
    motion_param pour_param;
    motion_param circle_param;
    motion_param spin_param;

    int take_open_flag = 0;
    int pour_open_flag = 0;
    int cir_open_flag = 0;
    int spin_open_flag = 0;

    int making_mode = READY_MOTION;
    int making_flag = 0;
    int making_start = 0;
    int first_start_flag = 1;

    int motion_size = 0;
    int motion_speed = 0;

    int m_timer = 0;
    int motion_end = 0;
    int motion_flag = 0;

    int Circle_time = 0;
    int Spin_time = 0;

    int i = 0;
private Q_SLOTS:

    void on_making_ready_clicked();
    void on_making_start_clicked();
    void making_cotton_candy();

    void take_sugar_motion();
    void pour_sugar_motion();
    void circle_motion();
    void spin_motion();
    void end_motion();

    void motion_time();

    void on_stop_pb_clicked();

    void on_take_sugar_motion_clicked();

    void on_pour_motion_pb_clicked();

    void on_circle_motion_pb_clicked();

    void on_spin_motion_pb_clicked();


    void on_circle_time_textChanged(const QString &arg1);

    void on_spin_time_textChanged(const QString &arg1);


    void on_motion_setting_save_clicked();

    void on_motion_setting_open_clicked();

    void motion_end_callback();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QTimer* making_t, * motions_t;
};

}  // namespace kuro_cotton_candy

#endif // kuro_cotton_candy_MAIN_WINDOW_H
