/**
 * @file /include/ds4_control_kuro/main_window.hpp
 *
 * @brief Qt based gui for ds4_control_kuro.
 *
 * @date November 2010
 **/
#ifndef ds4_control_kuro_MAIN_WINDOW_H
#define ds4_control_kuro_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace ds4_control_kuro {

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
        QSerialPort *serial2mobile;
        bool open_serial();

        //PS4
        msg_generate::ps4 ps4Info;
        msg_generate::ps4arm_msg ps4armInfo;

        struct arm_data
        {
            double leftArmX = 0.0;
            double leftArmY = 0.0;
            double leftArmZ = 0.0;
            double rightArmX = 0.0;
            double rightArmY = 0.0;
            double rightArmZ = 0.0;
        };
        struct mobile_data
        {
            int mobileRPM = 0.0;
            int mobileLinear = 0.0;
        };
        mobile_data kuro_mobile;
        arm_data kuro_arm;


        QPixmap zelda;
        QTimer* timer;
        QRect changeRect;

        bool frontFlag;
        bool backFlag;
        bool leftFlag;
        bool rightFlag;

        void kuro_ds4_engine();
        void moveFront();
        void moveBack();
        void moveLeft();
        void moveRight();
        void TxKuroMobile(unsigned char X,unsigned char Y,unsigned char Yaw,unsigned char Linear);

        double rcoordinateLimit;
        double lcoordinateLimit;
        bool squarebutton_flag;
        bool _squarebutton_flag;
        bool trianglebutton_flag;
        bool _trianglebutton_flag;
        bool circlebutton_flag;
        bool _circlebutton_flag;
        bool crossbutton_flag;
        bool _crossbutton_flag;
        int yaw_direction;
        int moveCount;

        void paintEvent(QPaintEvent *);

public Q_SLOTS:
        void ps4_callback();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace ds4_control_kuro

#endif // ds4_control_kuro_MAIN_WINDOW_H
