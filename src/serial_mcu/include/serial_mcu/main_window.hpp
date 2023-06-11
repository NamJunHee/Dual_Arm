/**
 * @file /include/serial_mcu/main_window.hpp
 *
 * @brief Qt based gui for serial_mcu.
 *
 * @date November 2010
 **/
#ifndef serial_mcu_MAIN_WINDOW_H
#define serial_mcu_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "Motor_msg.h"
#include "Mt2Serial_msg.h"


/*****************************************************************************
** Namespace
*****************************************************************************/

namespace serial_mcu {

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
        QSerialPort *serial;
        serial_mcu::motion_end motion_msg;
        bool open_serial();


public Q_SLOTS:
        void DynamixelCallback();
        void MotionCallback();
        void RxCallBack();
        void TxData(unsigned char ID, unsigned char Instruction,
                    unsigned char ParameterLength, unsigned char Motion_Mode, unsigned char MotionNum);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace serial_mcu

#endif // serial_mcu_MAIN_WINDOW_H
