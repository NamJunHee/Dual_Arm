/**
 * @file /include/dynamixel_serial/main_window.hpp
 *
 * @brief Qt based gui for dynamixel_serial.
 *
 * @date November 2010
 **/
#ifndef dynamixel_serial_MAIN_WINDOW_H
#define dynamixel_serial_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QSocketNotifier>
#include <QDebug>
#include <QObject>

#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <dynamixel_workbench_msgs/DynamixelInfo.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_MOVING_SPEED            32
#define ADDR_MX_PRESENT_POSITION        36 // Read
#define ADDR_MX_MOVING                  46 // Read
#define ADDR_MX_PRESENT_LOAD            40 // Read
#define ADDR_MX_P_GAIN                  28
#define ADDR_MX_I_GAIN                  27
#define ADDR_MX_D_GAIN                  26


#define ADDR_PRO_TORQUE_ENABLE          562                 // Control table address is different in Dynamixel model
#define ADDR_PRO_LED_RED                563
#define ADDR_PRO_GOAL_POSITION          596
#define ADDR_PRO_PRESENT_POSITION       611
// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_MOVING_SPEED             2
#define LEN_MX_MOVING                   1
#define LEN_MX_PRESENT_LOAD             2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_P_GAIN                   1
#define LEN_MX_I_GAIN                   1
#define LEN_MX_D_GAIN                   1
#define LEN_PRO_LED_RED                 1
#define LEN_PRO_GOAL_POSITION           4
#define LEN_PRO_PRESENT_POSITION        4

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        500000
#define DEVICENAME                      "/dev/ttyUSB1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      100                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      4000                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold
#define ESC_ASCII_VALUE                 0x1b

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace dynamixel_serial {
using namespace std;


struct motionData
{
    int nowPosition = 0;
    int inputPosition = 0;
    double proportion = 0;
    int id;
};


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
    void DynamixelInit();
	~MainWindow();


public Q_SLOTS:
    void on_pushButton_TxPosition_clicked();
    void on_pushButton_portSet_clicked();
    void on_pushButton_read_clicked();
    void on_pushButton_torqueoff_clicked();
    void on_pushButton_readLoad_clicked();
    void on_pushButton_PIDSet_clicked();
    void DynamixelCallback();
    void MotionCallback();
    void motionPlay();
    void on_horizontalSlider_valueChanged(int value);
    void Return_persent_position();
    void wheel_Mode_callback();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QSerialPort *m_serial;

    std::string device_name_;
    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;
    dynamixel::GroupSyncWrite *groupSyncWrite;


    uint8_t dxl_id_;
    int32_t goal_position_;
    int32_t current_position_;
    int spd = 0;

    QTimer* mTimer;
    int motionStep = 0;
    bool isStepFinish = true;
    bool isMotion = false;
    bool isDelay = false;

    vector<motionData> m_motionData;
    int m_nowTime =0;
    int m_delayTime = 0;
    int m_repeat = 0;
    int motion_end = 0;

    msg_generate::Motion_msg motion_Info;

    order_kuro::Rec_arm_id dxl_id_Info;
    int R_arm_id[5] = {1,3,5,7,9};
    int L_arm_id[5] = {0,2,4,6,8};

    msg_generate::Arm_path_position_msg dxl_pos_Info;
    msg_generate::motion_end motion_end_Info;

private:
    bool initDynamixelController();
    bool shutdownDynamixelController();
    void checkLoop();
    void writeDynamixelRegister(uint8_t id, uint16_t addr, uint16_t length, int32_t value);
    void dxlNowPosition();
    int ReadPosition(int id);
    int ReadLoad(int id);
    void PIDSet(int id, int P, int I, int D);
    void wheelMode(int id, int value);
};

}  // namespace dynamixel_serial

#endif // dynamixel_serial_MAIN_WINDOW_H
