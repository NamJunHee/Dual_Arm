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
#include <QDebug>
#include <QMessageBox>
#include <iostream>
#include <sstream>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "../include/serial_mcu/main_window.hpp"
#include "../include/serial_mcu/serial_msg.h"
#include "../include/serial_mcu/motor_dxl.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace serial_mcu {


unsigned char txdata[128] = {1,2,3,4,5,6,7,8, };
QByteArray Tx_data;
using namespace Qt;
using namespace std;
bool Tx_motion_flag = false;
unsigned char TxBuffer[128] = { 0, };
unsigned char RxBuffer[10] = { 0, };
unsigned char Parameter[128] = { 0, };


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();
    if(!open_serial())
        exit(0);

    QObject::connect(&qnode, SIGNAL(Rx_MotorData()), this, SLOT(DynamixelCallback()));
    QObject::connect(&qnode, SIGNAL(Rx_MotionData()), this, SLOT(MotionCallback()));
    QObject::connect(serial, SIGNAL(readyRead()), this, SLOT(RxCallBack()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));


}


bool MainWindow::open_serial()
{
    serial = new QSerialPort(this);
    serial -> setPortName("ttyUSB0");
    serial -> setBaudRate(QSerialPort::Baud115200);
    serial -> setDataBits(QSerialPort::Data8);
    serial -> setParity(QSerialPort::NoParity);
    serial -> setStopBits(QSerialPort::OneStop);
    serial -> setFlowControl(QSerialPort::NoFlowControl);
    if(!serial -> open(QIODevice::ReadWrite))
    {
        cout<<"**********Serial port open fail!!**********"<<endl;
        return false;
    }
    else
        return true;
}

void MainWindow::DynamixelCallback()
{
    cout<<"#### DynamixelCallback ####"<<endl<<endl;
    unsigned int parameterCount = 0;
    unsigned int dynamixelLength = qnode.Dx_msg.id.size();
    if(!Tx_motion_flag)
    {
        switch(qnode.Dx_msg.mode)
        {
            case 1: //position

            Parameter[parameterCount++] = 30;
            Parameter[parameterCount++] = 2;
            for (size_t i = 0; i < qnode.Dx_msg.id.size(); i++)
            {
                Parameter[parameterCount++] = qnode.Dx_msg.id[i];
                Parameter[parameterCount++] = (qnode.Dx_msg.position[i] & 0x00FF);
                Parameter[parameterCount++] = (qnode.Dx_msg.position[i] >> 8);
            }
            TxData(0xFE, INST_SYNC_WRITE, 2 + (dynamixelLength * 3), 0, 0);

            break;

            case 2: //speed

            Parameter[parameterCount++] = 32;
            Parameter[parameterCount++] = 2;
            for (size_t i = 0; i < qnode.Dx_msg.id.size(); i++)
            {
                Parameter[parameterCount++] = qnode.Dx_msg.id[i];
                Parameter[parameterCount++] = (qnode.Dx_msg.speed[i] & 0x00FF);
                Parameter[parameterCount++] = (qnode.Dx_msg.speed[i] >> 8);
            }
            TxData(0xFE, INST_SYNC_WRITE, 2 + (dynamixelLength * 3), 0, 0);

            break;

            case 3: //speed and position

            Parameter[parameterCount++] = 30;
            Parameter[parameterCount++] = 4;
            for (size_t i = 0; i < qnode.Dx_msg.id.size(); i++)
            {
                Parameter[parameterCount++] = qnode.Dx_msg.id[i];
                Parameter[parameterCount++] = (qnode.Dx_msg.position[i] & 0x00FF);
                Parameter[parameterCount++] = (qnode.Dx_msg.position[i] >> 8);
                Parameter[parameterCount++] = (qnode.Dx_msg.speed[i] & 0x00FF);
                Parameter[parameterCount++] = (qnode.Dx_msg.speed[i] >> 8);
            }
            TxData(0xFE, INST_SYNC_WRITE, 2 + (dynamixelLength * 5), 0, 0);

            break;

            case 4: //torque off

            Parameter[parameterCount++] = 24;
            Parameter[parameterCount++] = 0;

            TxData(qnode.Dx_msg.id[0], 0x03, 2, 0, 0);

            break;

            case 5:

            Parameter[parameterCount++] = 73;
            Parameter[parameterCount++] = 1;
            for (size_t i = 0; i < qnode.Dx_msg.id.size(); i++)
            {
                Parameter[parameterCount++] = qnode.Dx_msg.id[i];
                Parameter[parameterCount++] = (qnode.Dx_msg.acceleration[i]);
            }
            TxData(0xFE, INST_SYNC_WRITE, 2 + (dynamixelLength * 2), 0, 0);

            break;

            default:
            break;
        }
    }

}

void MainWindow::MotionCallback()
{
    cout<<"#### MotionCallback ####"<<endl<<endl;

    Tx_motion_flag = true;
    cout<<"Motion_Mode = " <<qnode.Motion_msg.Motion_Mode<<endl;
    cout<<"Motion_Num = " <<qnode.Motion_msg.Motion_Num<<endl;
    TxData(0,0,0,qnode.Motion_msg.Motion_Mode,qnode.Motion_msg.Motion_Num);
}

void MainWindow::RxCallBack()
{
    motion_msg.motion_end = 1;
    qnode.Serial_pub.publish(motion_msg);
    //cout<<"#####################RXRX##############"<<endl<<endl;
}

void MainWindow::TxData(unsigned char ID, unsigned char Instruction,
                        unsigned char ParameterLength, unsigned char Motion_Mode, unsigned char MotionNum)
{
    unsigned char Count;
    cout<<"Motion_Mode = "<<Motion_Mode<<endl;
    if(Motion_Mode)
    {
        TxBuffer[0] = 0xAA;
        TxBuffer[1] = 0xBB;
        TxBuffer[2] = MotionNum;
        TxBuffer[3] = ~MotionNum;

        for(int i = 0; i < 4; i++)
        {
            Tx_data.push_back(TxBuffer[i]);
        }

        for(int i = 0; i < 4;i++)
            printf("TxBuffer[%d] = %d\n",i,TxBuffer[i]);

        serial -> write(Tx_data,4);       
        Tx_data.clear();

        Tx_motion_flag = false;
    }
    else
    {
        // Header
        TxBuffer[0] = 0xFF;
        TxBuffer[1] = 0xFF;

        TxBuffer[2] = ID;
        TxBuffer[3] = ParameterLength + 2;// Length(Instruction + ParamterLength + Checksum)
        TxBuffer[4] = Instruction;

        for (Count = 0; Count < ParameterLength; Count++)
        {
            TxBuffer[Count + 5] = Parameter[Count];
        }

        unsigned char CheckSum = 0;
        unsigned char PacketLength = ParameterLength + 6;

        for (Count = 2; Count < PacketLength - 1; Count++)  // Except 0xFF, Checksum
        {
            CheckSum += TxBuffer[Count];
        }

        TxBuffer[Count] = ~CheckSum;


        for(int i = 0; i < PacketLength; i++)
        {
            Tx_data.push_back(TxBuffer[i]);
        }

        for(int i = 0; i < PacketLength;i++)
            printf("TxBuffer[%d] = %d\n",i,TxBuffer[i]);

        serial -> write(Tx_data,PacketLength);
        Tx_data.clear();

    }

}

MainWindow::~MainWindow() {
    serial->close();
}



}  // namespace serial_mcu


