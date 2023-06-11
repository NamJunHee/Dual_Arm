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
#include "../include/ds4_control_kuro/main_window.hpp"
#include <QPainter>
#include <QTimer>
#include <math.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ds4_control_kuro {

using namespace Qt;
using namespace std;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

unsigned char txdata_mobile[128] = {1,2,3,4,5,6,7,8,};
unsigned char TxBuffer_mobile[128] = {0,};
unsigned char Parameter_mobile[128] = {0,};
QByteArray Tx_data_mobile;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
        qnode.init();
        QObject::connect(&qnode,SIGNAL(QnodePs4Callback()),this,SLOT(ps4_callback()));

        //zelda.load("/home/kingkh/Downloads/sprite.png");
        changeRect.setRect(0,525,120,125);

        timer = new QTimer(this);
        timer->setInterval(50);
        timer->start();

        if(!open_serial())
            exit(0);
}

MainWindow::~MainWindow() {}
bool MainWindow::open_serial()
{
    serial2mobile = new QSerialPort(this);

    serial2mobile->setPortName("ttyUSB0");

    serial2mobile->setBaudRate(QSerialPort::Baud57600);

    serial2mobile->setDataBits(QSerialPort::Data8);

    serial2mobile->setParity(QSerialPort::NoParity);

    serial2mobile->setStopBits(QSerialPort::OneStop);

    serial2mobile->setFlowControl(QSerialPort::NoFlowControl);

    /*if(!serial2mobile -> open(QIODevice::ReadWrite))
    {
        cout<<"**********Serial port open fail!!(mobile)**********"<<endl;
        return false;
    }
    else
        return true;
*/


    /*if(!serial2arm -> open(QIODevice::ReadWrite))
    {
        cout<<"**********Serial port open fail!!(arm)**********"<<endl;
        return false;
    }*/
    /*else if(serial2mobile -> open(QIODevice::ReadWrite) && serial2arm -> open(QIODevice::ReadWrite))
    {
        return true;
    }*/


}
void MainWindow::TxKuroMobile(unsigned char X,unsigned char Y,unsigned char Yaw,unsigned char Linear)
{
    TxBuffer_mobile[0] = 0xAA;
    TxBuffer_mobile[1] = 0xBB;
    TxBuffer_mobile[2] = X;
    TxBuffer_mobile[3] = Y;
    TxBuffer_mobile[4] = Yaw;
    TxBuffer_mobile[5] = Linear;
    TxBuffer_mobile[6] = ~X;
    TxBuffer_mobile[7] = ~Y;
    TxBuffer_mobile[8] = ~Yaw;
    TxBuffer_mobile[9] = ~Linear;

    for(int i = 0; i < 10; i++)
    {
        Tx_data_mobile.push_back(TxBuffer_mobile[i]);
    }


    serial2mobile -> write(Tx_data_mobile,10);

    Tx_data_mobile.clear();
}

void MainWindow::ps4_callback()
{
    ps4Info.buttonCircle = qnode.ps4Info.buttonCircle;
    ps4Info.buttonCross = qnode.ps4Info.buttonCross;
    ps4Info.buttonSquare = qnode.ps4Info.buttonSquare;
    ps4Info.buttonTriangle =qnode.ps4Info.buttonTriangle;

    ps4Info.buttonUpDown = qnode.ps4Info.buttonUpDown;
    ps4Info.buttonLeftRight =qnode.ps4Info.buttonLeftRight;

    ps4Info.buttonL1 = qnode.ps4Info.buttonL1;
    ps4Info.buttonL2 = qnode.ps4Info.buttonL2;
    ps4Info.buttonR1 = qnode.ps4Info.buttonR1;
    ps4Info.buttonR2 = qnode.ps4Info.buttonR2;

    ps4Info.buttonShare = qnode.ps4Info.buttonShare;
    ps4Info.buttonOptions = qnode.ps4Info.buttonOptions;
    ps4Info.buttonLogo = qnode.ps4Info.buttonLogo;

    ps4Info.lJoystickUpDown = qnode.ps4Info.lJoystickUpDown;
    ps4Info.lJoystickLeftRight = qnode.ps4Info.lJoystickLeftRight;
    ps4Info.rJoystickUpDown = qnode.ps4Info.rJoystickUpDown;
    ps4Info.rJoystickLeftRight = qnode.ps4Info.rJoystickLeftRight;
    kuro_ds4_engine();
}
void MainWindow::kuro_ds4_engine()
{
    kuro_arm.leftArmX = kuro_arm.leftArmX+ps4Info.lJoystickUpDown;
    kuro_arm.leftArmY = kuro_arm.leftArmY+ps4Info.lJoystickLeftRight;
    if(ps4Info.buttonL2 != -1)ps4Info.buttonL2=0;
    kuro_arm.leftArmZ = kuro_arm.leftArmZ+ps4Info.buttonL1+ps4Info.buttonL2;

    kuro_arm.rightArmX = kuro_arm.rightArmX+ps4Info.rJoystickUpDown;
    kuro_arm.rightArmY = kuro_arm.rightArmY+ps4Info.rJoystickLeftRight;
    if(ps4Info.buttonR2 != -1)ps4Info.buttonR2=0;
    kuro_arm.rightArmZ = kuro_arm.rightArmZ+ps4Info.buttonR1+ps4Info.buttonR2;



    rcoordinateLimit=sqrt(kuro_arm.rightArmX*kuro_arm.rightArmX + kuro_arm.rightArmY*kuro_arm.rightArmY + kuro_arm.rightArmZ*kuro_arm.rightArmZ);
    cout<<"rcoordinateLimit"<<rcoordinateLimit<<endl;
    if(rcoordinateLimit>500)
    {
        kuro_arm.rightArmX = kuro_arm.rightArmX-ps4Info.rJoystickUpDown;
        kuro_arm.rightArmY = kuro_arm.rightArmY-ps4Info.rJoystickLeftRight;
        if(ps4Info.buttonR2 != -1)ps4Info.buttonR2=0;
        kuro_arm.rightArmZ = kuro_arm.rightArmZ-ps4Info.buttonR1+ps4Info.buttonR2;
    }
    lcoordinateLimit=sqrt(kuro_arm.leftArmX*kuro_arm.leftArmX + kuro_arm.leftArmY*kuro_arm.leftArmY + kuro_arm.leftArmZ*kuro_arm.leftArmZ);
    cout<<"lcoordinateLimit"<<lcoordinateLimit<<endl;
    if(lcoordinateLimit>500)
    {
        kuro_arm.leftArmX = kuro_arm.leftArmX-ps4Info.lJoystickUpDown;
        kuro_arm.leftArmY = kuro_arm.leftArmY-ps4Info.lJoystickLeftRight;
        if(ps4Info.buttonL2 != -1)ps4Info.buttonL2=0;
        kuro_arm.leftArmZ = kuro_arm.leftArmZ-ps4Info.buttonL1+ps4Info.buttonL2;
    }







    if(ps4Info.buttonSquare)
    {

        _squarebutton_flag=ps4Info.buttonSquare;
        if(squarebutton_flag != _squarebutton_flag)
        {
            squarebutton_flag = _squarebutton_flag;
            kuro_mobile.mobileRPM=kuro_mobile.mobileRPM+5;
            ui.lcdNumber_motor_rpm->display(kuro_mobile.mobileRPM);
        }
    }
    else
    {
        squarebutton_flag=false;
    }

    if(ps4Info.buttonCross)
    {

        _crossbutton_flag=ps4Info.buttonCross;
        if(crossbutton_flag != _crossbutton_flag)
        {
            crossbutton_flag = _crossbutton_flag;
            kuro_mobile.mobileRPM=kuro_mobile.mobileRPM-5;
            if(kuro_mobile.mobileRPM<0)kuro_mobile.mobileRPM=0;

            ui.lcdNumber_motor_rpm->display(kuro_mobile.mobileRPM);
        }
    }
    else
    {
        crossbutton_flag=false;
    }

    if(ps4Info.buttonTriangle)
    {

        _trianglebutton_flag=ps4Info.buttonTriangle;
        if(trianglebutton_flag != _trianglebutton_flag)
        {
            trianglebutton_flag = _trianglebutton_flag;
            kuro_mobile.mobileLinear=kuro_mobile.mobileLinear+5;
            ui.lcdNumber_linear->display(kuro_mobile.mobileLinear);
        }
    }
    else
    {
        trianglebutton_flag=false;
    }


    if(ps4Info.buttonCircle)
    {

        _circlebutton_flag=ps4Info.buttonCircle;
        if(circlebutton_flag != _circlebutton_flag)
        {
            circlebutton_flag = _circlebutton_flag;
            kuro_mobile.mobileLinear=kuro_mobile.mobileLinear-5;
            if(kuro_mobile.mobileLinear<0)kuro_mobile.mobileLinear=0;

            ui.lcdNumber_linear->display(kuro_mobile.mobileLinear);
        }
    }
    else
    {
        circlebutton_flag=false;
    }

    if(ps4Info.buttonShare)
    {
        yaw_direction=1;
    }
    else if(ps4Info.buttonOptions)
    {
        yaw_direction=-1;
    }
    else
    {
        yaw_direction=0;
    }





    if(ps4Info.buttonUpDown==1)
    {
        frontFlag = true;
    }
    else
    {
        frontFlag = false;
    }
    if(ps4Info.buttonLeftRight==1)
    {
        leftFlag = true;
    }
    else
    {
        leftFlag = false;
    }

    if(ps4Info.buttonUpDown==-1)
    {
        backFlag = true;
    }
    else
    {
        backFlag = false;
    }

    if(ps4Info.buttonLeftRight==-1)
    {
        rightFlag = true;
    }
    else
    {
        rightFlag = false;
    }


    if(frontFlag)moveFront();
    else if(backFlag)moveBack();
    else if(leftFlag)moveLeft();
    else if(rightFlag)moveRight();


    if(kuro_arm.leftArmX>0)
    ui.lcdNumber_lx->setPalette(Qt::red);
    else if(kuro_arm.leftArmX<0)
    ui.lcdNumber_lx->setPalette(Qt::blue);
    else
    ui.lcdNumber_lx->setPalette(Qt::white);

    if(kuro_arm.leftArmY>0)
    ui.lcdNumber_ly->setPalette(Qt::red);
    else if(kuro_arm.leftArmY<0)
    ui.lcdNumber_ly->setPalette(Qt::blue);
    else
    ui.lcdNumber_ly->setPalette(Qt::white);

    if(kuro_arm.leftArmZ>0)
    ui.lcdNumber_lz->setPalette(Qt::red);
    else if(kuro_arm.leftArmZ<0)
    ui.lcdNumber_lz->setPalette(Qt::blue);
    else
    ui.lcdNumber_lz->setPalette(Qt::white);

    if(kuro_arm.rightArmX>0)
    ui.lcdNumber_rx->setPalette(Qt::red);
    else if(kuro_arm.rightArmX<0)
    ui.lcdNumber_rx->setPalette(Qt::blue);
    else
    ui.lcdNumber_rx->setPalette(Qt::white);

    if(kuro_arm.rightArmY>0)
    ui.lcdNumber_ry->setPalette(Qt::red);
    else if(kuro_arm.rightArmY<0)
    ui.lcdNumber_ry->setPalette(Qt::blue);
    else
    ui.lcdNumber_ry->setPalette(Qt::white);

    if(kuro_arm.rightArmZ>0)
    ui.lcdNumber_rz->setPalette(Qt::red);
    else if(kuro_arm.rightArmZ<0)
    ui.lcdNumber_rz->setPalette(Qt::blue);
    else
    ui.lcdNumber_rz->setPalette(Qt::white);



    ui.lcdNumber_lx->display(kuro_arm.leftArmX);
    ui.lcdNumber_ly->display(kuro_arm.leftArmY);
    ui.lcdNumber_lz->display(kuro_arm.leftArmZ);
    ui.lcdNumber_rx->display(kuro_arm.rightArmX);
    ui.lcdNumber_ry->display(kuro_arm.rightArmY);
    ui.lcdNumber_rz->display(kuro_arm.rightArmZ);

    ps4armInfo.lArmX = kuro_arm.leftArmX;
    ps4armInfo.lArmY = kuro_arm.leftArmY;
    ps4armInfo.lArmZ = kuro_arm.leftArmZ;

    ps4armInfo.rArmX = kuro_arm.rightArmX;
    ps4armInfo.rArmY = kuro_arm.rightArmY;
    ps4armInfo.rArmZ = kuro_arm.rightArmZ;


    TxKuroMobile(kuro_mobile.mobileRPM*ps4Info.buttonUpDown,kuro_mobile.mobileRPM*ps4Info.buttonLeftRight,kuro_mobile.mobileRPM*yaw_direction,kuro_mobile.mobileLinear);
    qnode.ps4arm_pub.publish(ps4armInfo);


}
void MainWindow::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    painter.setBrush(QBrush(Qt::black,Qt::SolidPattern));
    painter.setPen(QPen(Qt::transparent));
    painter.drawPixmap(QPoint(150,450),zelda,changeRect);
}
void MainWindow::moveFront()
{
    if(moveCount>=10)
        moveCount=0;

    changeRect.setRect(moveCount*120,785,120,125);
    moveCount++;
    update();
}
void MainWindow::moveBack()
{
    if(moveCount>=10)
        moveCount=0;

    changeRect.setRect(moveCount*120,525,120,125);
    moveCount++;
    update();
}
void MainWindow::moveLeft()
{
    if(moveCount>=10)
        moveCount=0;

    changeRect.setRect(moveCount*120,655,120,125);
    moveCount++;
    update();
}
void MainWindow::moveRight()
{
    if(moveCount>=10)
        moveCount=0;

    changeRect.setRect(moveCount*120,920,120,125);
    moveCount++;
    update();
}


}  // namespace ds4_control_kuro

