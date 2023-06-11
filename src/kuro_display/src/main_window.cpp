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
#include "../include/kuro_display/main_window.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>
#include <stdio.h>
#include <opencv2/core.hpp>
#include <QTimer>
#include <QDebug>
#define UI_display_mp4 "/home/robit/catkin_ws/src/kuro_display/kuro_emotion/46779208-ac8ab900-cccb-11e8-88c3-886af4faf6ea.mp4"
#define UI_icon_png "/home/robit/catkin_ws/src/kuro_display/kuro_emotion/80329358.png"
#define UI_display_kuro_initial_mp4 "/home/robit/catkin_ws/src/kuro_display/kuro_emotion/kuro_initial.mp4"
#define UI_display_kuro_smile_mp4 "/home/robit/catkin_ws/src/kuro_display/kuro_emotion/kuro_smile.mp4"
#define UI_display_kuro_sad_mp4 "/home/robit/catkin_ws/src/kuro_display/kuro_emotion/kuro_sad.mp4"
#define UI_display_kuro_angry_mp4 "/home/robit/catkin_ws/src/kuro_display/kuro_emotion/kuro_angry.mp4"



/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kuro_display {

using namespace Qt;
using namespace cv;
using namespace std;
/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
VideoCapture getgif;


MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
        setWindowFlags(Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
        //setAttribute(Qt::WA_TranslucentBackground,true); //프로그램을 완전 투명하게 하는 함수

        getgif.open(UI_display_kuro_initial_mp4);
        screensize = QApplication::desktop()->screenGeometry();

        cv::Mat a;
        getgif >> a;
        cv::resize(a,a,cv::Size(screensize.width(),screensize.height()));
        ui.label->setFixedSize(a.cols,a.rows);

        timer = new QTimer();
        connect(timer, SIGNAL(timeout()), this, SLOT(slotTimerAlarm()));
        timer->start(100);

        setFixedSize(screensize.width(),screensize.height());
        setWindowIcon(QPixmap(UI_icon_png));

}

MainWindow::~MainWindow() {}


void MainWindow::updateImage()
{
    cv::Mat a;
    getgif >> a;

    if(changeDisplaySignal)
    {
        switch (displayChannel) {
        case 0:
            getgif.release();
            getgif.open(UI_display_kuro_initial_mp4);
            changeDisplaySignal=false;
            break;
        case 1:
            getgif.release();
            getgif.open(UI_display_kuro_smile_mp4);
            changeDisplaySignal=false;
            break;
        case 2:
            getgif.release();
            getgif.open(UI_display_kuro_sad_mp4);
            changeDisplaySignal=false;
            break;
        case 3:
            getgif.release();
            getgif.open(UI_display_kuro_angry_mp4);
            changeDisplaySignal=false;
            break;

        default:
            break;
        }
    }

    if(!a.empty())
    {
        cv::resize(a,a,cv::Size(screensize.width(),screensize.height()));
        QImage image_org((const unsigned char*)(a.data), a.cols, a.rows, QImage::Format_RGB888);
        ui.label->setPixmap(QPixmap::fromImage(image_org.rgbSwapped()));
    }
    else
    {
        switch (displayChannel) {

        case 0:
            getgif.release();
            getgif.open(UI_display_kuro_initial_mp4);
            break;
        case 1:
            getgif.release();
            getgif.open(UI_display_kuro_smile_mp4);
            break;
        case 2:
            getgif.release();
            getgif.open(UI_display_kuro_sad_mp4);
            break;
        case 3:
            getgif.release();
            getgif.open(UI_display_kuro_angry_mp4);
            break;
        default:
            break;
        }

    }


}
void MainWindow::slotTimerAlarm()
{
    updateImage();
}
void MainWindow::changeDisplay(int channel)
{
    changeDisplaySignal=true;
    displayChannel= channel;

}
void MainWindow::keyPressEvent(QKeyEvent *e)
{
    if(e->key()==Qt::Key_Q)
    {
        changeDisplay(0);
    }
    else if(e->key()==Qt::Key_W)
    {
        changeDisplay(1);
    }
    else if(e->key()==Qt::Key_E)
    {
        changeDisplay(2);
    }
    else if(e->key()==Qt::Key_R)
    {
        changeDisplay(3);
    }
    else if(e->key()==Qt::Key_Escape)
    {
        this->close();
    }
    else if(e->key()==Qt::Key_S)
    {
    }

}


}  // namespace kuro_display


