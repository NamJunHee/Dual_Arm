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
#include "../include/kuro_cotton_ui/main_window.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

/*****************************************************************************
** Graphic
*****************************************************************************/
#define UI_Background_Image "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/kuro_face_blur.mp4"
#define UI_Icon_ready_m "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/ready_m.mp4"
#define UI_Icon_start_m "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/start_m.mp4"
#define UI_Icon_stop_m "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/stop_m.mp4"
#define UI_Icon_ready_s "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/ready_s.mp4"
#define UI_Icon_start_s "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/start_s.mp4"
cv::VideoCapture getBackground_kuro;
cv::VideoCapture getIcon_ready;
cv::VideoCapture getIcon_start;
cv::VideoCapture getIcon_stop;
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kuro_cotton_ui {

//msg_generate::kuro_cotton_candy cotton_candy_msg;
using namespace Qt;
using namespace std;
using namespace cv;



bool ready_flag = false;
bool start_flag = false;
bool stop_flag = false;
/************************************S*****************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();


    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(updateUI()),this,SLOT(ui_update_callback()));

    screensize = QApplication::desktop()->screenGeometry();



   /* screensize = QApplication::desktop()->screenGeometry();
    QPixmap pix(UI_Background_Image);
    ui.background_label->setPixmap(pix);
    ui.background_label->setGeometry(0,0,1920,1080);*/

}

MainWindow::~MainWindow() {}

void MainWindow::ui_update_callback()
{


     cv::Mat bgk;
     getBackground_kuro >> bgk;
     if(!bgk.empty())
     {
     cv::resize(bgk,bgk,cv::Size(screensize.width(),screensize.height()));
     QImage image_org((const unsigned char*)(bgk.data), bgk.cols, bgk.rows, QImage::Format_RGB888);
     ui.background_label->setPixmap(QPixmap::fromImage(image_org.rgbSwapped()));
     ui.background_label->setFixedSize(bgk.cols,bgk.rows);
     }
     else
     {
         getBackground_kuro.open(UI_Background_Image);
     }






     ////////////////////////////////////////////////////////////
     /// \brief ready_icon
     ///
     /// ////////////////////////////////////////////////////////
     cv::Mat ready_icon;
     getIcon_ready >> ready_icon;

     if(!ready_icon.empty())
     {
        cv::resize(ready_icon,ready_icon,cv::Size(360,360));
        QImage image_org((const unsigned char*)(ready_icon.data),ready_icon.cols,ready_icon.rows,QImage::Format_RGB888);
        QIcon ButtonIcon(QPixmap::fromImage(image_org.rgbSwapped()));

        ui.ready_btn->setIcon(ButtonIcon);
        ui.ready_btn->setIconSize(image_org.rect().size());
     }
     else
     {
         getIcon_ready.release();
         if(ready_flag)
         {
             getIcon_ready.open(UI_Icon_ready_m);
         }
         else
         {
             getIcon_ready.open(UI_Icon_ready_s);
         }


     }

     //////////////////////////////////////////////////////////////
     /// \brief start_icon
     //////////////////////////////////////////////////////////////
     cv::Mat start_icon;
     getIcon_start >> start_icon;

     if(!start_icon.empty())
     {
         cv::resize(start_icon,start_icon,cv::Size(360,360));
         QImage image_org((const unsigned char*)(start_icon.data),start_icon.cols,start_icon.rows,QImage::Format_RGB888);
         QIcon ButtonIcon(QPixmap::fromImage(image_org.rgbSwapped()));

         ui.start_btn->setIcon(ButtonIcon);
         ui.start_btn->setIconSize(image_org.rect().size());
     }
     else
     {
         //getIcon_start.release();
         if(start_flag)
         {
             getIcon_start.open(UI_Icon_start_m);
         }
         else
         {
             getIcon_start.open(UI_Icon_start_s);
         }
     }
     ///////////////////////////////////////////////////////////////////
     /// \brief stop_icon
     ///////////////////////////////////////////////////////////////////
     cv::Mat stop_icon;
     getIcon_stop >> stop_icon;

     if(!stop_icon.empty())
     {
         cv::resize(stop_icon,stop_icon,cv::Size(360,360));
         QImage image_org((const unsigned char*)(stop_icon.data),stop_icon.cols,stop_icon.rows,QImage::Format_RGB888);
         QIcon ButtonIcon(QPixmap::fromImage(image_org.rgbSwapped()));

         ui.stop_btn->setIcon(ButtonIcon);
         ui.stop_btn->setIconSize(image_org.rect().size());
     }
     else
     {
         getIcon_stop.open(UI_Icon_stop_m);
     }









}
void kuro_cotton_ui::MainWindow::on_ready_btn_clicked()
{
    msg_cotton_candy_state.ready=true;
    msg_cotton_candy_state.start=false;
    msg_cotton_candy_state.stop=false;
    cotton_candy_state_pub.publish(msg_cotton_candy_state);

    ready_flag = true;
    start_flag = false;
    stop_flag = false;

    getIcon_ready.release();
    getIcon_start.release();

}

void kuro_cotton_ui::MainWindow::on_start_btn_clicked()
{
    msg_cotton_candy_state.ready=false;
    msg_cotton_candy_state.start=true;
    msg_cotton_candy_state.stop=false;
    cotton_candy_state_pub.publish(msg_cotton_candy_state);
    ready_flag = false;
    start_flag = true;
    stop_flag = false;
    getIcon_ready.release();
    getIcon_start.release();
}

void MainWindow::on_stop_btn_clicked()
{
    msg_cotton_candy_state.ready=false;
    msg_cotton_candy_state.start=false;
    msg_cotton_candy_state.stop=true;
    cotton_candy_state_pub.publish(msg_cotton_candy_state);
    ready_flag = false;
    start_flag = false;
    stop_flag = true;
    this->close();
}


}  // namespace kuro_cotton_ui

