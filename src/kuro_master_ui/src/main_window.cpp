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
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <QtGui>
#include <QMessageBox>
#include <QTimer>
#include <iostream>
#include "../include/kuro_master_ui/main_window.hpp"


#define UI_Icon_cotton_candy "/home/robit/catkin_ws/src/kuro_master_ui/resources/images/cc.mp4"
#define UI_Background_Image "/home/robit/catkin_ws/src/kuro_master_ui/resources/images/kuro_face.mp4"
#define UI_Icon_start_m "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/start_m.mp4"
#define UI_Icon_stop_m "/home/robit/catkin_ws/src/kuro_master_ui/resources/images/exit_m.mp4"
#define UI_Icon_ready_s "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/ready_s.mp4"
#define UI_Icon_start_s "/home/robit/catkin_ws/src/kuro_cotton_ui/resources/images/start_s.mp4"
#define UI_Icon_video "/home/robit/catkin_ws/src/kuro_master_ui/resources/images/robit_video_m.mp4"
#define UI_Icon_display "/home/robit/catkin_ws/src/kuro_master_ui/resources/images/display_m.mp4"
#define UI_Icon_pose_detection "/home/robit/catkin_ws/src/kuro_master_ui/resources/images/pose_detection_m.mp4"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kuro_master_ui {

using namespace cv;
using namespace Qt;
VideoCapture getBackground_kuro;
VideoCapture getIcon_cotton_candy;
VideoCapture getIcon_skeleton;
VideoCapture getIcon_video;
VideoCapture getIcon_display;
VideoCapture getIcon_stop;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();
    //setWindowFlags(Qt::FramelessWindowHint|Qt::WindowStaysOnTopHint);
    QObject::connect(&qnode,SIGNAL(updateUI()),this,SLOT(ui_update_callback()));

    screensize = QApplication::desktop()->screenGeometry();


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


    ////////////////////////////////////////////
    /// \brief cotton_candy_icon
    ////////////////////////////////////////////
    cv::Mat cotton_candy_icon;
    getIcon_cotton_candy >> cotton_candy_icon;
    if(!cotton_candy_icon.empty())
    {
       cv::resize(cotton_candy_icon,cotton_candy_icon,cv::Size(360,360));
       QImage image_org((const unsigned char*)(cotton_candy_icon.data),cotton_candy_icon.cols,cotton_candy_icon.rows,QImage::Format_RGB888);
       QIcon ButtonIcon(QPixmap::fromImage(image_org.rgbSwapped()));

       ui.cotton_candy_btn->setIcon(ButtonIcon);
       ui.cotton_candy_btn->setIconSize(image_org.rect().size());
    }
    else
    {
            getIcon_cotton_candy.open(UI_Icon_cotton_candy);
    }
    ////////////////////////////////////////////
    /// \brief skeleton_icon
    ////////////////////////////////////////////
    cv::Mat skeleton_icon;
    getIcon_skeleton >> skeleton_icon;
    if(!skeleton_icon.empty())
    {
       cv::resize(skeleton_icon,skeleton_icon,cv::Size(360,360));
       QImage image_org((const unsigned char*)(skeleton_icon.data),skeleton_icon.cols,skeleton_icon.rows,QImage::Format_RGB888);
       QIcon ButtonIcon(QPixmap::fromImage(image_org.rgbSwapped()));

       ui.skeleton_btn->setIcon(ButtonIcon);
       ui.skeleton_btn->setIconSize(image_org.rect().size());
    }
    else
    {
            getIcon_skeleton.open(UI_Icon_pose_detection);
    }
    ///////////////////////////////////////
    /// \brief video_icon
    ////////////////////////////////////////
    cv::Mat video_icon;
    getIcon_video >> video_icon;
    if(!video_icon.empty())
    {
       cv::resize(video_icon,video_icon,cv::Size(360,360));
       QImage image_org((const unsigned char*)(video_icon.data),video_icon.cols,video_icon.rows,QImage::Format_RGB888);
       QIcon ButtonIcon(QPixmap::fromImage(image_org.rgbSwapped()));

       ui.video_btn->setIcon(ButtonIcon);
       ui.video_btn->setIconSize(image_org.rect().size());
    }
    else
    {
            getIcon_video.open(UI_Icon_video);
    }
    //////////////////////////////////////////////////
    /// \brief display_icon
    //////////////////////////////////////////////////
    cv::Mat display_icon;
    getIcon_display >> display_icon;
    if(!display_icon.empty())
    {
       cv::resize(display_icon,display_icon,cv::Size(360,360));
       QImage image_org((const unsigned char*)(display_icon.data),display_icon.cols,display_icon.rows,QImage::Format_RGB888);
       QIcon ButtonIcon(QPixmap::fromImage(image_org.rgbSwapped()));

       ui.display_btn->setIcon(ButtonIcon);
       ui.display_btn->setIconSize(image_org.rect().size());
    }
    else
    {
            getIcon_display.open(UI_Icon_display);
    }
    /////////////////////////////////////////////////////
    /// \brief stop_icon
    /////////////////////////////////////////////////////
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
void MainWindow::on_video_btn_clicked()
{
    std::system("gnome-terminal -x sh -c 'rosrun kuro_video_ui kuro_video_ui;'");
}

void MainWindow::on_cotton_candy_btn_clicked()
{
    std::system("gnome-terminal -x sh -c 'rosrun kuro_cotton_ui kuro_cotton_ui;'");
}

void MainWindow::on_skeleton_btn_clicked()
{

}

void MainWindow::on_display_btn_clicked()
{
    std::system("gnome-terminal -x sh -c 'rosrun kuro_display_ui kuro_display_ui;'");
}

void MainWindow::on_stop_btn_clicked()
{
    this->close();
}




}  // namespace kuro_master_ui

