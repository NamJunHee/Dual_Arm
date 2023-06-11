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
#include <iostream>
#include "../include/kuro_video_ui/main_window.hpp"

#define UI_Background_Image "/home/robit/catkin_ws/src/kuro_video_ui/resources/images/robit_video.mp4"
#define UI_Stop_Icon "/home/robit/catkin_ws/src/kuro_video_ui/resources/images/stop_m.mp4"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kuro_video_ui {
using namespace cv;
using namespace Qt;


VideoCapture getBackground_kuro;
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
    ui.video_label->setPixmap(QPixmap::fromImage(image_org.rgbSwapped()));
    ui.video_label->setFixedSize(bgk.cols,bgk.rows);
    }
    else
    {
        getBackground_kuro.open(UI_Background_Image);
    }

    cv::Mat stop_icon;
    getIcon_stop >> stop_icon;
    if(!stop_icon.empty())
    {
    cv::resize(stop_icon,stop_icon,cv::Size(140,140));
    QImage image_org((const unsigned char*)(stop_icon.data), stop_icon.cols, stop_icon.rows, QImage::Format_RGB888);
    QIcon ButtonIcon(QPixmap::fromImage(image_org.rgbSwapped()));
    ui.stop_btn->setIcon(ButtonIcon);
    ui.stop_btn->setIconSize(image_org.rect().size());
    }
    else
    {
        getIcon_stop.open(UI_Stop_Icon);
    }

}

void MainWindow::on_stop_btn_clicked()
{
    this->close();
}

}  // namespace kuro_video_ui

