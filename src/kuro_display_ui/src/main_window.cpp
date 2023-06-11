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
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "../include/kuro_display_ui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/
#define UI_Background_Image "/home/robit/catkin_ws/src/kuro_display_ui/resources/images/kuro_face.mp4"
#define UI_Stop_Icon "/home/robit/catkin_ws/src/kuro_display_ui/resources/images/stop_m.mp4"
namespace kuro_display_ui {

using namespace Qt;
using namespace cv;
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
    ui.background_label->setPixmap(QPixmap::fromImage(image_org.rgbSwapped()));
    ui.background_label->setFixedSize(bgk.cols,bgk.rows);
    }
    else
    {
        getBackground_kuro.open(UI_Background_Image);
    }


}

void MainWindow::on_stop_btn_clicked()
{
    this->close();
}


}  // namespace kuro_display_ui

