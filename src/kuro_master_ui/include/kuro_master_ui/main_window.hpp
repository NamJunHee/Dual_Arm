/**
 * @file /include/kuro_master_ui/main_window.hpp
 *
 * @brief Qt based gui for kuro_master_ui.
 *
 * @date November 2010
 **/
#ifndef kuro_master_ui_MAIN_WINDOW_H
#define kuro_master_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtGui/QMessageBox>
#include <QPainter>
#include <QEvent>
#include <QTimer>
#include <QDebug>
#include <QString>
#include <QtCore>
#include <QPen>
#include <stdio.h>
#include <stdlib.h>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QThread>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kuro_master_ui {

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


public Q_SLOTS:
    void ui_update_callback();
    void on_video_btn_clicked();
    void on_cotton_candy_btn_clicked();
    void on_skeleton_btn_clicked();
    void on_display_btn_clicked();
    void on_stop_btn_clicked();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QRect screensize;

    void updateIconImage();
};

}  // namespace kuro_master_ui

#endif // kuro_master_ui_MAIN_WINDOW_H
