/**
 * @file /include/kuro_cotton_ui/main_window.hpp
 *
 * @brief Qt based gui for kuro_cotton_ui.
 *
 * @date November 2010
 **/
#ifndef kuro_cotton_ui_MAIN_WINDOW_H
#define kuro_cotton_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QWidget>
#include <QThread>
#include "qnode.hpp"
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
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtGui/QMessageBox>
#include <QPainter>
#include <QEvent>
#include <msg_generate/kuro_cotton_candy.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kuro_cotton_ui {
extern ros::Publisher cotton_candy_state_pub;
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
    //msg_generate::kuro_cotton_candy cotton_candy_msg;

void updateIconImage();
msg_generate::kuro_cotton_candy msg_cotton_candy_state;
public Q_SLOTS:

    void on_ready_btn_clicked();
    void on_start_btn_clicked();
    void on_stop_btn_clicked();
    void ui_update_callback();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QRect screensize;
};

}  // namespace kuro_cotton_ui

#endif // kuro_cotton_ui_MAIN_WINDOW_H
