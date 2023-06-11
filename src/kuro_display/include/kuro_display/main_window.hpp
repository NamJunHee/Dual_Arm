/**
 * @file /include/kuro_display/main_window.hpp
 *
 * @brief Qt based gui for kuro_display.
 *
 * @date November 2010
 **/
#ifndef kuro_display_MAIN_WINDOW_H
#define kuro_display_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtGui/QMessageBox>
#include <QPainter>
#include <QEvent>
#include <QTime>
#include <QDebug>
#include <QPainter>
#include <QTimer>
#include <QString>
#include <QtCore>
#include <QPen>
#include <stdio.h>
#include <stdlib.h>
#include <QWidget>
#include <QDesktopWidget>
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kuro_display {

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

private:
	Ui::MainWindowDesign ui;
        QNode qnode;
        QTimer* timer;

        QRect screensize;

        int displayChannel;
        bool changeDisplaySignal;

        void keyPressEvent(QKeyEvent *e);
        void updateImage();
        void selcetImage();
        void changeDisplay(int channel);

public Q_SLOTS:

        void slotTimerAlarm();



};

}  // namespace kuro_display

#endif // kuro_display_MAIN_WINDOW_H
