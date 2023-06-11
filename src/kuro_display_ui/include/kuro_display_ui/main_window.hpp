/**
 * @file /include/kuro_display_ui/main_window.hpp
 *
 * @brief Qt based gui for kuro_display_ui.
 *
 * @date November 2010
 **/
#ifndef kuro_display_ui_MAIN_WINDOW_H
#define kuro_display_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kuro_display_ui {

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
    void on_stop_btn_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QRect screensize;
};

}  // namespace kuro_display_ui

#endif // kuro_display_ui_MAIN_WINDOW_H
