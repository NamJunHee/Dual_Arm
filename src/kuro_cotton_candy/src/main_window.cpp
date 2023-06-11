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
#include <QString>
#include <fstream>

#include "../include/kuro_cotton_candy/main_window.hpp"
#include "../include/kuro_cotton_candy/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kuro_cotton_candy {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.init();

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(motion_end_callback()), this, SLOT(motion_end_callback()));


    //motion_right_clicked..............
    ui.take_sugar_motion->installEventFilter(this);
    ui.pour_motion_pb->installEventFilter(this);
    ui.circle_motion_pb->installEventFilter(this);
    ui.spin_motion_pb->installEventFilter(this);
}

MainWindow::~MainWindow() {}

bool MainWindow::eventFilter(QObject *object, QEvent *event)
{
    if(object == ui.take_sugar_motion && event->type() == QEvent::MouseButtonPress)
    {
        QMouseEvent *keyEvent = static_cast<QMouseEvent *>(event);
        if(keyEvent->button() == Qt::RightButton)
        {
//            on_take_sugar_motion_rightclicked();
            return true;
        }
        return false;
    }

    if(object == ui.pour_motion_pb && event->type() == QEvent::MouseButtonPress)
    {
        QMouseEvent *keyEvent = static_cast<QMouseEvent *>(event);
        if(keyEvent->button() == Qt::RightButton)
        {
//            on_pour_motion_pb_rightclicked();
            return true;
        }
        else
            return false;
    }

    if(object == ui.circle_motion_pb && event->type() == QEvent::MouseButtonPress)
    {
        QMouseEvent *keyEvent = static_cast<QMouseEvent *>(event);
        if(keyEvent->button() == Qt::RightButton)
        {
//            on_circle_motion_pb_rightclicked();
            return true;
        }
        else
            return false;
    }

    if(object == ui.spin_motion_pb && event->type() == QEvent::MouseButtonPress)
    {
        QMouseEvent *keyEvent = static_cast<QMouseEvent *>(event);
        if(keyEvent->button() == Qt::RightButton)
        {
//            on_spin_motion_pb_rightclicked();
            return true;
        }
        else
            return false;
    }
}

//Cotton_Candy_Making..................................................................
void kuro_cotton_candy::MainWindow::on_making_ready_clicked()
{
    making_flag = 1;

    elbow_updown_info.elbow_updown = EL_DOWN;
    qnode.elbow_updown_pub.publish(elbow_updown_info);

    motion_set.circle_time = ui.circle_time->text().toInt();
    motion_set.spin_time = ui.spin_time->text().toInt();

    making_t = new QTimer(this);
    connect(making_t, SIGNAL(timeout()), this, SLOT(making_cotton_candy()));
    making_t->start(50); //0.05 sec

    motions_t = new QTimer(this);
    connect(motions_t,SIGNAL(timeout()), this, SLOT(motion_time()));
    motions_t->start(1000); //1 sec
}

void kuro_cotton_candy::MainWindow::on_making_start_clicked()
{
    making_start = 1;
}

void MainWindow::motion_time()
{
    m_timer++; //1 sec ++
}

void MainWindow::making_cotton_candy()
{
    switch (making_mode)
    {

    case READY_MOTION:

        cout << "READY_MOTION" << endl;

        if(!motion_flag)
        {
            if(first_start_flag)
            {
                motion_name_info.before_motion_name = " ";
                motion_name_info.now_motion_name = "first_init";
                cout << "first_init" << endl;
            }
            else
            {
                motion_name_info.before_motion_name = "end_motion";
                motion_name_info.now_motion_name = "Init_motion";
                cout << "Init_motion";
            }
            qnode.motion_name_pub.publish(motion_name_info);

            motion_flag = 1;
        }

        if(motion_end)
        {
            motion_flag = 0;
            motion_end = 0;
            m_timer = 0;
            making_mode = TAKE_SUGAR_MOTION;
        }

        break;

    case TAKE_SUGAR_MOTION:

        cout << "TAKE_SUGAR" << endl;

        if(!motion_flag)
        {
            motion_name_info.before_motion_name = " ";
            motion_name_info.now_motion_name = "take_sugar_motion";
            qnode.motion_name_pub.publish(motion_name_info);
            motion_flag = 1;
        }

        if(motion_end)
        {
            motion_flag = 0;
            motion_end = 0;
            m_timer = 0;
            making_mode = POUR_SUGAR_MOTION;
        }

        break;

    case POUR_SUGAR_MOTION:

        cout << "POUR_SUGAR" << endl;

        if(!motion_flag && making_start)
        {
            motion_name_info.before_motion_name = "take_sugar_motion";
            motion_name_info.now_motion_name = "pour_motion";
            qnode.motion_name_pub.publish(motion_name_info);
            motion_flag = 1;
            making_start = 0;
        }

        if(motion_end)
        {
            elbow_updown_info.elbow_updown = EL_UP;
            qnode.elbow_updown_pub.publish(elbow_updown_info);

            motion_flag = 0;
            motion_end = 0;
            m_timer = 0;
            making_mode = CIRCLE_SETTING_MOTION;
        }

        break;

    case CIRCLE_SETTING_MOTION:

        cout << "CIRCLE_SETTING_MOTION" << endl;

        if(!motion_flag)
        {
            motion_name_info.before_motion_name = "pour_motion";
            motion_name_info.now_motion_name = "circle_setting_motion";
            qnode.motion_name_pub.publish(motion_name_info);
            motion_flag = 1;
        }

        if(motion_end)
        {
            motion_flag = 0;
            motion_end = 1;
            m_timer = 0;
            making_mode = CIRCLE_MOTION;
        }

        break;

    case CIRCLE_MOTION:

        cout << "CIRCLE_MOTION" << endl;
        cout << "Circle_time >> " << motion_set.circle_time << endl;
        cout << "m_timer >> " << m_timer << endl;

        if(motion_end)
        {
            if(m_timer >= motion_set.circle_time && motion_end)
            {
                motion_end = 0;
                motion_flag = 0;
                m_timer = 0;
                making_mode = CIRCLE_END_MOTION;
            }
            else
            {
                if(!motion_flag)
                {
                    motion_name_info.before_motion_name = "circle_setting_motion";
                    motion_name_info.now_motion_name = "circle_motion";
                    qnode.motion_name_pub.publish(motion_name_info);

                    motion_flag = 1;
                }
                else
                {
                    motion_name_info.before_motion_name = "circle_motion";
                    motion_name_info.now_motion_name = "circle_motion";
                    qnode.motion_name_pub.publish(motion_name_info);

                    motion_end = 0;
                }
            }
        }

        break;

    case CIRCLE_END_MOTION:

        cout << "CIRCLE_END" << endl;

        if(!motion_flag)
        {
            elbow_updown_info.elbow_updown = EL_DOWN;
            qnode.elbow_updown_pub.publish(elbow_updown_info);

            motion_name_info.before_motion_name = "circle_motion";
            motion_name_info.now_motion_name = "circle_end_motion";
            qnode.motion_name_pub.publish(motion_name_info);
            motion_flag = 1;
        }

        if(motion_end)
        {
            motion_flag = 0;
            motion_end = 0;
            m_timer = 0;
            making_mode = SPIN_MOTION;
        }

        break;

    case SPIN_MOTION:

        cout << "SPIN_MOTION_2" << endl;
        cout << "Spin_time >> " << motion_set.spin_time << endl;
        cout << "m_timer >> " << m_timer << endl;

        if(!motion_flag)
        {
            motion_name_info.before_motion_name = "circle_end_motion";
            motion_name_info.now_motion_name = "spin_motion";
            qnode.motion_name_pub.publish(motion_name_info);

            wheel_mode_info.id = 60;
            wheel_mode_info.speed = 500;
            qnode.wheel_mode_pub.publish(wheel_mode_info);

            motion_flag = 1;
        }

        if(motion_end && m_timer >= motion_set.spin_time)
        {
            motion_flag = 0;
            motion_end = 0;
            m_timer = 0;
            making_mode = END_MOTION;
        }

        break;

    case END_MOTION:

        cout << "END_MOTION" << endl;

        if(!motion_flag)
        {
            motion_name_info.before_motion_name = "spin_motion";
            motion_name_info.now_motion_name = "end_motion";
            qnode.motion_name_pub.publish(motion_name_info);

            wheel_mode_info.id = 60;
            wheel_mode_info.speed = 0;
            qnode.wheel_mode_pub.publish(wheel_mode_info);

            motion_flag = 1;
        }

        if(motion_end)
        {
            first_start_flag = 0;
            motion_flag = 0;
            motion_end = 0;
            m_timer = 0;
            making_mode = READY_MOTION;

            making_t->stop();
            delete making_t;

            motions_t->stop();
            delete motions_t;

            making_flag = 0;
        }
        break;
    }
}
void MainWindow::motion_end_callback()
{
    motion_end = qnode.motion_end_info.motion_end;
}

void kuro_cotton_candy::MainWindow::take_sugar_motion()
{

}
void kuro_cotton_candy::MainWindow::pour_sugar_motion()
{

}
void kuro_cotton_candy::MainWindow::circle_motion()
{

}
void kuro_cotton_candy::MainWindow::spin_motion()
{

}
void kuro_cotton_candy::MainWindow::end_motion()
{

}

void kuro_cotton_candy::MainWindow::on_stop_pb_clicked()
{
    cout << "STOP!!" << endl;

    if(making_flag)
    {
        making_t->stop();
        delete making_t;
        making_flag = 0;
    }
}

//Motion_Setting.......................................................................

void kuro_cotton_candy::MainWindow::on_motion_setting_save_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("Save file"), "/home/robit/catkin_ws/src/kuro_cotton_candy/work");

    if(fileName.isEmpty()==true)
        qDebug() << "save Cancel";
    else
    {
        QFile *file = new QFile;
        file->setFileName(fileName);
        file->open(QIODevice::WriteOnly);
        QTextStream out(file);

        out << motion_set.circle_time << endl
        << motion_set.spin_time << endl;
        file->close();
    }
}

void kuro_cotton_candy::MainWindow::on_motion_setting_open_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open file"), "/home/robit/catkin_ws/src/kuro_cotton_candy/work");

    if(fileName.isEmpty() == true)
        qDebug() << "Open cancel";
    else
    {
        ifstream is;

        is.open(fileName.toStdString().c_str());

        is >> motion_set.circle_time;
        is >> motion_set.spin_time;
        is.close();

        ui.circle_time->setText(QString::number(motion_set.circle_time));
        ui.spin_time->setText(QString::number(motion_set.spin_time));
     }
}

void kuro_cotton_candy::MainWindow::on_take_sugar_motion_clicked()
{
    motion_name_info.before_motion_name = " ";
    motion_name_info.now_motion_name = "take_sugar_motion";
    qnode.motion_name_pub.publish(motion_name_info);
}
void kuro_cotton_candy::MainWindow::on_pour_motion_pb_clicked()
{
    motion_name_info.before_motion_name = "take_sugar_motion";
    motion_name_info.now_motion_name = "pour_motion";
    qnode.motion_name_pub.publish(motion_name_info);

    elbow_updown_info.elbow_updown = EL_UP;
    qnode.elbow_updown_pub.publish(elbow_updown_info);
}
void kuro_cotton_candy::MainWindow::on_circle_motion_pb_clicked()
{
    if(i == 0)
    {
        motion_name_info.before_motion_name = "pour_motion";
        motion_name_info.now_motion_name = "circle_setting_motion";
        qnode.motion_name_pub.publish(motion_name_info);

        i++;
    }
    else if(i == 1)
    {
        motion_name_info.before_motion_name = "circle_setting_motion";
        motion_name_info.now_motion_name = "circle_motion";
        qnode.motion_name_pub.publish(motion_name_info);

        i++;
    }
    else if(i == 2)
    {
        motion_name_info.before_motion_name = "circle_motion";
        motion_name_info.now_motion_name = "circle_motion";
        qnode.motion_name_pub.publish(motion_name_info);
    }

}
void kuro_cotton_candy::MainWindow::on_spin_motion_pb_clicked()
{
    motion_name_info.now_motion_name = "spin_motion";
    qnode.motion_name_pub.publish(motion_name_info);

    wheel_mode_info.id = 60;
    wheel_mode_info.speed = 500;
    qnode.wheel_mode_pub.publish(wheel_mode_info);
}

void kuro_cotton_candy::MainWindow::on_circle_time_textChanged(const QString &arg1)
{
    motion_set.circle_time = ui.circle_time->text().toInt();
}

void kuro_cotton_candy::MainWindow::on_spin_time_textChanged(const QString &arg1)
{
    motion_set.spin_time = ui.spin_time->text().toInt();
}

/*****************************************************************************
** Functions
*****************************************************************************/


}  // namespace kuro_cotton_candy

