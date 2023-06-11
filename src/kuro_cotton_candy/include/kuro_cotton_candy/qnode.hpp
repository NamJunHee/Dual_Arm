/**
 * @file /include/kuro_cotton_candy/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef kuro_cotton_candy_QNODE_HPP_
#define kuro_cotton_candy_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include "motion_name.h"
#include <msg_generate/elbow_updown_msg.h>
#include <msg_generate/dxl_wheel_msg.h>
#include <msg_generate/motion_end.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kuro_cotton_candy {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT

public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
        void run();

    ros::Subscriber motion_end_sub;

    ros::Publisher motion_name_pub;
    ros::Publisher elbow_updown_pub;
    ros::Publisher wheel_mode_pub;

    void motion_end_callback(const msg_generate::motion_end::ConstPtr& msg);
    msg_generate::motion_end motion_end_info;

Q_SIGNALS:
    void rosShutdown();
    void motion_end_callback();

private:
	int init_argc;
	char** init_argv;

};

}  // namespace kuro_cotton_candy

#endif /* kuro_cotton_candy_QNODE_HPP_ */
