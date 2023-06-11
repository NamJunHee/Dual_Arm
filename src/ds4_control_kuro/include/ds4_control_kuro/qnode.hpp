/**
 * @file /include/ds4_control_kuro/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ds4_control_kuro_QNODE_HPP_
#define ds4_control_kuro_QNODE_HPP_

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
#include </home/robit/catkin_ws/devel/include/msg_generate/ps4.h>
#include </home/robit/catkin_ws/devel/include/msg_generate/ps4arm_msg.h>
//#include <msg_generate/ps4.h>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ds4_control_kuro {

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

        ros::Subscriber ps4_sub;
        void ps4Callback(const msg_generate::ps4::ConstPtr &msg);
        msg_generate::ps4 ps4Info;

        ros::Publisher ps4arm_pub;


Q_SIGNALS:
    void rosShutdown();
    void QnodePs4Callback();

private:
	int init_argc;
	char** init_argv;

};

}  // namespace ds4_control_kuro

#endif /* ds4_control_kuro_QNODE_HPP_ */
