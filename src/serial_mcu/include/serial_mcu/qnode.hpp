/**
 * @file /include/serial_mcu/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef serial_mcu_QNODE_HPP_
#define serial_mcu_QNODE_HPP_

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
#include "Motor_msg.h"
#include "Mt2Serial_msg.h"
#include "motion_end.h"


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace serial_mcu {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
        serial_mcu::Motor_msg Dx_msg;
        serial_mcu::Mt2Serial_msg Motion_msg;
        ros::Publisher Serial_pub;
	bool init();
	void run();


Q_SIGNALS:
        void Rx_MotorData();
        void Rx_MotionData();
        void rosShutdown();


private:
	int init_argc;
	char** init_argv;
        void DxCallback(const serial_mcu::Motor_msg::ConstPtr &msg);
        void MotionCallback(const serial_mcu::Mt2Serial_msg::ConstPtr &motion_msg);
        ros::Subscriber Motor_sub;
        ros::Subscriber Motion_sub;


};

}  // namespace serial_mcu

#endif /* serial_mcu_QNODE_HPP_ */
