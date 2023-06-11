/**
 * @file /include/dynamixel_serial/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef dynamixel_serial_QNODE_HPP_
#define dynamixel_serial_QNODE_HPP_

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
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <msg_generate/Motor_msg.h>
#include <msg_generate/Motion_msg.h>
#include <msg_generate/Step_msg.h>
#include <msg_generate/Arm_path_position_msg.h>
#include <msg_generate/dxl_wheel_msg.h>
#include <msg_generate/motion_end.h>
#include "Rec_arm_id.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dynamixel_serial {

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

    ros::Publisher dxl_position_pub;
    ros::Publisher motion_end_pub;

    msg_generate::Motor_msg Dx_msg;
    msg_generate::Motion_msg Motion_msg;
    order_kuro::Rec_arm_id dxl_id_msg;
    msg_generate::dxl_wheel_msg wheel_msg;

Q_SIGNALS:
    void rosShutdown();
    void Rx_MotorData();
    void Rx_MotionData();
    void Rx_Dxl_id();
    void wheel_mode();


private:
	int init_argc;
	char** init_argv;
    void DxCallback(const msg_generate::Motor_msg::ConstPtr &msg);
    void MotionCallback(const msg_generate::Motion_msg::ConstPtr &msg);
    void Dxl_idCallback(const order_kuro::Rec_arm_id::ConstPtr &msg);
    void wheel_mode_Callback(const msg_generate::dxl_wheel_msg::ConstPtr &msg);

    ros::Subscriber Motor_sub;
    ros::Subscriber Motion_sub;
    ros::Subscriber dxl_id_sub;
    ros::Subscriber wheel_sub;


};

}  // namespace dynamixel_serial

#endif /* dynamixel_serial_QNODE_HPP_ */
