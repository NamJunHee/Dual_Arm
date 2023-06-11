
#ifndef order_kuro_QNODE_HPP_
#define order_kuro_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <iostream>

//..msg.....................
#include "order.h"
#include "joint_state.h"
#include "loca_order.h"
#include "fk_loca.h"
#include "Rec_arm_id.h"
#include "motion_name.h"
#include <msg_generate/Arm_path_coordinate_msg.h>
#include <msg_generate/elbow_updown_msg.h>

using namespace std;

namespace order_kuro {

class QNode : public QThread {
    Q_OBJECT
public:
        QNode(int argc, char** argv );
        virtual ~QNode();
        bool init();
        void run();

        ros::Publisher order_pub;
        ros::Publisher loca_order_pub;
        ros::Publisher loca_play_pub;
        ros::Publisher dxl_id_pub;
        ros::Publisher motion_name_pub;
        ros::Publisher elbow_updown_pub;

        ros::Subscriber state_sub;
        ros::Subscriber fk_sub;
        ros::Subscriber moving_path_coor_sub;

        void joint_stateCallback(const move_DualArm::joint_state::ConstPtr& msg);
        move_DualArm::joint_state jointinfo;

        void fk_location_Callback(const move_DualArm::fk_loca::ConstPtr& msg);
        move_DualArm::fk_loca fk_info;

        void moving_path_coordinate_Callback(const msg_generate::Arm_path_coordinate_msg::ConstPtr& msg);
        msg_generate::Arm_path_coordinate_msg moving_path_info;

Q_SIGNALS:
    void rosShutdown();
    void js_callback();
    void fk_callback();
    void moving_path_callback();

private:
	int init_argc;
	char** init_argv;

};

}  // namespace order_kuro

#endif /* order_kuro_QNODE_HPP_ */
