#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "../include/kuro/angle.h"
#include <functional>
#include <ignition/math/Vector3.hh>

using namespace std;

namespace gazebo
{
 class simulation_kuro : public ModelPlugin
 {
 private:

     physics::JointController * JC;
     physics::ModelPtr model;

     physics::JointPtr r_shoulder_roll;
     physics::JointPtr r_shoulder_pitch;
     physics::JointPtr r_shoulder_yaw;
     physics::JointPtr r_elbow_pitch;
     physics::JointPtr r_wrist_yaw;

     physics::JointPtr l_shoulder_roll;
     physics::JointPtr l_shoulder_pitch;
     physics::JointPtr l_shoulder_yaw;
     physics::JointPtr l_elbow_pitch;
     physics::JointPtr l_wrist_yaw;

     event::ConnectionPtr updateConnection;

     ros::NodeHandle n;
     ros::Subscriber sub;

     double r_ang[7] = {0.0, };
     double l_ang[7] = {0.0, };
     int time;

 public:

     simulation_kuro(){}
     ~simulation_kuro(){this->n.shutdown();}

     void Load(physics::ModelPtr _parent, sdf::ElementPtr)
     {
         int argc = 0;
         char** argv = NULL;

         ros::init(argc, argv, "simulation_kuro");
         sub = n.subscribe("angle", 100, &simulation_kuro::msgcallback, this);

         model = _parent;

         JC = new physics::JointController(model);

         r_shoulder_yaw   = model->GetJoint("r_shoulder_yaw");
         r_shoulder_roll  = model->GetJoint("r_shoulder_roll");
         r_shoulder_pitch = model->GetJoint("r_shoulder_pitch");
         r_elbow_pitch    = model->GetJoint("r_elobow_pitch");
         r_wrist_yaw      = model->GetJoint("r_wrist_yaw");

         l_shoulder_yaw   = model->GetJoint("l_shoulder_yaw");
         l_shoulder_roll  = model->GetJoint("l_shoulder_roll");
         l_shoulder_pitch = model->GetJoint("l_shoulder_pitch");
         l_elbow_pitch    = model->GetJoint("l_elbow_pitch");
         l_wrist_yaw      = model->GetJoint("l_wrist_yaw");

         //updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&simulation_kuro::OnUpdate,this));
         updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&simulation_kuro::OnUpdate, this, _1));

         time = 0;
     }

     void msgcallback(const kuro::angle::ConstPtr &msg)
     {
         r_ang[0]  = ((msg->r_ang_1)*3.141592)/180.0;
         r_ang[1]  = ((msg->r_ang_2)*(-1)*3.141592)/180.0;
         r_ang[2]  = (((msg->r_ang_3)+180)*(-1)*3.141592)/180.0;
         r_ang[3]  = ((msg->r_ang_4)*(-1)*3.141592)/180.0;
//         r_ang[4]  = ((msg->r_ang_5)*3.141592)/180.0;
//         r_ang[5]  = ((msg->r_ang_6)*3.141592)/180.0;
         r_ang[6]  = ((msg->r_ang_7)*3.141592)/180.0;

         l_ang[0]  = ((msg->l_ang_1)*3.141592)/180.0;
         l_ang[1]  = ((msg->l_ang_2)*3.141592)/180.0;
         l_ang[2]  = (((msg->l_ang_3)+180)*(-1)*3.141592)/180.0;
         l_ang[3]  = ((msg->l_ang_4)*3.141592)/180.0;
//         l_ang[4]  = ((msg->l_ang_5)*3.141592)/180.0;
//         l_ang[5]  = ((msg->l_ang_6)*3.141592)/180.0;
         l_ang[6]  = ((msg->l_ang_7)*3.141592)/180.0;

//         for(int i =0; i < 7; i++)
//         {
//             cout << "l_ang[" << i << "]" << l_ang[i] * 180 / 3.141592 << endl;
//         }
//         cout << endl;
     }

     void OnUpdate(const common::UpdateInfo &)
     {
         time += 1;
         if(time >= 100){
          time = 0;
         }

         JC->SetJointPosition(r_shoulder_roll,  r_ang[0]);
         JC->SetJointPosition(r_shoulder_pitch, r_ang[1]);
         JC->SetJointPosition(r_shoulder_yaw,   r_ang[2]);
         JC->SetJointPosition(r_elbow_pitch,    r_ang[3]);
         JC->SetJointPosition(r_wrist_yaw,      r_ang[6]);
\
         JC->SetJointPosition(l_shoulder_roll,  l_ang[0]);
         JC->SetJointPosition(l_shoulder_pitch, l_ang[1]);
         JC->SetJointPosition(l_shoulder_yaw,   l_ang[2]);
         JC->SetJointPosition(l_elbow_pitch,    l_ang[3]);
         JC->SetJointPosition(l_wrist_yaw,      l_ang[6]);
     }
 };

  GZ_REGISTER_MODEL_PLUGIN(simulation_kuro); //set plugin name
}

