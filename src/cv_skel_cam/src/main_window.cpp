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
#include <cmath>
#include "../include/cv_skel_cam/main_window.hpp"


geometry_msgs::Point standard;
bool m_visibleUsers[MAX_USERS] = {false};
float revleft_arm[3][3];
float revright_arm[3][3];

nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

#define USER_MESSAGE(msg) \
{printf("[%08lu] User #%d:\t%s\n",ts, user.getId(),msg);}

#define FILTERDATA 10
#define PI 3.14159265359
#define MAX_RIGHT_ANGLE 55
#define MAX_LEFT_ANGLE 60
#define SHOULDER_ELBOW 180//200
#define ELBOW_HAND 210//289    //319

//int data[FILTERDATA] = {0, };
std::vector<skeleton_message::UserData> data;
skeleton_message::UserData avg (skeleton_message::UserData a)
{
    static int count = 0;

    if(count++ > FILTERDATA)
    {
        unsigned char i;

        skeleton_message::UserData sum, average;
        sum = a;
        average = a;
        for(i = 0; i < a.skeletonJoints.size(); i++)
        {
            for (int k = 0; k < FILTERDATA - 1; k++)
            {
                data[k].skeletonJoints[i].location.x = data[k+1].skeletonJoints[i].location.x;
                data[k].skeletonJoints[i].location.y = data[k+1].skeletonJoints[i].location.y;
                data[k].skeletonJoints[i].location.z = data[k+1].skeletonJoints[i].location.z;
            }
        }
        for(i = 0; i < a.skeletonJoints.size(); i++)
        {
            data[FILTERDATA-1].skeletonJoints[i].location.x = a.skeletonJoints[i].location.x;
            data[FILTERDATA-1].skeletonJoints[i].location.y = a.skeletonJoints[i].location.y;
            data[FILTERDATA-1].skeletonJoints[i].location.z = a.skeletonJoints[i].location.z;

            for (int k = 0; k < FILTERDATA; k++)
            {
                sum.skeletonJoints[i].location.x += data[k].skeletonJoints[i].location.x;
                sum.skeletonJoints[i].location.y += data[k].skeletonJoints[i].location.y ;
                sum.skeletonJoints[i].location.z += data[k].skeletonJoints[i].location.z ;
            }
            average.skeletonJoints[i].location.x = sum.skeletonJoints[i].location.x/ FILTERDATA;
            average.skeletonJoints[i].location.y = sum.skeletonJoints[i].location.y/ FILTERDATA;
            average.skeletonJoints[i].location.z = sum.skeletonJoints[i].location.z/ FILTERDATA;
        }
        return average;
    }
    else
    {
        count ++;

        unsigned char i;



        for(i = 0; i < a.skeletonJoints.size(); i++)
        {

            for (int k = 0; k < FILTERDATA - 1; k++)
            {
                data.push_back(a);
            }
        }
        return a;
    }

}

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace cv_skel_cam {

Skeleton_Viewer* Skeleton_Viewer::ms_self = NULL;

using namespace std;
using namespace Qt;
using namespace ros;


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/
Skeleton_Viewer user_trakcing("tracking");

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    setWindowIcon(QIcon(":/images/icon.png"));

    qnode.init();



    cout<<"afafaf"<<endl;

    user_trakcing.tracking_Init(argc,argv);
    cout<<"fin Init"<<endl;

    ros::init(argc, argv, "pub_img");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_pub = it.advertise("/camera/color", 100);

    mtimer = new QTimer(this);
    connect(mtimer, SIGNAL(timeout()),this, SLOT(TrackingCallback()));
    mtimer->start(10);

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    cv_bridge::CvImagePtr cv_ptr_m(new cv_bridge::CvImage);
    cv_ptr = cv_ptr_m;
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Functions
*****************************************************************************/

Skeleton_Viewer::Skeleton_Viewer(const char* strSampleName) :
    m_poseUser(0)
{
    ms_self = this;
    strncpy(m_strSampleName, strSampleName, ONI_MAX_STR);
    m_pUserTracker = new nite::UserTracker;
    cv::Mat a(640,480,CV_8UC3);
    color_Mat = a.clone();

}

int Skeleton_Viewer::tracking_Init(int argc, char **argv){

    openni::Status rc = openni::STATUS_OK;

    rc = openni::OpenNI::initialize();

    if (rc != openni::STATUS_OK)
    {
        printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
        return rc;
    }

    const char* deviceURI = openni::ANY_DEVICE;
    for (int i = 1; i < argc-1; ++i)
    {
        if (strcmp(argv[i], "-device") == 0)
        {
            deviceURI = argv[i+1];
            break;
        }
    }


    rc = m_device.open(deviceURI);
    if (rc != openni::STATUS_OK)
    {
        printf("SimpleViewer: Device open failed:\n%s\n", openni::OpenNI::getExtendedError());
        openni::OpenNI::shutdown();
        return 1;
    }


    nite::NiTE::initialize();

    if (m_pUserTracker->create(&m_device) != nite::STATUS_OK)
    {
        return openni::STATUS_ERROR;
    }


    printf("After initialization:\n%s\n", openni::OpenNI::getExtendedError());
    cout<<"DEPTH_RC"<<endl<<endl;
    rc = m_depthStream.create(m_device, openni::SENSOR_DEPTH);
    if (rc == openni::STATUS_OK)
    {
        rc = m_depthStream.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start depth stream:\n%s\n", openni::OpenNI::getExtendedError());
            m_depthStream.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find depth stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    cout<<"COLOR_RC"<<endl<<endl;
    rc = m_colorStream.create(m_device, openni::SENSOR_COLOR);
    if (rc == openni::STATUS_OK)
    {
        rc = m_colorStream.start();
        if (rc != openni::STATUS_OK)
        {
            printf("SimpleViewer: Couldn't start color stream:\n%s\n", openni::OpenNI::getExtendedError());
            m_colorStream.destroy();
        }
    }
    else
    {
        printf("SimpleViewer: Couldn't find color stream:\n%s\n", openni::OpenNI::getExtendedError());
    }

    if (!m_depthStream.isValid() || !m_colorStream.isValid())
    {
        printf("SimpleViewer: No valid streams. Exiting\n");
        openni::OpenNI::shutdown();
        return 2;
    }
    cout<<"Ros_init"<<endl<<endl;


    ros::init(argc, argv, "skeleton_tracker");
    ros::NodeHandle nh, nh_priv("~");

    UserPub = nh.advertise<skeleton_message::Users>("Skeleton_User",100);


    cout<<"main init ros"<<endl<<endl;

    colors[0] = Scalar(0,0,0);
    colors[1] = Scalar(255,0,0);
    colors[2] = Scalar(0,255,0);
    colors[3] = Scalar(0,0,255);
    colors[4] = Scalar(255,255,0);
    colors[5] = Scalar(0,255,255);
    colors[6] = Scalar(255,0,255);
    colors[7] = Scalar(255,255,255);
    colors[8] = Scalar(100,100,100);
    colors[9] = Scalar(200,200,200);


    return openni::STATUS_OK;


}


Skeleton_Viewer::~Skeleton_Viewer()
{
    Finalize();

    delete[] m_pTexMap;

    ms_self = NULL;
}

void Skeleton_Viewer::Finalize()
{
    delete m_pUserTracker;
    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();
}


void Skeleton_Viewer::updateUserState(const nite::UserData &user, uint64_t ts)
{
    if (user.isNew()){
        USER_MESSAGE("New")
                a[user.getId()] = true;
    }
    else if (user.isVisible() && !m_visibleUsers[user.getId()])
        USER_MESSAGE("Visible")
                else if (!user.isVisible() && m_visibleUsers[user.getId()]){
            USER_MESSAGE("Out of Scene")
                    a[user.getId()] = false;
        }
            else if (user.isLost()){
            USER_MESSAGE("Lost")
                    a[user.getId()] = false;
        }

    m_visibleUsers[user.getId()] = user.isVisible();


    if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
    {
        switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
        {
        case nite::SKELETON_NONE:
            USER_MESSAGE("Stopped tracking.")
                    a[user.getId()] = false;
            break;
        case nite::SKELETON_CALIBRATING:
            USER_MESSAGE("Calibrating...")
                    break;
        case nite::SKELETON_TRACKED:
            USER_MESSAGE("Tracking!")
                    break;
        case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
        case nite::SKELETON_CALIBRATION_ERROR_HANDS:
        case nite::SKELETON_CALIBRATION_ERROR_LEGS:
        case nite::SKELETON_CALIBRATION_ERROR_HEAD:
        case nite::SKELETON_CALIBRATION_ERROR_TORSO:
            USER_MESSAGE("Calibration Failed... :-|")
                    break;
        }
    }
}

void Skeleton_Viewer::convertMat(){


    m_colorStream.readFrame(&m_colorFrame);
    if(!m_colorFrame.isValid()){
        throw std::runtime_error( "failed can not retrieve depth frame" );
        std::exit( EXIT_FAILURE );
    }

    color_width = m_colorFrame.getWidth();
    color_height = m_colorFrame.getHeight();

    color_Mat = cv::Mat(color_height,color_width,CV_BGR2RGB,const_cast<void*>(m_colorFrame.getData()));

    m_depthStream.readFrame(&m_depthFrame);
    if( !m_depthFrame.isValid() ){
        throw std::runtime_error( "failed can not retrieve depth frame" );
        std::exit( EXIT_FAILURE );
    }
    depth_width = m_depthFrame.getWidth();
    depth_height = m_depthFrame.getHeight();

    depth_Mat = cv::Mat( depth_height, depth_width, CV_16UC1, const_cast<void*>( m_depthFrame.getData() ) );

    depth_Mat.convertTo( depth_Mat, CV_8U, -255.0 / 10000.0, 255.0 ); // 0-10000 -> 255(white)-0(black)
    //    depth_Mat.convertTo( depth_Mat, CV_8U, 255.0 / 10000.0, 0.0 ); // 0-10000 -> 0(black)-255(white)

    cv::cvtColor( depth_Mat, depth_Mat, cv::COLOR_GRAY2RGB );
}

geometry_msgs::Point getPoint(const nite::SkeletonJoint& m_Joint){
    geometry_msgs::Point tmp;
    if(m_Joint.getType() == 0)
    {
        standard.x =m_Joint.getPosition().x;
        standard.y =m_Joint.getPosition().y;
        standard.z =m_Joint.getPosition().z;
        tmp.y = 0;
        tmp.z = 0;
        tmp.x = 0;
    }
    else
    {
        tmp.y = -m_Joint.getPosition().x + standard.x;
        tmp.z = m_Joint.getPosition().y - standard.y;
        tmp.x = -(m_Joint.getPosition().z - standard.z);
    }
    return tmp;
}
nite::JointType& operator ++ (nite::JointType& e)
{
    if (e == nite::JointType::JOINT_RIGHT_FOOT) {
        //        throw std::out_of_range("for E& operator ++ (JointType&)");
    }
    e = nite::JointType(static_cast<std::underlying_type<nite::JointType>::type>(e) + 1);
    return e;
}

float length(float x1,float y1,float z1,float x2,float y2,float z2,float *arr,cv::String compare)
{
    float m_result, result, ratio;
    m_result = pow((x1-x2),2) + pow((y1-y2),2) + pow((z1-z2),2);
    result = sqrt(m_result);

    if(compare == "ELBOW")
    {

        ratio = SHOULDER_ELBOW/result;
        arr[0] = ratio * (x1-x2);
        arr[1] = ratio * (y1-y2);
        arr[2] = ratio * (z1-z2);

    }
    else if(compare == "HAND")
    {
        ratio = ELBOW_HAND/result;
        arr[0] = ratio * (x1-x2);
        arr[1] = ratio * (y1-y2);
        arr[2] = ratio * (z1-z2);
    }
    return result;
}
skeleton_message::UserData arm_length (skeleton_message::UserData m_UD,float left_arm[][3],float right_arm[][3])
{
    float add[3]={0,0,0};
    cout<<"RIGHT Shoulder ~  Elbow AAA==============             "<<length(left_arm[0][0],left_arm[0][1],left_arm[0][2],left_arm[1][0],left_arm[1][1],left_arm[1][2],add,"ELBOW")<<endl<<endl;
    if(left_arm[1][0]>=0)
        m_UD.skeletonJoints[4].location.x = abs(add[0])+left_arm[0][0];
    else
        m_UD.skeletonJoints[4].location.x = left_arm[0][0]-abs(add[0]);

    if(left_arm[1][1]>=0)
        m_UD.skeletonJoints[4].location.y = abs(add[1])+left_arm[0][1];
    else
        m_UD.skeletonJoints[4].location.y = left_arm[0][1]-abs(add[1]);
    if(left_arm[1][2]>left_arm[0][2])
        m_UD.skeletonJoints[4].location.z = abs(add[2])+left_arm[0][2];
    else
        m_UD.skeletonJoints[4].location.z = left_arm[0][2]-abs(add[2]);

    cout<<"RIGHT Shoulder ~  Elbow FFF==============             "<<length(left_arm[0][0],left_arm[0][1],left_arm[0][2],
            m_UD.skeletonJoints[4].location.x,m_UD.skeletonJoints[4].location.y,m_UD.skeletonJoints[4].location.z,
            add,"ELBOW")<<endl<<endl;

    /**********************************************************************************************************************************************************************/
    cout<<"LEFT Shoulder ~  Elbow AAA==============             "<<length(right_arm[0][0],right_arm[0][1],right_arm[0][2],right_arm[1][0],right_arm[1][1],right_arm[1][2],add,"ELBOW")<<endl<<endl;
    if(right_arm[1][0]>=0)
        m_UD.skeletonJoints[5].location.x = abs(add[0])+right_arm[0][0];
    else
        m_UD.skeletonJoints[5].location.x = right_arm[0][0]-abs(add[0]);

    if(right_arm[1][1]>=0)
        m_UD.skeletonJoints[5].location.y = abs(add[1])+right_arm[0][1];
    else
        m_UD.skeletonJoints[5].location.y = right_arm[0][1]-abs(add[1]);

    if(right_arm[1][2]>right_arm[0][2])
        m_UD.skeletonJoints[5].location.z = abs(add[2])+right_arm[0][2];
    else
        m_UD.skeletonJoints[5].location.z = right_arm[0][2]-abs(add[2]);

    cout<<"LEFT Shoulder ~  Elbow FFF==============             "<<length(right_arm[0][0],right_arm[0][1],right_arm[0][2],
            m_UD.skeletonJoints[5].location.x,m_UD.skeletonJoints[5].location.y,m_UD.skeletonJoints[5].location.z,
            add,"ELBOW")<<endl<<endl;

    /**********************************************************************************************************************************************************************/
    cout<<"RIGHT Elbow ~  Hand AAA==============            "<<length(m_UD.skeletonJoints[4].location.x,m_UD.skeletonJoints[4].location.y,m_UD.skeletonJoints[4].location.z,
            left_arm[2][0],left_arm[2][1],left_arm[2][2],
            add,"HAND")<<endl<<endl;
    if(left_arm[2][0]>=0)
        m_UD.skeletonJoints[6].location.x = abs(add[0])+m_UD.skeletonJoints[4].location.x;
    else
        m_UD.skeletonJoints[6].location.x = m_UD.skeletonJoints[4].location.x-abs(add[0]);

    if(left_arm[2][1]>=left_arm[0][2])
        m_UD.skeletonJoints[6].location.y = abs(add[1])+m_UD.skeletonJoints[4].location.y;
    else
        m_UD.skeletonJoints[6].location.y = m_UD.skeletonJoints[4].location.y-abs(add[1]);

    if(left_arm[2][2]>=left_arm[0][2])
        m_UD.skeletonJoints[6].location.z = abs(add[2])+m_UD.skeletonJoints[4].location.z;
    else
        m_UD.skeletonJoints[6].location.z = m_UD.skeletonJoints[4].location.z-abs(add[2]);

    cout<<"RIGHT Elbow ~  Hand FFF==============            "<<length(m_UD.skeletonJoints[4].location.x,m_UD.skeletonJoints[4].location.y,m_UD.skeletonJoints[4].location.z,
            m_UD.skeletonJoints[6].location.x,m_UD.skeletonJoints[6].location.y,m_UD.skeletonJoints[6].location.z,
            add,"HAND")<<endl<<endl;
    /**********************************************************************************************************************************************************************/
    cout<<"LEFT Elbow ~  Hand AAA==============            "<<length(m_UD.skeletonJoints[5].location.x,m_UD.skeletonJoints[5].location.y,m_UD.skeletonJoints[5].location.z,
            right_arm[2][0],right_arm[2][1],right_arm[2][2],
            add,"HAND")<<endl<<endl;
    if(right_arm[2][0]>=0)
        m_UD.skeletonJoints[7].location.x = abs(add[0])+m_UD.skeletonJoints[5].location.x;
    else
        m_UD.skeletonJoints[7].location.x = m_UD.skeletonJoints[5].location.x-abs(add[0]);

    if(right_arm[2][1]>=0)
        m_UD.skeletonJoints[7].location.y = abs(add[1])+m_UD.skeletonJoints[5].location.y;
    else
        m_UD.skeletonJoints[7].location.y = m_UD.skeletonJoints[5].location.y-abs(add[1]);

    if(right_arm[2][2]>=right_arm[0][2])
        m_UD.skeletonJoints[7].location.z = abs(add[2])+m_UD.skeletonJoints[5].location.z;
    else
        m_UD.skeletonJoints[7].location.z = m_UD.skeletonJoints[5].location.z-abs(add[2]);

    cout<<"LEFT Elbow ~  Hand FFF==============            "<<length(m_UD.skeletonJoints[5].location.x,m_UD.skeletonJoints[5].location.y,m_UD.skeletonJoints[5].location.z,
            m_UD.skeletonJoints[7].location.x,m_UD.skeletonJoints[7].location.y,m_UD.skeletonJoints[7].location.z,
            add,"HAND")<<endl<<endl;

    return m_UD;
}
skeleton_message::UserData motion_revise(skeleton_message::UserData m_UD,float left_arm[][3],float right_arm[][3]){

    bool left_arm_flag = true;
    bool right_arm_flag = true;
    /**********************************************************************************************************************************************************************/
    //몸통 안쪽으로 손이 들어가는 것을 방지하는 부분 ->tracking 실패하는 것을 방지
    for(int i = 0; i<3 ; i++){
        left_arm[i][1] = m_UD.skeletonJoints[2*(i+1)].location.y;
    }
    for(int i = 0; i<3 ; i++){
        right_arm[i][1] = m_UD.skeletonJoints[2*(i+1) +1].location.y ;
    }

    /**********************************************************************************************************************************************************************/
    for(int i = 1; i<3 ; i++){
        m_UD.skeletonJoints[2*(i+1)].location.y = -left_arm[i][1];
    }
    for(int i = 1; i<3 ; i++){
        m_UD.skeletonJoints[2*(i+1) +1].location.y = -right_arm[i][1];
    }
    /**********************************************************************************************************************************************************************/
    //손이 얼굴쪽으로 가면 tracking Error 가 발생하여 생기는 오류 방지
    if(left_arm_flag == true){
        for(int i = 0; i<3;i++)
        {
            revleft_arm[i][0]= m_UD.skeletonJoints[2*(i+1)].location.x;
            revleft_arm[i][1]= m_UD.skeletonJoints[2*(i+1)].location.y;
            revleft_arm[i][2]= m_UD.skeletonJoints[2*(i+1)].location.z;
        }
    }
    else{
        for(int i = 0; i<3;i++)
        {
            m_UD.skeletonJoints[2*(i+1)].location.x = revleft_arm[i][0];
            m_UD.skeletonJoints[2*(i+1)].location.y = revleft_arm[i][1];
            m_UD.skeletonJoints[2*(i+1)].location.z = revleft_arm[i][2];
        }
        left_arm_flag = true;
    }
    if(right_arm_flag == true){
        for(int i = 0; i<3;i++)
        {
            revright_arm[i][0]= m_UD.skeletonJoints[2*(i+1)+1].location.x;
            revright_arm[i][1]= m_UD.skeletonJoints[2*(i+1)+1].location.y;
            revright_arm[i][2]= m_UD.skeletonJoints[2*(i+1)+1].location.z;
        }
    }
    else{
        for(int i = 0; i<3;i++)
        {
            m_UD.skeletonJoints[2*(i+1)+1].location.x = revright_arm[i][0];
            m_UD.skeletonJoints[2*(i+1)+1].location.y = revright_arm[i][1];
            m_UD.skeletonJoints[2*(i+1)+1].location.z = revright_arm[i][2];
        }
        right_arm_flag = true;
    }


    return m_UD;
}
geometry_msgs::Point crossProduct(geometry_msgs::Point a, geometry_msgs::Point b,geometry_msgs::Point& c){
    c.x = (a.y*b.z) - (a.z*b.y);
    c.y = (a.z*b.x) - (a.x*b.z);
    c.z = (a.x*b.y) - (a.y*b.x);
}

void Skeleton_Viewer::PublishSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& user){

    skeleton_message::UserData m_UD;
    skeleton_message::SkeletonJoint m_SK;
    float left_arm[3][3]={{0},};
    float right_arm[3][3]={{0},};
    geometry_msgs::Point m_COM;
    m_COM.y = user.getCenterOfMass().x;
    m_COM.z = user.getCenterOfMass().y;
    m_COM.x = user.getCenterOfMass().z;
    m_Users.numberofUsers = user.getId();
    for(nite::JointType i = nite::JOINT_HEAD; i < nite::JOINT_RIGHT_FOOT +1; ++i)
    {
        const nite::SkeletonJoint& m_tmp = user.getSkeleton().getJoint(i);
        m_SK.type = i;
        m_SK.location = getPoint(m_tmp);
        m_UD.skeletonJoints.push_back(m_SK);
    }
    float right_arm_angle = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER).getOrientation().w)*(180/PI);
    float trans_rf_arm_angle = (right_arm_angle-43)*(MAX_RIGHT_ANGLE/12);
    float left_arm_angle = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER).getOrientation().w)*(180/PI);
    float trans_lf_arm_angle = (left_arm_angle-45)*(MAX_LEFT_ANGLE/12);
        /**********************************************************************************************************************************************************************/
    //양 팔의 어깨를 (0,0,0)으로 초기화
    left_arm[0][0] = m_UD.skeletonJoints[2].location.x;
    left_arm[0][1] = m_UD.skeletonJoints[2].location.y;
    left_arm[0][2] = m_UD.skeletonJoints[2].location.z;

    right_arm[0][0] = m_UD.skeletonJoints[3].location.x;
    right_arm[0][1] = m_UD.skeletonJoints[3].location.y;
    right_arm[0][2] = m_UD.skeletonJoints[3].location.z;
    for(int i = 1; i<3 ; i++){
        left_arm[i][0] = m_UD.skeletonJoints[2*(i+1)].location.x - left_arm[0][0];
        left_arm[i][1] = m_UD.skeletonJoints[2*(i+1)].location.y - left_arm[0][1];
        left_arm[i][2] = m_UD.skeletonJoints[2*(i+1)].location.z - left_arm[0][2];
    }
    for(int i = 1; i<3 ; i++){
        right_arm[i][0] = m_UD.skeletonJoints[2*(i+1) +1].location.x - right_arm[0][0];
        right_arm[i][1] = m_UD.skeletonJoints[2*(i+1) +1].location.y - right_arm[0][1];
        right_arm[i][2] = m_UD.skeletonJoints[2*(i+1) +1].location.z - right_arm[0][2];
    }

    left_arm[0][0] = 0;
    left_arm[0][1] = 0;
    left_arm[0][2] = 0;

    right_arm[0][0] = 0;
    right_arm[0][1] = 0;
    right_arm[0][2] = 0;

    m_UD.skeletonJoints[2].location.x = 0;
    m_UD.skeletonJoints[2].location.y = 0;
    m_UD.skeletonJoints[2].location.z = 0;

    m_UD.skeletonJoints[3].location.x = 0;
    m_UD.skeletonJoints[3].location.y = 0;
    m_UD.skeletonJoints[3].location.z = 0;
    /**********************************************************************************************************************************************************************/
    m_UD = arm_length(m_UD,left_arm,right_arm);//팔 크기(비율) 계산
    // 좌표 전송시 1의 자리는 제거한 후 10의 자리 이상부터 전송하는 part
    int num_change[3] = {0,};
    for(nite::JointType i = nite::JOINT_HEAD; i < nite::JOINT_RIGHT_FOOT +1; ++i)
    {
        num_change[0] = m_UD.skeletonJoints[i].location.x / 10 ;
        num_change[1] = m_UD.skeletonJoints[i].location.y / 10;
        num_change[2] = m_UD.skeletonJoints[i].location.z / 10;
        m_UD.skeletonJoints[i].location.x = num_change[0] * 10;
        m_UD.skeletonJoints[i].location.y = num_change[1] * 10;
        m_UD.skeletonJoints[i].location.z = num_change[2] * 10;
    }

    m_UD = motion_revise(m_UD,left_arm,right_arm);//Pose LIMIT Function
    /**********************************************************************************************************************************************************************/
    if(trans_lf_arm_angle < 60 && trans_lf_arm_angle>5)
        m_UD.skeletonJoints[0].location.x = trans_lf_arm_angle;//왼쪽 팔 각도 넣어주는 부분
    if(trans_rf_arm_angle < 55 && trans_rf_arm_angle>5)
        m_UD.skeletonJoints[0].location.y = -trans_rf_arm_angle;//오른쪽 팔 각도 넣어주는 부분
    /**********************************************************************************************************************************************************************/
    m_UD = avg(m_UD);//중간값 필터링
    /**********************************************************************************************************************************************************************/
    if(m_UD.skeletonJoints[6].location.x<3) //손이 뒤쪽으로 이동하는 것을 방지
        m_UD.skeletonJoints[6].location.x = -m_UD.skeletonJoints[6].location.x;
    if(m_UD.skeletonJoints[7].location.x<3)
        m_UD.skeletonJoints[7].location.x = -m_UD.skeletonJoints[7].location.x;
    /**********************************************************************************************************************************************************************/
    m_UD.UserID = user.getId();
    m_UD.UserState = a[user.getId()];
    m_UD.centerOfMass = m_COM;
    m_Users.Users.push_back(m_UD);
}

inline nite::Status Skeleton_Viewer::convertJointCoordinatesToDepth( const float x, const float y, const float z, float * pOutX, float * pOutY )
{
    Rs2PointPixel proj = { 0.0 };
    proj.point[0] = x;
    proj.point[1] = y;
    proj.point[2] = z;

    OPENNI_CHECK( m_device.invoke( RS2_PROJECT_POINT_TO_PIXEL, reinterpret_cast< void* >( &proj ), static_cast< int >( sizeof( proj ) ) ) );

    *pOutX = proj.pixel[0];
    *pOutY = depth_height - proj.pixel[1];

    return nite::Status::STATUS_OK;
}

void Skeleton_Viewer::DrawSkeleton(nite::UserTracker *pUserTracker, const nite::UserData &user){
    Point2f a;
    int id = user.getId();
    for(nite::JointType i = nite::JOINT_HEAD; i < nite::JOINT_RIGHT_FOOT +1; ++i)
    {

        const nite::Point3f& tmpPos = user.getSkeleton().getJoint(i).getPosition();
        NITE_CHECK(convertJointCoordinatesToDepth(tmpPos.x,tmpPos.y,tmpPos.z, &a.x,&a.y));
        cv::circle(depth_Mat,a,2,colors[id],4);
    }
}

void Skeleton_Viewer::DrawCOM(nite::UserTracker *pUserTracker, const nite::UserData &user){
    int id = user.getId();
    string UserId = " User : " + to_string(id);
    Point2f a;
    const nite::Point3f& tmpCOM = user.getCenterOfMass();

    NITE_CHECK(convertJointCoordinatesToDepth(tmpCOM.x,tmpCOM.y,tmpCOM.z, &a.x,&a.y));
    cv::circle(depth_Mat,a,5,colors[id],4);
    cv::putText(depth_Mat, UserId, a, CV_FONT_HERSHEY_PLAIN, 1 , colors[id],4);

    //    cout<<UserId<<endl;

}

void Skeleton_Viewer::trackingLoop(){
    nite::UserTrackerFrameRef userTrackerFrame;
    nite::Status rc = m_pUserTracker->readFrame(&userTrackerFrame);

    const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
    m_Users.Users.clear();
    for (int i = 0; i < users.getSize(); ++i)
    {
        const nite::UserData& user = users[i];

        updateUserState(user, userTrackerFrame.getTimestamp());

        if (user.isNew())
        {

            m_pUserTracker->startSkeletonTracking(user.getId());
            m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
        }
        else if (!user.isLost())
        {
            //        if (g_drawStatusLabel)
            //        {
            //          DrawStatusLabel(m_pUserTracker, user);
            //        }
            //        if (g_drawCenterOfMass)
            //        {
            //          DrawCenterOfMass(m_pUserTracker, user);
            //        }
            //        if (g_drawBoundingBox)
            //        {
            //          DrawBoundingBox(user);
            //        }

            if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)
            {
                DrawSkeleton(m_pUserTracker, user);
                DrawCOM(m_pUserTracker,user);
                PublishSkeleton(m_pUserTracker, user);
            }

        }


    }

    if(users.getSize()){
        UserPub.publish(m_Users);
    }

}
void MainWindow::PublishColor(cv::Mat a){

    //    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);


    ros::Time time = ros::Time::now();
    cv_ptr->encoding = "bgr8";
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "/camera/color";
    //    cv_ptr->image = a.clone();
    QImage qimageOrg((const unsigned char*)(a.data), a.cols, a.rows, QImage::Format_RGB888);
    ui.label->QLabel::setPixmap(QPixmap::fromImage(qimageOrg));

    image_pub.publish(cv_ptr->toImageMsg());
}

void MainWindow::TrackingCallback(){

    user_trakcing.convertMat();

    user_trakcing.trackingLoop();

    //    static int asd = 0;
    //    if(asd++ > 10000)
    //        PublishColor(user_trakcing.color_Mat);


    //    static int i = 0;
    //    cv::circle(user_trakcing.depth_Mat,Point(320,240),5,user_trakcing.colors[i++/10],4);
    //    if(i>100)
    //        i = 0;


    //    QImage qimageOrg((const unsigned char*)(user_trakcing.color_Mat.data), user_trakcing.color_Mat.cols, user_trakcing.color_Mat.rows, QImage::Format_RGB888);
    //    ui.label->QLabel::setPixmap(QPixmap::fromImage(qimageOrg));

    QImage qimageDepth((const unsigned char*)(user_trakcing.depth_Mat.data), user_trakcing.depth_Mat.cols, user_trakcing.depth_Mat.rows, QImage::Format_RGB888);
    ui.label_2->QLabel::setPixmap(QPixmap::fromImage(qimageDepth));
}

}  // namespace cv_skel_cam

