/**
 * @file /include/cv_skel_cam/main_window.hpp
 *
 * @brief Qt based gui for cv_skel_cam.
 *
 * @date November 2010
 **/
#ifndef cv_skel_cam_MAIN_WINDOW_H
#define cv_skel_cam_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "QTimer"
#include <iostream>

#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <skeleton_message/Users.h>
#include <skeleton_message/UserData.h>
#include <skeleton_message/SkeletonJoint.h>

#include "util.h"
//#include "NiteSampleUtilities.h"

#define MAX_USERS 10
#define MAX_DEPTH 10000
#define RS2_PROJECT_POINT_TO_PIXEL 0x1000

struct Rs2PointPixel
{
    float point[3];
    float pixel[2];
};
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace cv_skel_cam {
using namespace cv;

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */

class Skeleton_Viewer
{
public:
    //  int main(int argc, char **argv);
    skeleton_message::Users m_Users;
    Skeleton_Viewer(const char* strSampleName);
    int tracking_Init(int argc, char** argv);

    void trackingLoop();



    virtual ~Skeleton_Viewer();

    void convertMat();

    ros::Publisher UserPub;


    bool a[99] = {0,};

    std::array<cv::Scalar, MAX_USERS> colors;

    cv::Mat color_Mat;
    cv::Mat depth_Mat;

protected:

    void Finalize();
private:
    Skeleton_Viewer(const Skeleton_Viewer&);
    Skeleton_Viewer& operator=(Skeleton_Viewer&);

    static Skeleton_Viewer* ms_self;

    float length(float x1,float y1,float z1,float x2,float y2,float z2,float *arr,cv::String compare);
    void PublishSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& user);
    void updateUserState(const nite::UserData& user, uint64_t ts);
    void DrawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& user);
    void DrawCOM(nite::UserTracker* pUserTracker, const nite::UserData& user);
    inline nite::Status convertJointCoordinatesToDepth( const float x, const float y, const float z, float* pOutX, float* pOutY );
    \
    float				m_pDepthHist[MAX_DEPTH];
    char			m_strSampleName[ONI_MAX_STR];
    openni::RGB888Pixel*		m_pTexMap;
    unsigned int		m_nTexMapX;
    unsigned int		m_nTexMapY;




    openni::Device		m_device;

    nite::UserTracker* m_pUserTracker;

    nite::UserId m_poseUser;
    uint64_t m_poseTime;


    openni::VideoFrameRef		m_depthFrame;
    openni::VideoFrameRef		m_colorFrame;

    openni::VideoStream			m_depthStream;
    openni::VideoStream			m_colorStream;

    //  openni::Device&			m_device2;
    //  openni::VideoStream&			m_depthStream;
    //  openni::VideoStream&			m_colorStream;
    //  openni::VideoStream**		m_streams;


    unsigned int color_width,color_height,depth_width,depth_height;


};

class MainWindow : public QMainWindow/*,Skeleton_Viewer*/{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    image_transport::Publisher image_pub;
    sensor_msgs::ImagePtr m_msg;
//    ros::Time time;
    //     Skeleton_Viewer a;
public Q_SLOTS:

    void TrackingCallback();
    void PublishColor(cv::Mat a);
private:

    cv_bridge::CvImagePtr cv_ptr;
    Ui::MainWindowDesign ui;
    QNode qnode;
    QTimer* mtimer;

};

}  // namespace cv_skel_cam

#endif // cv_skel_cam_MAIN_WINDOW_H
