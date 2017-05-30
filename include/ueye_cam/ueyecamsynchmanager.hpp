#ifndef UEYECAMSYNCHMANAGER_HPP
#define UEYECAMSYNCHMANAGER_HPP
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <mavros_msgs/CommandTriggerControl.h>
#include "ueye_cam/Exposure.h"

namespace ueye_cam
{
using namespace std;

class UeyeCamSynchManager: public nodelet::Nodelet
{
public:
    const string      DEFAULT_MASTER_EXPOSURE_TOPIC = "/cam0/exposure";
    const string      DEFAULT_TRIGGER_CONTROL_SRV_NAME = "/px4/cmd/trigger_control";
    const string      DEFAULT_CAMERA_NAMES = "cam0";
    const bool        DEFAULT_TRIGGER_CONTROL_SRV_IGNORE_RESP = false;
    const double      DEFAULT_FRAME_RATE = 20.0;

    UeyeCamSynchManager();
    virtual ~UeyeCamSynchManager();

    // Nodelet interface
private:
    void onInit();

    //PX4 camera trigger service
    bool mIgnoreCameraTriggerResponse;
    ros::ServiceClient mCameraTriggerControlClient;
    mavros_msgs::CommandTriggerControl mCameraTriggerCall;

    map<string, ros::ServiceClient> mCameraControlClients;
    ros::Subscriber mMasterExposureSubscriber;

    string mMasterExposureTopic;
    double mFrameRate;

    bool sendCameraTriggerControl (bool enable);
    void masterExposureHandler (const ueye_cam::ExposureConstPtr &masterExposurePtr);
    bool resetAllCameras();
    bool waitForAllCameras();
};

}
#endif // UEYECAMSYNCHMANAGER_HPP
