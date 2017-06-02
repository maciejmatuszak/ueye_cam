#include "ueye_cam/ueye_cam_synch_nodelet.hpp"
#include "ueye_cam/CameraControl.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>

namespace ueye_cam
{
using namespace std;

UeyeCamSynchNodelet::UeyeCamSynchNodelet():
    nodelet::Nodelet()
{

}
UeyeCamSynchNodelet::~UeyeCamSynchNodelet()
{
    sendCameraTriggerControl (false);
    mStopThread = true;
    if (mSynchThread.joinable())
    {
        mSynchThread.join();
    }
}

void UeyeCamSynchNodelet::onInit()
{
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &local_nh = getPrivateNodeHandle();
    string triggerControlSrvName;
    vector<string> camNamesV;
    string temp;


    local_nh.param<double> ("frame_rate", mFrameRate, DEFAULT_FRAME_RATE );
    local_nh.param<string> ("master_exposure_topic", mMasterExposureTopic, DEFAULT_MASTER_EXPOSURE_TOPIC );
    local_nh.param<string> ("trigger_control_srv", triggerControlSrvName, DEFAULT_TRIGGER_CONTROL_SRV_NAME );
    local_nh.param<bool> ("trigger_control_ignore_response", mIgnoreCameraTriggerResponse, DEFAULT_TRIGGER_CONTROL_SRV_IGNORE_RESP );

    local_nh.param<string> ("camera_names", temp, "");
    boost::split (camNamesV, temp, boost::is_any_of (","));
    BOOST_FOREACH (const string & camName, camNamesV)
    {
        ROS_INFO_STREAM ("Camera:" << camName << " creating control service client");
        ros::ServiceClient sc = getMTNodeHandle().serviceClient<ueye_cam::CameraControl> ("/" + camName + "/camera_control");
        mCameraControlClients[camName] = sc;
    }

    //TODO:Evaluate the use of multithreaded callback
    mCameraTriggerControlClient = getMTNodeHandle().serviceClient<mavros_msgs::CommandTriggerControl> (triggerControlSrvName);

    mMasterExposureSubscriber = nh.subscribe (mMasterExposureTopic, 1, &UeyeCamSynchNodelet::masterExposureHandler, this);
    mStopThread = false;

    mSynchThread = thread (bind (&UeyeCamSynchNodelet::SynchThread, this));


}



void UeyeCamSynchNodelet::masterExposureHandler (const ueye_cam::ExposureConstPtr &masterExposurePtr)
{
    BOOST_FOREACH (const string camName, mCameraControlClients | boost::adaptors::map_keys)
    {
        ros::ServiceClient sc = mCameraControlClients[camName];
        CameraControl ccCall;
        ccCall.request.action = CameraControlRequest::ACTION_SET_EXPOSURE;
        ccCall.request.arg1 = masterExposurePtr->exposure_ms;
        sc.call (ccCall);
        if (ccCall.response.success == false)
        {
            ROS_ERROR_STREAM ("Camera: " << camName << "; failed to set exposure");
        }
    }
}

bool UeyeCamSynchNodelet::waitForAllCameras ()
{
    BOOST_FOREACH (const string camName, mCameraControlClients | boost::adaptors::map_keys)
    {
        ros::ServiceClient sc = mCameraControlClients[camName];
        //wait for 5 sec
        bool cameraExists = sc.waitForExistence (ros::Duration (5));
        if (cameraExists == false)
        {
            ROS_ERROR_STREAM ("Camera: " << camName << "; does not exists");
            return false;
        }
    }
    return true;
}


bool UeyeCamSynchNodelet::resetAllCameras ()
{
    CameraControl svcCall;
    //stop all cameras
    BOOST_FOREACH (const string camName, mCameraControlClients | boost::adaptors::map_keys)
    {
        ros::ServiceClient sc = mCameraControlClients[camName];

        svcCall.request.action = CameraControlRequest::ACTION_STOP;
        sc.call (svcCall);
        if (svcCall.response.success == false)
        {
            ROS_ERROR_STREAM ("Failed to stop camera: " << camName << "; message:" << svcCall.response.message);
            return false;
        }
    }

    BOOST_FOREACH (const string camName, mCameraControlClients | boost::adaptors::map_keys)
    {
        ros::ServiceClient sc = mCameraControlClients[camName];

        svcCall.request.action = CameraControlRequest::ACTION_START;
        sc.call (svcCall);
        if (svcCall.response.success == false)
        {
            ROS_ERROR_STREAM ("Failed to start camera: " << camName << "; message:" << svcCall.response.message);
            return false;
        }
    }
    return true;
}


bool UeyeCamSynchNodelet::sendCameraTriggerControl (bool enable)
{
    bool result = false;

    //Due to changes in PX4/mavling (no mavros yet) trigger control the cycle_time is mapped to "reset sequence"!!! F!
    //mCameraTriggerCall.request.cycle_time = static_cast<float> (enable ? (1000.0 / mFrameRate) : 0.0);
    mCameraTriggerCall.request.cycle_time = enable ? 1 : 0;
    mCameraTriggerCall.request.trigger_enable = enable;
    ROS_INFO ("Calling trigger control service: trigger_enable=%s; cycle_time=%f[ms]",
              (enable ? "true" : "false"),
              static_cast<double> (mCameraTriggerCall.request.cycle_time));

    result = mCameraTriggerControlClient.call (mCameraTriggerCall);

    if (result == true)
    {
        ROS_INFO ("Successfully enabled camera trigger");
        return true;
    }
    else
    {
        if (mIgnoreCameraTriggerResponse)
        {
            ROS_INFO ("Failed to call trigger_control service - ignoring");
            return true;
        }
        else
        {
            ROS_ERROR ("Failed to call trigger_control service");
            return false;
        }
    }
}


void UeyeCamSynchNodelet::SynchThread()
{

    //waiting for cameras

    bool result = true;
    sendCameraTriggerControl (false);

    ros::Rate delayReady (ros::Duration (3));
    delayReady.sleep();
    result = waitForAllCameras();
    if (result == false)
    {
        ROS_FATAL_STREAM ("Failed to wait for all cameras");
        exit (-1);
    }
    ROS_INFO_STREAM ("All cameras present");

    result = resetAllCameras();
    if (result == false)
    {
        ROS_FATAL_STREAM ("Failed to reset all cameras");
        exit (-2);
    }
    ROS_INFO_STREAM ("All cameras reseted");
    //give the cameras time to start
    ros::Rate delay (ros::Duration (1));
    delay.sleep();

    sendCameraTriggerControl (true);


    ros::Rate rate (1);
    while (ros::ok() && (mStopThread == false))
    {
        rate.sleep();

    }

}


} //namespace



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (ueye_cam::UeyeCamSynchNodelet, nodelet::Nodelet)
