#include "ueye_cam/ueyecamsynchmanager.hpp"
#include "ueye_cam/CameraControl.h"
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>

namespace ueye_cam
{
using namespace std;

UeyeCamSynchNodelet::UeyeCamSynchNodelet():
    nodelet::Nodelet ()
{

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
        ros::ServiceClient sc = nh.serviceClient<ueye_cam::CameraControl> ("/" + camName + "/camera_control");
        mCameraControlClients[camName] = sc;
    }

    mCameraTriggerControlClient = nh.serviceClient<mavros_msgs::CommandTriggerControl> (triggerControlSrvName);

    mMasterExposureSubscriber = nh.subscribe (mMasterExposureTopic, 1, &UeyeCamSynchNodelet::masterExposureHandler, this);

    bool result = true;
    result = waitForAllCameras();
    if (result == false)
    {
        ROS_FATAL_STREAM ("Failed to wait for all cameras");
        exit (-1);
    }

    result = resetAllCameras();
    if (result == false)
    {
        ROS_FATAL_STREAM ("Failed to reset all cameras");
        exit (-2);
    }
    ros::spin();

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


    mCameraTriggerCall.request.cycle_time = static_cast<float> (enable ? (1000.0 / mFrameRate) : 0.0);
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

}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (ueye_cam::UeyeCamSynchNodelet, nodelet::Nodelet)
