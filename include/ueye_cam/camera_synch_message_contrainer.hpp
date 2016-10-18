#ifndef CAMERASYNCHMESSAGECONTRAINER_H
#define CAMERASYNCHMESSAGECONTRAINER_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <ueye_cam/logging_macros.hpp>

namespace ueye_cam {


    class CameraSynchMessageContrainer
    {
    public:
        CameraSynchMessageContrainer();
         sensor_msgs::ImageConstPtr cameraImagePtr;
         sensor_msgs::CameraInfoConstPtr cameraInfoPtr;
         mavros_msgs::CamIMUStampPtr timeStampPtr;

         void init(){
             ROS_DEBUG_STREAM("Creating instance of CameraSynchMessageContrainer");
         }

        CameraSynchMessageContrainer( sensor_msgs::ImageConstPtr image,  sensor_msgs::CameraInfoConstPtr info)
        {
            init();
            this->cameraImagePtr = image;
            this->cameraInfoPtr = info;
            this->timeStampPtr = nullptr;
        }

        CameraSynchMessageContrainer( mavros_msgs::CamIMUStampPtr timestampPtr)
        {
            init();
            this->cameraImagePtr = nullptr;
            this->cameraInfoPtr = nullptr;
            this->timeStampPtr = timestampPtr;
        }
        ~CameraSynchMessageContrainer()
        {
            ROS_DEBUG_STREAM("Disposing of CameraSynchMessageContrainer");
        }
    };


    typedef boost::shared_ptr<CameraSynchMessageContrainer> CameraSynchMessageContrainerPtr;
    typedef boost::shared_ptr<CameraSynchMessageContrainer const> CameraSynchMessageContrainerConstPtr;
}
#endif // CAMERASYNCHMESSAGECONTRAINER_H
