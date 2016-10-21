#ifndef CAMERASYNCHMESSAGECONTRAINER_H
#define CAMERASYNCHMESSAGECONTRAINER_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <ueye_cam/logging_macros.hpp>

namespace ueye_cam {


    class CameraSynchMessageContainer
    {
    public:
        CameraSynchMessageContainer();
        sensor_msgs::ImagePtr cameraImagePtr;
        sensor_msgs::CameraInfoPtr cameraInfoPtr;
        mavros_msgs::CamIMUStampPtr timeStampPtr;

        CameraSynchMessageContainer(const sensor_msgs::ImagePtr& cameraImagePtr,  const sensor_msgs::CameraInfoPtr& cameraInfoPtr)
        {
            this->cameraImagePtr = cameraImagePtr;
            this->cameraInfoPtr = cameraInfoPtr;
        }

        CameraSynchMessageContainer(const mavros_msgs::CamIMUStampPtr& timeStampPtr)
        {
            this->timeStampPtr = timeStampPtr;
        }

        bool isComplette()
        {
            return (cameraImagePtr != nullptr && timeStampPtr != nullptr);
        }
    };


    typedef boost::shared_ptr<CameraSynchMessageContainer> CameraSynchMessageContainerPtr;
    typedef boost::shared_ptr<CameraSynchMessageContainer const> CameraSynchMessageContainerConstPtr;
}
#endif // CAMERASYNCHMESSAGECONTRAINER_H
