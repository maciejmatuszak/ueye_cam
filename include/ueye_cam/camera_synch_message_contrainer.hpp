#ifndef CAMERASYNCHMESSAGECONTRAINER_H
#define CAMERASYNCHMESSAGECONTRAINER_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <mavros_msgs/CamIMUStamp.h>
#include <ueye_cam/logging_macros.hpp>
#include <cv_bridge/cv_bridge.h>

namespace ueye_cam {


    class CameraSynchMessageContainer
    {
    public:

        sensor_msgs::ImagePtr cameraImagePtr;
        cv_bridge::CvImageConstPtr cameraImageCvPtr;
        sensor_msgs::CameraInfoPtr cameraInfoPtr;
        unsigned int timeStampFrameSeq;
        ros::Time timeStampTimestamp;


        CameraSynchMessageContainer():timeStampTimestamp(0,0)
        {

        }

        CameraSynchMessageContainer(const sensor_msgs::ImagePtr& cameraImagePtr,  const sensor_msgs::CameraInfoPtr& cameraInfoPtr, const cv_bridge::CvImageConstPtr& cameraImageCvPtr):
            timeStampTimestamp(0,0)
        {
            this->cameraImagePtr = cameraImagePtr;
            this->cameraImageCvPtr = cameraImageCvPtr;
            this->cameraInfoPtr = cameraInfoPtr;
        }

        CameraSynchMessageContainer(const mavros_msgs::CamIMUStampPtr& timeStampPtr)
        {
            updateCamIMUStamp(timeStampPtr);
        }

        void updateCamIMUStamp(const mavros_msgs::CamIMUStampPtr& timeStampPtr)
        {
            this->timeStampTimestamp.sec = timeStampPtr->frame_stamp.sec;
            this->timeStampTimestamp.nsec = timeStampPtr->frame_stamp.nsec;
            this->timeStampFrameSeq = timeStampPtr->frame_seq_id;
        }


        void reset()
        {
            this->timeStampTimestamp.sec = 0;
            this->timeStampTimestamp.nsec = 0;
            this->timeStampFrameSeq = 0;
            this->cameraImagePtr = nullptr;
            this->cameraInfoPtr = nullptr;
            this->cameraImageCvPtr = nullptr;
        }

        bool isComplette()
        {
            return (cameraImagePtr != nullptr && !timeStampTimestamp.isZero());
        }

    };


    typedef boost::shared_ptr<CameraSynchMessageContainer> CameraSynchMessageContainerPtr;
    typedef boost::shared_ptr<CameraSynchMessageContainer const> CameraSynchMessageContainerConstPtr;
}
#endif // CAMERASYNCHMESSAGECONTRAINER_H
