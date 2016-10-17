#include "ros/ros.h"
#include "mavros_msgs/CommandTriggerControl.h"
#include <cstdlib>
#include <string>
#include <std_srvs/Trigger.h>
#include "ueye_cam/CameraReady.h"


using namespace std;

namespace ueye_cam
{
    class TriggerReady
    {

    public:

        const static string DEFAULT_TRIGGER_CONTROL_SRV_NAME;
        const static bool   DEFAULT_TRIGGER_CONTROL_SRV_IGNORE_RESP;

        TriggerReady():
            privateN_("~")
        {

            privateN_.param<string>("trigger_control_srv", triggerControlSrvName_, DEFAULT_TRIGGER_CONTROL_SRV_NAME );
            privateN_.param<bool>("trigger_control_ignore_response", ignoreTriggerResponse_ , DEFAULT_TRIGGER_CONTROL_SRV_IGNORE_RESP );
            ROS_INFO_STREAM("Trigger Control service set to: " << triggerControlSrvName_);
            ROS_INFO_STREAM("Trigger Control service ignore response set to: " << (ignoreTriggerResponse_?"true":"false"));
            parseCameraNames();

            framerate_hz_ = 18.0; // default framerate TODO get this from the ueye node

            //master camera is not set yet
            camera_master_set_ = false;

            triggerControlClient_ = globalN_.serviceClient<mavros_msgs::CommandTriggerControl>(triggerControlSrvName_);

            serverCameraReady_ = privateN_.advertiseService("camera_ready", &TriggerReady::servCameraReady, this);
        }

        void parseCameraNames() {
            string parameter_string;
            if(privateN_.getParam("cameras", parameter_string))
            {

                //the camera names are delimited with space
                char delim = ' ';
                stringstream parameter_stream;
                parameter_stream.str(parameter_string);
                string item;
                while (getline(parameter_stream, item, delim))
                {
                    this->camera_status_[item] = false;
                    ROS_DEBUG_STREAM("Initialized camera status for: " << item);
                }
            }
            else
            {
                ROS_FATAL("the parameter \"cameras has to be provided with spacce separated camera names\"");
                exit(3);
            }
        }

        /**
         * @brief servCameraReady processes CameraReady service request from cameras
         * @param req
         * @param resp
         * @return
         */
        bool servCameraReady(ueye_cam::CameraReady::Request &req, ueye_cam::CameraReady::Response &resp)
        {
            map<string,bool>::iterator it = camera_status_.find(req.camera_name);
            bool result = false;

            algorithm need to be checked

            //find the camera entry
            if(it != camera_status_.end())
            {
                //check for duplicate names
                if(it->second == true)
                {
                    //this camera is reporting ready for second time!!!
                    ROS_FATAL_STREAM("Camera: " << req.camera_name << " was already processed. Possible two cameras with the same name?");
                    result = false;
                }
                else
                {
                    //if it is master camera then set the frequency
                    if(req.is_master)
                    {
                        if(camera_master_set_)
                        {
                            ROS_FATAL_STREAM("Camera: " << req.camera_name << " is reporting as master. You have more than one configured as master");
                        }
                        else
                        {
                            result = true;
                            camera_master_set_= true;
                            framerate_hz_ = req.frame_rate > 0.0 ? req.frame_rate : framerate_hz_;
                            ROS_INFO_STREAM("Camera: " << req.camera_name << " is master setting framerate to: " << req.frame_rate << " [Hz]");
                        }
                    }

                    //ok camera is ready, mark it as such
                    it->second = true;
                    ROS_INFO_STREAM("Camera: " << req.camera_name << " is primed for trigger");
                }

            }
            else
            {
                //we do not know this camera
                ROS_ERROR_STREAM("Camera: " << req.camera_name << " was not configred in the trigger");
                resp.message = "Camera: " + req.camera_name + " was not configred in the trigger";
            }

            resp.success = result;
            return result;
        }


        /**
         * @brief allCamerasReady checks if all cameras reported ready
         * @return false if at least one camera is not ready, true otherwise
         */
        bool allCamerasReady()
        {

            for (auto it = camera_status_.begin(); it != camera_status_.end(); ++it)
            {
                if(it->second == false)
                {
                    return false;
                }
            }
            return true;
        }


        /**
         * @brief enableTrigger - calls trigger service to enable/disable hardware trigger line
         * @return
         */
        bool setTriggerControl(bool enable)
        {
            bool result = false;

            triggerControlMsg_.request.cycle_time = enable ? (1000.0 / framerate_hz_) : 0.0;
            triggerControlMsg_.request.trigger_enable = enable;

            result = triggerControlClient_.call(triggerControlMsg_);

            if(result == true)
            {
                ROS_INFO("Successfully enabled camera trigger");
                return true;
            }
            else
            {
                if(ignoreTriggerResponse_)
                {
                    ROS_INFO("Failed to call trigger_control service - ignoring");
                    return true;
                }
                else
                {
                    ROS_ERROR("Failed to call trigger_control service");
                    return false;
                }
            }
        }


    private:

        /**
         * @brief framerate_hz_ received from camera rady service from master camera, it is used to set the frame rate on PX4 via trigger_control
         */
        float framerate_hz_;
        map<string, bool> camera_status_;
        string triggerControlSrvName_;

        /**
         * unless you modified the PX4 Firmware you will not get reply from trigger control service
         */
        bool ignoreTriggerResponse_;

        /**
         * @brief camera_master_set_ used to detect multiple masters
         */
        bool camera_master_set_;

        ros::NodeHandle globalN_;
        ros::NodeHandle privateN_;

        ros::ServiceClient triggerControlClient_;
        mavros_msgs::CommandTriggerControl triggerControlMsg_;

        ros::ServiceServer serverCameraReady_;

    };

    const string TriggerReady::DEFAULT_TRIGGER_CONTROL_SRV_NAME = "/mavros/cmd/trigger_control";
    const bool   TriggerReady::DEFAULT_TRIGGER_CONTROL_SRV_IGNORE_RESP = false;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "StartTrigger");
    ueye_cam::TriggerReady tr;

    ros::Rate waiting_for_trigger_resonseRate(5); // Hz

    // Define time update rate to call callback function if necessary
    // while waiting for cameras we want to be fearly quick
    ros::Rate waiting_for_camerasRate(100); // Hz


    //for the start lets disable the wh trigger line
    while (!tr.setTriggerControl(false) && ros::ok())
    {
        ROS_INFO_STREAM("Retrying reaching pixhawk");
        waiting_for_trigger_resonseRate.sleep();
    }

    //now lets wait for all cameras to report ready
    while (!tr.allCamerasReady() && ros::ok())
    {
        ros::spinOnce();
        waiting_for_camerasRate.sleep();
    }

    // All cameras are ready - fire
    // Send start trigger command to Pixhawk
    while (!tr.setTriggerControl(true) && ros::ok())
    {
        ROS_INFO_STREAM("Retrying reaching pixhawk");
        waiting_for_trigger_resonseRate.sleep();
    }

}
