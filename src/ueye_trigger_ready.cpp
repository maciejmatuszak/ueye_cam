#include "ros/ros.h"
#include "mavros_msgs/CommandTriggerControl.h"
#include <cstdlib>
#include <string>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int16.h>

class TriggerReady
{

public:
	TriggerReady()
	{
		cam0_OK_ = false;
		cam1_OK_ = false;
		exposure_ms_ = 0;
		framerate_hz_ = 30; // default framerate
		triggerClient_ = n_.serviceClient<mavros_msgs::CommandTriggerControl>("/mavros/cmd/trigger_control");
		advertiseService();
		subscribeCameras();
	}

	void cam0Ready(const std_msgs::Int16ConstPtr &msg)
	{
		exposure_ms_ = msg->data; // Set exposure from cam0
		if (exposure_ms_ > 1000/framerate_hz_) {
			ROS_WARN("Exposure time %u ms does not allow %u Hz frame-rate!", exposure_ms_, framerate_hz_);
		}
		ROS_INFO("Camera 0 waiting for trigger. Exposure set to %u ms", exposure_ms_);
	}

	void cam1Ready(const std_msgs::Int16ConstPtr &msg)
	{
		ROS_INFO("Camera 1 waiting for trigger. Exposure set to %u ms", exposure_ms_); //XXX check if the exposures aren't same.
	}
	void subscribeCameras()
	{
		cam0_Sub_ = n_.subscribe("cam0/waiting_trigger", 0, &TriggerReady::cam0Ready, this);
		cam1_Sub_ = n_.subscribe("cam1/waiting_trigger", 0, &TriggerReady::cam1Ready, this);
	}

	bool servCam0(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
	{
		cam0_OK_ = true;
		resp.success = true;
		ROS_INFO_STREAM("Camera 0 is primed for trigger");
		return true;
	}

	bool servCam1(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
	{
		cam1_OK_ = true;
		resp.success = true;
		ROS_INFO_STREAM("Camera 1 is primed for trigger");
		return true;
	}

	bool cam0_OK()
	{
		return cam0_OK_;
	}

	bool cam1_OK()
	{
		return cam1_OK_;
	}

	int sendTriggerCommand()
	{
		srv_.request.integration_time = 1000/framerate_hz_;
		srv_.request.trigger_enable = true;

		if (triggerClient_.call(srv_)) {
			ROS_INFO("Successfully called service trigger_control");

		} else {
			ROS_ERROR("Failed to call service trigger_control");
			return 1;
		}

		return 0;
	}

	void advertiseService()
	{
		serverCam0_ = n_.advertiseService("cam0/trigger_ready", &TriggerReady::servCam0, this);
		serverCam1_ = n_.advertiseService("cam1/trigger_ready", &TriggerReady::servCam1, this);
	}


private:

	bool cam0_OK_;
	bool cam1_OK_;
	int exposure_ms_;
	int framerate_hz_;

	ros::NodeHandle n_;

	ros::Subscriber cam0_Sub_;
	ros::Subscriber cam1_Sub_;

	ros::ServiceClient triggerClient_;
	mavros_msgs::CommandTriggerControl srv_;

	ros::ServiceServer serverCam0_;
	ros::ServiceServer serverCam1_;

};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "StartTrigger");
	TriggerReady tr;

	// Define time update rate to call callback function if necessary
	ros::Rate r(100); // Hz

	while (!(tr.cam0_OK() && tr.cam1_OK()) && ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}

	// Send start trigger command to Pixhawk
	ros::Rate r2(10); // Hz

	while (tr.sendTriggerCommand() && ros::ok()) {
		ROS_INFO_STREAM("Retrying reaching pixhawk");
		r2.sleep();
	}

}

