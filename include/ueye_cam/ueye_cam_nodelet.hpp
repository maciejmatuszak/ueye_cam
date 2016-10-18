/*******************************************************************************
* DO NOT MODIFY - AUTO-GENERATED
*
*
* DISCLAMER:
*
* This project was created within an academic research setting, and thus should
* be considered as EXPERIMENTAL code. There may be bugs and deficiencies in the
* code, so please adjust expectations accordingly. With that said, we are
* intrinsically motivated to ensure its correctness (and often its performance).
* Please use the corresponding web repository tool (e.g. github/bitbucket/etc.)
* to file bugs, suggestions, pull requests; we will do our best to address them
* in a timely manner.
*
*
* SOFTWARE LICENSE AGREEMENT (BSD LICENSE):
*
* Copyright (c) 2013-2016, Anqi Xu and contributors
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the School of Computer Science, McGill University,
*    nor the names of its contributors may be used to endorse or promote
*    products derived from this software without specific prior written
*    permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#ifndef UEYE_CAM_NODELET_HPP_
#define UEYE_CAM_NODELET_HPP_


#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <ueye_cam/UEyeCamConfig.h>
#include <boost/thread/mutex.hpp>
#include <ueye_cam/ueye_cam_driver.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include "ueye_cam/camera_synch_message_contrainer.hpp"
#include "ueye_cam/Exposure.h"
#include "ueye_cam/CameraReady.h"


namespace ueye_cam {


typedef dynamic_reconfigure::Server<ueye_cam::UEyeCamConfig> ReconfigureServer;


/**
 * ROS interface nodelet for UEye camera API from IDS Imaging Development Systems GMBH.
 */
class UEyeCamNodelet : public nodelet::Nodelet, public UEyeCamDriver {
public:
  constexpr static unsigned int RECONFIGURE_RUNNING = 0;
  constexpr static unsigned int RECONFIGURE_STOP = 1;
  constexpr static unsigned int RECONFIGURE_CLOSE = 3;
  constexpr static int DEFAULT_IMAGE_WIDTH = 640;  // NOTE: these default values do not matter, as they
  constexpr static int DEFAULT_IMAGE_HEIGHT = 480; // are overwritten by queryCamParams() during connectCam()
  constexpr static double DEFAULT_EXPOSURE = 10.0;
  constexpr static double DEFAULT_FRAME_RATE = 10.0;
  constexpr static int DEFAULT_PIXEL_CLOCK = 25;
  constexpr static int DEFAULT_FLASH_DURATION = 1000;

  const static std::string DEFAULT_FRAME_NAME;
  const static std::string DEFAULT_CAMERA_NAME;
  const static std::string DEFAULT_CAMERA_IMU_TOPIC;
  const static std::string DEFAULT_CAMERA_READY_SERVICE;
  const static std::string DEFAULT_CAMERA_TOPIC;
  const static std::string DEFAULT_CAMERA_TOPIC_RECT;
  const static std::string DEFAULT_TIMEOUT_TOPIC;
  const static std::string DEFAULT_COLOR_MODE;
  const static bool        DEFAULT_CAMERA_IS_MASTER;


  UEyeCamNodelet();

  virtual ~UEyeCamNodelet();

  /**
   * Initializes ROS environment, loads static ROS parameters, initializes UEye camera,
   * and starts live capturing / frame grabbing thread.
   */
  virtual void onInit();

  /**
   * Handles callbacks from dynamic_reconfigure.
   */
  void configCallback(ueye_cam::UEyeCamConfig& config, uint32_t level);


protected:
  /**
   * Calls UEyeCamDriver::syncCamConfig(), then updates ROS camera info
   * and ROS image settings.
   */
  virtual INT syncCamConfig(std::string dft_mode_str = "mono8");

  /**
   * Reads parameter values from currently selected camera.
   */
  INT queryCamParams();

  /**
   * Loads, validates, and updates static ROS parameters.
   */
  INT parseROSParams(ros::NodeHandle& local_nh);

  /**
   * Initializes the camera handle, loads UEye INI configuration, refreshes
   * parameters from camera, loads and sets static ROS parameters, and starts
   * the frame grabber thread.
   */
  virtual INT connectCam();

  /**
   * Stops the frame grabber thread, closes the camera handle,
   * and releases all local variables.
   */
  virtual INT disconnectCam();

  /**
   * (ROS Service) Updates the camera's intrinsic parameters over the ROS topic,
   * and saves the parameters to a flatfile.
   */
  bool setCamInfo(sensor_msgs::SetCameraInfo::Request& req,
      sensor_msgs::SetCameraInfo::Response& rsp);

  /**
   * Loads the camera's intrinsic parameters from camIntrFilename.
   */
  void loadIntrinsicsFile();


  /**
   * Saves the camera's intrinsic parameters to camIntrFilename.
   */
  bool saveIntrinsicsFile();

  /**
   * Main ROS interface "spin" loop.
   */
  void frameGrabLoop();
  void startFrameGrabber();
  void stopFrameGrabber();

  const static std::map<INT, std::string> ENCODING_DICTIONARY;
  /**
   * Transfers the current frame content into given sensor_msgs::Image,
   * therefore writes the fields width, height, encoding, step and
   * data of img.
   */
  bool fillMsgData(sensor_msgs::Image& img) const;

  /**
   * Returns image's timestamp or current wall time if driver call fails.
   */
  ros::Time getImageTimestamp();

  /**
   * @brief setSlaveExposure
   * @param msg
   */
  void setSlaveExposure(const ueye_cam::Exposure& msg);

  /**
   * @brief sendTriggerReady
   */
  void sendTriggerReady();

  /**
   * @brief sendSlaveExposure
   */
  void sendSlaveExposure();

  /**
   * @brief processAndPublish
   * @param containerptr
   */
  void bufferTimestamp(const mavros_msgs::CamIMUStampPtr& msg);
  void bufferImages(sensor_msgs::CameraInfoPtr cam_info_msg_ptr, sensor_msgs::ImagePtr img_msg_ptr);
  void publishImages(CameraSynchMessageContrainerPtr containerptr);

  void trim_message_buffer();


  /**
   * Returns image's timestamp based on device's internal clock or current wall time if driver call fails.
   */
  ros::Time getImageTickTimestamp();

  virtual void handleTimeout();

  std::thread frame_grab_thread_;
  bool frame_grab_alive_;

  ReconfigureServer* ros_cfg_;
  boost::recursive_mutex ros_cfg_mutex_;
  bool cfg_sync_requested_;

  image_transport::CameraPublisher ros_cam_pub_;
  image_transport::Publisher ros_rect_pub_;

  /**
   * @brief ros_exposure_pub_ publishes exposure details for slave cameras
   */
  ros::Publisher ros_exposure_pub_;

  /**
   * @brief ros_exposure_sub_ slave cameras subscribe for exposure details from master
   */
  ros::Subscriber ros_exposure_sub_;

  /**
   * @brief ros_timestamp_sub_ subscriber for time synch messages from mavros (Pixhawk / PX4)
   */
  ros::Subscriber ros_timestamp_sub_;

  std::map<unsigned int, CameraSynchMessageContrainerPtr> message_buffer_;

  ros::ServiceClient camera_ready_srv_client_;

  sensor_msgs::Image ros_image_;
  sensor_msgs::CameraInfo ros_cam_info_;
  unsigned int ros_frame_count_;
  ros::Publisher timeout_pub_;
  unsigned long long int timeout_count_;

  // Image rectification
  image_geometry::PinholeCameraModel camera_model_;


  ros::ServiceServer set_cam_info_srv_;

  std::string frame_name_;
  std::string cam_topic_;
  std::string cam_topic_rect_;
  std::string timeout_topic_;
  std::string cam_intr_filename_;
  std::string cam_params_filename_; // should be valid UEye INI file
  ueye_cam::UEyeCamConfig cam_params_;

  ros::Time init_ros_time_; // for processing frames
  uint64_t init_clock_tick_;

  ros::Time init_publish_time_; // for throttling frames from being published (see cfg.output_rate)
  uint64_t prev_output_frame_idx_; // see init_publish_time_
  boost::mutex output_rate_mutex_;
};


} // namespace ueye_cam


#endif /* UEYE_CAM_NODELET_HPP_ */
