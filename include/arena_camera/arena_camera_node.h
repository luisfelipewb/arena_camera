/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef ARENA_CAMERA_ARENA_CAMERA_NODE_H
#define ARENA_CAMERA_ARENA_CAMERA_NODE_H

// STD
#include <string>

// ROS sys dep
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
 
#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Arena
#include <arena_camera/arena_camera_parameter.h>

namespace arena_camera
{

/**
* The ROS-node of the arena_camera interface
*/
class ArenaCameraNode
{
public:
  ArenaCameraNode();
  virtual ~ArenaCameraNode();

  /**
  * initialize the camera and the ros node.
  * calls ros::shutdown if an error occurs.
  */
  void init();

  /**
  * spin the node
  */
  virtual void spin();

  /**
  * Getter for the frame rate set by the launch script or from the ros parameter
  * server
  * @return the desired frame rate.
  */
  const double& frameRate() const;

  /**
  * Getter for the tf frame.
  * @return the camera frame.
  */
  const std::string& cameraFrame() const;

protected:
  /**
  * Creates the camera instance and starts the services and action servers.
  * @return false if an error occurred
  */
  bool initAndRegister();

  /**
  * Start the camera and initialize the messages
  * @return
  */
  bool startGrabbing();

  bool setImageEncoding(const std::string& ros_encoding);

  /**
  * Initializing of img_rect_pub_, grab_img_rect_as_ and the pinhole_model_,
  * in case that a valid camera info has been set
  * @return
  */
  void setupRectification();

  /**
  * Returns the total number of subscribers on any advertised image topic.
  */
  uint32_t getNumSubscribers() const;

  /**
  * Returns the number of subscribers for the raw image topic
  */
  uint32_t getNumSubscribersRaw() const;

  /**
  * Returns the number of subscribers for the rect image topic
  */
  uint32_t getNumSubscribersRect() const;

  /**
  * Grabs an image and stores the image in img_raw_msg_
  * @return false if an error occurred.
  */
  virtual bool grabImage();

  /**
  * Fills the ros CameraInfo-Object with the image dimensions
  */
  virtual void setupInitialCameraInfo(sensor_msgs::CameraInfo& cam_info_msg);

  /**
  * Update area of interest in the camera image
  * @param target_roi the target roi
  * @param reached_roi the roi that could be set
  * @return true if the targeted roi could be reached
  */
  bool setROI(const sensor_msgs::RegionOfInterest target_roi, sensor_msgs::RegionOfInterest& reached_roi);

  bool setExposureValue(const float& target_exposure, float& reached_exposure);

  /**
  * Update the exposure value on the camera
  * @param target_exposure the targeted exposure
  * @param reached_exposure the exposure that could be reached
  * @return true if the targeted exposure could be reached
  */
  bool setExposure(const float& target_exposure, float& reached_exposure);


  /**
  * Sets the target brightness which is the intensity-mean over all pixels.
  * If the target exposure time is not in the range of Arena's auto target
  * brightness range the extended brightness search is started.
  * The Auto function of the Arena-API supports values from [50 - 205].
  * Using a binary search, this range will be extended up to [1 - 255].
  * @param target_brightness is the desired brightness. Range is [1...255].
  * @return true if the brightness could be reached or false otherwise.
  */
  bool setBrightness(const int& target_brightness);

  /**
  * Update the gain from the camera to a target gain in percent
  * @param target_gain the targeted gain in percent
  * @return true if the targeted gain could be reached
  */
  bool setGain(const float& target_gain);

  /**
  * Returns true if the camera was put into sleep mode
  * @return true if in sleep mode
  */
  bool isSleeping();


  /**
  * This function will recursively be called from above setupSamplingIndices()
  * to generate the indices of pixels given the actual ROI.
  * @return indices describing the subset of points
  */
  void genSamplingIndicesRec(std::vector<std::size_t>& indices, const std::size_t& min_window_height,
                             const cv::Point2i& start, const cv::Point2i& end);


  void initCalibrationMatrices(sensor_msgs::CameraInfo& info, const cv::Mat& D, const cv::Mat& K);


  ros::NodeHandle nh_;
  ArenaCameraParameter arena_camera_parameter_set_;

  image_transport::ImageTransport* it_;
  image_transport::CameraPublisher img_raw_pub_;

  ros::Publisher* img_rect_pub_;
  image_geometry::PinholeCameraModel* pinhole_model_;

  sensor_msgs::Image img_raw_msg_;
  cv_bridge::CvImage* cv_bridge_img_rect_;

  camera_info_manager::CameraInfoManager* camera_info_manager_;

  std::vector<std::size_t> sampling_indices_;

  bool is_sleeping_;
  boost::recursive_mutex grab_mutex_;

};

}  // namespace arena_camera

#endif  // ARENA_CAMERA_ARENA_CAMERA_NODE_H
