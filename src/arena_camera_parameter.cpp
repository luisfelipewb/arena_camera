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

#define CAMERA_FRAME "arena_camera"
#define DEVICE_USER_ID ""
#define FRAME_RATE 10.0
#define CAMERA_INFO_URL ""
#define IMAGE_ENCODING "bayer_rggb8"
#define EXPOSURE 10000.0
#define GAIN 0.0
#define BRIGHTNESS 100
#define BRIGHTNESS_CONTINUOUS true
#define EXPOSURE_AUTO true
#define GAIN_AUTO true
#define EXPOSURE_SEARCH_TIMEOUT 5.0
#define AUTO_EXP_UPPER_LIM 10000000.0
#define MTU_SIZE 1500
#define INTER_PKG_DELAY 1000
#define SHUTTER_MODE_STR "global"

#include <arena_camera/arena_camera_parameter.h>

namespace arena_camera
{
ArenaCameraParameter::ArenaCameraParameter()
  : camera_frame_(CAMERA_FRAME)
  , device_user_id_(DEVICE_USER_ID)
  , frame_rate_(FRAME_RATE)
  , camera_info_url_(CAMERA_INFO_URL)
  , image_encoding_(IMAGE_ENCODING)
  , exposure_(EXPOSURE)
  , gain_(GAIN)
  , brightness_(BRIGHTNESS)
  , brightness_continuous_(BRIGHTNESS_CONTINUOUS)
  , exposure_auto_(EXPOSURE_AUTO)
  , gain_auto_(GAIN_AUTO)
  , exposure_search_timeout_(EXPOSURE_SEARCH_TIMEOUT)
  , auto_exp_upper_lim_(AUTO_EXP_UPPER_LIM)
  , mtu_size_(MTU_SIZE)
  , inter_pkg_delay_(INTER_PKG_DELAY)
  , shutter_mode_(SM_GLOBAL)
{
}

ArenaCameraParameter::~ArenaCameraParameter()
{
}

void ArenaCameraParameter::readFromRosParameterServer(const ros::NodeHandle& nh)
{
  if (nh.param<std::string>("camera_frame" , camera_frame_ , CAMERA_FRAME)) {
    ROS_INFO_STREAM("Parameter provided: camera_frame=" << camera_frame_);
  }

  if (nh.param<std::string>("device_user_id" , device_user_id_ , DEVICE_USER_ID)) {
    ROS_INFO_STREAM("Parameter provided: device_user_id=" << device_user_id_);
  }

  if (nh.param<double>("frame_rate" , frame_rate_ , FRAME_RATE)) {
    ROS_INFO_STREAM("Parameter provided: frame_rate=" << frame_rate_);
  }

  if (nh.param<std::string>("camera_info_url" , camera_info_url_ , CAMERA_INFO_URL)) {
    ROS_INFO_STREAM("Parameter provided: camera_info_url=" << camera_info_url_);
  }

  if (nh.param<std::string>("image_encoding" , image_encoding_ , IMAGE_ENCODING)) {
    ROS_INFO_STREAM("Parameter provided: image_encoding=" << image_encoding_);
  }

  if (nh.param<double>("exposure" , exposure_ , EXPOSURE)) {
    ROS_INFO_STREAM("Parameter provided: exposure=" << exposure_);
  }

  if (nh.param<double>("gain" , gain_ , GAIN)) {
    ROS_INFO_STREAM("Parameter provided: gain=" << gain_);
  }

  if (nh.param<int>("brightness" , brightness_ , BRIGHTNESS)) {
    ROS_INFO_STREAM("Parameter provided: brightness=" << brightness_);
  }

  if (nh.param<bool>("brightness_continuous" , brightness_continuous_ , BRIGHTNESS_CONTINUOUS)) {
    ROS_INFO_STREAM("Parameter provided: brightness_continuous=" << brightness_continuous_);
  }

  if (nh.param<bool>("exposure_auto" , exposure_auto_ , EXPOSURE_AUTO)) {
    ROS_INFO_STREAM("Parameter provided: exposure_auto=" << exposure_auto_);
  }

  if (nh.param<bool>("gain_auto" , gain_auto_ , GAIN_AUTO)) {
    ROS_INFO_STREAM("Parameter provided: gain_auto=" << gain_auto_);
  }

  if (nh.param<double>("exposure_search_timeout" , exposure_search_timeout_ , EXPOSURE_SEARCH_TIMEOUT)) {
    ROS_INFO_STREAM("Parameter provided: exposure_search_timeout=" << exposure_search_timeout_);
  }

  if (nh.param<double>("auto_exp_upper_lim" , auto_exp_upper_lim_ , AUTO_EXP_UPPER_LIM)) {
    ROS_INFO_STREAM("Parameter provided: auto_exp_upper_lim=" << auto_exp_upper_lim_);
  }

  if (nh.param<int>("mtu_size" , mtu_size_ , MTU_SIZE)) {
    ROS_INFO_STREAM("Parameter provided: mtu_size=" << mtu_size_);
  }

  if (nh.param<int>("inter_pkg_delay" , inter_pkg_delay_ , INTER_PKG_DELAY)) {
    ROS_INFO_STREAM("Parameter provided: inter_pkg_delay=" << inter_pkg_delay_);
  }

  std::string shutter_param_string;
  if (nh.param<std::string>("shutter_mode" , shutter_param_string , "global")) {
    ROS_INFO_STREAM("Parameter provided: shutter_mode=" << shutter_param_string);
    if (shutter_param_string == "rolling")
    {
      shutter_mode_ = SM_ROLLING;
    }
    else if (shutter_param_string == "global")
    {
      shutter_mode_ = SM_GLOBAL;
    }
    else if (shutter_param_string == "global_reset")
    {
      shutter_mode_ = SM_GLOBAL_RESET_RELEASE;
    }
    else
    {
      shutter_mode_ = SM_DEFAULT;
    }
  }

  validateParameterSet(nh);
  return;
}


void ArenaCameraParameter::validateParameterSet(const ros::NodeHandle& nh)
{
  if (device_user_id_.empty())
  {
    ROS_INFO_STREAM("No Device User ID set -> Will open the camera device found first");
  }

  if (frame_rate_ < 0 && frame_rate_ != -1)
  {
    ROS_WARN_STREAM("Invalid frame rate (" << frame_rate_ << "Hz). Using " << FRAME_RATE << " Hz instead");
    setFrameRate(nh, FRAME_RATE);
  }

  if (exposure_ <= 0.0 || exposure_ > 1e7)
  {
    ROS_WARN_STREAM("Invalid exposure (" << exposure_ << "ms) Using " << EXPOSURE << " instead");
    setExposure(nh, EXPOSURE);
  }

  if (gain_ < 0.0 || gain_ > 1.0)
  {
    ROS_WARN_STREAM("Invalid gain (" << gain_ << ") Using " << gain_ << " instead");
    setGain(nh, GAIN);
  }

  if (brightness_ < 0.0 || brightness_ > 255)
  {
    ROS_WARN_STREAM("Invalid brightness (" << gain_ << ") Using " << BRIGHTNESS << " instead");
    setBrightness(nh, BRIGHTNESS);
  }

  if (exposure_search_timeout_ < 5.)
  {
    ROS_WARN_STREAM("Low timeout for exposure search detected! Exposure search may fail.");
  }

  return;
}

const std::string& ArenaCameraParameter::deviceUserID() const
{
  return device_user_id_;
}

std::string ArenaCameraParameter::shutterModeString() const
{
  if (shutter_mode_ == SM_ROLLING)
  {
    return "rolling";
  }
  else if (shutter_mode_ == SM_GLOBAL)
  {
    return "global";
  }
  else if (shutter_mode_ == SM_GLOBAL_RESET_RELEASE)
  {
    return "global_reset";
  }
  else
  {
    return "default_shutter_mode";
  }
}

const std::string& ArenaCameraParameter::imageEncoding() const
{
  return image_encoding_;
}

const std::string& ArenaCameraParameter::cameraFrame() const
{
  return camera_frame_;
}

const double& ArenaCameraParameter::frameRate() const
{
  return frame_rate_;
}


const std::string& ArenaCameraParameter::cameraInfoURL() const
{
  return camera_info_url_;
}

void ArenaCameraParameter::setFrameRate(const ros::NodeHandle& nh, const double& frame_rate)
{
  frame_rate_ = frame_rate;
  nh.setParam("frame_rate", frame_rate_);
}

void ArenaCameraParameter::setExposure(const ros::NodeHandle& nh, const double& exposure)
{
  exposure_ = exposure;
  nh.setParam("exposure", exposure_);
}

void ArenaCameraParameter::setGain(const ros::NodeHandle& nh, const double& gain)
{
  gain_ = gain;
  nh.setParam("gain", gain_);
}

void ArenaCameraParameter::setBrightness(const ros::NodeHandle& nh, const int& brightness)
{
  brightness_ = brightness;
  nh.setParam("brightness", brightness_);
}

void ArenaCameraParameter::setCameraInfoURL(const ros::NodeHandle& nh, const std::string& camera_info_url)
{
  camera_info_url_ = camera_info_url;
  nh.setParam("camera_info_url", camera_info_url_);
}

}  // namespace arena_camera
