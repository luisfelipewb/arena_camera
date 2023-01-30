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

// Arena
#include <ArenaApi.h>
#include <GenApi/GenApi.h>
#include <GenApiCustom.h>

// Arena node
#include <arena_camera/arena_camera_node.h>
#include <arena_camera/encoding_conversions.h>

namespace arena_camera
{
Arena::ISystem* pSystem_ = nullptr;
Arena::IDevice* pDevice_ = nullptr;
Arena::IImage* pImage_ = nullptr;
const uint8_t* pData_ = nullptr;
GenApi::INodeMap* pNodeMap_ = nullptr;

using sensor_msgs::CameraInfo;
using sensor_msgs::CameraInfoPtr;

ArenaCameraNode::ArenaCameraNode()
  : nh_("~")
  , arena_camera_parameter_set_()
  // Arena
  , arena_camera_(nullptr)
  // others
  , it_(new image_transport::ImageTransport(nh_))
  , img_raw_pub_(it_->advertiseCamera("image_raw", 1))
  , img_rect_pub_(nullptr)
  , pinhole_model_(nullptr)
  , cv_bridge_img_rect_(nullptr)
  , camera_info_manager_(new camera_info_manager::CameraInfoManager(nh_)) // should this be freed in ~() ?
  , sampling_indices_()
  , is_sleeping_(false)
{
  init();
}

void ArenaCameraNode::init()
{
  // reading all necessary parameter to open the desired camera from the
  // ros-parameter-server. In case that invalid parameter values can be
  // detected, the interface will reset them to the default values.
  // These parameters furthermore contain the intrinsic calibration matrices,
  // in case they are provided
  arena_camera_parameter_set_.readFromRosParameterServer(nh_);

  // setting the camera info URL to produce rectified image. Can substitute
  // any desired file path or comment out this line if only producing raw
  // images.
  //  arena_camera_parameter_set_.setCameraInfoURL(nh_,
  //  "file://${ROS_HOME}/camera_info/camera.yaml");

  // creating the target ArenaCamera-Object with the specified
  // device_user_id, registering the Software-Trigger-Mode, starting the
  // communication with the device and enabling the desired startup-settings
  if (!initAndRegister())
  {
    ros::shutdown();
    return;
  }

  // starting the grabbing procedure with the desired image-settings
  if (!startGrabbing())
  {
    ros::shutdown();
    return;
  }
}

bool createDevice(const std::string& device_user_id_to_open)
{
  pSystem_ = Arena::OpenSystem();
  pSystem_->UpdateDevices(100);
  std::vector<Arena::DeviceInfo> deviceInfos = pSystem_->GetDevices();

  if (deviceInfos.size() == 0)
  {
    Arena::CloseSystem(pSystem_);
    pSystem_ = nullptr;
    return false;
  }
  else
  {
    if (device_user_id_to_open.empty())
    {
      pDevice_ = pSystem_->CreateDevice(deviceInfos[0]);
      return true;
    }
    else
    {
      std::vector<Arena::DeviceInfo>::iterator it;
      bool found_desired_device = false;

      for (it = deviceInfos.begin(); it != deviceInfos.end(); ++it)
      {
        std::string device_user_id_found(it->UserDefinedName());
        if ((0 == device_user_id_to_open.compare(device_user_id_found)) ||
            (device_user_id_to_open.length() < device_user_id_found.length() &&
             (0 ==
              device_user_id_found.compare(device_user_id_found.length() - device_user_id_to_open.length(),
                                           device_user_id_to_open.length(), device_user_id_to_open))))
        {
          found_desired_device = true;
          break;
        }
      }
      if (found_desired_device)
      {
        ROS_INFO_STREAM("Found the desired camera with DeviceUserID " << device_user_id_to_open << ": ");

        pDevice_ = pSystem_->CreateDevice(*it);
        return true;
      }
      else
      {
        ROS_ERROR_STREAM("Couldn't find the camera that matches the "
                         << "given DeviceUserID: " << device_user_id_to_open << "! "
                         << "Either the ID is wrong or the cam is not yet connected");
        return false;
      }
    }
  }
}

bool ArenaCameraNode::initAndRegister()
{
  bool device_found_ = false;
  device_found_ = createDevice(arena_camera_parameter_set_.deviceUserID());

  if (device_found_ == false)
  {
    // wait and retry until a camera is present
    ros::Time end = ros::Time::now() + ros::Duration(15.0);
    ros::Rate r(0.5);
    while (ros::ok() && device_found_ == false)
    {
      device_found_ = createDevice(arena_camera_parameter_set_.deviceUserID());
      if (ros::Time::now() > end)
      {
        ROS_WARN_STREAM("No camera present. Keep waiting ...");
        end = ros::Time::now() + ros::Duration(15.0);
      }
      r.sleep();
      ros::spinOnce();
    }
  }
  else
  {
    ROS_INFO_STREAM("Camera " << arena_camera_parameter_set_.deviceUserID() << " is found!");
  }

  if (!ros::ok())
  {
    return false;
  }

  return true;
}

sensor_msgs::RegionOfInterest currentROI()
{
  sensor_msgs::RegionOfInterest roi;
  roi.width = pImage_->GetWidth();
  roi.height = pImage_->GetHeight();
  ;
  roi.x_offset = pImage_->GetOffsetX();
  roi.y_offset = pImage_->GetOffsetY();
  return roi;
}

float currentGamma()
{
  GenApi::CFloatPtr pGamma = pDevice_->GetNodeMap()->GetNode("Gamma");

  if (!pGamma || !GenApi::IsReadable(pGamma))
  {
    ROS_WARN_STREAM("No gamma value, returning -1");
    return -1.;
  }
  else
  {
    float gammaValue = pGamma->GetValue();
    return gammaValue;
  }
}

int64_t currentBinningX()
{
  GenApi::CIntegerPtr BinningHorizontal = pDevice_->GetNodeMap()->GetNode("BinningHorizontal");

  if (!BinningHorizontal || !GenApi::IsReadable(BinningHorizontal))
  {
    ROS_WARN_STREAM("No binningY value, returning -1");
    return -1;
  }
  else
  {
    float binningXValue = BinningHorizontal->GetValue();
    return binningXValue;
  }
}

int64_t currentBinningY()
{
  GenApi::CIntegerPtr BinningVertical = pDevice_->GetNodeMap()->GetNode("BinningVertical");

  if (!BinningVertical || !GenApi::IsReadable(BinningVertical))
  {
    ROS_WARN_STREAM("No binningY value, returning -1");
    return -1;
  }
  else
  {
    float binningYValue = BinningVertical->GetValue();
    return binningYValue;
  }
}

float currentGain()
{
  GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");

  if (!pGain || !GenApi::IsReadable(pGain))
  {
    ROS_WARN_STREAM("No gain value");
    return -1.;
  }
  else
  {
    float gainValue = pGain->GetValue();
    return gainValue;
  }
}

float currentExposure()
{
  GenApi::CFloatPtr pExposureTime = pDevice_->GetNodeMap()->GetNode("ExposureTime");

  if (!pExposureTime || !GenApi::IsReadable(pExposureTime))
  {
    ROS_WARN_STREAM("No exposure time value, returning -1");
    return -1.;
  }
  else
  {
    float exposureValue = pExposureTime->GetValue();
    return exposureValue;
  }
}

std::string currentROSEncoding()
{
  std::string gen_api_encoding(Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat"));
  std::string ros_encoding("");
  if (!encoding_conversions::genAPI2Ros(gen_api_encoding, ros_encoding))
  {
    std::stringstream ss;
    ss << "No ROS equivalent to GenApi encoding '" << gen_api_encoding << "' found! This is bad because this case "
                                                                          "should never occur!";
    throw std::runtime_error(ss.str());
    return "NO_ENCODING";
  }
  return ros_encoding;
}

bool ArenaCameraNode::setImageEncoding(const std::string& ros_encoding)
{
  std::string gen_api_encoding;
  bool conversion_found = encoding_conversions::ros2GenAPI(ros_encoding, gen_api_encoding);
  if (!conversion_found)
  {
    if (ros_encoding.empty())
    {
      return false;
    }
    else
    {
      std::string fallbackPixelFormat = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat").c_str();
      ROS_ERROR_STREAM("Can't convert ROS encoding '" << ros_encoding
                                                      << "' to a corresponding GenAPI encoding! Will use current "
                                                      << "pixel format ( "
                                                      << fallbackPixelFormat
                                                      << " ) as fallback!"); 
      return false;
    }
  }
  try
  {
    GenApi::CEnumerationPtr pPixelFormat = pDevice_->GetNodeMap()->GetNode("PixelFormat");
    if (GenApi::IsWritable(pPixelFormat))
    {
      Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "PixelFormat", gen_api_encoding.c_str());
      if (currentROSEncoding() == "16UC3" || currentROSEncoding() == "16UC4")
        ROS_WARN_STREAM("ROS grabbing image data from 3D pixel format, unable to display in image viewer");
    }
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("An exception while setting target image encoding to '" << ros_encoding
                                                                             << "' occurred: " << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::startGrabbing()
{
  auto  pNodeMap = pDevice_->GetNodeMap();

  try
  {
    //
    // Arena device prior streaming settings
    //

    //
    // PIXELFORMAT
    //
    setImageEncoding(arena_camera_parameter_set_.imageEncoding());

    //
    // TRIGGER MODE
    //
    GenApi::CStringPtr pTriggerMode = pNodeMap->GetNode("TriggerMode");
    if (GenApi::IsWritable(pTriggerMode))
    {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerMode", "On");
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "TriggerSource", "Software");
    }

    //
    // FRAMERATE
    //
    auto cmdlnParamFrameRate = arena_camera_parameter_set_.frameRate();
    auto currentFrameRate = Arena::GetNodeValue<double>(pNodeMap , "AcquisitionFrameRate");
    auto maximumFrameRate = GenApi::CFloatPtr(pNodeMap->GetNode("AcquisitionFrameRate"))->GetMax();

    // requested framerate larger than device max so we trancate it
    if (cmdlnParamFrameRate >= maximumFrameRate)
    {
      arena_camera_parameter_set_.setFrameRate(nh_, maximumFrameRate);
      
      ROS_WARN("Desired framerate %.2f Hz (rounded) is higher than max possible. Will limit "
              "framerate device max : %.2f Hz (rounded)", cmdlnParamFrameRate, maximumFrameRate);
    }
    // special case:
    // dues to inacurate float comparision we skip. If we set it it might
    // throw becase it could be a lil larger than the max avoid the exception (double accuracy issue when setting the node) 
    // request frame rate very close to device max
    else if (cmdlnParamFrameRate == maximumFrameRate){
      ROS_INFO("Framerate is %.2f Hz", cmdlnParamFrameRate);
    }
    // requested max frame rate
    else if (cmdlnParamFrameRate == -1) // speacial for max frame rate available
    {
      arena_camera_parameter_set_.setFrameRate(nh_, maximumFrameRate);
      
      ROS_WARN("Framerate is set to device max : %.2f Hz", maximumFrameRate);
    }
    // requested framerate is valid so we set it to the device
    else{
      Arena::SetNodeValue<bool>(pNodeMap, "AcquisitionFrameRateEnable", true);
      Arena::SetNodeValue<double>(pNodeMap, "AcquisitionFrameRate" , 
                                      cmdlnParamFrameRate);
      ROS_INFO("Framerate is set to: %.2f Hz", cmdlnParamFrameRate);
    }

    //
    // EXPOSURE AUTO & EXPOSURE
    //

    // exposure_auto_ will be already set to false if exposure_given_ is true
    // read params () solved the priority between them
    if (arena_camera_parameter_set_.exposure_auto_)
    {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Continuous");
      // todo update parameter on the server
      ROS_INFO_STREAM("Settings Exposure to auto/Continuous");
    }
    else
    {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "ExposureAuto", "Off");
      // todo update parameter on the server
      ROS_INFO_STREAM("Settings Exposure to off/false");
    }

    if (arena_camera_parameter_set_.exposure_given_)
     {
      float reached_exposure;
      if (setExposure(arena_camera_parameter_set_.exposure_, reached_exposure))
      {
        // Note: ont update the ros param because it might keep 
        // decreasing or incresing overtime when rerun
        ROS_INFO_STREAM("Setting exposure to " << arena_camera_parameter_set_.exposure_
                                               << ", reached: " << reached_exposure);
      }
    }

    //
    // GAIN
    //
    
    // gain_auto_ will be already set to false if gain_given_ is true
    // read params () solved the priority between them
    if (arena_camera_parameter_set_.gain_auto_)
    {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Continuous");
      // todo update parameter on the server
      ROS_INFO_STREAM("Settings Gain to auto/Continuous");
    }
    else
    {
      Arena::SetNodeValue<GenICam::gcstring>(pNodeMap, "GainAuto", "Off");
      // todo update parameter on the server
      ROS_INFO_STREAM("Settings Gain to off/false");
    }

    if (arena_camera_parameter_set_.gain_given_)
    {
      float reached_gain;
      if (setGain(arena_camera_parameter_set_.gain_))
      {
        // Note: ont update the ros param because it might keep 
        // decreasing or incresing overtime when rerun
        ROS_INFO_STREAM("Setting gain to: " << arena_camera_parameter_set_.gain_);
      }
    }

    // ------------------------------------------------------------------------

    //
    //  Initial setting of the CameraInfo-msg, assuming no calibration given
    CameraInfo initial_cam_info;
    setupInitialCameraInfo(initial_cam_info);
    camera_info_manager_->setCameraInfo(initial_cam_info);

    if (arena_camera_parameter_set_.cameraInfoURL().empty() ||
        !camera_info_manager_->validateURL(arena_camera_parameter_set_.cameraInfoURL()))
    {
      ROS_INFO_STREAM("CameraInfoURL needed for rectification! ROS-Param: "
                      << "'" << nh_.getNamespace() << "/camera_info_url' = '"
                      << arena_camera_parameter_set_.cameraInfoURL() << "' is invalid!");
      ROS_DEBUG_STREAM("CameraInfoURL should have following style: "
                       << "'file:///full/path/to/local/file.yaml' or "
                       << "'file://${ROS_HOME}/camera_info/${NAME}.yaml'");
      ROS_WARN_STREAM("Will only provide distorted /image_raw images!");
    }
    else
    {
      // override initial camera info if the url is valid
      if (camera_info_manager_->loadCameraInfo(arena_camera_parameter_set_.cameraInfoURL()))
      {
        setupRectification();
        // set the correct tf frame_id
        CameraInfoPtr cam_info(new CameraInfo(camera_info_manager_->getCameraInfo()));
        cam_info->header.frame_id = img_raw_msg_.header.frame_id;
        camera_info_manager_->setCameraInfo(*cam_info);
      }
      else
      {
        ROS_WARN_STREAM("Will only provide distorted /image_raw images!");
      }
    }

    // if (arena_camera_parameter_set_.image_encoding_given_)
    // {
    // 	float reached_image_encoding;
    // 	if (setImageEncoding(arena_camera_parameter_set_.image_encoding_))
    // 	{
    // 		ROS_INFO_STREAM("Setting exposure to "
    // 						<< arena_camera_parameter_set_.image_encoding_);
    // 	}
    // }

    // Configuring StreamPacketResendEnable significantly improves performance
    // TODO: Cleanup
    GenICam::gcstring triggerSelectorInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSelector");
    GenICam::gcstring triggerModeInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerMode");
    GenICam::gcstring triggerSourceInitial = Arena::GetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "TriggerSource");

    ROS_DEBUG_STREAM( "Initial stream configuration"
                    << "\ttriggerSelectorInitial: " << triggerSelectorInitial
                    << "\ttriggerModeInitial: " << triggerModeInitial
                    << "\ttriggerSourceInitial: " << triggerSourceInitial << std::endl);

    // enable stream auto negotiate packet size
	  Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
	  // enable stream packet resend
	  Arena::SetNodeValue<bool>(pDevice_->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
    // TODO: Confirm if necessary
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");

    //
    // Trigger Image
    //

    pDevice_->StartStream();
    bool isTriggerArmed = false;

    if (GenApi::IsWritable(pTriggerMode))
    {
      do
      {
        isTriggerArmed = Arena::GetNodeValue<bool>(pNodeMap, "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pNodeMap, "TriggerSoftware");
    }

    pImage_ = pDevice_->GetImage(5000);
    pData_ = pImage_->GetData();

    img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
    memcpy(&img_raw_msg_.data[0], pImage_->GetData(), img_raw_msg_.height * img_raw_msg_.step);
  }
  catch (GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("Error while grabbing first image occurred: \r\n" << e.GetDescription());
    return false;
  }

  // --------------------------------------------------------------------------

  img_raw_msg_.header.frame_id = arena_camera_parameter_set_.cameraFrame();
  // Encoding of pixels -- channel meaning, ordering, size
  // taken from the list of strings in include/sensor_msgs/image_encodings.h
  img_raw_msg_.encoding = currentROSEncoding();
  img_raw_msg_.height = pImage_->GetHeight();
  img_raw_msg_.width = pImage_->GetWidth();
  // step = full row length in bytes, img_size = (step * rows), imagePixelDepth
  // already contains the number of channels
  img_raw_msg_.step = img_raw_msg_.width * (pImage_->GetBitsPerPixel() / 8);

  if (!camera_info_manager_->setCameraName(
          std::string(Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID").c_str())))
  {
    // valid name contains only alphanumeric signs and '_'
    ROS_WARN_STREAM(
        "DeviceUserID [" << std::string(Arena::GetNodeValue<GenICam::gcstring>(pNodeMap, "DeviceUserID").c_str())
            << "] name not valid for camera_info_manager");
  }

  ROS_INFO_STREAM("Startup settings: "
                  << "encoding = '" << currentROSEncoding() << "', "
                  << "exposure = " << currentExposure() << ", "
                  << "gain = " << currentGain() << ", "
                  << "shutter mode = " << arena_camera_parameter_set_.shutterModeString());

  pDevice_->RequeueBuffer(pImage_);
  return true;
}

void ArenaCameraNode::setupRectification()
{
  if (!img_rect_pub_)
  {
    img_rect_pub_ = new ros::Publisher(nh_.advertise<sensor_msgs::Image>("image_rect", 1));
  }


  if (!pinhole_model_)
  {
    pinhole_model_ = new image_geometry::PinholeCameraModel();
  }

  pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
  if (!cv_bridge_img_rect_)
  {
    cv_bridge_img_rect_ = new cv_bridge::CvImage();
  }
  cv_bridge_img_rect_->header = img_raw_msg_.header;
  cv_bridge_img_rect_->encoding = img_raw_msg_.encoding;
}

struct CameraPublisherImpl
{
  image_transport::Publisher image_pub_;
  ros::Publisher info_pub_;
  bool unadvertised_;
  // double constructed_;
};

class CameraPublisherLocal
{
public:
  struct Impl;
  typedef boost::shared_ptr<Impl> ImplPtr;
  typedef boost::weak_ptr<Impl> ImplWPtr;

  CameraPublisherImpl* impl_;
};

uint32_t ArenaCameraNode::getNumSubscribersRaw() const
{
  return ((CameraPublisherLocal*)(&img_raw_pub_))->impl_->image_pub_.getNumSubscribers();
}

void ArenaCameraNode::spin()
{
  if (camera_info_manager_->isCalibrated())
  {
    ROS_INFO_ONCE("Camera is calibrated");
  }
  else
  {
    ROS_INFO_ONCE("Camera not calibrated");
  }

  if (pDevice_->IsConnected() == false)
  {
    ROS_ERROR("Arena camera has been removed, trying to reset");
    pSystem_->DestroyDevice(pDevice_);
    pDevice_ = nullptr;
    Arena::CloseSystem(pSystem_);
    pSystem_ = nullptr;

    ros::Duration(0.5).sleep();  // sleep for half a second
    init();
    return;
  }

  if (!isSleeping() && (img_raw_pub_.getNumSubscribers() || getNumSubscribersRect()))
  {
    if (getNumSubscribersRaw() || getNumSubscribersRect())
    {
      if (!grabImage())
      {
        ROS_INFO("did not get image");
        return;
      }
    }

    if (img_raw_pub_.getNumSubscribers() > 0)
    {
      // get actual cam_info-object in every frame, because it might have
      // changed due to a 'set_camera_info'-service call
      sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(camera_info_manager_->getCameraInfo()));
      cam_info->header.stamp = img_raw_msg_.header.stamp;

      // Publish via image_transport
      img_raw_pub_.publish(img_raw_msg_, *cam_info);
      ROS_INFO_ONCE("Number subscribers received");
    }

    if (getNumSubscribersRect() > 0 && camera_info_manager_->isCalibrated())
    {
      cv_bridge_img_rect_->header.stamp = img_raw_msg_.header.stamp;
      assert(pinhole_model_->initialized());
      cv_bridge::CvImagePtr cv_img_raw = cv_bridge::toCvCopy(img_raw_msg_, img_raw_msg_.encoding);
      pinhole_model_->fromCameraInfo(camera_info_manager_->getCameraInfo());
      pinhole_model_->rectifyImage(cv_img_raw->image, cv_bridge_img_rect_->image);
      img_rect_pub_->publish(*cv_bridge_img_rect_);
      ROS_INFO_ONCE("Number subscribers rect received");
    }
  }
}

bool ArenaCameraNode::grabImage()
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  try
  {
    GenApi::CStringPtr pTriggerMode = pDevice_->GetNodeMap()->GetNode("TriggerMode");
    if (GenApi::IsWritable(pTriggerMode))
    {
      bool isTriggerArmed = false;

      do
      {
        isTriggerArmed = Arena::GetNodeValue<bool>(pDevice_->GetNodeMap(), "TriggerArmed");
      } while (isTriggerArmed == false);
      Arena::ExecuteNode(pDevice_->GetNodeMap(), "TriggerSoftware");
    }
    pImage_ = pDevice_->GetImage(5000);
    pData_ = pImage_->GetData();

    img_raw_msg_.data.resize(img_raw_msg_.height * img_raw_msg_.step);
    memcpy(&img_raw_msg_.data[0], pImage_->GetData(), img_raw_msg_.height * img_raw_msg_.step);

    img_raw_msg_.header.stamp = ros::Time::now();

    pDevice_->RequeueBuffer(pImage_);
    return true;
  }
  catch (GenICam::GenericException& e)
  {
    return false;
  }
}


const double& ArenaCameraNode::frameRate() const
{
  return arena_camera_parameter_set_.frameRate();
}

const std::string& ArenaCameraNode::cameraFrame() const
{
  return arena_camera_parameter_set_.cameraFrame();
}

uint32_t ArenaCameraNode::getNumSubscribersRect() const
{
  return camera_info_manager_->isCalibrated() ? img_rect_pub_->getNumSubscribers() : 0;
}

uint32_t ArenaCameraNode::getNumSubscribers() const
{
  return img_raw_pub_.getNumSubscribers() + img_rect_pub_->getNumSubscribers();
}

void ArenaCameraNode::setupInitialCameraInfo(sensor_msgs::CameraInfo& cam_info_msg)
{
  std_msgs::Header header;
  header.frame_id = arena_camera_parameter_set_.cameraFrame();
  header.stamp = ros::Time::now();

  // http://www.ros.org/reps/rep-0104.html
  // If the camera is uncalibrated, the matrices D, K, R, P should be left
  // zeroed out. In particular, clients may assume that K[0] == 0.0
  // indicates an uncalibrated camera.
  cam_info_msg.header = header;

  // The image dimensions with which the camera was calibrated. Normally
  // this will be the full camera resolution in pixels. They remain fix, even
  // if binning is applied
  // rows and colums
  cam_info_msg.height = Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Height");
  cam_info_msg.width = Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "Width");

  // The distortion model used. Supported models are listed in
  // sensor_msgs/distortion_models.h. For most cameras, "plumb_bob" - a
  // simple model of radial and tangential distortion - is sufficient.
  // Empty D and distortion_model indicate that the CameraInfo cannot be used
  // to rectify points or images, either because the camera is not calibrated
  // or because the rectified image was produced using an unsupported
  // distortion model, e.g. the proprietary one used by Bumblebee cameras
  // [http://www.ros.org/reps/rep-0104.html].
  cam_info_msg.distortion_model = "";

  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3) -> float64[] D.
  cam_info_msg.D = std::vector<double>(5, 0.);

  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]  --> 3x3 row-major matrix
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel coordinates
  // using the focal lengths (fx, fy) and principal point (cx, cy).
  cam_info_msg.K.assign(0.0);

  // Rectification matrix (stereo cameras only)
  // A rotation matrix aligning the camera coordinate system to the ideal
  // stereo image plane so that epipolar lines in both stereo images are
  // parallel.
  cam_info_msg.R.assign(0.0);

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]  --> # 3x4 row-major matrix
  //     [ 0   0   1   0]
  // By convention, this matrix specifies the intrinsic (camera) matrix of the
  // processed (rectified) image. That is, the left 3x3 portion is the normal
  // camera intrinsic matrix for the rectified image. It projects 3D points
  // in the camera coordinate frame to 2D pixel coordinates using the focal
  // lengths (fx', fy') and principal point (cx', cy') - these may differ from
  // the values in K. For monocular cameras, Tx = Ty = 0. Normally, monocular
  // cameras will also have R = the identity and P[1:3,1:3] = K.
  // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
  // position of the optical center of the second camera in the first
  // camera's frame. We assume Tz = 0 so both cameras are in the same
  // stereo image plane. The first camera always has Tx = Ty = 0.
  // For the right (second) camera of a horizontal stereo pair,
  // Ty = 0 and Tx = -fx' * B, where B is the baseline between the cameras.
  // Given a 3D point [X Y Z]', the projection (x, y) of the point onto the
  // rectified image is given by:
  // [u v w]' = P * [X Y Z 1]'
  //        x = u / w
  //        y = v / w
  //  This holds for both images of a stereo pair.
  cam_info_msg.P.assign(0.0);

  // Binning refers here to any camera setting which combines rectangular
  // neighborhoods of pixels into larger "super-pixels." It reduces the
  // resolution of the output image to (width / binning_x) x (height /
  // binning_y). The default values binning_x = binning_y = 0 is considered the
  // same as binning_x = binning_y = 1 (no subsampling).
  //  cam_info_msg.binning_x = currentBinningX();
  //  cam_info_msg.binning_y = currentBinningY();

  // Region of interest (subwindow of full camera resolution), given in full
  // resolution (unbinned) image coordinates. A particular ROI always denotes
  // the same window of pixels on the camera sensor, regardless of binning
  // settings. The default setting of roi (all values 0) is considered the same
  // as full resolution (roi.width = width, roi.height = height).

  // todo? do these has ti be set via Arena::GetNodeValue<int64_t>(pDevice_->GetNodeMap(), "OffsetX"); or so ?
  cam_info_msg.roi.x_offset = cam_info_msg.roi.y_offset = 0;
  cam_info_msg.roi.height = cam_info_msg.roi.width = 0;
}

bool ArenaCameraNode::setROI(const sensor_msgs::RegionOfInterest target_roi, sensor_msgs::RegionOfInterest& reached_roi)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  // TODO: set ROI
  return true;
}


bool ArenaCameraNode::setExposureValue(const float& target_exposure, float& reached_exposure)
{
  try
  {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Off");

    GenApi::CFloatPtr pExposureTime = pDevice_->GetNodeMap()->GetNode("ExposureTime");

    float exposure_to_set = target_exposure;
    if (exposure_to_set < pExposureTime->GetMin())
    {
      ROS_WARN_STREAM("Desired exposure (" << exposure_to_set << ") "
                                           << "time unreachable! Setting to lower limit: " << pExposureTime->GetMin());
      exposure_to_set = pExposureTime->GetMin();
    }
    else if (exposure_to_set > pExposureTime->GetMax())
    {
      ROS_WARN_STREAM("Desired exposure (" << exposure_to_set << ") "
                                           << "time unreachable! Setting to upper limit: " << pExposureTime->GetMax());
      exposure_to_set = pExposureTime->GetMax();
    }

    pExposureTime->SetValue(exposure_to_set);
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("An exception while setting target exposure to " << target_exposure
                                                                      << " occurred:" << e.GetDescription());
    return false;
  }
  return true;
}

bool ArenaCameraNode::setExposure(const float& target_exposure, float& reached_exposure)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  // if ( !pylon_camera_->isReady() )
  // {
  //     ROS_WARN("Error in setExposure(): pylon_camera_ is not ready!");
  //     return false;
  // }

  if (ArenaCameraNode::setExposureValue(target_exposure, reached_exposure))
  {
    // success if the delta is smaller then the exposure step
    return true;
  }
  else  // retry till timeout
  {
    // wait for max 5s till the cam has updated the exposure
    ros::Rate r(10.0);
    ros::Time timeout(ros::Time::now() + ros::Duration(5.0));
    while (ros::ok())
    {
      if (ArenaCameraNode::setExposureValue(target_exposure, reached_exposure))
      {
        // success if the delta is smaller then the exposure step
        return true;
      }

      if (ros::Time::now() > timeout)
      {
        break;
      }
      r.sleep();
    }
    ROS_ERROR_STREAM("Error in setExposure(): Unable to set target"
                     << " exposure before timeout");
    return false;
  }
}

bool ArenaCameraNode::setGain(const float& target_gain)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  //TODO: Cleanup
  float reached_gain;
  try
  {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Off");

    GenApi::CFloatPtr pGain = pDevice_->GetNodeMap()->GetNode("Gain");
    float truncated_gain = target_gain;
    if (truncated_gain < pGain->GetMin())
    {
      ROS_WARN_STREAM("Desired gain (" << target_gain << ") in "
                                       << "percent out of range [0.0 - 1.0]! Setting to lower "
                                       << "limit: 0.0");
      truncated_gain = pGain->GetMin();
    }
    else if (truncated_gain > pGain->GetMax())
    {
      ROS_WARN_STREAM("Desired gain (" << target_gain << ") in "
                                       << "percent out of range [0.0 - 1.0]! Setting to upper "
                                       << "limit: 1.0");
      truncated_gain = pGain->GetMax();
    }

    float gain_to_set = pGain->GetMin() + truncated_gain * (pGain->GetMax() - pGain->GetMin());
    pGain->SetValue(gain_to_set);
  }
  catch (const GenICam::GenericException& e)
  {
    ROS_ERROR_STREAM("An exception while setting target gain to " << target_gain
                                                                  << " occurred: " << e.GetDescription());
    return false;
  }
  return true;
}


void disableAllRunningAutoBrightessFunctions()
{
  GenApi::CStringPtr pExposureAuto = pNodeMap_->GetNode("ExposureAuto");
  GenApi::CStringPtr pGainAuto = pNodeMap_->GetNode("GainAuto");
  if (!pExposureAuto || !GenApi::IsWritable(pExposureAuto) || !pGainAuto || !GenApi::IsWritable(pGainAuto))
  {
    ROS_WARN_STREAM("Unable to disable auto brightness");
    return;
  }

  else
  {
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "ExposureAuto", "Off");
    Arena::SetNodeValue<GenICam::gcstring>(pDevice_->GetNodeMap(), "GainAuto", "Off");
  }
}

bool ArenaCameraNode::setBrightness(const int& target_brightness)
{
  boost::lock_guard<boost::recursive_mutex> lock(grab_mutex_);
  // TODO: clenaup
  Arena::SetNodeValue<int64_t>(pDevice_->GetNodeMap(), "TargetBrightness", target_brightness);
  ROS_INFO_STREAM("WOLF setBrightness: " << target_brightness);
  // double exposureTime = Arena::GetNodeValue<double>(pDevice_->GetNodeMap(), "ExposureTime");
  return true;
}


bool ArenaCameraNode::isSleeping()
{
  return is_sleeping_;
}

ArenaCameraNode::~ArenaCameraNode()
{
  if (pDevice_ != nullptr)
  {
    pSystem_->DestroyDevice(pDevice_);
  }

  if (pSystem_ != nullptr)
  {
    Arena::CloseSystem(pSystem_);
  }

  if (it_)
  {
    delete it_;
    it_ = nullptr;
  }

  if (img_rect_pub_)
  {
    delete img_rect_pub_;
    img_rect_pub_ = nullptr;
  }

  if (cv_bridge_img_rect_)
  {
    delete cv_bridge_img_rect_;
    cv_bridge_img_rect_ = nullptr;
  }

  if (pinhole_model_)
  {
    delete pinhole_model_;
    pinhole_model_ = nullptr;
  }
}

}  // namespace arena_camera
