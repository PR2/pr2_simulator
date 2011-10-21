/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/*
 @mainpage
   Desc: GazeboRosProsilica plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b GazeboRosProsilica plugin mimics after prosilica_camera package
 */

#include <algorithm>
#include <assert.h>

#include <pr2_gazebo_plugins/gazebo_ros_prosilica.h>

#include "gazebo.h"
#include "physics/World.hh"
#include "physics/HingeJoint.hh"
#include "sensors/Sensor.hh"
#include "sdf/interface/SDF.hh"
#include "sdf/interface/Param.hh"
#include "common/Exception.hh"
#include "sensors/CameraSensor.hh"
#include "sensors/SensorTypes.hh"
#include "rendering/Camera.hh"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <cv_bridge/CvBridge.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <cv.h>
#include <cvwimage.h>

#include <boost/scoped_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/thread.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <string>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosProsilica::GazeboRosProsilica()
{
  this->imageConnectCount = 0;
  this->infoConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosProsilica::~GazeboRosProsilica()
{
  // Finalize the controller
  this->parentSensor->SetActive(false);
  this->rosnode_->shutdown();
#ifdef USE_CBQ
  this->prosilica_queue_.clear();
  this->prosilica_queue_.disable();
  this->prosilica_callback_queue_thread_.join();
#else
  this->ros_spinner_thread_.join();
#endif

  this->poll_srv_.shutdown();
  this->image_pub_.shutdown();
  this->camera_info_pub_.shutdown();




  delete this->rosnode_;

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosProsilica::Load(sensors::SensorPtr &_parent, sdf::ElementPtr &_sdf)
{



  // Get then name of the parent sensor
  this->parentSensor = _parent;


  // Get the world name.
  std::string worldName = _sdf->GetWorldName();
  this->world = physics::get_world(worldName);

  gzdbg << "plugin parent sensor name: " << this->parentSensor->GetName() << "\n";

  //this->node = transport::NodePtr(new transport::Node());
  //this->node->Init(worldName);
  //this->statsSub = this->node->Subscribe("~/world_stats", &GazeboRosProsilica::OnStats, this);

  this->parentCameraSensor = boost::shared_dynamic_cast<sensors::CameraSensor>(this->parentSensor);

  if (!this->parentCameraSensor)
    gzthrow("GazeboRosProsilica controller requires a Camera Sensor as its parent");





  //this->robotNamespaceP->Load(node);
  this->robotNamespace = ""; // FIXME

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  //this->cameraNameP->Load(node);
  this->cameraName = "prosilica"; // FIXME: this->cameraNameP->GetValue();
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace+"/"+this->cameraName);
  this->rosnode_->setCallbackQueue(&this->prosilica_queue_);
  this->itnode_ = new image_transport::ImageTransport(*this->rosnode_);

  //this->imageTopicNameP->Load(node);
  // this->cameraInfoTopicNameP->Load(node);
  // this->pollServiceNameP->Load(node);
  // this->frameNameP->Load(node);
  // this->CxPrimeP->Load(node);
  // this->CxP->Load(node);
  // this->CyP->Load(node);
  // this->focal_lengthP->Load(node);
  // this->hack_baselineP->Load(node);
  // this->distortion_k1P->Load(node);
  // this->distortion_k2P->Load(node);
  // this->distortion_k3P->Load(node);
  // this->distortion_t1P->Load(node);
  // this->distortion_t2P->Load(node);
  
  this->imageTopicName = "image_raw"; // FIXME: this->imageTopicNameP->GetValue();
  this->cameraInfoTopicName = "camera_info"; // FIXME: this->cameraInfoTopicNameP->GetValue();
  this->pollServiceName = "request_image"; // FIXME: this->pollServiceNameP->GetValue();
  this->frameName = "high_def_optical_frame"; // FIXME: this->frameNameP->GetValue();
  this->CxPrime = 320; // FIXME: this->CxPrimeP->GetValue();
  this->Cx = 320; // FIXME: this->CxP->GetValue();
  this->Cy = 240; // FIXME: this->CyP->GetValue();
  this->focal_length = 554.256; // FIXME: this->focal_lengthP->GetValue();
  this->hack_baseline = 0; // FIXME: this->hack_baselineP->GetValue();
  this->distortion_k1 = 0; // FIXME: this->distortion_k1P->GetValue();
  this->distortion_k2 = 0; // FIXME: this->distortion_k2P->GetValue();
  this->distortion_k3 = 0; // FIXME: this->distortion_k3P->GetValue();
  this->distortion_t1 = 0; // FIXME: this->distortion_t1P->GetValue();
  this->distortion_t2 = 0; // FIXME: this->distortion_t2P->GetValue();
  if ((this->distortion_k1 != 0.0) || (this->distortion_k2 != 0.0) ||
      (this->distortion_k3 != 0.0) || (this->distortion_t1 != 0.0) ||
      (this->distortion_t2 != 0.0))
    ROS_WARN("gazebo_ros_prosilica simulation does not support non-zero distortion parameters right now, your simulation maybe wrong.");

  // camera mode for prosilica:
  // prosilica::AcquisitionMode mode_; /// @todo Make this property of Camera
  std::string mode_param_name;

  //ROS_ERROR("before trigger_mode %s %s",mode_param_name.c_str(),this->mode_.c_str());

  if (!this->rosnode_->searchParam("trigger_mode",mode_param_name)) ///\@todo: hardcoded per prosilica_camera wiki api, make this an urdf parameter
      mode_param_name = "trigger_mode";

  if (!this->rosnode_->getParam(mode_param_name,this->mode_))
      this->mode_ = "streaming";

  ROS_INFO("trigger_mode %s %s",mode_param_name.c_str(),this->mode_.c_str());


  if (this->mode_ == "polled")
  {
      poll_srv_ = polled_camera::advertise(*this->rosnode_,this->pollServiceName,&GazeboRosProsilica::pollCallback,this);
  }
  else if (this->mode_ == "streaming")
  {
      ROS_DEBUG("do nothing here,mode: %s",this->mode_.c_str());
  }
  else
  {
      ROS_ERROR("trigger_mode is invalid: %s, using streaming mode",this->mode_.c_str());
  }

  /// advertise topics for image and camera info
  this->image_pub_ = this->itnode_->advertise(
    this->imageTopicName,1,
    boost::bind( &GazeboRosProsilica::ImageConnect,this),
    boost::bind( &GazeboRosProsilica::ImageDisconnect,this), ros::VoidPtr(), &this->prosilica_queue_);

  ros::AdvertiseOptions camera_info_ao = ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    this->cameraInfoTopicName,1,
    boost::bind( &GazeboRosProsilica::InfoConnect,this),
    boost::bind( &GazeboRosProsilica::InfoDisconnect,this), ros::VoidPtr(), &this->prosilica_queue_);
  this->camera_info_pub_ = this->rosnode_->advertise(camera_info_ao);

#ifdef SIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE
  if (!this->cameraName.empty()) {
    dyn_srv_ = new dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>(*this->rosnode_);
    dynamic_reconfigure::Server<gazebo_plugins::GazeboRosCameraConfig>::CallbackType f = boost::bind(&GazeboRosProsilica::configCallback, this, _1, _2);
    dyn_srv_->setCallback(f);
  }
#endif

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosProsilica::UpdateChild, this));

}

#ifdef SIMULATOR_GAZEBO_GAZEBO_ROS_CAMERA_DYNAMIC_RECONFIGURE
////////////////////////////////////////////////////////////////////////////////
// Dynamic Reconfigure Callback
void GazeboRosProsilica::configCallback(gazebo_plugins::GazeboRosCameraConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request for the gazebo ros camera: %s. New rate: %.2f", this->cameraName.c_str(), config.imager_rate);

  gzerr << "SetUpdateRate for new camera has not been implemented yet\n";
  //(dynamic_cast<OgreCamera*>(this->parentSensor))->SetUpdateRate(update_rate->data);
}
#endif

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosProsilica::InitChild()
{
  // sensor generation off by default
  this->parentSensor->SetActive(false);

  // set buffer size
  this->width            = this->parentCameraSensor->GetCamera()->GetImageWidth();
  this->height           = this->parentCameraSensor->GetCamera()->GetImageHeight();
  this->depth            = this->parentCameraSensor->GetCamera()->GetImageDepth();
  //ROS_INFO("image format in urdf is %s\n",this->parentCameraSensor->GetCamera()->GetImageFormat().c_str());
  if (this->parentCameraSensor->GetCamera()->GetImageFormat() == "L8")
  {
    this->type = sensor_msgs::image_encodings::MONO8;
    this->skip = 1;
  }
  else if (this->parentCameraSensor->GetCamera()->GetImageFormat() == "R8G8B8")
  {
    this->type = sensor_msgs::image_encodings::RGB8;
    this->skip = 3;
  }
  else if (this->parentCameraSensor->GetCamera()->GetImageFormat() == "B8G8R8")
  {
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }
  else if (this->parentCameraSensor->GetCamera()->GetImageFormat() == "BAYER_RGGB8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip = 1;
  }
  else if (this->parentCameraSensor->GetCamera()->GetImageFormat() == "BAYER_BGGR8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip = 1;
  }
  else if (this->parentCameraSensor->GetCamera()->GetImageFormat() == "BAYER_GBRG8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip = 1;
  }
  else if (this->parentCameraSensor->GetCamera()->GetImageFormat() == "BAYER_GRBG8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_GRBG8;
    this->skip = 1;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat for Prosilica, using BGR8\n");
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }

  /// Compute camera parameters if set to 0
  if (this->CxPrime == 0)
    this->CxPrime = ((double)this->width+1.0) /2.0;
  if (this->Cx == 0)
    this->Cx = ((double)this->width+1.0) /2.0;
  if (this->Cy == 0)
    this->Cy = ((double)this->height+1.0) /2.0;
  if (this->focal_length == 0)
    this->focal_length = ((double)this->width) / (2.0 *tan(this->parentCameraSensor->GetCamera()->GetHFOV().GetAsRadian()/2.0));

#ifdef USE_CBQ
  // start custom queue for prosilica
  this->prosilica_callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosProsilica::ProsilicaQueueThread,this ) );
#else
  // start ros spinner as it is done in prosilica node
  this->ros_spinner_thread_ = boost::thread( boost::bind( &GazeboRosProsilica::ProsilicaROSThread,this ) );
#endif

}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosProsilica::ImageConnect()
{
  this->imageConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosProsilica::ImageDisconnect()
{
  this->imageConnectCount--;

  if ((this->infoConnectCount == 0) && (this->imageConnectCount == 0))
    this->parentSensor->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosProsilica::InfoConnect()
{
  this->infoConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosProsilica::InfoDisconnect()
{
  this->infoConnectCount--;

  if ((this->infoConnectCount == 0) && (this->imageConnectCount == 0))
    this->parentSensor->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosProsilica::UpdateChild()
{

  // should do nothing except turning camera on/off, as we are using service.
  /// @todo: consider adding thumbnailing feature here if subscribed.

  // as long as ros is connected, parent is active
  //ROS_ERROR("debug image count %d",this->imageConnectCount);
  if (!this->parentSensor->IsActive())
  {
    if (this->imageConnectCount > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    // publish if in continuous mode, otherwise triggered by poll
    if (this->mode_ == "streaming")
      this->PutCameraData();
  }

  /// publish CameraInfo if in continuous mode, otherwise triggered by poll
  if (this->infoConnectCount > 0)
    if (this->mode_ == "streaming")
      this->PublishCameraInfo();
}
////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosProsilica::PutCameraData()
{
  const unsigned char *src;

  //boost::recursive_mutex::scoped_lock mr_lock(*this->world->GetMRMutex());

  // Get a pointer to image data
  src = this->parentCameraSensor->GetCamera()->GetImageData(0);

  if (src)
  {
    //double tmpT0 = this->world->GetWallTime();

    //unsigned char dst[this->width*this->height];

    this->lock.lock();
    // copy data into image
    this->imageMsg.header.frame_id = this->frameName;
    this->imageMsg.header.stamp.fromSec(this->world->GetSimTime().Double());

    //double tmpT1 = this->world->GetWallTime();
    //double tmpT2;

    /// @todo: don't bother if there are no subscribers
    if (this->image_pub_.getNumSubscribers() > 0)
    {

      //ROS_ERROR("debug %d %d %d %d", this->type, this->height, this->width, this->skip);

      // copy from src to imageMsg
      fillImage(this->imageMsg,
                this->type,
                this->height,
                this->width,
                this->skip*this->width,
                (void*)src );

      //tmpT2 = this->world->GetWallTime();

      // publish to ros
      this->image_pub_.publish(this->imageMsg);
    }

    //double tmpT3 = this->world->GetWallTime();

    this->lock.unlock();
  }

}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosProsilica::PublishCameraInfo()
{
  // fill CameraInfo
  this->cameraInfoMsg.header.frame_id = this->frameName;
  this->cameraInfoMsg.header.stamp.fromSec(this->world->GetSimTime().Double());
  this->cameraInfoMsg.height = this->height;
  this->cameraInfoMsg.width  = this->width;

  // distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
  this->cameraInfoMsg.distortion_model = "plumb_bob";
  this->cameraInfoMsg.D.resize(5);
#endif
  this->cameraInfoMsg.D[0] = this->distortion_k1;
  this->cameraInfoMsg.D[1] = this->distortion_k2;
  this->cameraInfoMsg.D[2] = this->distortion_k3;
  this->cameraInfoMsg.D[3] = this->distortion_t1;
  this->cameraInfoMsg.D[4] = this->distortion_t2;
  // original camera matrix
  this->cameraInfoMsg.K[0] = this->focal_length;
  this->cameraInfoMsg.K[1] = 0.0;
  this->cameraInfoMsg.K[2] = this->Cx;
  this->cameraInfoMsg.K[3] = 0.0;
  this->cameraInfoMsg.K[4] = this->focal_length;
  this->cameraInfoMsg.K[5] = this->Cy;
  this->cameraInfoMsg.K[6] = 0.0;
  this->cameraInfoMsg.K[7] = 0.0;
  this->cameraInfoMsg.K[8] = 1.0;
  // rectification
  this->cameraInfoMsg.R[0] = 1.0;
  this->cameraInfoMsg.R[1] = 0.0;
  this->cameraInfoMsg.R[2] = 0.0;
  this->cameraInfoMsg.R[3] = 0.0;
  this->cameraInfoMsg.R[4] = 1.0;
  this->cameraInfoMsg.R[5] = 0.0;
  this->cameraInfoMsg.R[6] = 0.0;
  this->cameraInfoMsg.R[7] = 0.0;
  this->cameraInfoMsg.R[8] = 1.0;
  // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
  this->cameraInfoMsg.P[0] = this->focal_length;
  this->cameraInfoMsg.P[1] = 0.0;
  this->cameraInfoMsg.P[2] = this->Cx;
  this->cameraInfoMsg.P[3] = -this->focal_length * this->hack_baseline;
  this->cameraInfoMsg.P[4] = 0.0;
  this->cameraInfoMsg.P[5] = this->focal_length;
  this->cameraInfoMsg.P[6] = this->Cy;
  this->cameraInfoMsg.P[7] = 0.0;
  this->cameraInfoMsg.P[8] = 0.0;
  this->cameraInfoMsg.P[9] = 0.0;
  this->cameraInfoMsg.P[10] = 1.0;
  this->cameraInfoMsg.P[11] = 0.0;
  this->camera_info_pub_.publish(this->cameraInfoMsg);
}

////////////////////////////////////////////////////////////////////////////////
// new prosilica interface.
void GazeboRosProsilica::pollCallback(polled_camera::GetPolledImage::Request& req,
                                      polled_camera::GetPolledImage::Response& rsp,
                                      sensor_msgs::Image& image, sensor_msgs::CameraInfo& info)
{
  /// @todo Support binning (maybe just cv::resize)
  /// @todo Don't adjust K, P for ROI, set CameraInfo.roi fields instead
  /// @todo D parameter order is k1, k2, t1, t2, k3

  if (this->mode_ != "polled")
  {
    rsp.success = false;
    rsp.status_message = "Camera is not in triggered mode";
    return;
  }

  if (req.binning_x > 1 || req.binning_y > 1)
  {
    rsp.success = false;
    rsp.status_message = "Gazebo Prosilica plugin does not support binning";
    return;
  }

/*
  // fill out the cam info part
  info.header.frame_id = this->frameName;
  info.header.stamp.fromSec(this->world->GetSimTime().Double());
  info.height = this->parentCameraSensor->GetCamera()->GetImageHeight();
  info.width  = this->parentCameraSensor->GetCamera()->GetImageWidth() ;
  // distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
  info.distortion_model = "plumb_bob";
  info.D.resize(5);
#endif
  info.D[0] = this->distortion_k1;
  info.D[1] = this->distortion_k2;
  info.D[2] = this->distortion_k3;
  info.D[3] = this->distortion_t1;
  info.D[4] = this->distortion_t2;
  // original camera matrix
  info.K[0] = this->focal_length;
  info.K[1] = 0.0;
  info.K[2] = this->Cx;
  info.K[3] = 0.0;
  info.K[4] = this->focal_length;
  info.K[5] = this->Cy;
  info.K[6] = 0.0;
  info.K[7] = 0.0;
  info.K[8] = 1.0;
  // rectification
  info.R[0] = 1.0;
  info.R[1] = 0.0;
  info.R[2] = 0.0;
  info.R[3] = 0.0;
  info.R[4] = 1.0;
  info.R[5] = 0.0;
  info.R[6] = 0.0;
  info.R[7] = 0.0;
  info.R[8] = 1.0;
  // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
  info.P[0] = this->focal_length;
  info.P[1] = 0.0;
  info.P[2] = this->Cx;
  info.P[3] = -this->focal_length * this->hack_baseline;
  info.P[4] = 0.0;
  info.P[5] = this->focal_length;
  info.P[6] = this->Cy;
  info.P[7] = 0.0;
  info.P[8] = 0.0;
  info.P[9] = 0.0;
  info.P[10] = 1.0;
  info.P[11] = 0.0;
*/

  // get region from request
  if (req.roi.x_offset <= 0 || req.roi.y_offset <= 0 || req.roi.width <= 0 || req.roi.height <= 0)
  {
    req.roi.x_offset = 0;
    req.roi.y_offset = 0;
    req.roi.width = this->width;
    req.roi.height = this->height;
  }
  const unsigned char *src = NULL;
  ROS_ERROR("roidebug %d %d %d %d", req.roi.x_offset, req.roi.y_offset, req.roi.width, req.roi.height);

  // signal sensor to start update
  this->ImageConnect();
  // wait until an image has been returned
  while(!src)
  {
    {
      // boost::recursive_mutex::scoped_lock lock(*this->world->GetMRMutex());  // FIXME: no more mutex?
      // Get a pointer to image data
      src = this->parentCameraSensor->GetCamera()->GetImageData(0);

      if (src)
      {

        // fill CameraInfo
        this->roiCameraInfoMsg = &info;
        this->roiCameraInfoMsg->header.frame_id = this->frameName;

        // FIXME: this->roiCameraInfoMsg->header.stamp.fromSec( (dynamic_cast<OgreCamera*>(this->parentSensor))->GetLastRenderTime().Double());
        common::Time roiLastRenderTime;
        this->roiCameraInfoMsg->header.stamp.sec = roiLastRenderTime.sec;
        this->roiCameraInfoMsg->header.stamp.nsec = roiLastRenderTime.nsec;

        this->roiCameraInfoMsg->width  = req.roi.width; //this->parentSensor->GetImageWidth() ;
        this->roiCameraInfoMsg->height = req.roi.height; //this->parentSensor->GetImageHeight();
        // distortion
#if ROS_VERSION_MINIMUM(1, 3, 0)
        this->roiCameraInfoMsg->distortion_model = "plumb_bob";
        this->roiCameraInfoMsg->D.resize(5);
#endif
        this->roiCameraInfoMsg->D[0] = this->distortion_k1;
        this->roiCameraInfoMsg->D[1] = this->distortion_k2;
        this->roiCameraInfoMsg->D[2] = this->distortion_k3;
        this->roiCameraInfoMsg->D[3] = this->distortion_t1;
        this->roiCameraInfoMsg->D[4] = this->distortion_t2;
        // original camera matrix
        this->roiCameraInfoMsg->K[0] = this->focal_length;
        this->roiCameraInfoMsg->K[1] = 0.0;
        this->roiCameraInfoMsg->K[2] = this->Cx - req.roi.x_offset;
        this->roiCameraInfoMsg->K[3] = 0.0;
        this->roiCameraInfoMsg->K[4] = this->focal_length;
        this->roiCameraInfoMsg->K[5] = this->Cy - req.roi.y_offset;
        this->roiCameraInfoMsg->K[6] = 0.0;
        this->roiCameraInfoMsg->K[7] = 0.0;
        this->roiCameraInfoMsg->K[8] = 1.0;
        // rectification
        this->roiCameraInfoMsg->R[0] = 1.0;
        this->roiCameraInfoMsg->R[1] = 0.0;
        this->roiCameraInfoMsg->R[2] = 0.0;
        this->roiCameraInfoMsg->R[3] = 0.0;
        this->roiCameraInfoMsg->R[4] = 1.0;
        this->roiCameraInfoMsg->R[5] = 0.0;
        this->roiCameraInfoMsg->R[6] = 0.0;
        this->roiCameraInfoMsg->R[7] = 0.0;
        this->roiCameraInfoMsg->R[8] = 1.0;
        // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
        this->roiCameraInfoMsg->P[0] = this->focal_length;
        this->roiCameraInfoMsg->P[1] = 0.0;
        this->roiCameraInfoMsg->P[2] = this->Cx - req.roi.x_offset;
        this->roiCameraInfoMsg->P[3] = -this->focal_length * this->hack_baseline;
        this->roiCameraInfoMsg->P[4] = 0.0;
        this->roiCameraInfoMsg->P[5] = this->focal_length;
        this->roiCameraInfoMsg->P[6] = this->Cy - req.roi.y_offset;
        this->roiCameraInfoMsg->P[7] = 0.0;
        this->roiCameraInfoMsg->P[8] = 0.0;
        this->roiCameraInfoMsg->P[9] = 0.0;
        this->roiCameraInfoMsg->P[10] = 1.0;
        this->roiCameraInfoMsg->P[11] = 0.0;
        this->camera_info_pub_.publish(*this->roiCameraInfoMsg);

        // copy data into imageMsg, then convert to roiImageMsg(image)
        this->imageMsg.header.frame_id    = this->frameName;

        // FIXME: this->imageMsg.header.stamp.fromSec( (dynamic_cast<OgreCamera*>(this->parentSensor))->GetLastRenderTime().Double());
        common::Time lastRenderTime;
        this->imageMsg.header.stamp.sec = lastRenderTime.sec;
        this->imageMsg.header.stamp.nsec = lastRenderTime.nsec;

        //unsigned char dst[this->width*this->height];

        /// @todo: don't bother if there are no subscribers

        // copy from src to imageMsg
        fillImage(this->imageMsg,
                  this->type,
                  this->height,
                  this->width,
                  this->skip*this->width,
                  (void*)src );

        /// @todo: publish to ros, thumbnails and rect image in the Update call?

        this->image_pub_.publish(this->imageMsg);

        {
          // copy data into ROI image
          this->roiImageMsg = &image;
          this->roiImageMsg->header.frame_id = this->frameName;
          // FIXME: this->roiImageMsg->header.stamp.fromSec( (dynamic_cast<OgreCamera*>(this->parentSensor))->GetLastRenderTime().Double());
          common::Time roiLastRenderTime;
          this->roiImageMsg->header.stamp.sec = roiLastRenderTime.sec;
          this->roiImageMsg->header.stamp.nsec = roiLastRenderTime.nsec;

          //sensor_msgs::CvBridge img_bridge_(&this->imageMsg);
          //IplImage* cv_image;
          //img_bridge_.to_cv( &cv_image );

          sensor_msgs::CvBridge img_bridge_;
          img_bridge_.fromImage(this->imageMsg);

          //cvNamedWindow("showme",CV_WINDOW_AUTOSIZE);
          //cvSetMouseCallback("showme", &GazeboRosProsilica::mouse_cb, this);
          //cvStartWindowThread();

          //cvShowImage("showme",img_bridge_.toIpl());
          cvSetImageROI(img_bridge_.toIpl(),cvRect(req.roi.x_offset,req.roi.y_offset,req.roi.width,req.roi.height));
          IplImage *roi = cvCreateImage(cvSize(req.roi.width,req.roi.height),
                                       img_bridge_.toIpl()->depth,
                                       img_bridge_.toIpl()->nChannels);
          cvCopy(img_bridge_.toIpl(),roi);

          img_bridge_.fromIpltoRosImage(roi,*this->roiImageMsg);

          cvReleaseImage(&roi);
        }
      }
    }
    usleep(100000);
  }
  this->ImageDisconnect();
  rsp.success = true;
  return;
}


#ifdef USE_CBQ
////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosProsilica::ProsilicaQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->prosilica_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#else
void GazeboRosProsilica::ProsilicaROSThread()
{
  ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());

  ros::Rate rate(1000);

  while (this->rosnode_->ok())
  {
    //rate.sleep(); // using rosrate gets stuck on model delete
    usleep(1000);
    ros::spinOnce();
  }
}
#endif

/*
void GazeboRosProsilica::OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
{
  this->simTime  = msgs::Convert( _msg->sim_time() );

  math::Pose pose;
  pose.pos.x = 0.5*sin(0.01*this->simTime.Double());
  gzdbg << "plugin simTime [" << this->simTime.Double() << "] update pose [" << pose.pos.x << "]\n";
}
*/

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosProsilica)


}
