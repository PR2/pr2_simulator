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

#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include "gazebo/MonoCameraSensor.hh"


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

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_prosilica", GazeboRosProsilica);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosProsilica::GazeboRosProsilica(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<MonoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosProsilica controller requires a Camera Sensor as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->imageTopicNameP = new ParamT<std::string>("imageTopicName","image_raw", 0);
  this->cameraInfoTopicNameP = new ParamT<std::string>("cameraInfoTopicName","camera_info", 0);
  this->pollServiceNameP = new ParamT<std::string>("pollServiceName","request_image", 0);
  this->frameNameP = new ParamT<std::string>("frameName","camera", 0);
  // camera parameters 
  this->CxPrimeP = new ParamT<double>("CxPrime",320, 0); // for 640x480 image
  this->CxP  = new ParamT<double>("Cx" ,320, 0); // for 640x480 image
  this->CyP  = new ParamT<double>("Cy" ,240, 0); // for 640x480 image
  this->focal_lengthP  = new ParamT<double>("focal_length" ,554.256, 0); // == image_width(px) / (2*tan( hfov(radian) /2))
  this->hack_baselineP  = new ParamT<double>("hackBaseline" ,0, 0); // hack for right stereo camera
  this->distortion_k1P  = new ParamT<double>("distortion_k1" ,0, 0);
  this->distortion_k2P  = new ParamT<double>("distortion_k2" ,0, 0);
  this->distortion_k3P  = new ParamT<double>("distortion_k3" ,0, 0);
  this->distortion_t1P  = new ParamT<double>("distortion_t1" ,0, 0);
  this->distortion_t2P  = new ParamT<double>("distortion_t2" ,0, 0);
  Param::End();

  this->imageConnectCount = 0;
  this->infoConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosProsilica::~GazeboRosProsilica()
{
  delete this->robotNamespaceP;
  delete this->rosnode_;
  delete this->imageTopicNameP;
  delete this->cameraInfoTopicNameP;
  delete this->pollServiceNameP;
  delete this->frameNameP;
  delete this->CxPrimeP;
  delete this->CxP;
  delete this->CyP;
  delete this->focal_lengthP;
  delete this->hack_baselineP;
  delete this->distortion_k1P;
  delete this->distortion_k2P;
  delete this->distortion_k3P;
  delete this->distortion_t1P;
  delete this->distortion_t2P;
#ifdef USE_CBQ
  delete this->prosilica_callback_queue_thread_;
#else
  delete this->ros_spinner_thread_;
#endif

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosProsilica::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);
  this->rosnode_->setCallbackQueue(&this->prosilica_queue_);

  this->imageTopicNameP->Load(node);
  this->cameraInfoTopicNameP->Load(node);
  this->pollServiceNameP->Load(node);
  this->frameNameP->Load(node);
  this->CxPrimeP->Load(node);
  this->CxP->Load(node);
  this->CyP->Load(node);
  this->focal_lengthP->Load(node);
  this->hack_baselineP->Load(node);
  this->distortion_k1P->Load(node);
  this->distortion_k2P->Load(node);
  this->distortion_k3P->Load(node);
  this->distortion_t1P->Load(node);
  this->distortion_t2P->Load(node);
  
  this->imageTopicName = this->imageTopicNameP->GetValue();
  this->cameraInfoTopicName = this->cameraInfoTopicNameP->GetValue();
  this->pollServiceName = this->pollServiceNameP->GetValue();
  this->frameName = this->frameNameP->GetValue();
  this->CxPrime = this->CxPrimeP->GetValue();
  this->Cx = this->CxP->GetValue();
  this->Cy = this->CyP->GetValue();
  this->focal_length = this->focal_lengthP->GetValue();
  this->hack_baseline = this->hack_baselineP->GetValue();
  this->distortion_k1 = this->distortion_k1P->GetValue();
  this->distortion_k2 = this->distortion_k2P->GetValue();
  this->distortion_k3 = this->distortion_k3P->GetValue();
  this->distortion_t1 = this->distortion_t1P->GetValue();
  this->distortion_t2 = this->distortion_t2P->GetValue();

  // camera mode for prosilica:
  // prosilica::AcquisitionMode mode_; /// @todo Make this property of Camera
  std::string mode_param_name;

  //ROS_ERROR("before trigger_mode %s %s",mode_param_name.c_str(),this->mode_.c_str());

  if (this->rosnode_->searchParam("trigger_mode",mode_param_name)) ///\@todo: hardcoded per prosilica_camera wiki api, make this an urdf parameter
  {
      this->rosnode_->getParam(mode_param_name,this->mode_);
  }
  else
  {
      ROS_DEBUG("defaults to streaming");
      this->mode_ = "streaming";
  }

  //this->rosnode_->getParam(mode_param_name,this->mode_);
  //ROS_ERROR("trigger_mode %s %s",mode_param_name.c_str(),this->mode_.c_str());

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
  ros::AdvertiseOptions image_ao = ros::AdvertiseOptions::create<sensor_msgs::Image>(
    this->imageTopicName,1,
    boost::bind( &GazeboRosProsilica::ImageConnect,this),
    boost::bind( &GazeboRosProsilica::ImageDisconnect,this), ros::VoidPtr(), &this->prosilica_queue_);
  this->image_pub_ = this->rosnode_->advertise(image_ao);

  ros::AdvertiseOptions camera_info_ao = ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
    this->cameraInfoTopicName,1,
    boost::bind( &GazeboRosProsilica::InfoConnect,this),
    boost::bind( &GazeboRosProsilica::InfoDisconnect,this), ros::VoidPtr(), &this->prosilica_queue_);
  this->camera_info_pub_ = this->rosnode_->advertise(camera_info_ao);

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosProsilica::InitChild()
{
  // sensor generation off by default
  this->myParent->SetActive(false);

  // set buffer size
  this->width            = this->myParent->GetImageWidth();
  this->height           = this->myParent->GetImageHeight();
  this->depth            = this->myParent->GetImageDepth();
  //ROS_INFO("image format in urdf is %s\n",this->myParent->GetImageFormat().c_str());
  if (this->myParent->GetImageFormat() == "L8")
  {
    this->type = sensor_msgs::image_encodings::MONO8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "R8G8B8")
  {
    this->type = sensor_msgs::image_encodings::RGB8;
    this->skip = 3;
  }
  else if (this->myParent->GetImageFormat() == "B8G8R8")
  {
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_RGGB8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_RGGB8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_BGGR8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_BGGR8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_GBRG8")
  {
    this->type = sensor_msgs::image_encodings::BAYER_GBRG8;
    this->skip = 1;
  }
  else if (this->myParent->GetImageFormat() == "BAYER_GRBG8")
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
    this->focal_length = ((double)this->width) / (2.0 *tan(this->myParent->GetHFOV().GetAsRadian()/2.0));

#ifdef USE_CBQ
  // start custom queue for prosilica
  this->prosilica_callback_queue_thread_ = new boost::thread( boost::bind( &GazeboRosProsilica::ProsilicaQueueThread,this ) );
#else
  // start ros spinner as it is done in prosilica node
  this->ros_spinner_thread_ = new boost::thread( boost::bind( &ros::spin ) );
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
    this->myParent->SetActive(false);
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
    this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosProsilica::UpdateChild()
{

  // should do nothing except turning camera on/off, as we are using service.
  /// @todo: consider adding thumbnailing feature here if subscribed.

  // as long as ros is connected, parent is active
  //ROS_ERROR("debug image count %d",this->imageConnectCount);
  if (!this->myParent->IsActive())
  {
    if (this->imageConnectCount > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->myParent->SetActive(true);
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

  //boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());

  // Get a pointer to image data
  src = this->myParent->GetImageData(0);

  if (src)
  {
    //double tmpT0 = Simulator::Instance()->GetWallTime();

    unsigned char dst[this->width*this->height];

    this->lock.lock();
    // copy data into image
    this->imageMsg.header.frame_id = this->frameName;
    this->imageMsg.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
    this->imageMsg.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

    //double tmpT1 = Simulator::Instance()->GetWallTime();
    //double tmpT2;

    /// @todo: don't bother if there are no subscribers
    if (this->image_pub_.getNumSubscribers() > 0)
    {

      // do last minute conversion if Bayer pattern is requested, go from R8G8B8
      if (this->myParent->GetImageFormat() == "BAYER_RGGB8")
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // RG
            // GB
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
            else // odd column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_BGGR8")
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // BG
            // GR
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
            else // odd column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_GBRG8")
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // GB
            // RG
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
            else // odd column
              if (i%2) // even row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
          }
        }
        src=dst;
      }
      else if (this->myParent->GetImageFormat() == "BAYER_GRBG8")
      {
        for (int i=0;i<this->width;i++)
        {
          for (int j=0;j<this->height;j++)
          {
            //
            // GR
            // BG
            //
            // determine position
            if (j%2) // even column
              if (i%2) // even row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd row, red
                dst[i+j*this->width] = src[i*3+j*this->width*3+0];
            else // odd column
              if (i%2) // even row, blue
                dst[i+j*this->width] = src[i*3+j*this->width*3+2];
              else // odd row, green
                dst[i+j*this->width] = src[i*3+j*this->width*3+1];
          }
        }
        src=dst;
      }

      //ROS_ERROR("debug %d %d %d %d", this->type, this->height, this->width, this->skip);

      // copy from src to imageMsg
      fillImage(this->imageMsg,
                this->type,
                this->height,
                this->width,
                this->skip*this->width,
                (void*)src );

      //tmpT2 = Simulator::Instance()->GetWallTime();

      // publish to ros
      this->image_pub_.publish(this->imageMsg);
    }

    //double tmpT3 = Simulator::Instance()->GetWallTime();

    this->lock.unlock();
  }

}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosProsilica::PublishCameraInfo()
{
  // fill CameraInfo
  this->cameraInfoMsg.header.frame_id = this->frameName;
  this->cameraInfoMsg.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  this->cameraInfoMsg.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;
  this->cameraInfoMsg.height = this->height;
  this->cameraInfoMsg.width  = this->width;

  // distortion
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
bool GazeboRosProsilica::pollCallback(polled_camera::GetPolledImage::Request& req,
                                      sensor_msgs::Image& image, sensor_msgs::CameraInfo& info)
{

  if (this->mode_ != "polled")
  {
    ROS_ERROR("Poll service called but camera is not in triggered mode");
    return false;
  }

/*
  // fill out the cam info part
  info.header.frame_id = this->frameName;
  info.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  info.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;
  info.height = this->myParent->GetImageHeight();
  info.width  = this->myParent->GetImageWidth() ;
  // distortion
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
      boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
      // Get a pointer to image data
      src = this->myParent->GetImageData(0);

      if (src)
      {

        // fill CameraInfo
        this->roiCameraInfoMsg = &info;
        this->roiCameraInfoMsg->header.frame_id = this->frameName;
        this->roiCameraInfoMsg->header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
        this->roiCameraInfoMsg->header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;
        this->roiCameraInfoMsg->width  = req.roi.width; //this->myParent->GetImageWidth() ;
        this->roiCameraInfoMsg->height = req.roi.height; //this->myParent->GetImageHeight();
        // distortion
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
        this->imageMsg.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
        this->imageMsg.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

        unsigned char dst[this->width*this->height];

        /// @todo: don't bother if there are no subscribers

        // do last minute conversion if Bayer pattern is requested, go from R8G8B8
        if (this->myParent->GetImageFormat() == "BAYER_RGGB8")
        {
          for (int i=0;i<this->width;i++)
          {
            for (int j=0;j<this->height;j++)
            {
              //
              // RG
              // GB
              //
              // determine position
              if (j%2) // even column
                if (i%2) // even row, red
                  dst[i+j*this->width] = src[i*3+j*this->width*3+0];
                else // odd row, green
                  dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd column
                if (i%2) // even row, green
                  dst[i+j*this->width] = src[i*3+j*this->width*3+1];
                else // odd row, blue
                  dst[i+j*this->width] = src[i*3+j*this->width*3+2];
            }
          }
          src=dst;
        }
        else if (this->myParent->GetImageFormat() == "BAYER_BGGR8")
        {
          for (int i=0;i<this->width;i++)
          {
            for (int j=0;j<this->height;j++)
            {
              //
              // BG
              // GR
              //
              // determine position
              if (j%2) // even column
                if (i%2) // even row, blue
                  dst[i+j*this->width] = src[i*3+j*this->width*3+2];
                else // odd row, green
                  dst[i+j*this->width] = src[i*3+j*this->width*3+1];
              else // odd column
                if (i%2) // even row, green
                  dst[i+j*this->width] = src[i*3+j*this->width*3+1];
                else // odd row, red
                  dst[i+j*this->width] = src[i*3+j*this->width*3+0];
            }
          }
          src=dst;
        }
        else if (this->myParent->GetImageFormat() == "BAYER_GBRG8")
        {
          for (int i=0;i<this->width;i++)
          {
            for (int j=0;j<this->height;j++)
            {
              //
              // GB
              // RG
              //
              // determine position
              if (j%2) // even column
                if (i%2) // even row, green
                  dst[i+j*this->width] = src[i*3+j*this->width*3+1];
                else // odd row, blue
                  dst[i+j*this->width] = src[i*3+j*this->width*3+2];
              else // odd column
                if (i%2) // even row, red
                  dst[i+j*this->width] = src[i*3+j*this->width*3+0];
                else // odd row, green
                  dst[i+j*this->width] = src[i*3+j*this->width*3+1];
            }
          }
          src=dst;
        }
        else if (this->myParent->GetImageFormat() == "BAYER_GRBG8")
        {
          for (int i=0;i<this->width;i++)
          {
            for (int j=0;j<this->height;j++)
            {
              //
              // GR
              // BG
              //
              // determine position
              if (j%2) // even column
                if (i%2) // even row, green
                  dst[i+j*this->width] = src[i*3+j*this->width*3+1];
                else // odd row, red
                  dst[i+j*this->width] = src[i*3+j*this->width*3+0];
              else // odd column
                if (i%2) // even row, blue
                  dst[i+j*this->width] = src[i*3+j*this->width*3+2];
                else // odd row, green
                  dst[i+j*this->width] = src[i*3+j*this->width*3+1];
            }
          }
          src=dst;
        }

        // copy from src to imageMsg
        fillImage(this->imageMsg,
                  this->type,
                  this->height,
                  this->width,
                  this->skip*this->width,
                  (void*)src );

        /// @todo: publish to ros, thumbnails and rect image in the Update call?

        this->image_pub_.publish(this->imageMsg);

        if ((this->myParent->GetImageFormat() == "BAYER_RGGB8") ||
            (this->myParent->GetImageFormat() == "BAYER_BGGR8") ||
            (this->myParent->GetImageFormat() == "BAYER_GBRG8") ||
            (this->myParent->GetImageFormat() == "BAYER_GRBG8") )
        {
          ROS_ERROR("prosilica does not support bayer roi, using full image");

          // copy from src to imageMsg
          fillImage(image,
                    this->type,
                    this->height,
                    this->width,
                    this->skip*this->width,
                    (void*)src );
        }
        else
        {
          // copy data into ROI image
          this->roiImageMsg = &image;
          this->roiImageMsg->header.frame_id = this->frameName;
          this->roiImageMsg->header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
          this->roiImageMsg->header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

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
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosProsilica::FiniChild()
{
  this->myParent->SetActive(false);
#ifdef USE_CBQ
  this->prosilica_queue_.clear();
  this->prosilica_queue_.disable();
  ros::requestShutdown();
  this->prosilica_callback_queue_thread_->join();
#else
  this->ros_spinner_thread_->join();
#endif

  this->poll_srv_.shutdown();
  this->image_pub_.shutdown();
  this->camera_info_pub_.shutdown();

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
#endif


}
