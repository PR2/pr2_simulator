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
   Desc: GazeboRosCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b GazeboRosCamera plugin broadcasts ROS Image messages
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <pr2_gazebo_plugins/gazebo_ros_camera.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/MonoCameraSensor.hh>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"
using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_camera", GazeboRosCamera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosCamera::GazeboRosCamera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<MonoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("GazeboRosCamera controller requires a Camera Sensor as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace","/",0);
  this->imageTopicNameP = new ParamT<std::string>("imageTopicName","generic_camera/image_raw", 0);
  this->cameraInfoTopicNameP = new ParamT<std::string>("cameraInfoTopicName","generic_camera/camera_info", 0);
  this->frameNameP = new ParamT<std::string>("frameName","generic_camera_link", 0);
  // camera parameters 
  this->CxPrimeP = new ParamT<double>("CxPrime",0, 0); // default to 0 for compute on the fly
  this->CxP  = new ParamT<double>("Cx" ,0, 0); // default to 0 for compute on the fly
  this->CyP  = new ParamT<double>("Cy" ,0, 0); // default to 0 for compute on the fly
  this->focal_lengthP  = new ParamT<double>("focal_length" ,0, 0); // == image_width(px) / (2*tan( hfov(radian) /2)), default to 0 for compute on the fly
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
GazeboRosCamera::~GazeboRosCamera()
{
  delete this->robotNamespaceP;
  delete this->rosnode_;
  delete this->imageTopicNameP;
  delete this->cameraInfoTopicNameP;
  delete this->frameNameP;
  delete this->CxPrimeP;
  delete this->CxP;
  delete this->CyP;
  delete this->focal_lengthP;
  delete this->distortion_k1P;
  delete this->distortion_k2P;
  delete this->distortion_k3P;
  delete this->distortion_t1P;
  delete this->distortion_t2P;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosCamera::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo",ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->imageTopicNameP->Load(node);
  this->cameraInfoTopicNameP->Load(node);
  this->frameNameP->Load(node);
  this->CxPrimeP->Load(node);
  this->CxP->Load(node);
  this->CyP->Load(node);
  this->focal_lengthP->Load(node);
  this->distortion_k1P->Load(node);
  this->distortion_k2P->Load(node);
  this->distortion_k3P->Load(node);
  this->distortion_t1P->Load(node);
  this->distortion_t2P->Load(node);
  this->imageTopicName = this->imageTopicNameP->GetValue();
  this->cameraInfoTopicName = this->cameraInfoTopicNameP->GetValue();
  this->frameName = this->frameNameP->GetValue();
  this->CxPrime = this->CxPrimeP->GetValue();
  this->Cx = this->CxP->GetValue();
  this->Cy = this->CyP->GetValue();
  this->focal_length = this->focal_lengthP->GetValue();
  this->distortion_k1 = this->distortion_k1P->GetValue();
  this->distortion_k2 = this->distortion_k2P->GetValue();
  this->distortion_k3 = this->distortion_k3P->GetValue();
  this->distortion_t1 = this->distortion_t1P->GetValue();
  this->distortion_t2 = this->distortion_t2P->GetValue();

  this->image_pub_ = this->rosnode_->advertise<sensor_msgs::Image>(this->imageTopicName,1,
    boost::bind( &GazeboRosCamera::ImageConnect, this),
    boost::bind( &GazeboRosCamera::ImageDisconnect, this));
  this->camera_info_pub_ = this->rosnode_->advertise<sensor_msgs::CameraInfo>(this->cameraInfoTopicName,1,
    boost::bind( &GazeboRosCamera::InfoConnect, this),
    boost::bind( &GazeboRosCamera::InfoDisconnect, this));
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCamera::InfoConnect()
{
  this->infoConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCamera::InfoDisconnect()
{
  this->infoConnectCount--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosCamera::ImageConnect()
{
  this->imageConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosCamera::ImageDisconnect()
{
  this->imageConnectCount--;

  if (this->imageConnectCount == 0)
    this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosCamera::InitChild()
{
  // sensor generation off by default
  this->myParent->SetActive(false);

  // set buffer size
  this->width            = this->myParent->GetImageWidth();
  this->height           = this->myParent->GetImageHeight();
  this->depth            = this->myParent->GetImageDepth();
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
    //this->type = sensor_msgs::image_encodings::BGR8;
    this->type = sensor_msgs::image_encodings::RGB8; // FIXME: gazebo does not produce BGR correctly
    this->skip = 3;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
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
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosCamera::UpdateChild()
{

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
    this->PutCameraData();
  }

  /// publish CameraInfo
  if (this->infoConnectCount > 0)
    this->PublishCameraInfo();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosCamera::FiniChild()
{
  this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosCamera::PutCameraData()
{
  const unsigned char *src;

  //boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());

  // Get a pointer to image data
  src = this->myParent->GetImageData(0);

  if (src)
  {
    //double tmpT0 = Simulator::Instance()->GetWallTime();

    this->lock.lock();
    // copy data into image
    this->imageMsg.header.frame_id = this->frameName;
    this->imageMsg.header.stamp.sec = (unsigned long)floor(Simulator::Instance()->GetSimTime());
    this->imageMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  Simulator::Instance()->GetSimTime() - this->imageMsg.header.stamp.sec) );

    //double tmpT1 = Simulator::Instance()->GetWallTime();
    //double tmpT2;

    /// @todo: don't bother if there are no subscribers
    if (this->image_pub_.getNumSubscribers() > 0)
    {
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
void GazeboRosCamera::PublishCameraInfo()
{
  // fill CameraInfo
  this->cameraInfoMsg.header.frame_id = this->frameName;
  this->cameraInfoMsg.header.stamp    = ros::Time((unsigned long)floor(Simulator::Instance()->GetSimTime()));
  this->cameraInfoMsg.height = this->height,
  this->cameraInfoMsg.width  = this->width,
  // distortion
  this->cameraInfoMsg.D[0] = 0.0;
  this->cameraInfoMsg.D[1] = 0.0;
  this->cameraInfoMsg.D[2] = 0.0;
  this->cameraInfoMsg.D[3] = 0.0;
  this->cameraInfoMsg.D[4] = 0.0;
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
  this->cameraInfoMsg.P[3] = 0.0;
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
