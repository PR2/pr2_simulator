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
 * Desc: Stereo camera controller.
 * Author: Nathan Koenig
 * Date: 06 April 2008
 * SVN info: $Id: gazebo_ros_stereo_camera.cpp 4436 2008-03-24 17:42:45Z robotos $
 */

#include <algorithm>
#include <assert.h>

#include <pr2_gazebo_plugins/gazebo_ros_stereo_camera.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Model.hh>
#include <gazebo/MonoCameraSensor.hh>
#include <gazebo/Body.hh>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_stereo_camera", GazeboRosStereoCamera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosStereoCamera::GazeboRosStereoCamera(Entity *parent)
    : Controller(parent)
{
  this->myParent = parent;

  if (!dynamic_cast<Model*>(this->myParent))
    gzthrow("GazeboRosStereoCamera controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  // camera sensor names
  this->leftCameraNameP = new ParamT<std::string>("leftCamera","", 1);
  this->rightCameraNameP = new ParamT<std::string>("rightCamera","", 1);
  // raw_stereo topic name
  this->topicNameP = new ParamT<std::string>("topicName","stereo/raw_stereo", 0);
  // camera frame names
  this->frameNameP = new ParamT<std::string>("frameName","stereo_link", 0);
  // camera parameters 
  this->CxPrimeP = new ParamT<double>("CxPrime",320, 0); // for 640x480 image
  this->CxP  = new ParamT<double>("Cx" ,320, 0); // for 640x480 image
  this->CyP  = new ParamT<double>("Cy" ,240, 0); // for 640x480 image
  this->focal_lengthP  = new ParamT<double>("focal_length" ,554.256, 0); // == image_width(px) / (2*tan( hfov(radian) /2))
  this->distortion_k1P  = new ParamT<double>("distortion_k1" ,0, 0);
  this->distortion_k2P  = new ParamT<double>("distortion_k2" ,0, 0);
  this->distortion_k3P  = new ParamT<double>("distortion_k3" ,0, 0);
  this->distortion_t1P  = new ParamT<double>("distortion_t1" ,0, 0);
  this->distortion_t2P  = new ParamT<double>("distortion_t2" ,0, 0);
  this->baselineP  = new ParamT<double>("baseline" ,0.05, 0); // distance from left to right camera
  Param::End();

  // RawStereo.msg
  this->leftImageMsg  = &(this->rawStereoMsg.left_image);
  this->rightImageMsg = &(this->rawStereoMsg.right_image);
  this->leftCameraInfoMsg  = &(this->rawStereoMsg.left_info);
  this->rightCameraInfoMsg = &(this->rawStereoMsg.right_info);
  this->stereoInfoMsg = &(this->rawStereoMsg.stereo_info);
  ROS_DEBUG("stereo: done with constuctor");

  this->imageConnectCount = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosStereoCamera::~GazeboRosStereoCamera()
{
  delete this->robotNamespaceP;
  delete this->rosnode_;
  delete this->leftCameraNameP;
  delete this->rightCameraNameP;
  delete this->frameNameP;
  delete this->topicNameP;
  delete this->CxPrimeP;
  delete this->CxP;
  delete this->CyP;
  delete this->focal_lengthP;
  delete this->distortion_k1P;
  delete this->distortion_k2P;
  delete this->distortion_k3P;
  delete this->distortion_t1P;
  delete this->distortion_t2P;
  delete this->baselineP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosStereoCamera::LoadChild(XMLConfigNode *node)
{

  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo",ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->leftCameraNameP->Load(node);
  this->rightCameraNameP->Load(node);
  this->topicNameP->Load(node);
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
  this->baselineP->Load(node);

  this->leftCameraName = this->leftCameraNameP->GetValue();
  this->rightCameraName = this->rightCameraNameP->GetValue();
  this->topicName = this->topicNameP->GetValue();
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
  this->baseline = this->baselineP->GetValue();

  ROS_DEBUG("stereo: done with Loading params from XML");

}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void GazeboRosStereoCamera::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->leftCameraNameP) << "\n";
  stream << prefix << *(this->rightCameraNameP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosStereoCamera::InitChild()
{
  ROS_DEBUG("stereo: running InitChild");

  // advertise node topics
  ROS_DEBUG("stereo: advertise topicName %s\n",this->topicName.c_str());
  this->pub_ = this->rosnode_->advertise<stereo_msgs::RawStereo>(this->topicName, 1,
    boost::bind( &GazeboRosStereoCamera::ImageConnect, this),
    boost::bind( &GazeboRosStereoCamera::ImageDisconnect, this));

  // iterate through children of the model parent to find left and right camera sensors
  std::vector<Entity*> sibling = this->myParent->GetChildren();
  std::vector<Entity*>::iterator iter;

  this->leftCamera = NULL;
  this->rightCamera = NULL;

  ROS_DEBUG("stereo: children of model parent %d\n",sibling.size());

  while (!this->leftCamera || !this->rightCamera)
  {
    for (iter = sibling.begin(); iter != sibling.end(); iter++)
    {
      Body* body = dynamic_cast<Body*>(*iter);
      if (body)
      {
        ROS_DEBUG("stereo: children body %s\n",(*iter)->GetName().c_str());
        std::vector<Sensor*> sensors = body->GetSensors();
        std::vector<Sensor*>::iterator sensorIter;
        for (sensorIter = sensors.begin(); sensorIter != sensors.end(); sensorIter++)
        {
          ROS_DEBUG("stereo: children sensor %s\n",(*sensorIter)->GetName().c_str());
          MonoCameraSensor* mcs = dynamic_cast<MonoCameraSensor*>(*sensorIter);
          if (mcs != NULL)
          {
            ROS_DEBUG("stereo: sensors %s is a MCS compare with %s and %s\n",mcs->GetName().c_str(),this->leftCameraName.c_str(),this->rightCameraName.c_str());
            if (mcs->GetName() == this->leftCameraName)
              this->leftCamera = mcs;
            else if (mcs->GetName() == this->rightCameraName)
              this->rightCamera = mcs;
          }
        }
      }
    }

    ROS_WARN("GazeboRosStereoCamera controller requires 2 MonoCameraSensors, waiting for them to be created in simulation...");
    //gzthrow("GazeboRosStereoCamera controller requires 2 MonoCameraSensors");
    usleep(500000);
  }

  // set parent sensor to inactive automatically
  this->leftCamera->SetActive(false);
  this->rightCamera->SetActive(false);

  if( leftCamera->GetImageFormat() == "L8" ) 
  {
      left_type = sensor_msgs::image_encodings::MONO8;
      left_type = sensor_msgs::image_encodings::MONO8;
      left_skip = 1;
  }
  else if ( leftCamera->GetImageFormat() == "R8G8B8" ) 
  {
      left_type = sensor_msgs::image_encodings::RGB8;
      left_skip = 3;
  }
  else if ( leftCamera->GetImageFormat() == "B8G8R8" ) 
  {
      left_type = sensor_msgs::image_encodings::BGR8;
      left_skip = 3;
  }
   
  if( rightCamera->GetImageFormat() == "L8" ) 
  {
      right_type = sensor_msgs::image_encodings::MONO8;
      right_skip = 1;
  }
  else if ( rightCamera->GetImageFormat() == "R8G8B8" ) 
  {
      right_type = sensor_msgs::image_encodings::RGB8;
      right_skip = 3;
  }
  else if ( rightCamera->GetImageFormat() == "B8G8R8" ) 
  {
      right_type = sensor_msgs::image_encodings::BGR8;
      right_skip = 3;
  }

  ROS_DEBUG("stereo: set sensors active\n");

}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboRosStereoCamera::ImageConnect()
{
  this->imageConnectCount++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboRosStereoCamera::ImageDisconnect()
{
  this->imageConnectCount--;
  if (this->imageConnectCount == 0)
  {
    this->leftCamera->SetActive(false);
    this->rightCamera->SetActive(false);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosStereoCamera::UpdateChild()
{
  // as long as ros is connected, parent is active
  //ROS_ERROR("debug image count %d",this->imageConnectCount);
  if (!this->leftCamera->IsActive() || !this->rightCamera->IsActive())
  {
    // do this first so there's chance for sensor to run 1 frame after activate
    if (this->imageConnectCount > 0)
    {
      this->leftCamera->SetActive(true);
      this->rightCamera->SetActive(true);
    }
  }
  else
  {
    this->PutCameraData();
  }

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosStereoCamera::FiniChild()
{
  this->leftCamera->SetActive(false);
  this->rightCamera->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void GazeboRosStereoCamera::PutCameraData()
{
  //CameraData *camera_data = new CameraData();
  const unsigned char *left_src = NULL;
  const unsigned char *right_src = NULL;
  // Get a pointer to image data
  left_src = this->leftCamera->GetImageData(0);
  right_src = this->rightCamera->GetImageData(0);

  if (left_src && right_src)
  {
    /// @todo: don't bother if there are no subscribers
    if (this->pub_.getNumSubscribers() > 0)
    {
      this->lock.lock();
      // setup header
      this->rawStereoMsg.header.frame_id = this->frameName;
      this->rawStereoMsg.header.stamp.sec = (unsigned long)floor(Simulator::Instance()->GetSimTime());
      this->rawStereoMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  Simulator::Instance()->GetSimTime() - this->rawStereoMsg.header.stamp.sec) );

      // copy from src to leftImageMsg
      this->leftImageMsg->header = this->rawStereoMsg.header;
      fillImage(*(this->leftImageMsg),
                this->left_type,
                this->leftCamera->GetImageHeight(),
                this->leftCamera->GetImageWidth(),
                this->left_skip*this->leftCamera->GetImageWidth(),
                (void*)left_src );

      // copy from src to rightImageMsg
      this->rightImageMsg->header = this->rawStereoMsg.header;
      fillImage(*(this->rightImageMsg),
                this->right_type,
                this->rightCamera->GetImageHeight(),
                this->rightCamera->GetImageWidth(),
                this->right_skip*this->rightCamera->GetImageWidth(),
                (void*)right_src );

      // fill StereoInfo stereo_info
      this->stereoInfoMsg->header = this->rawStereoMsg.header;
      this->stereoInfoMsg->height = this->leftCamera->GetImageHeight();
      this->stereoInfoMsg->width  = this->leftCamera->GetImageWidth() ;
      // pose of right cam in left cam coords
      this->stereoInfoMsg->T[0]  = this->baseline;
      this->stereoInfoMsg->T[1]  = 0.0;
      this->stereoInfoMsg->T[2]  = 0.0;
      // rotation
      this->stereoInfoMsg->Om[0] = 0.0;
      this->stereoInfoMsg->Om[1] = 0.0;
      this->stereoInfoMsg->Om[2] = 0.0;
      // reprojection matrix, see videre manual
      this->stereoInfoMsg->RP[0] = 1.0;
      this->stereoInfoMsg->RP[1] = 0.0;
      this->stereoInfoMsg->RP[2] = 0.0;
      this->stereoInfoMsg->RP[3] = -this->Cx;
      this->stereoInfoMsg->RP[4] = 0.0;
      this->stereoInfoMsg->RP[5] = 1.0;
      this->stereoInfoMsg->RP[6] = 0.0;
      this->stereoInfoMsg->RP[7] = -this->Cy;
      this->stereoInfoMsg->RP[8] = 0.0;
      this->stereoInfoMsg->RP[9] = 0.0;
      this->stereoInfoMsg->RP[10] = 0.0;
      this->stereoInfoMsg->RP[11] = this->focal_length;
      this->stereoInfoMsg->RP[12] = 0.0;
      this->stereoInfoMsg->RP[13] = 0.0;
      this->stereoInfoMsg->RP[14] = -1.0/this->baseline;
      this->stereoInfoMsg->RP[15] = (this->Cx-this->CxPrime)/this->baseline;

      // fill CameraInfo left_info
      this->leftCameraInfoMsg->header = this->rawStereoMsg.header;
      this->leftCameraInfoMsg->height = this->leftCamera->GetImageHeight();
      this->leftCameraInfoMsg->width  = this->leftCamera->GetImageWidth() ;
      // distortion
      this->leftCameraInfoMsg->D[0] = 0.0;
      this->leftCameraInfoMsg->D[1] = 0.0;
      this->leftCameraInfoMsg->D[2] = 0.0;
      this->leftCameraInfoMsg->D[3] = 0.0;
      this->leftCameraInfoMsg->D[4] = 0.0;
      // original camera matrix
      this->leftCameraInfoMsg->K[0] = this->focal_length;
      this->leftCameraInfoMsg->K[1] = 0.0;
      this->leftCameraInfoMsg->K[2] = this->Cx;
      this->leftCameraInfoMsg->K[3] = 0.0;
      this->leftCameraInfoMsg->K[4] = this->focal_length;
      this->leftCameraInfoMsg->K[5] = this->Cy;
      this->leftCameraInfoMsg->K[6] = 0.0;
      this->leftCameraInfoMsg->K[7] = 0.0;
      this->leftCameraInfoMsg->K[8] = 1.0;
      // rectification
      this->leftCameraInfoMsg->R[0] = 1.0;
      this->leftCameraInfoMsg->R[1] = 0.0;
      this->leftCameraInfoMsg->R[2] = 0.0;
      this->leftCameraInfoMsg->R[3] = 0.0;
      this->leftCameraInfoMsg->R[4] = 1.0;
      this->leftCameraInfoMsg->R[5] = 0.0;
      this->leftCameraInfoMsg->R[6] = 0.0;
      this->leftCameraInfoMsg->R[7] = 0.0;
      this->leftCameraInfoMsg->R[8] = 1.0;
      // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
      this->leftCameraInfoMsg->P[0] = this->focal_length;
      this->leftCameraInfoMsg->P[1] = 0.0;
      this->leftCameraInfoMsg->P[2] = this->Cx;
      this->leftCameraInfoMsg->P[3] = 0.0;
      this->leftCameraInfoMsg->P[4] = 0.0;
      this->leftCameraInfoMsg->P[5] = this->focal_length;
      this->leftCameraInfoMsg->P[6] = this->Cy;
      this->leftCameraInfoMsg->P[7] = 0.0;
      this->leftCameraInfoMsg->P[8] = 0.0;
      this->leftCameraInfoMsg->P[9] = 0.0;
      this->leftCameraInfoMsg->P[10] = 1.0;
      this->leftCameraInfoMsg->P[11] = 0.0;

      // fill CameraInfo right_info
      this->rightCameraInfoMsg->header = this->rawStereoMsg.header;
      this->rightCameraInfoMsg->height = this->rightCamera->GetImageHeight();
      this->rightCameraInfoMsg->width  = this->rightCamera->GetImageWidth() ;
      // distortion
      this->rightCameraInfoMsg->D[0] = 0.0;
      this->rightCameraInfoMsg->D[1] = 0.0;
      this->rightCameraInfoMsg->D[2] = 0.0;
      this->rightCameraInfoMsg->D[3] = 0.0;
      this->rightCameraInfoMsg->D[4] = 0.0;
      // original camera matrix
      this->rightCameraInfoMsg->K[0] = this->focal_length;
      this->rightCameraInfoMsg->K[1] = 0.0;
      this->rightCameraInfoMsg->K[2] = this->Cx;
      this->rightCameraInfoMsg->K[3] = 0.0;
      this->rightCameraInfoMsg->K[4] = this->focal_length;
      this->rightCameraInfoMsg->K[5] = this->Cy;
      this->rightCameraInfoMsg->K[6] = 0.0;
      this->rightCameraInfoMsg->K[7] = 0.0;
      this->rightCameraInfoMsg->K[8] = 1.0;
      // rectification
      this->rightCameraInfoMsg->R[0] = 1.0;
      this->rightCameraInfoMsg->R[1] = 0.0;
      this->rightCameraInfoMsg->R[2] = 0.0;
      this->rightCameraInfoMsg->R[3] = 0.0;
      this->rightCameraInfoMsg->R[4] = 1.0;
      this->rightCameraInfoMsg->R[5] = 0.0;
      this->rightCameraInfoMsg->R[6] = 0.0;
      this->rightCameraInfoMsg->R[7] = 0.0;
      this->rightCameraInfoMsg->R[8] = 1.0;
      // camera projection matrix (same as camera matrix due to lack of distortion/rectification) (is this generated?)
      this->rightCameraInfoMsg->P[0] = this->focal_length;
      this->rightCameraInfoMsg->P[1] = 0.0;
      this->rightCameraInfoMsg->P[2] = this->Cx;
      this->rightCameraInfoMsg->P[3] = -this->focal_length*this->baseline;
      this->rightCameraInfoMsg->P[4] = 0.0;
      this->rightCameraInfoMsg->P[5] = this->focal_length;
      this->rightCameraInfoMsg->P[6] = this->Cy;
      this->rightCameraInfoMsg->P[7] = 0.0;
      this->rightCameraInfoMsg->P[8] = 0.0;
      this->rightCameraInfoMsg->P[9] = 0.0;
      this->rightCameraInfoMsg->P[10] = 1.0;
      this->rightCameraInfoMsg->P[11] = 0.0;

      // fill uint8 left_type
      this->rawStereoMsg.left_type = this->rawStereoMsg.IMAGE;
      // fill uint8 right_type
      this->rawStereoMsg.right_type = this->rawStereoMsg.IMAGE;
      // fill uint8 has_disparity
      this->rawStereoMsg.has_disparity = 0;
      // publish to ros
      this->pub_.publish(this->rawStereoMsg);
      this->lock.unlock();
    }


  }

}

