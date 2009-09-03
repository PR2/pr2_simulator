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
   Desc: RosCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b RosCamera plugin broadcasts ROS Image messages
 */

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>

#include <pr2_gazebo_plugins/ros_camera.h>

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

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_camera", RosCamera);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosCamera::RosCamera(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<MonoCameraSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("RosCamera controller requires a Camera Sensor as its parent");

  Param::Begin(&this->parameters);
  this->topicNameP = new ParamT<std::string>("topicName","stereo/raw_stereo", 0);
  this->frameNameP = new ParamT<std::string>("frameName","stereo_link", 0);
  Param::End();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosCamera::~RosCamera()
{
  delete this->rosnode_;
  delete this->topicNameP;
  delete this->frameNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosCamera::LoadChild(XMLConfigNode *node)
{
  this->topicNameP->Load(node);
  this->frameNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->frameName = this->frameNameP->GetValue();

  ROS_DEBUG("================= %s", this->topicName.c_str());
  this->pub_ = this->rosnode_->advertise<sensor_msgs::Image>(this->topicName,1);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosCamera::InitChild()
{
  // set parent sensor to active automatically
  this->myParent->SetActive(true);

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
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }
  else
  {
    ROS_ERROR("Unsupported Gazebo ImageFormat\n");
    this->type = sensor_msgs::image_encodings::BGR8;
    this->skip = 3;
  }

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosCamera::UpdateChild()
{

  // do this first so there's chance for sensor to run 1 frame after activate
  if (this->myParent->IsActive())
    this->PutCameraData();
  else
    this->myParent->SetActive(true); // as long as this plugin is running, parent is active

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosCamera::FiniChild()
{
  this->myParent->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void RosCamera::PutCameraData()
{
  const unsigned char *src;

  boost::recursive_mutex::scoped_lock mr_lock(*Simulator::Instance()->GetMRMutex());

  // Get a pointer to image data
  src = this->myParent->GetImageData(0);

  //std::cout << " updating camera " << this->topicName << " " << this->width << std::endl;

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
    if (this->pub_.getNumSubscribers() > 0)
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
      this->pub_.publish(this->imageMsg);
    }

    //double tmpT3 = Simulator::Instance()->GetWallTime();

    this->lock.unlock();
  }

}

