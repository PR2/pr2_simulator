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
   Desc: RosSimIface plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b RosSimIface plugin reads ROS Odometry messages
 */

#include <algorithm>
#include <assert.h>

#include <pr2_gazebo_plugins/ros_sim_iface.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Model.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_sim_iface", RosSimIface);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosSimIface::RosSimIface(Entity *parent)
    : Controller(parent)
{
  this->myParent = parent;

  if (!this->myParent)
    gzthrow("RosSimIface controller requires an Entity as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->topicNameP = new ParamT<std::string>("topicName","simiface_pose", 0);
  this->frameNameP = new ParamT<std::string>("frameName","map", 0);
  this->modelNameP = new ParamT<std::string>("modelName","pr2_model", 1);
  this->xyzP  = new ParamT<Vector3>("xyz" ,Vector3(0,0,0), 0);
  this->rpyP  = new ParamT<Vector3>("rpy" ,Vector3(0,0,0), 0);
  this->velP  = new ParamT<Vector3>("vel" ,Vector3(0,0,0), 0);
  this->angVelP  = new ParamT<Vector3>("angVel" ,Vector3(0,0,0), 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosSimIface::~RosSimIface()
{
  delete this->rosnode_;

  delete this->robotNamespaceP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->modelNameP;
  delete this->xyzP;
  delete this->rpyP;
  delete this->velP;
  delete this->angVelP;

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosSimIface::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->topicNameP->Load(node);
  this->frameNameP->Load(node);
  this->modelNameP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->velP->Load(node);
  this->angVelP->Load(node);

  this->topicName = this->topicNameP->GetValue();
  this->frameName = this->frameNameP->GetValue();
  this->modelName = this->modelNameP->GetValue();
  this->xyz = this->xyzP->GetValue();
  this->rpy = this->rpyP->GetValue();
  this->vel = this->velP->GetValue();
  this->angVel = this->angVelP->GetValue();

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosSimIface::InitChild()
{

  ROS_DEBUG("ros simiface subscribing to %s", this->topicName.c_str());
  this->sub_ = this->rosnode_->subscribe(this->topicName.c_str(), 10, &RosSimIface::UpdateObjectPose,this);

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosSimIface::UpdateObjectPose(const nav_msgs::Odometry::ConstPtr& poseMsg)
{

  this->lock.lock();

  Model* model = gazebo::World::Instance()->GetModelByName(this->modelName);
  Vector3 pos(poseMsg->pose.pose.position.x,poseMsg->pose.pose.position.y,poseMsg->pose.pose.position.z);
  Quatern rot(poseMsg->pose.pose.orientation.w,poseMsg->pose.pose.orientation.x,poseMsg->pose.pose.orientation.y,poseMsg->pose.pose.orientation.z);
  Pose3d modelPose(pos,rot);
  model->SetPose(modelPose);

  this->lock.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosSimIface::UpdateChild()
{



}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosSimIface::FiniChild()
{
}


