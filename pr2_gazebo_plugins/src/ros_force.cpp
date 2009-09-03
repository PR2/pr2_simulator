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
   Desc: RosForce plugin for manipulating objects in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b RosForce plugin reads ROS geometry_msgs/Wrench messages
 */

#include <algorithm>
#include <assert.h>

#include <pr2_gazebo_plugins/ros_force.h>

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_force", RosForce);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosForce::RosForce(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("RosForce controller requires an Model as its parent");

  Param::Begin(&this->parameters);
  this->topicNameP = new ParamT<std::string>("topicName","sim_force", 0);
  this->bodyNameP = new ParamT<std::string>("bodyName","link", 1);
  Param::End();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosForce::~RosForce()
{
  delete this->rosnode_;

  delete this->topicNameP;
  delete this->bodyNameP;

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosForce::LoadChild(XMLConfigNode *node)
{
  this->topicNameP->Load(node);
  this->bodyNameP->Load(node);

  this->topicName = this->topicNameP->GetValue();
  this->bodyName = this->bodyNameP->GetValue();


  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(bodyName)) == NULL)
    ROS_FATAL("ros_force plugin error: bodyName: %s does not exist\n",bodyName.c_str());

  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosForce::InitChild()
{

  ROS_DEBUG("ros force: subscribing to %s", this->topicName.c_str());
  this->sub_ = this->rosnode_->subscribe(this->topicName.c_str(), 10, &RosForce::UpdateObjectForce,this);

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosForce::UpdateObjectForce(const geometry_msgs::Wrench::ConstPtr& wrenchMsg)
{

  this->lock.lock();

  Vector3 force(wrenchMsg->force.x,wrenchMsg->force.y,wrenchMsg->force.z);
  Vector3 torque(wrenchMsg->torque.x,wrenchMsg->torque.y,wrenchMsg->torque.z);
  this->myBody->SetForce(force);
  this->myBody->SetTorque(torque);

  this->lock.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosForce::UpdateChild()
{



}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosForce::FiniChild()
{
}


