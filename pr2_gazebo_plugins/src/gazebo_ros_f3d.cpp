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
   Desc: Force Feed Back Ground Truth
   Author: Sachin Chitta and John Hsu
   Date: 1 June 2008
   SVN info: $Id$
 @htmlinclude manifest.html
 @b RosF3D plugin broadcasts forces acting on the body specified by name.
 */

#include <pr2_gazebo_plugins/gazebo_ros_f3d.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_f3d", RosF3D);


////////////////////////////////////////////////////////////////////////////////
// Constructor
RosF3D::RosF3D(Entity *parent )
   : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("RosF3D controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->bodyNameP = new ParamT<std::string>("bodyName", "", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "default_f3d_topic", 0);
  this->frameNameP = new ParamT<std::string>("frameName", "default_f3d_frame", 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosF3D::~RosF3D()
{
  delete this->robotNamespaceP;
  delete this->bodyNameP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosF3D::LoadChild(XMLConfigNode *node)
{

  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo",ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->bodyNameP->Load(node);
  this->bodyName = this->bodyNameP->GetValue();
  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(bodyName));
  if (!this->myBody)
    ROS_FATAL("gazebo_ros_f3d plugin error: bodyName: %s does not exist\n",bodyName.c_str());

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();

  this->pub_ = this->rosnode_->advertise<geometry_msgs::Vector3Stamped>(this->topicName,10);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosF3D::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosF3D::UpdateChild()
{
  Vector3 torque;
  Vector3 force;

  // get force on body
  force = this->myBody->GetForce();

  // get torque on body
  torque = this->myBody->GetTorque();

  //FIXME:  toque not published, need new message type of new topic name for torque
  //FIXME:  use name space of body id (link name)?
  this->lock.lock();
  // copy data into pose message
  this->vector3Msg.header.frame_id = this->frameName;
  this->vector3Msg.header.stamp.sec = (unsigned long)floor(Simulator::Instance()->GetSimTime());
  this->vector3Msg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  Simulator::Instance()->GetSimTime() - this->vector3Msg.header.stamp.sec) );

  this->vector3Msg.vector.x    = force.x;
  this->vector3Msg.vector.y    = force.y;
  this->vector3Msg.vector.z    = force.z;

  //std::cout << "RosF3D: " << this->topicName
  //          << "  f: " << force
  //          << "  t: " << torque << std::endl;
  // publish to ros
  this->pub_.publish(this->vector3Msg);
  this->lock.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosF3D::FiniChild()
{
}
