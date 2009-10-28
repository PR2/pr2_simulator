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
 * Desc: Empty gazebo plugin
 * Author: John Hsu
 * Date: 24 July 2009
 * SVN info: $Id$
 */


#include <algorithm>
#include <assert.h>

#include <pr2_gazebo_plugins/ros_step_world_state.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_step_world_state", RosStepWorldState);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosStepWorldState::RosStepWorldState(Entity *parent)
    : Controller(parent)
{
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboMechanismControl controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "gazebo_world_state", 0);
  this->frameNameP = new ParamT<std::string>("frameName", "base_link", 0);
  Param::End();

  this->all_bodies.clear();
  this->models.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosStepWorldState::~RosStepWorldState()
{
  delete this->robotNamespaceP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosStepWorldState::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo",ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();

  this->sub_ = rosnode_->subscribe(this->topicName,100,&RosStepWorldState::WorldStateCallback,this);

  // spawn 2 threads by default, ///@todo: make this a parameter
  ros::MultiThreadedSpinner s(2);
  boost::thread spinner_thread( boost::bind( &ros::spin, s ) );
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void RosStepWorldState::WorldStateCallback(const pr2_gazebo_plugins::WorldStateConstPtr& worldStateMsg)
{
  ROS_DEBUG("received state message");

  // get information from message
  // ignore frame_id for now, everything inertial.  this->worldStateMsg->header.frame_id
  gazebo::Simulator::Instance()->SetSimTime(worldStateMsg->header.stamp.toSec());

  int object_count = worldStateMsg->get_name_size();

  //this->lock.lock();
  for (int count = 0; count < object_count; count++)
  {
    boost::recursive_mutex::scoped_lock lock(*gazebo::Simulator::Instance()->GetMRMutex());
    std::map<std::string,gazebo::Body*>::iterator body = this->all_bodies.find(worldStateMsg->name[count]);
    if (body == this->all_bodies.end())
    {
      //ROS_ERROR("body %s is not in this world!",worldStateMsg->name[count].c_str());
    }
    else
    {
      //ROS_ERROR("setting pose for body %s.",worldStateMsg->name[count].c_str());
      Vector3 pos;
      Quatern rot;
      pos.x = worldStateMsg->pose[count].position.x;
      pos.y = worldStateMsg->pose[count].position.y;
      pos.z = worldStateMsg->pose[count].position.z;
      rot.x = worldStateMsg->pose[count].orientation.x;
      rot.y = worldStateMsg->pose[count].orientation.y;
      rot.z = worldStateMsg->pose[count].orientation.z;
      rot.u = worldStateMsg->pose[count].orientation.w;
      body->second->SetPose(Pose3d(pos,rot));
    }
  }
  //this->lock.unlock();

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosStepWorldState::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosStepWorldState::UpdateChild()
{

  /********************************************************************/
  /*                                                                  */
  /*  build list of all bodies in the world (across models)           */
  /*                                                                  */
  /*  need a faster way to hook up each body with incoming states     */
  /*                                                                  */
  /********************************************************************/

  // FIXME: this test simply checks the number of models for now, better
  //        to setup a flag to indicate whether models in the world have changed
  if (this->models.size() != gazebo::World::Instance()->GetModels().size())
  {
    this->models = gazebo::World::Instance()->GetModels();

    // aggregate all bodies into a single vector
    for (std::vector<gazebo::Model*>::iterator miter = this->models.begin(); miter != this->models.end(); miter++)
    {
      // list of all bodies in the current model
      const std::map<std::string,gazebo::Body*> *bodies = (*miter)->GetBodies();
      // Iterate through all bodies
      std::map<std::string, Body*>::const_iterator biter;
      for (biter=bodies->begin(); biter!=bodies->end(); biter++)
      {
        this->all_bodies.insert(make_pair(biter->first,biter->second));
      }
    }
  }
  //ROS_ERROR("debug: %d",this->all_bodies.size());

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosStepWorldState::FiniChild()
{
}



