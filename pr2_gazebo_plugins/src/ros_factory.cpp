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

#include <pr2_gazebo_plugins/ros_factory.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_factory", RosFactory);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosFactory::RosFactory(Entity *parent)
    : Controller(parent)
{

  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("RosFactory controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->spawnModelServiceNameP = new ParamT<std::string>("spawnModelServiceName","spawn_model_service", 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosFactory::~RosFactory()
{
  delete this->robotNamespaceP;
  delete this->spawnModelServiceNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosFactory::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->spawnModelServiceNameP->Load(node);
  this->spawnModelServiceName = this->spawnModelServiceNameP->GetValue();

  this->spawnService = this->rosnode_->advertiseService(this->spawnModelServiceName,&RosFactory::spawnModel, this);
}

////////////////////////////////////////////////////////////////////////////////
// Service
bool RosFactory::spawnModel(pr2_gazebo_plugins::GazeboModel::Request &req,
                            pr2_gazebo_plugins::GazeboModel::Response &res)
{
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosFactory::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosFactory::UpdateChild()
{
    /***************************************************************/
    /*                                                             */
    /*  this is called at every update simulation step             */
    /*                                                             */
    /***************************************************************/
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosFactory::FiniChild()
{
}



