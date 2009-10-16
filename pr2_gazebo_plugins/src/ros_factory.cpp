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
  // extract information from request
  std::string model_name = req.model_name;
  // convert

  // push to factory iface

  // wait and verify that model is spawned

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Open Factory Iface and Push Model to Factory
bool RosFactory::pushToFactory(std::string gazebo_model_xml)
{
  //************************************/
  /*  Connect to Gazebo Iface Server   */
  //************************************/
  gazebo::Client *client = new gazebo::Client();
  gazebo::FactoryIface *factoryIface = new gazebo::FactoryIface();

  int serverId = 0;

  bool connected_to_server = false;
  /// Connect to the libgazebo server
  while (!connected_to_server)
  {
    try
    {
      ROS_INFO("spawn_gazebo_model waiting for gazebo factory, usually launched by 'roslaunch `rospack find gazebo`/launch/empty_world.launch'");
      client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
      connected_to_server = true;
    }
    catch (gazebo::GazeboError e)
    {
      ROS_ERROR("Gazebo error: Unable to connect\n %s\n",e.GetErrorStr().c_str());
      usleep(1000000);
      connected_to_server = false;
    }
  }

  //************************************/
  /*    Open the Factory interface     */
  //************************************/
  try
  {
    factoryIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    ROS_ERROR("Gazebo error: Unable to connect to the factory interface\n%s\n",e.GetErrorStr().c_str());
    return false;
  }

  //************************************/
  /*  Copy model to a string           */
  //************************************/
  std::ostringstream stream;
  stream << gazebo_model_xml;
  std::string gazebo_model_xml_string = stream.str();
  ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());

  bool writing_iface = true;
  while (writing_iface)
  {
    factoryIface->Lock(1);
    if (strcmp((char*)factoryIface->data->newModel,"")==0)
    {
      // don't overwrite data, only write if iface data is empty
      strcpy((char*)factoryIface->data->newModel, gazebo_model_xml_string.c_str());
      writing_iface = false;
    }
    factoryIface->Unlock();
  }
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



