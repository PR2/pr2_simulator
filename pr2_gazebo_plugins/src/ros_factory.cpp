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

#include <tinyxml/tinyxml.h>

#include <pr2_gazebo_plugins/ros_factory.h>
#include <gazebo_tools/urdf2gazebo.h>

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
  this->spawnModelServiceNameP = new ParamT<std::string>("spawnModelServiceName","spawn_model", 0);
  this->deleteModelServiceNameP = new ParamT<std::string>("deleteModelServiceName","delete_model", 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosFactory::~RosFactory()
{
  delete this->robotNamespaceP;
  delete this->spawnModelServiceNameP;
  delete this->deleteModelServiceNameP;
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
  this->spawnModelService = this->rosnode_->advertiseService(this->spawnModelServiceName,&RosFactory::spawnModel, this);

  this->deleteModelServiceNameP->Load(node);
  this->deleteModelServiceName = this->deleteModelServiceNameP->GetValue();
  this->deleteModelService = this->rosnode_->advertiseService(this->deleteModelServiceName,&RosFactory::deleteModel, this);
}

////////////////////////////////////////////////////////////////////////////////
// utilites for checking incoming string
bool RosFactory::IsURDF(std::string robot_model)
{
  TiXmlDocument doc_in;
  doc_in.Parse(robot_model.c_str());
  if (doc_in.FirstChild("robot"))
    return true;
  else
    return false;
}

////////////////////////////////////////////////////////////////////////////////
// utilites for checking incoming string
bool RosFactory::IsGazeboModelXML(std::string robot_model)
{
  TiXmlDocument doc_in;
  doc_in.Parse(robot_model.c_str());
  if (doc_in.FirstChild("model:physical"))
    return true;
  else
    return false;
}
////////////////////////////////////////////////////////////////////////////////
// Service for deleting models in Gazebo
bool RosFactory::deleteModel(pr2_gazebo_plugins::DeleteModel::Request &req,
                             pr2_gazebo_plugins::DeleteModel::Response &res)
{
  if (!this->pushToDeleteQueue(req.model_name))
  {
    ROS_ERROR("Failed to push robot model to deletion queue iface");
    return 1;
  }

  // wait and verify that model is spawned
  while (gazebo::World::Instance()->GetModelByName(req.model_name))
  {
    ROS_DEBUG("Waiting for model deletion (%s)",req.model_name.c_str());
    usleep(500000);
  }

  // set result
  res.success = true;
  res.status_message = std::string("successfully spawned robot");

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Service for spawning models in Gazebo
bool RosFactory::spawnModel(pr2_gazebo_plugins::SpawnModel::Request &req,
                            pr2_gazebo_plugins::SpawnModel::Response &res)
{
  // check to see if model name already exist as a model
  std::string model_name = req.model_name;
  if (gazebo::World::Instance()->GetModelByName(model_name))
  {
    ROS_ERROR("model name %s already exist.",model_name.c_str());
    return 1;
  }

  // get name space for the corresponding model plugins
  std::string robot_namespace = req.robot_namespace;

  // get model initial pose
  geometry_msgs::Pose initial_pose = req.initial_pose;

  // get incoming string containg either an URDF or a Gazebo Model XML
  // check type depending on the xml_type flag
  // grab from parameter server if necessary
  // convert to Gazebo Model XML if necessary
  std::string robot_model = req.robot_model; // incoming robot model string
  bool convert_urdf2gazebo = false;
  if (req.xml_type == req.URDF)
  {
    if (this->IsURDF(robot_model))
    {
      // incoming robot model string is a string containing URDF
      convert_urdf2gazebo = true;
    }
    else
    {
      ROS_ERROR("input xml type does not match xml_type in request: input URDF XML must begin with <robot>");
      return 1;
    }
  }
  else if (req.xml_type == req.GAZEBO_XML)
  {
    if (this->IsGazeboModelXML(robot_model))
    {
      // incoming robot model string is a string containing a Gazebo Model XML
      convert_urdf2gazebo = false;
    }
    else
    {
      ROS_ERROR(" input Gazebo Model XML must begin with <model:physical>\n");
      return 1;
    }

  }
  else if (req.xml_type == req.URDF_PARAM_NAME)
  {
    // incoming robot model string contains the parameter server name for the URDF
    // get the URDF off the parameter server
    std::string full_param_name;
    rosnode_->searchParam(robot_model,full_param_name);
    rosnode_->getParam(full_param_name.c_str(),robot_model);
    // incoming robot model string is a string containing URDF
    convert_urdf2gazebo = true;

    if (robot_model.c_str()==NULL)
    {
      ROS_ERROR("Unable to load robot model from param server robot_description\n");  
      return 1;
    }
    else if (!this->IsURDF(robot_model))
    {
      ROS_ERROR("input xml type does not match xml_type in request: input URDF XML must begin with <robot>");
      return 1;
    }
  }
  else if (req.xml_type == req.GAZEBO_XML_PARAM_NAME)
  {
    // incoming robot model string contains the parameter server name for the Gazebo Model XML
    // get the Gazebo Model XML off the parameter server
    std::string full_param_name;
    rosnode_->searchParam(robot_model,full_param_name);
    rosnode_->getParam(full_param_name.c_str(),robot_model);
    // incoming robot model string is a string containing a Gazebo Model XML
    convert_urdf2gazebo = false;

    if (robot_model.c_str()==NULL)
    {
      ROS_ERROR("Unable to load robot model from param server robot_description\n");  
      return 1;
    }
    else if (!this->IsGazeboModelXML(robot_model))
    {
      ROS_ERROR(" input Gazebo Model XML must begin with <model:physical>\n");
      return 1;
    }
  }
  else if (req.xml_type == req.URDF_FILE_NAME)
  {
    // incoming robot model string contains the filename for the URDF
    // get the URDF from file (or use resource retriever?)
    ROS_WARN("spawning from resource_retriever using a URDF filename is not supported right now");

    TiXmlDocument robot_model_xml(robot_model);
    robot_model_xml.LoadFile();
    // copy tixml to a string
    std::ostringstream stream;
    stream << robot_model_xml;
    robot_model = stream.str();
    convert_urdf2gazebo = true;

    if (robot_model.c_str()==NULL)
    {
      ROS_ERROR("Unable to load robot model from param server robot_description\n");  
      return 1;
    }
    else if (!this->IsURDF(robot_model))
    {
      ROS_ERROR("input xml type does not match xml_type in request: input URDF XML must begin with <robot>");
      return 1;
    }
  }
  else if (req.xml_type == req.GAZEBO_XML_FILE_NAME)
  {
    // incoming robot model string contains the file name for the Gazebo Model XML
    // get the Gazebo Model XML from file (or use the resource retriever?)
    ROS_WARN("spawning from resource_retriever using a Gazebo Model XML filename is not supported right now");

    TiXmlDocument robot_model_xml(robot_model);
    robot_model_xml.LoadFile();
    // copy tixml to a string
    std::ostringstream stream;
    stream << robot_model_xml;
    robot_model = stream.str();
    convert_urdf2gazebo = false;

    if (robot_model.c_str()==NULL)
    {
      ROS_ERROR("Unable to load robot model from param server robot_description\n");  
      return 1;
    }
    else if (!this->IsGazeboModelXML(robot_model))
    {
      ROS_ERROR(" input Gazebo Model XML must begin with <model:physical>\n");
      return 1;
    }
  }
  else
  {
    ROS_ERROR("type of robot model to be spawned is incorrect, options are:(URDF,GAZEBO_XML,URDF_PARAM_NAME,GAZEBO_XML_PARAM_NAME)");
    return 1;
  }

  // get options for conversions
  // get initial xyz
  double initial_x = req.initial_pose.position.x;
  double initial_y = req.initial_pose.position.y;
  double initial_z = req.initial_pose.position.z;
  urdf::Vector3 initial_xyz(initial_x,initial_y,initial_z);
  // get initial roll pitch yaw (fixed frame transform)
  urdf::Rotation initial_q(req.initial_pose.orientation.x,req.initial_pose.orientation.y,req.initial_pose.orientation.z,req.initial_pose.orientation.w);
  double initial_rx,initial_ry,initial_rz;
  initial_q.getRPY(initial_rx,initial_ry,initial_rz);
  urdf::Vector3 initial_rpy(initial_rx,initial_ry,initial_rz);
  // get flag on joint limits
  bool disable_urdf_joint_limits = req.disable_urdf_joint_limits;

  // do the conversion if necessary
  urdf2gazebo::URDF2Gazebo u2g;
  TiXmlDocument gazebo_model_xml; // resulting Gazebo Model XML to be sent to Factory Iface
  if (convert_urdf2gazebo)
  {
    //************************************/
    /*    convert urdf to gazebo model   */
    //************************************/
    TiXmlDocument robot_model_xml;
    robot_model_xml.Parse(robot_model.c_str());
    u2g.convert(robot_model_xml, gazebo_model_xml, disable_urdf_joint_limits, initial_xyz, initial_rpy, model_name, robot_namespace);
  }
  else
  {
    //************************************/
    /*    prepare gazebo model xml for   */
    /*    factory                        */
    //************************************/
    /// STRIP DECLARATION <? ... xml version="1.0" ... ?> from robot_model
    /// @todo: does tinyxml have functionality for this?
    /// @todo: should gazebo take care of the declaration?
    std::string open_bracket("<?");
    std::string close_bracket("?>");
    int pos1 = robot_model.find(open_bracket,0);
    int pos2 = robot_model.find(close_bracket,0);
    robot_model.replace(pos1,pos2-pos1+2,std::string(""));

    // put string in TiXmlDocument for manipulation
    gazebo_model_xml.Parse(robot_model.c_str());

    // optional model manipulations:
    //  * update initial pose
    //  * replace model name
    TiXmlElement* model;
    model = gazebo_model_xml.FirstChildElement("model:physical");
    if (model)
    {
      // replace initial pose of robot
      // find first instance of xyz and rpy, replace with initial pose
      TiXmlElement* xyz_key = model->FirstChildElement("xyz");
      if (xyz_key)
        model->RemoveChild(xyz_key);
      TiXmlElement* rpy_key = model->FirstChildElement("rpy");
      if (rpy_key)
        model->RemoveChild(rpy_key);

      xyz_key = new TiXmlElement("xyz");
      rpy_key = new TiXmlElement("rpy");

      std::ostringstream xyz_stream, rpy_stream;
      xyz_stream << initial_x << " " << initial_y << " " << initial_z;
      rpy_stream << initial_rx << " " << initial_ry << " " << initial_rz;

      TiXmlText* xyz_txt = new TiXmlText(xyz_stream.str());
      TiXmlText* rpy_txt = new TiXmlText(rpy_stream.str());

      xyz_key->LinkEndChild(xyz_txt);
      rpy_key->LinkEndChild(rpy_txt);

      model->LinkEndChild(xyz_key);
      model->LinkEndChild(rpy_key);


      // replace model name if one is specified by the user
      if (!model_name.empty())
      {
        model->RemoveAttribute("name");
        model->SetAttribute("name",model_name);
      }

    }
  }

  // push to factory iface
  std::ostringstream stream;
  stream << gazebo_model_xml;
  std::string gazebo_model_xml_string = stream.str();
  ROS_DEBUG("Gazebo Model XML\n\n%s\n\n ",gazebo_model_xml_string.c_str());

  if (!this->pushToFactory(gazebo_model_xml_string))
  {
    ROS_ERROR("Failed to push robot model to factory iface");
    return 1;
  }

  // wait and verify that model is spawned
  while (!gazebo::World::Instance()->GetModelByName(model_name))
  {
    ROS_DEBUG("Waiting for spawning model (%s)",model_name.c_str());
    usleep(500000);
  }

  // set result
  res.success = true;
  res.status_message = std::string("successfully spawned robot");

  return 0;
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
// Push model name to Iface for deletion
bool RosFactory::pushToDeleteQueue(std::string model_name)
{
  // connect to gazebo
  gazebo::Client *client = new gazebo::Client();
  gazebo::FactoryIface *factoryIface = new gazebo::FactoryIface();

  int serverId = 0;

  bool connected_to_server = false;
  /// Connect to the libgazebo server
  while (!connected_to_server)
  {
    try
    {
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

  /// Open the Factory interface
  try
  {
    factoryIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    ROS_ERROR("Gazebo error: Unable to connect to the factory interface\n%s\n",e.GetErrorStr().c_str());
    return -1;
  }

  bool writing_iface = true;
  while (writing_iface)
  {
    factoryIface->Lock(1);
    if (strcmp((char*)factoryIface->data->deleteModel,"")==0)
    {
      ROS_INFO("Deleting Robot Model Name:%s in Gazebo\n",model_name.c_str());
      // don't overwrite data, only write if iface data is empty
      strcpy((char*)factoryIface->data->deleteModel, model_name.c_str());
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



