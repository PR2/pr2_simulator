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
 * Desc: Actuator array controller for a Pr2 robot.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */


#include <algorithm>
#include <assert.h>

#include <pr2_gazebo_plugins/gazebo_ros_time.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_time", GazeboRosTime);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosTime::GazeboRosTime(Entity *parent)
    : Controller(parent)
{
  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosTime::~GazeboRosTime()
{
  delete this->rosnode_;
  delete this->robotNamespaceP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosTime::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo",ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  // for rostime
  this->pub_ = this->rosnode_->advertise<roslib::Time>("time",10);

  // broadcasting sim time, so set parameter, this should really be in the launch script param tag, so it's set before nodes start
  this->rosnode_->setParam("/use_sim_time", true);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosTime::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosTime::UpdateChild()
{
    // pass time to robot
    double currentTime = Simulator::Instance()->GetSimTime();
    //std::cout << "sim time: " << currentTime << std::endl;

    /***************************************************************/
    /*                                                             */
    /*  publish time to ros                                        */
    /*                                                             */
    /***************************************************************/
    this->lock.lock();
    timeMsg.rostime.fromSec(currentTime);
    this->pub_.publish(timeMsg);
    this->lock.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosTime::FiniChild()
{
}



