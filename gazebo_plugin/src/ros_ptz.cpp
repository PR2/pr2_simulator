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
 * Desc: A ros ptz controller
 * Author: Nathan Koenig
 * Date: 26 Nov 2007
 * SVN: $Id:$
 */

#include <algorithm>
#include <assert.h>

#include <gazebo_plugin/ros_ptz.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/HingeJoint.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_ptz", RosPTZ);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosPTZ::RosPTZ(Entity *parent)
    : Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("RosPTZ controller requires a Model as its parent");

  this->panJoint = NULL;
  this->tiltJoint = NULL;

  this->panJoint = NULL;
  this->tiltJoint = NULL;

  Param::Begin(&this->parameters);
  this->panJointNameP = new ParamT<std::string>("panJoint", "", 1);
  this->tiltJointNameP = new ParamT<std::string>("tiltJoint", "", 1);
  this->motionGainP = new ParamT<double>("motionGain",2,0);
  this->forceP = new ParamT<double>("force",10,0);
  this->commandTopicNameP = new ParamT<std::string>("commandTopicName","PTZ_cmd",0);
  this->stateTopicNameP = new ParamT<std::string>("stateTopicName","PTZ_state",0);
  Param::End();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosPTZ::~RosPTZ()
{
  delete this->rosnode_;
  //if (this->panJoint)
  //  delete this->panJoint;
  //if (this->tiltJoint)
  //  delete this->tiltJoint;

  this->panJoint = NULL;
  this->tiltJoint = NULL;

  delete this->panJointNameP;
  delete this->tiltJointNameP;
  delete this->motionGainP;
  delete this->forceP;
  delete this->commandTopicNameP;
  delete this->stateTopicNameP;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosPTZ::LoadChild(XMLConfigNode *node)
{

  this->panJointNameP->Load(node);
  this->tiltJointNameP->Load(node);
  this->motionGainP->Load(node);
  this->forceP->Load(node);
  this->commandTopicNameP->Load(node);
  this->stateTopicNameP->Load(node);

  this->panJoint = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(this->panJointNameP->GetValue()));
  this->tiltJoint = dynamic_cast<HingeJoint*>(this->myParent->GetJoint(this->tiltJointNameP->GetValue()));

  this->commandTopicName = this->commandTopicNameP->GetValue();
  this->stateTopicName = this->stateTopicNameP->GetValue();

  this->panFrameName = this->panJointNameP->GetValue();
  this->tiltFrameName = this->tiltJointNameP->GetValue();

  ROS_DEBUG(" publishing state topic for ptz %s", this->stateTopicName.c_str());
  ROS_DEBUG(" subscribing command topic for ptz %s", this->commandTopicName.c_str());

  this->pub_ = this->rosnode_->advertise<axis_cam::PTZActuatorState>(this->stateTopicName,10);
  this->sub_ = this->rosnode_->subscribe(this->commandTopicName.c_str(), 10, &RosPTZ::PTZCommandReceived,this);

  if (!this->panJoint)
    gzthrow("couldn't get pan hinge joint");

  if (!this->tiltJoint)
    gzthrow("couldn't get tilt hinge joint");

}

void RosPTZ::PTZCommandReceived(const axis_cam::PTZActuatorCmdConstPtr& in)
{
  this->lock.lock();
  this->cmdPan  = in->pan.cmd*M_PI/180.0;
  this->cmdTilt = in->tilt.cmd*M_PI/180.0;
  this->lock.unlock();
}

////////////////////////////////////////////////////////////////////////////////
/// Save the controller.
void RosPTZ::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->panJointNameP) << "\n";
  stream << prefix << *(this->tiltJointNameP) << "\n";
  stream << prefix << *(this->motionGainP) << "\n";
  stream << prefix << *(this->forceP) << "\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosPTZ::InitChild()
{
  // initialize pan/tilt angles
  this->cmdPan  = 0.0;
  this->cmdTilt = 0.0;
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void RosPTZ::ResetChild()
{

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosPTZ::UpdateChild()
{
  // Apply joint limits to commanded pan/tilt angles
  if (this->cmdTilt > M_PI*0.3)
    this->cmdTilt = M_PI*0.3;
  else if (this->cmdTilt < -M_PI*0.3)
    this->cmdTilt = -M_PI*0.3;

  if (this->cmdPan > M_PI*0.3)
    this->cmdPan = M_PI*0.3;
  else if (this->cmdPan < -M_PI*0.3)
    this->cmdPan = -M_PI*0.3;

  // Apply limits on commanded zoom
  /*if (this->cmdZoom < this->zoomMin)
    this->cmdZoom = this->zoomMin;
  if (this->cmdZoom > this->zoomMax)
    this->cmdZoom = this->zoomMax;
    */

  // Set the pan and tilt motors; can't set angles so track cmds with
  // a proportional control
  float tilt = this->cmdTilt - this->tiltJoint->GetAngle();
  float pan = this->cmdPan - this->panJoint->GetAngle();

  //std::cout << "command received : " << this->cmdPan << ":" << this->cmdTilt;
  //std::cout << " state : " << this->panJoint->GetAngle() << ":" << this->tiltJoint->GetAngle() << std::endl;
  this->tiltJoint->SetParam( dParamVel, **(this->motionGainP) * tilt);
  this->tiltJoint->SetParam( dParamFMax, **(this->forceP) );

  this->panJoint->SetParam( dParamVel, **(this->motionGainP) * pan);
  this->panJoint->SetParam( dParamFMax, **(this->forceP) );

  this->PutPTZData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosPTZ::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void RosPTZ::PutPTZData()
{
  // Data timestamp, not used
  //double cur_time = Simulator::Instance()->GetSimTime();

  this->lock.lock();
  // also put data into ros message
  PTZStateMessage.pan.pos_valid =1;
  PTZStateMessage.pan.pos       = RTOD(this->panJoint->GetAngle());
  PTZStateMessage.tilt.pos_valid=1;
  PTZStateMessage.tilt.pos      = RTOD(this->tiltJoint->GetAngle());
  // publish topic
  this->pub_.publish(PTZStateMessage);
  this->lock.unlock();

}

