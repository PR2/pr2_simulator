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
 * Desc: Ros Joint Force controller
 * Author: John Hsu
 * Date: 24 Sept 2008
 */

#include <gazebo_plugin/ros_joint_force.h>

#include <gazebo/World.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>

#include "ros/console.h"

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_joint_force", RosJointForce);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosJointForce::RosJointForce(Entity *parent )
  : Controller(parent)
{
    this->myParent = dynamic_cast<Model*>(this->parent);

    if (!this->myParent)
        gzthrow("ControllerStub controller requires a Model as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosJointForce::~RosJointForce()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosJointForce::LoadChild(XMLConfigNode *node)
{
    XMLConfigNode *jNode;
    Joint *joint;
    std::string jointName;
    dJointFeedback *jFeedback = new dJointFeedback;
    int i =0;

    jNode = node->GetChild("joint");
    while(jNode && i < ROS_JOINT_FORCE_CONTROLLER_MAX_FEEDBACKS)
    {
        jointName = jNode->GetString("name","",1);
        joint = this->myParent->GetJoint(jointName);
        jFeedback = joint->GetFeedback();
        if(jFeedback) {
            this->jointfeedbacks[i] = jFeedback;
            i++;
        }
        jNode = jNode->GetNext("joint");
    }
    this->n_joints = i;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosJointForce::InitChild()
{

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosJointForce::UpdateChild()
{
  // this->myIface->Lock(1);
  //double current_time = Simulator::Instance()->GetSimTime();
  // // Let me explain this number: each joint reports 4 vectors: Force and torque
  // // on each jointed object, respectively. These vectors have 4 elements: x,y,z
  // // and a fourth one. So we transmit 4 dReals per vector.
  int data_count = n_joints*4*4*sizeof(dReal);
  ROS_DEBUG("data_count: %d", data_count);

  double data[1000];

  for(int i=0; i< n_joints; i++) {
    // Copy vector for force on first object
    memcpy(data + (i*4 + 0)*4*sizeof(dReal), this->jointfeedbacks[i]->f1, 4*sizeof(dReal));
    // Copy vector for torque on first object
    memcpy(data + (i*4 + 1)*4*sizeof(dReal), this->jointfeedbacks[i]->t1, 4*sizeof(dReal));
    // Copy vector for force on second object
    memcpy(data + (i*4 + 2)*4*sizeof(dReal), this->jointfeedbacks[i]->f2, 4*sizeof(dReal));
    // Copy vector for torque on second object
    memcpy(data + (i*4 + 3)*4*sizeof(dReal), this->jointfeedbacks[i]->t2, 4*sizeof(dReal));

  }
  ROS_DEBUG_STREAM(" data: " << data << std::endl);
  // this->myIface->Unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosJointForce::FiniChild()
{
}
