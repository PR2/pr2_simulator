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
 * Desc: Bumper controller
 * Author: Nate Koenig
 * Date: 09 Setp. 2008
 */

#include <pr2_gazebo_plugins/ros_bumper.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/ContactSensor.hh>
#include <gazebo/World.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Body.hh>
#include <gazebo/Body.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_bumper", RosBumper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
RosBumper::RosBumper(Entity *parent )
  : Controller(parent)
{
  this->myParent = dynamic_cast<ContactSensor*>(this->parent);

  if (!this->myParent)
    gzthrow("Bumper controller requires a Contact Sensor as its parent");

  Param::Begin(&this->parameters);
  this->bumperTopicNameP = new ParamT<std::string>("bumperTopicName", "bumper", 0);
  Param::End();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosBumper::~RosBumper()
{
  delete this->bumperTopicNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosBumper::LoadChild(XMLConfigNode *node)
{
  this->bumperTopicNameP->Load(node);
  this->bumperTopicName = this->bumperTopicNameP->GetValue();
  ROS_DEBUG("publishing contact/collisions to topic name: %s", 
           this->bumperTopicName.c_str());
  //std::cout << " publishing contact/collisions to topic name: " << this->bumperTopicName << std::endl;

  this->info_pub_ = this->rosnode_->advertise<std_msgs::String>(this->bumperTopicName+std::string("/info"),100);
  this->force_pub_ = this->rosnode_->advertise<geometry_msgs::Vector3Stamped>(this->bumperTopicName+std::string("/force"),100);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosBumper::InitChild()
{
      this->forceMsg.vector.x = 0;
      this->forceMsg.vector.y = 0;
      this->forceMsg.vector.z = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosBumper::UpdateChild()
{
  boost::mutex::scoped_lock sclock(this->lock);

  unsigned int num_contacts = this->myParent->GetContactCount();

  std::string my_geom_name;
  int i_hit_geom;
  double when_i_hit;
  std::string geom_i_hit;
  dJointFeedback contact_forces;

  for (unsigned int i=0; i < num_contacts; i++)
  {
    my_geom_name = this->myParent->GetGeomName(i); 
    i_hit_geom = this->myParent->GetContactState(i); 
    when_i_hit= this->myParent->GetContactTime(i); 
    geom_i_hit = this->myParent->GetContactGeomName(i); 
    contact_forces = this->myParent->GetContactFeedback(i); 
    if (i_hit_geom == 1)
    {
      std::ostringstream stream;
      stream    << "touched!    i:" << i
                << "      my geom:" << my_geom_name
                << "   other geom:" << geom_i_hit
                << "         time:" << when_i_hit
                << " f1:" << contact_forces.f1[0]<<","<<contact_forces.f1[1]<<","<<contact_forces.f1[2]
                << " f2:" << contact_forces.f2[0]<<","<<contact_forces.f2[1]<<","<<contact_forces.f2[2]
                << std::endl;
      //std::cout << stream.str();
      this->bumperMsg.data = stream.str();

      // for the force feedback
      double cur_time = Simulator::Instance()->GetSimTime();
      Body* cur_body = dynamic_cast<Body*>((this->myParent)->GetParent());

      if (cur_body)
        this->forceMsg.header.frame_id = cur_body->GetName();  // @todo: this is the link name
      this->forceMsg.header.stamp.sec = (unsigned long)floor(cur_time);
      this->forceMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  cur_time - this->forceMsg.header.stamp.sec) );

      double eps = 0.10; // very crude low pass filter
      this->forceMsg.vector.x = (1.0-eps)*this->forceMsg.vector.x + eps*contact_forces.f1[0];
      this->forceMsg.vector.y = (1.0-eps)*this->forceMsg.vector.y + eps*contact_forces.f1[1];
      this->forceMsg.vector.z = (1.0-eps)*this->forceMsg.vector.z + eps*contact_forces.f1[2];

      this->info_pub_.publish(this->bumperMsg);
      this->force_pub_.publish(this->forceMsg);
    }
  }

  this->myParent->ResetContactStates();

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosBumper::FiniChild()
{
}
