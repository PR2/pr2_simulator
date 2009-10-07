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
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 * SVN info: $Id$
 */

#include <pr2_gazebo_plugins/ros_imu.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("ros_imu", RosIMU);


////////////////////////////////////////////////////////////////////////////////
// Constructor
RosIMU::RosIMU(Entity *parent )
   : Controller(parent)
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("RosIMU controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->bodyNameP = new ParamT<std::string>("bodyName", "", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "default_f3d_topic", 0);
  this->frameNameP = new ParamT<std::string>("frameName", "default_f3d_frame", 0);
  this->xyzOffsetsP    = new ParamT<Vector3>("xyzOffsets", Vector3(0,0,0),0);
  this->rpyOffsetsP    = new ParamT<Vector3>("rpyOffsets", Vector3(0,0,0),0);
  this->gaussianNoiseP = new ParamT<double>("gaussianNoise",0.0,0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RosIMU::~RosIMU()
{
  delete this->robotNamespaceP;
  delete this->bodyNameP;
  delete this->topicNameP;
  delete this->frameNameP;
  delete this->xyzOffsetsP;
  delete this->rpyOffsetsP;
  delete this->gaussianNoiseP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void RosIMU::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->bodyNameP->Load(node);
  this->bodyName = this->bodyNameP->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName)) == NULL)
    ROS_FATAL("ros_imu plugin error: bodyName: %s does not exist\n",this->bodyName.c_str());

  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName));

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();
  this->frameNameP->Load(node);
  this->frameName = this->frameNameP->GetValue();
  this->xyzOffsetsP->Load(node);
  this->xyzOffsets = this->xyzOffsetsP->GetValue();
  this->rpyOffsetsP->Load(node);
  this->rpyOffsets = this->rpyOffsetsP->GetValue();
  this->gaussianNoiseP->Load(node);
  this->gaussianNoise = this->gaussianNoiseP->GetValue();

  if (this->topicName != "")
    this->pub_ = this->rosnode_->advertise<sensor_msgs::Imu>(this->topicName,10);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosIMU::InitChild()
{
  this->last_time = Simulator::Instance()->GetSimTime();
  this->last_vpos = this->myBody->GetPositionRate(); // get velocity in gazebo frame
  this->last_veul = this->myBody->GetEulerRate(); // get velocity in gazebo frame
  this->apos = 0;
  this->aeul = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosIMU::UpdateChild()
{
  Pose3d pose;
  Quatern rot;
  Vector3 pos;

  // Get Pose/Orientation ///@todo: verify correctness
  pose = this->myBody->GetPose(); // - this->myBody->GetCoMPose();

  // apply xyz offsets and get position and rotation components
  pos = pose.pos + this->xyzOffsets;
  rot = pose.rot;
  // std::cout << " --------- RosIMU rot " << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.u << std::endl;

  // apply rpy offsets
  Quatern qOffsets;
  qOffsets.SetFromEuler(this->rpyOffsets);
  rot = qOffsets*rot;
  rot.Normalize();

  double cur_time = Simulator::Instance()->GetSimTime();
  
  // get Rates
  Vector3 vpos = this->myBody->GetPositionRate(); // get velocity in gazebo frame
  Quatern vrot = this->myBody->GetRotationRate(); // get velocity in gazebo frame
  Vector3 veul = this->myBody->GetEulerRate(); // get velocity in gazebo frame

  // differentiate to get accelerations
  double tmp_dt = this->last_time - cur_time;
  if (tmp_dt != 0)
  {
    this->apos = (this->last_vpos - vpos) / tmp_dt;
    this->aeul = (this->last_veul - veul) / tmp_dt;
    this->last_vpos = vpos;
    this->last_veul = veul;
  }

  this->lock.lock();


  if (this->topicName != "")
  {
    // copy data into pose message
    this->imuMsg.header.frame_id = "map";  // @todo: should this be changeable?
    this->imuMsg.header.stamp.sec = (unsigned long)floor(cur_time);
    this->imuMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  cur_time - this->imuMsg.header.stamp.sec) );

    // orientation quaternion
    this->imuMsg.orientation.x = rot.x;
    this->imuMsg.orientation.y = rot.y;
    this->imuMsg.orientation.z = rot.z;
    this->imuMsg.orientation.w = rot.u;

    // pass euler angular rates
    this->imuMsg.angular_velocity.x    = veul.x + this->GaussianKernel(0,this->gaussianNoise) ;
    this->imuMsg.angular_velocity.y    = veul.y + this->GaussianKernel(0,this->gaussianNoise) ;
    this->imuMsg.angular_velocity.z    = veul.z + this->GaussianKernel(0,this->gaussianNoise) ;

    // pass accelerations
    this->imuMsg.linear_acceleration.x    = apos.x;
    this->imuMsg.linear_acceleration.y    = apos.y;
    this->imuMsg.linear_acceleration.z    = apos.z;

    // fill in covariance matrix
    /// @todo: let user set separate linear and angular covariance values.
    this->imuMsg.orientation_covariance[0] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.orientation_covariance[4] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.orientation_covariance[8] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.angular_velocity_covariance[0] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.angular_velocity_covariance[4] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.angular_velocity_covariance[8] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.linear_acceleration_covariance[0] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.linear_acceleration_covariance[4] = this->gaussianNoise*this->gaussianNoise;
    this->imuMsg.linear_acceleration_covariance[8] = this->gaussianNoise*this->gaussianNoise;

    // publish to ros
    this->pub_.publish(this->imuMsg);
  }

  this->lock.unlock();

  // save last time stamp
  this->last_time = cur_time;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosIMU::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double RosIMU::GaussianKernel(double mu,double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}


