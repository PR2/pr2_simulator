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

#include <pr2_gazebo_plugins/gazebo_ros_p3d.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_p3d", RosP3D);


////////////////////////////////////////////////////////////////////////////////
// Constructor
RosP3D::RosP3D(Entity *parent )
   : Controller(parent)
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("RosP3D controller requires a Model as its parent");

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
RosP3D::~RosP3D()
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
void RosP3D::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo",ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->bodyNameP->Load(node);
  this->bodyName = this->bodyNameP->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName)) == NULL)
    ROS_FATAL("gazebo_ros_p3d plugin error: bodyName: %s does not exist\n",this->bodyName.c_str());

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
    this->pub_ = this->rosnode_->advertise<nav_msgs::Odometry>(this->topicName,10);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void RosP3D::InitChild()
{
  this->last_time = Simulator::Instance()->GetSimTime();
  this->last_vpos = this->myBody->GetPositionRate(); // get velocity in gazebo frame
  this->last_veul = this->myBody->GetEulerRate(); // get velocity in gazebo frame
  this->apos = 0;
  this->aeul = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void RosP3D::UpdateChild()
{
  Pose3d pose;
  Quatern rot;
  Vector3 pos;

  // Get Pose/Orientation ///@todo: verify correctness
  pose = this->myBody->GetPose(); // - this->myBody->GetCoMPose();

  // apply xyz offsets and get position and rotation components
  pos = pose.pos + this->xyzOffsets;
  rot = pose.rot;
  // std::cout << " --------- RosP3D rot " << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.u << std::endl;

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
    this->poseMsg.header.frame_id = "map";  // @todo: should this be changeable?
    this->poseMsg.header.stamp.sec = (unsigned long)floor(cur_time);
    this->poseMsg.header.stamp.nsec = (unsigned long)floor(  1e9 * (  cur_time - this->poseMsg.header.stamp.sec) );

    // pose is given in inertial frame for Gazebo, transform to the designated frame name

    this->poseMsg.pose.pose.position.x    = pos.x;
    this->poseMsg.pose.pose.position.y    = pos.y;
    this->poseMsg.pose.pose.position.z    = pos.z;

    this->poseMsg.pose.pose.orientation.x = rot.x;
    this->poseMsg.pose.pose.orientation.y = rot.y;
    this->poseMsg.pose.pose.orientation.z = rot.z;
    this->poseMsg.pose.pose.orientation.w = rot.u;

    this->poseMsg.twist.twist.linear.x        = vpos.x + this->GaussianKernel(0,this->gaussianNoise) ;
    this->poseMsg.twist.twist.linear.y        = vpos.y + this->GaussianKernel(0,this->gaussianNoise) ;
    this->poseMsg.twist.twist.linear.z        = vpos.z + this->GaussianKernel(0,this->gaussianNoise) ;
    // pass euler angular rates
    this->poseMsg.twist.twist.angular.x    = veul.x + this->GaussianKernel(0,this->gaussianNoise) ;
    this->poseMsg.twist.twist.angular.y    = veul.y + this->GaussianKernel(0,this->gaussianNoise) ;
    this->poseMsg.twist.twist.angular.z    = veul.z + this->GaussianKernel(0,this->gaussianNoise) ;

    // fill in covariance matrix
    /// @todo: let user set separate linear and angular covariance values.
    this->poseMsg.pose.covariance[0] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.pose.covariance[7] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.pose.covariance[14] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.pose.covariance[21] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.pose.covariance[28] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.pose.covariance[35] = this->gaussianNoise*this->gaussianNoise;

    this->poseMsg.twist.covariance[0] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.twist.covariance[7] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.twist.covariance[14] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.twist.covariance[21] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.twist.covariance[28] = this->gaussianNoise*this->gaussianNoise;
    this->poseMsg.twist.covariance[35] = this->gaussianNoise*this->gaussianNoise;

    // publish to ros
    this->pub_.publish(this->poseMsg);
  }

  this->lock.unlock();

  // save last time stamp
  this->last_time = cur_time;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void RosP3D::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double RosP3D::GaussianKernel(double mu,double sigma)
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


