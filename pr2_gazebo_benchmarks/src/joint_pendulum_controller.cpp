/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "pr2_gazebo_benchmarks/joint_pendulum_controller.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_DECLARE_CLASS(pr2_gazebo_benchmarks, JointPendulumController, controller::JointPendulumController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

JointPendulumController::JointPendulumController()
: joint_state_(NULL), command_(0),
  loop_count_(0),  initialized_(false), robot_(NULL), last_time_(0), last_position_(0), last_last_position_(0)
{
}

JointPendulumController::~JointPendulumController()
{
  sub_command_.shutdown();
}

bool JointPendulumController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
				   const control_toolbox::Pid &pid)
{
  assert(robot);
  robot_ = robot;
  last_time_ = robot->getTime();

  joint_state_ = robot_->getJointState(joint_name);
  if (!joint_state_)
  {
    ROS_ERROR("JointPendulumController could not find joint named \"%s\"\n",
              joint_name.c_str());
    return false;
  }
  if (!joint_state_->calibrated_)
  {
    ROS_ERROR("Joint %s not calibrated for JointPendulumController", joint_name.c_str());
    return false;
  }

  pid_controller_ = pid;

  return true;
}

bool JointPendulumController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;

  std::string joint_name;
  if (!node_.getParam("joint", joint_name)) {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  control_toolbox::Pid pid;
  if (!pid.init(ros::NodeHandle(node_, "pid")))
    return false;

  controller_state_publisher_.reset(
    new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
    (node_, "state", 1));

  sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &JointPendulumController::setCommandCB, this);

  return init(robot, joint_name, pid);
}


void JointPendulumController::setGains(const double &p, const double &i, const double &d, const double &i_max, const double &i_min)
{
  pid_controller_.setGains(p,i,d,i_max,i_min);
}

void JointPendulumController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_.getGains(p,i,d,i_max,i_min);
}

std::string JointPendulumController::getJointName()
{
  return joint_state_->joint_->name;
}

// Set the joint position command
void JointPendulumController::setCommand(double cmd)
{
  command_ = cmd;
}

// Return the current position command
void JointPendulumController::getCommand(double & cmd)
{
  cmd = command_;
}

void JointPendulumController::update()
{
  if (!joint_state_->calibrated_)
    return;

  assert(robot_ != NULL);
  double error(0);
  ros::Time time = robot_->getTime();
  assert(joint_state_->joint_);
  dt_= time - last_time_;

  if (!initialized_)
  {
    initialized_ = true;
    command_ = joint_state_->position_;
    last_position_ = joint_state_->position_;
    last_last_position_ = joint_state_->position_;
  }

  if(joint_state_->joint_->type == urdf::Joint::REVOLUTE)
  {
    error = joint_state_->position_ - command_;
  }
  else if(joint_state_->joint_->type == urdf::Joint::CONTINUOUS)
  {
    error = angles::shortest_angular_distance(command_, joint_state_->position_);
  }
  else //prismatic
  {
    error = joint_state_->position_ - command_;
  }

  double commanded_effort = pid_controller_.updatePid(error, dt_);
  //joint_state_->commanded_effort_ = commanded_effort;

  // hack for compensation
  double g = 1.0;
  double p = angles::shortest_angular_distance(joint_state_->position_,M_PI/2.0);
  //double f = g*sin(p);
  //joint_state_->commanded_effort_ = f;

  //ROS_ERROR("debug %f %f %f",g,p,f);



  // analytical solution of the pendulum
  // d^2 theta / dt^2 + m*g*l / I sin (theta) = 0
  // theta_new = 2*theta_1 - theta_2 - dt^2 * m*g*l / I * sin(theta_1)
  // we can check error per time step.
  // taking last position 
  double m = 1;
  double l = 1;
  double I = m*l*l;


  bool nj = 1000; // discretize time step into small steps
  ros::Time tau = time; // start time
  double dtau = dt_.toSec() / (double)nj;  // in seconds
  double theta = last_position_;
  // integrate from last position to this position
  for (int j=0;j<nj;j++)
  {
    // solve for new position
    theta = theta + dtau * (   (last_position_ - last_last_position_) / dt_.toSec()
                                      - 0.5 * (dtau + dt_.toSec()) *m * g * l / I * sin(theta) ) ;
  }
  ROS_WARN("pendulum %20.15f  %20.15f",joint_state_->position_,joint_state_->position_ - theta);


  // state publishes
  if(loop_count_ % 10 == 0)  // hard code to publish once every 10 steps
  {
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = joint_state_->position_;
      controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
      controller_state_publisher_->msg_.error = error;
      controller_state_publisher_->msg_.time_step = dt_.toSec();
      controller_state_publisher_->msg_.command = commanded_effort;

      double dummy;
      getGains(controller_state_publisher_->msg_.p,
               controller_state_publisher_->msg_.i,
               controller_state_publisher_->msg_.d,
               controller_state_publisher_->msg_.i_clamp,
               dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;

  last_time_ = time;
  last_last_position_ = last_position_;
  last_position_ = joint_state_->position_;
}

void JointPendulumController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

}
