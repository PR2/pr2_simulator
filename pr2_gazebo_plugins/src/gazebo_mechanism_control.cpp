/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <pr2_gazebo_plugins/gazebo_mechanism_control.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <set>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <urdf/model.h>
#include <map>

namespace gazebo {

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_mechanism_control", GazeboMechanismControl);

GazeboMechanismControl::GazeboMechanismControl(Entity *parent)
  : Controller(parent), hw_(0), mc_(0), fake_state_(NULL)
{
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboMechanismControl controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotParamP = new ParamT<std::string>("robotParam", "robot_description_new", 0);
  Param::End();

  if (getenv("CHECK_SPEEDUP"))
  {
    wall_start = Simulator::Instance()->GetWallTime();
    sim_start  = Simulator::Instance()->GetSimTime();
  }

  int argc = 0;
  char** argv = NULL;
  ros::init(argc,argv,"gazebo");
  this->rosnode_ = new ros::NodeHandle("pr2_mechanism_control");

  this->mc_ = new pr2_mechanism::MechanismControl(&hw_);
}

GazeboMechanismControl::~GazeboMechanismControl()
{
  delete this->robotParamP;
  delete this->mc_; 
  delete this->rosnode_;
}

void GazeboMechanismControl::LoadChild(XMLConfigNode *node)
{
  // get parameter name
  this->robotParamP->Load(node);
  this->robotParam = this->robotParamP->GetValue();

  // read pr2.xml (pr2_gazebo_mechanism_control.xml)
  // setup actuators, then setup mechanism control node
  ReadPr2Xml(node);

  // Initializes the fake state (for running the transmissions backwards).
  this->fake_state_ = new pr2_mechanism::RobotState(&this->mc_->model_, &this->hw_);

  // The gazebo joints and mechanism joints should match up.
  for (unsigned int i = 0; i < this->mc_->model_.joints_.size(); ++i)
  {
    std::string joint_name = this->mc_->model_.joints_[i]->name_;

    // fill in gazebo joints pointer
    gazebo::Joint *joint = this->parent_model_->GetJoint(joint_name);
    if (joint)
    {
      this->joints_.push_back(joint);
    }
    else
    {
      ROS_WARN("A joint named \"%s\" is not part of Mechanism Controlled joints.\n", joint_name.c_str());
      this->joints_.push_back(NULL);
    }

  }

  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime());
}

void GazeboMechanismControl::InitChild()
{
  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime());
}

void GazeboMechanismControl::UpdateChild()
{

  if (getenv("CHECK_SPEEDUP"))
  {
    double wall_elapsed = Simulator::Instance()->GetWallTime() - wall_start;
    double sim_elapsed  = Simulator::Instance()->GetSimTime()  - sim_start;
    std::cout << " real time: " <<  wall_elapsed
              << "  sim time: " <<  sim_elapsed
              << "  speed up: " <<  sim_elapsed / wall_elapsed
              << std::endl;
  }
  assert(this->joints_.size() == this->fake_state_->joint_states_.size());

  //--------------------------------------------------
  //  Pushes out simulation state
  //--------------------------------------------------

  // Copies the state from the gazebo joints into the mechanism joints.
  for (unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    if (!this->joints_[i])
      continue;

    this->fake_state_->joint_states_[i].applied_effort_ = this->fake_state_->joint_states_[i].commanded_effort_;

    switch(this->joints_[i]->GetType())
    {
    case Joint::HINGE: {
      HingeJoint *hj = (HingeJoint*)this->joints_[i];
      this->fake_state_->joint_states_[i].position_ = hj->GetAngle();
      this->fake_state_->joint_states_[i].velocity_ = hj->GetAngleRate();
      break;
    }
    case Joint::SLIDER: {
      SliderJoint *sj = (SliderJoint*)this->joints_[i];
      this->fake_state_->joint_states_[i].position_ = sj->GetPosition();
      this->fake_state_->joint_states_[i].velocity_ = sj->GetPositionRate();
      break;
    }
    default:
      abort();
    }
  }

  // Reverses the transmissions to propagate the joint position into the actuators.
  this->fake_state_->propagateStateBackwards();

  //--------------------------------------------------
  //  Runs Mechanism Control
  //--------------------------------------------------
  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime());
  try
  {
    this->mc_->update();
  }
  catch (const char* c)
  {
    if (strcmp(c,"dividebyzero")==0)
      ROS_WARN("pid controller reports divide by zero error");
    else
      ROS_WARN("unknown const char* exception: %s", c);
  }

  //--------------------------------------------------
  //  Takes in actuation commands
  //--------------------------------------------------

  // Reverses the transmissions to propagate the actuator commands into the joints.
  this->fake_state_->propagateEffortBackwards();

  // Copies the commands from the mechanism joints into the gazebo joints.
  for (unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    if (!this->joints_[i])
      continue;

    double damping_force;
    double effort = this->fake_state_->joint_states_[i].commanded_effort_;
    switch (this->joints_[i]->GetType())
    {
    case Joint::HINGE:
      //std::cout << " hinge joint name " << mc_->model_.joints_[i]->name_ << std::endl;
      //std::cout << " check damping coef " << mc_->model_.joints_[i]->joint_damping_coefficient_ << std::endl;
      damping_force = this->mc_->model_.joints_[i]->joint_damping_coefficient_ * ((HingeJoint*)this->joints_[i])->GetAngleRate();
      ((HingeJoint*)this->joints_[i])->SetTorque(effort - damping_force);
      break;
    case Joint::SLIDER:
      //std::cout << " slider joint name " << mc_->model_.joints_[i]->name_ << std::endl;
      //std::cout << " check damping coef " << mc_->model_.joints_[i]->joint_damping_coefficient_ << std::endl;
      damping_force = this->mc_->model_.joints_[i]->joint_damping_coefficient_ * ((SliderJoint*)this->joints_[i])->GetPositionRate();
      ((SliderJoint*)this->joints_[i])->SetSliderForce(effort - damping_force);
      break;
    default:
      abort();
    }
  }
}

void GazeboMechanismControl::FiniChild()
{
  ROS_DEBUG("Calling FiniChild in GazeboMechanismControl");

  this->hw_.~HardwareInterface();
  this->mc_->~MechanismControl();

  pr2_mechanism::deleteElements(&this->joints_);
  delete this->fake_state_;
}

void GazeboMechanismControl::ReadPr2Xml(XMLConfigNode *node)
{

  std::string urdf_param_name;
  this->rosnode_->searchParam(this->robotParam,urdf_param_name);
  std::string urdf_string;
  this->rosnode_->getParam(urdf_param_name,urdf_string);


  // wait for robot_description on param server
  while(urdf_string.empty())
  {
    ROS_WARN("gazebo mechanism control plugin is waiting for %s in param server.  run merge/roslaunch send.xml or similar.", this->robotParam.c_str());
    this->rosnode_->getParam(this->robotParam,urdf_string);
    usleep(100000);
  }

  ROS_INFO("gazebo mechanism control got pr2.xml from param server, parsing it...");
  //std::cout << urdf_string << std::endl;

  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf_string.c_str()))
  {
    ROS_ERROR("Could not load the gazebo mechanism_control plugin's configuration file: %s\n",
            urdf_string.c_str());
    abort();
  }
  //std::cout << *(doc.RootElement()) << std::endl;

  // Pulls out the list of actuators used in the robot configuration.
  struct GetActuators : public TiXmlVisitor
  {
    std::set<std::string> actuators;
    virtual bool VisitEnter(const TiXmlElement &elt, const TiXmlAttribute *)
    {
      if (elt.ValueStr() == std::string("actuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      else if (elt.ValueStr() == std::string("rightActuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      else if (elt.ValueStr() == std::string("leftActuator") && elt.Attribute("name"))
        actuators.insert(elt.Attribute("name"));
      return true;
    }
  } get_actuators;
  doc.RootElement()->Accept(&get_actuators);

  // Places the found actuators into the hardware interface.
  std::set<std::string>::iterator it;
  for (it = get_actuators.actuators.begin(); it != get_actuators.actuators.end(); ++it)
  {
    //std::cout << " adding actuator " << (*it) << std::endl;
    this->hw_.actuators_.push_back(new pr2_mechanism::Actuator(*it));
  }

  // Setup mechanism control node
  this->mc_->initXml(doc.RootElement());

  for (unsigned int i = 0; i < this->mc_->state_->joint_states_.size(); ++i)
    this->mc_->state_->joint_states_[i].calibrated_ = true;
}

} // namespace gazebo
