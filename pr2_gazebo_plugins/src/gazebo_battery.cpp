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

#include <pr2_gazebo_plugins/gazebo_battery.h>
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
#include <map>

namespace gazebo {

  GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_battery", GazeboBattery);

  GazeboBattery::GazeboBattery(Entity *parent): Controller(parent)
  {
    this->parent_model_ = dynamic_cast<Model*>(this->parent);

    if (!this->parent_model_)
       gzthrow("GazeboBattery controller requires a Model as its parent");

    Param::Begin(&this->parameters);
    this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
    this->stateTopicNameP = new ParamT<std::string>("stateTopicName","power_state",0);
    //this->diagnosticMessageTopicNameP = new ParamT<std::string>("diagnosticMessageTopicName","diagnostic",0);
    this->default_consumption_rateP       = new ParamT<double>("default_consumption_rate",-10.0,0);
    this->full_capacityP                  = new ParamT<double>("full_charge_energy",0.0,0);
    this->default_charge_rateP            = new ParamT<double>("default_charge_rate",-10.0,0);
    //this->diagnostic_rateP     = new ParamT<double>("diagnostic_rate",1.0,0);
    this->power_state_rateP  = new ParamT<double>("power_state_rate_",1.0,0);
    Param::End();

  }

  GazeboBattery::~GazeboBattery()
  {
    delete this->rosnode_;

    delete this->robotNamespaceP;
    delete this->stateTopicNameP;
    //delete this->diagnosticMessageTopicNameP;
    delete this->default_consumption_rateP;
    delete this->full_capacityP;
    delete this->default_charge_rateP;
    //delete this->diagnostic_rateP;
    delete this->power_state_rateP;
  }

  void GazeboBattery::LoadChild(XMLConfigNode *node)
  {

    this->robotNamespaceP->Load(node);
    this->robotNamespace = this->robotNamespaceP->GetValue();
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo");
    this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

    this->stateTopicNameP->Load(node);
    this->stateTopicName_ = this->stateTopicNameP->GetValue();
    this->pub_ = this->rosnode_->advertise<pr2_msgs::PowerState>(this->stateTopicName_,10);

    //this->diagnosticMessageTopicNameP->Load(node);
    //this->diagnosticMessageTopicName_ = this->diagnosticMessageTopicNameP->GetValue();
    //this->diag_pub_ = this->rosnode_->advertise<diagnostic_msgs::DiagnosticArray>(this->diagnosticMessageTopicName_,10);

    /// faking the plug and unplug of robot
    this->sub_ = this->rosnode_->subscribe("plugged_in",10,&GazeboBattery::SetPlug,this);

    this->default_consumption_rateP->Load(node);
    this->default_consumption_rate_       = this->default_consumption_rateP->GetValue();
    this->full_capacityP->Load(node);
    this->full_capacity_       = this->full_capacityP->GetValue();
    this->default_charge_rateP->Load(node);
    this->default_charge_rate_ = this->default_charge_rateP->GetValue();

    /// @todo make below useful
    //this->diagnostic_rateP->Load(node);
    //this->diagnostic_rate_     = //this->diagnostic_rateP->GetValue();
    /// @todo make below useful
    this->power_state_rateP->Load(node);
    this->power_state_rate_  = this->power_state_rateP->GetValue();
  }

  void GazeboBattery::SetPlug(const pr2_gazebo_plugins::PlugCommandConstPtr& plug_msg)
  {
    this->lock_.lock();
    if (plug_msg->status == "the robot is very much plugged into the wall")
    {
      this->consumption_rate_ = this->default_charge_rate_ + this->default_consumption_rate_;
      this->battery_state_.AC_present = 4;
    }
    else
    {
      this->consumption_rate_ = this->default_consumption_rate_;
      this->battery_state_.AC_present = 0;
    }
    this->lock_.unlock();
  }

  void GazeboBattery::InitChild()
  {
    this->current_time_ = Simulator::Instance()->GetSimTime();
    this->last_time_    = this->current_time_;

    /// initialize battery
    this->charge_           = this->full_capacity_; /// our convention is joules
    this->consumption_rate_ = this->default_consumption_rate_; /// time based decay rate in watts
  }

  void GazeboBattery::UpdateChild()
  {
    this->current_time_ = Simulator::Instance()->GetSimTime();

    /**********************************************************/
    /*                                                        */
    /*   update battery                                       */
    /*                                                        */
    /**********************************************************/
    this->charge_ = this->charge_ + (this->current_time_ - this->last_time_)*this->consumption_rate_;
    if (this->charge_ < 0) this->charge_ = 0;
    if (this->charge_ > this->full_capacity_) this->charge_ = this->full_capacity_;
    //std::cout << " battery charge remaining: " << this->charge_ << " Joules " << std::endl;

    /**********************************************************/
    /*                                                        */
    /* publish power state                                    */
    /*                                                        */
    /**********************************************************/
    //this->power_state_.header.frame_id = ; // no frame id for battery
    this->power_state_.header.stamp.sec = (unsigned long)floor(this->current_time_);
    this->power_state_.header.stamp.nsec = (unsigned long)floor(  1e9 * (  this->current_time_ - this->power_state_.header.stamp.sec) );
    if (this->consumption_rate_ > 0)
        this->power_state_.time_remaining = this->charge_ / this->consumption_rate_;
    else
        this->power_state_.time_remaining = 65535;
    //this->power_state_.energy_capacity = this->full_capacity_;
    this->power_state_.power_consumption = this->consumption_rate_;

    this->lock_.lock();
    this->pub_.publish(this->power_state_);
    this->lock_.unlock();
    
    /**********************************************************/
    /*                                                        */
    /* publish diagnostic message                             */
    /*                                                        */
    /**********************************************************/
    //this->diagnostic_status_.level = 0;
    //this->diagnostic_status_.name = "battery diagnostic";
    //this->diagnostic_status_.message = "battery ok";
    //this->diagnostic_message_.header = this->power_state_.header;
    //this->diagnostic_message_.set_status_size(1);
    //this->diagnostic_message_.status[0] = this->diagnostic_status_;
    //this->lock_.lock();
    //this->diag_pub_.publish(this->diagnosticMessageTopicName_,diagnostic_message_);
    //this->lock_.unlock();

    this->last_time_    = this->current_time_;
  }

  void GazeboBattery::FiniChild()
  {
    ROS_DEBUG("Calling FiniChild in GazeboBattery");
  }



} // namespace gazebo


