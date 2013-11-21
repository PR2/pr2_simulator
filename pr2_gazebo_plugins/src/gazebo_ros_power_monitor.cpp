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

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <diagnostic_updater/diagnostic_updater.h>

#include "pr2_gazebo_plugins/gazebo_ros_power_monitor.h"

using namespace std;

namespace gazebo {

GazeboRosPowerMonitor::GazeboRosPowerMonitor()
{
    // model_ = dynamic_cast<Model*>(parent);
    // if (!model_)
    //     gzthrow("GazeboRosPowerMonitor controller requires a Model as its parent");

    // // Initialize parameters
    // Param::Begin(&parameters);
    // robot_namespace_param_   = new ParamT<string>("robotNamespace",     "/",           0);
    // power_state_topic_param_ = new ParamT<string>("powerStateTopic",    "power_state", 0);
    // power_state_rate_param_  = new ParamT<double>("powerStateRate",        1.0,        0);
    // full_capacity_param_     = new ParamT<double>("fullChargeCapacity",   80.0,        0);
    // discharge_rate_param_    = new ParamT<double>("dischargeRate",      -500.0,        0);
    // charge_rate_param_       = new ParamT<double>("chargeRate",         1000.0,        0);
    // discharge_voltage_param_ = new ParamT<double>("dischargeVoltage",     16.0,        0);
    // charge_voltage_param_    = new ParamT<double>("chargeVoltage",        16.0,        0);
    // Param::End();
}

GazeboRosPowerMonitor::~GazeboRosPowerMonitor()
{
    // Do nothing
    this->rosnode_->shutdown();

    delete rosnode_;

    // delete robot_namespace_param_;
    // delete power_state_topic_param_;
    // delete power_state_rate_param_;
    // delete full_capacity_param_;
    // delete discharge_rate_param_;
    // delete charge_rate_param_;
    // delete discharge_voltage_param_;
    // delete charge_voltage_param_;
}

void GazeboRosPowerMonitor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Get the world name.
  this->world = _parent->GetWorld();

  // Get a pointer to the model
  this->parent_model_ = _parent;

  // Error message if the model couldn't be found
  if (!this->parent_model_)
    gzerr << "Unable to get parent model\n";

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosPowerMonitor::UpdateChild, this));
  gzdbg << "plugin model name: " << modelName << "\n";


    // Load parameters from XML
    this->robot_namespace_     = "";
    this->power_state_topic_   = "";
    this->power_state_rate_    = 0;
    this->full_capacity_       = 0;
    this->discharge_rate_      = 0;
    this->charge_rate_         = 0;
    this->discharge_voltage_   = 0;
    this->charge_voltage_      = 0;

    if (!ros::isInitialized())
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    }

    this->robot_namespace_ = "";
    rosnode_        = new ros::NodeHandle(this->robot_namespace_);
    power_state_pub_ = rosnode_->advertise<pr2_msgs::PowerState>(this->power_state_topic_, 10);
    plugged_in_sub_  = rosnode_->subscribe("plugged_in", 10, &GazeboRosPowerMonitor::SetPlug, this);
}

void GazeboRosPowerMonitor::InitChild()
{
    last_time_ = curr_time_ = this->world->GetSimTime().Double();

    // Initialize battery to full capacity
    charge_      = this->full_capacity_;
    charge_rate_ = this->discharge_rate_;
    voltage_     = this->discharge_voltage_;
}

void GazeboRosPowerMonitor::UpdateChild()
{
    // Update time
    curr_time_ = this->world->GetSimTime().Double();
    double dt = curr_time_ - last_time_;
    last_time_ = curr_time_;

    // Update charge
    double current = charge_rate_ / voltage_;
    charge_ += (dt / 3600) * current;   // charge is measured in ampere-hours, simulator time is measured in secs

    // Clamp to [0, full_capacity]
    if (charge_ < 0)
        charge_ = 0;
    if (charge_ > this->full_capacity_)
        charge_ = this->full_capacity_;

    // Publish power state (simulate the power_monitor node)
    power_state_.header.stamp.fromSec(curr_time_);

    power_state_.power_consumption = charge_rate_;

    if (current < 0.0)
        power_state_.time_remaining = ros::Duration((charge_ / -current) * 60);      // time remaining reported in hours
    else
    {
        double charge_to_full = this->full_capacity_ - charge_;
        if (charge_to_full == 0.0)
            power_state_.time_remaining = ros::Duration(0);
        else if (current == 0.0)
            power_state_.time_remaining = ros::Duration(65535, 65535);               // zero current - time_remaining is undefined
        else
            power_state_.time_remaining = ros::Duration((charge_to_full / current) * 60);
    }

    power_state_.prediction_method = "fuel gauge";
    power_state_.relative_capacity = (int) (100.0 * (charge_ / this->full_capacity_));

    lock_.lock();
    power_state_pub_.publish(power_state_);
    lock_.unlock();
}

void GazeboRosPowerMonitor::SetPlug(const pr2_gazebo_plugins::PlugCommandConstPtr& plug_msg)
{
    lock_.lock();

    if (plug_msg->charge_rate > 0.0)
    {
      this->charge_rate_ = plug_msg->charge_rate;
      ROS_DEBUG("debug: charge rate %f",this->charge_rate_);
    }
    if (plug_msg->discharge_rate < 0.0)
    {
      this->discharge_rate_ = plug_msg->discharge_rate;
      ROS_DEBUG("debug: discharge rate %f",this->discharge_rate_);
    }

    charge_ = plug_msg->charge;
    ROS_DEBUG("debug: charge %f",charge_);

    if (plug_msg->ac_present)
    {
        charge_rate_            = this->charge_rate_ + this->discharge_rate_;
        power_state_.AC_present = 4;
    }
    else
    {
        charge_rate_            = this->discharge_rate_;
        power_state_.AC_present = 0;
    }

    lock_.unlock();
}

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboRosPowerMonitor)

}
