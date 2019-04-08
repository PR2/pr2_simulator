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
{}

GazeboRosPowerMonitor::~GazeboRosPowerMonitor()
{
    this->update_connection_.reset();
    this->queue_.clear();
    this->queue_.disable();
    this->rosnode_->shutdown();
    this->callback_queue_thread_.join();
    delete rosnode_;
}

void GazeboRosPowerMonitor::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->sdf_ = _sdf;
  this->deferred_load_thread_ = boost::thread(
      boost::bind(&GazeboRosPowerMonitor::LoadThread, this));
}

void GazeboRosPowerMonitor::LoadThread()
{
  if (this->sdf_->HasElement("robotNamespace"))
    this->robot_namespace_ = this->sdf_->GetElement("robotNamespace")->Get<std::string>() + "/";
  else
    this->robot_namespace_ = "";

  if (!this->sdf_->HasElement("powerStateTopic"))
  {
    ROS_INFO_NAMED("power_monitor", "power_monitor plugin missing <powerStateTopic>, defaults to \"power_state\"");
    this->power_state_topic_ = "power_state";
  }
  else
    this->power_state_topic_ = this->sdf_->GetElement("powerStateTopic")->Get<std::string>();

  if (!this->sdf_->HasElement("powerStateRate"))
  {
    ROS_INFO_NAMED("power_monitor", "power_monitor plugin missing <powerStateRate>, defaults to 1.0");
    this->power_state_rate_ = 1.0;
  }
  else
    this->power_state_rate_ = this->sdf_->GetElement("powerStateRate")->Get<double>();

  if (!this->sdf_->HasElement("fullChargeCapacity"))
  {
    ROS_INFO_NAMED("power_monitor", "power_monitor plugin missing <fullChargeCapacity>, defaults to 80.0");
    this->full_capacity_ = 80.0;
  }
  this->full_capacity_ = this->sdf_->GetElement("fullChargeCapacity")->Get<double>();

  if (!this->sdf_->HasElement("dischargeRate"))
  {
    ROS_INFO_NAMED("power_monitor", "power_monitor plugin missing <dischargeRate>, defaults to -500.0");
    this->discharge_rate_ = -500.0;
  }
  else
    this->discharge_rate_ = this->sdf_->GetElement("dischargeRate")->Get<double>();

  if (!this->sdf_->HasElement("chargeRate"))
  {
    ROS_INFO_NAMED("power_monitor", "power_monitor plugin missing <chargeRate>, defaults to 1000.0");
    this->charge_rate_ = 1000.0;
  }
  else
    this->charge_rate_ = this->sdf_->GetElement("chargeRate")->Get<double>();

  if (!this->sdf_->HasElement("dischargeVoltage"))
  {
    ROS_INFO_NAMED("power_monitor", "power_monitor plugin missing <dischargeVoltage>, defaults to 16.0");
    this->discharge_voltage_ = 16.0;
  }
  else
    this->discharge_voltage_ = this->sdf_->GetElement("dischargeVoltage")->Get<double>();

  if (!this->sdf_->HasElement("chargeVoltage"))
  {
    ROS_INFO_NAMED("power_monitor", "power_monitor plugin missing <chargeVoltage>, defaults to 16.0");
    this->charge_voltage_ = 16.0;
  }
  else
    this->charge_voltage_ = this->sdf_->GetElement("chargeVoltage")->Get<double>();

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("power_monitor", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
                           << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Initialize ROS functions
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  ros::AdvertiseOptions opts = ros::AdvertiseOptions::create<pr2_msgs::PowerState>(
      this->power_state_topic_, 10,
      boost::bind(&GazeboRosPowerMonitor::ConnectCb, this),
      boost::bind(&GazeboRosPowerMonitor::DisconnectCb, this),
      ros::VoidPtr(), &this->queue_);
  this->power_state_pub_ = this->rosnode_->advertise(opts);

  this->plugged_in_sub_  = this->rosnode_->subscribe(
      "plugged_in", 10, &GazeboRosPowerMonitor::SetPlug, this);

  this->callback_queue_thread_ = boost::thread(
      boost::bind(&GazeboRosPowerMonitor::QueueThread, this));

  // Initialize internal variables
  charge_      = this->full_capacity_;
  charge_rate_ = this->discharge_rate_;
  voltage_     = this->discharge_voltage_;
#if GAZEBO_MAJOR_VERSION >= 8
  last_time_ = this->world_->SimTime().Double();
#else
  last_time_ = this->world_->GetSimTime().Double();
#endif

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosPowerMonitor::UpdateChild, this));
}

void GazeboRosPowerMonitor::ConnectCb()
{
  this->connect_count_++;
}

void GazeboRosPowerMonitor::DisconnectCb()
{
  this->connect_count_--;
}

void GazeboRosPowerMonitor::QueueThread()
{
  static const double timeout = 0.01;
  while (this->rosnode_->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosPowerMonitor::UpdateChild()
{
  // Update time
#if GAZEBO_MAJOR_VERSION >= 8
  double curr_time_ = this->world_->SimTime().Double();
#else
  double curr_time_ = this->world_->GetSimTime().Double();
#endif
  double dt = curr_time_ - last_time_;

  if (dt < this->power_state_rate_)
    return;

  last_time_ = curr_time_;

  if (connect_count_ == 0)
    return;

  // Update charge
  double current = 0.0;
  if (voltage_ > 0) {
    current = charge_rate_ / voltage_;
    // charge is measured in ampere-hours, simulator time is measured in secs
    charge_ += (dt / 3600) * current;
  }

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
