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

#include <gazebo/gazebo.h>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/Simulator.hh>
#include <pr2_gazebo_plugins/gazebo_ros_ocean_battery.h>
#include <diagnostic_updater/diagnostic_updater.h>

using namespace std;

namespace gazebo {

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_ocean_battery", GazeboRosOceanBattery);

GazeboRosOceanBattery::GazeboRosOceanBattery(Entity* parent) : Controller(parent)
{
    model_ = dynamic_cast<Model*>(parent);
    if (!model_)
        gzthrow("GazeboRosOceanBattery controller requires a Model as its parent");

    // Initialize parameters
    Param::Begin(&parameters);
    robot_namespace_param_   = new ParamT<string>("robotNamespace",     "/",           0);
    power_state_topic_param_ = new ParamT<string>("powerStateTopic",    "power_state", 0);
    power_state_rate_param_  = new ParamT<double>("powerStateRate",        1.0,        0);
    full_capacity_param_     = new ParamT<double>("fullChargeCapacity",   80.0,        0);
    discharge_rate_param_    = new ParamT<double>("dischargeRate",      -500.0,        0);
    charge_rate_param_       = new ParamT<double>("chargeRate",          500.0,        0);
    discharge_voltage_param_ = new ParamT<double>("dischargeVoltage",     16.0,        0);
    charge_voltage_param_    = new ParamT<double>("chargeVoltage",        16.0,        0);
    Param::End();
}

GazeboRosOceanBattery::~GazeboRosOceanBattery()
{
    delete ros_node_;

    delete robot_namespace_param_;
    delete power_state_topic_param_;
    delete power_state_rate_param_;
    delete full_capacity_param_;
    delete discharge_rate_param_;
    delete charge_rate_param_;
    delete discharge_voltage_param_;
    delete charge_voltage_param_;
}

void GazeboRosOceanBattery::LoadChild(XMLConfigNode* configNode)
{
    // Load parameters from XML
    robot_namespace_param_->Load(configNode);
    power_state_topic_param_->Load(configNode);
    power_state_rate_param_->Load(configNode);
    full_capacity_param_->Load(configNode);
    discharge_rate_param_->Load(configNode);
    charge_rate_param_->Load(configNode);
    discharge_voltage_param_->Load(configNode);
    charge_voltage_param_->Load(configNode);

    int argc = 0;
    char* argv = NULL;
    ros::init(argc, &argv, "gazebo_ros_ocean_battery", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);

    ros_node_        = new ros::NodeHandle(robot_namespace_param_->GetValue());
    power_state_pub_ = ros_node_->advertise<pr2_msgs::PowerState>(power_state_topic_param_->GetValue(), 10);
    plugged_in_sub_  = ros_node_->subscribe("plugged_in", 10, &GazeboRosOceanBattery::SetPlug, this);
}

void GazeboRosOceanBattery::InitChild()
{
    last_time_ = curr_time_ = Simulator::Instance()->GetSimTime();

    // Initialize ocean battery to full capacity
    charge_      = full_capacity_param_->GetValue();
    charge_rate_ = discharge_rate_param_->GetValue();
    voltage_     = discharge_voltage_param_->GetValue();
}

void GazeboRosOceanBattery::UpdateChild()
{
    // Update time
    curr_time_ = Simulator::Instance()->GetSimTime();
    Time dt = curr_time_ - last_time_;
    last_time_ = curr_time_;

    // Update ocean battery charge
    double current = charge_rate_ / voltage_;
    charge_ += (dt.Double() / 3600) * current;   // charge is measured in ampere-hours, simulator time is measured in secs

    // Clamp to [0, full_capacity]
    if (charge_ < 0)
        charge_ = 0;
    if (charge_ > full_capacity_param_->GetValue())
        charge_ = full_capacity_param_->GetValue();

    // Publish power state (simulate the power_monitor node)
    power_state_.header.stamp.sec = curr_time_.sec;
    power_state_.header.stamp.nsec = curr_time_.nsec;

    power_state_.power_consumption = charge_rate_;
    if (current < 0.0)
        power_state_.time_remaining = ros::Duration((-charge_ / current) * 60);  // time remaining reported in hours
    else
        power_state_.time_remaining = ros::Duration(65535,65535);
    power_state_.prediction_method = "fuel gauge";
    power_state_.relative_capacity = (int) (100.0 * (charge_ / full_capacity_param_->GetValue()));

    lock_.lock();
    power_state_pub_.publish(power_state_);
    lock_.unlock();
}

void GazeboRosOceanBattery::FiniChild()
{
    // Do nothing
}

void GazeboRosOceanBattery::SetPlug(const pr2_gazebo_plugins::PlugCommandConstPtr& plug_msg)
{
    lock_.lock();

    if (plug_msg->status == "the robot is very much plugged into the wall")
    {
        charge_rate_            = charge_rate_param_->GetValue() + discharge_rate_param_->GetValue();
        power_state_.AC_present = 4;
    }
    else
    {
        charge_rate_            = discharge_rate_param_->GetValue();
        power_state_.AC_present = 0;
    }

    lock_.unlock();
}

}
